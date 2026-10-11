/*
  Golden Motor EZkontrol dual-motor Wi-Fi dashboard

  Hardware: Waveshare ESP32-S3-RS485-CAN
    CAN TX GPIO15, CAN RX GPIO16
    Onboard TJA1051 CAN transceiver is permanently enabled
    Use the board's USB Type-C port for flashing/logging

  Controller setup:
    Port controller:      Part number 239 (0xEF)
    Starboard controller: Part number 240 (0xF0)
    Set in EZkontrol app: Setting -> Communication -> Part number

  Connect a phone/tablet/computer to:
    Wi-Fi:   EZkontrol-Dash
    Password: GoldenMotor
    Browser: http://192.168.4.1

  This edition is deliberately local-only:
    - The ESP32 creates its own Wi-Fi access point
    - The web dashboard and JSON API are served directly by the ESP32
    - It never joins another Wi-Fi network or sends data to the internet

  CAN behaviour:
    - 250 kbit/s, extended Golden Motor meter frames
    - Normal mode supplies ACK bits so both controllers keep transmitting
    - TX queue is disabled and this sketch never calls twai_transmit()
    - The web UI is display-only and exposes no control commands

  Serial monitor: 115200 baud
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <U8g2lib.h>
#include "driver/twai.h"

static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_15;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_16;
static constexpr int OLED_SDA_PIN = 1;  // J4 pin 1
static constexpr int OLED_SCL_PIN = 2;  // J4 pin 2
static constexpr uint8_t PORT_OLED_ADDRESS_7BIT = 0x3C;
static constexpr uint8_t PORT_OLED_ADDRESS_U8G2 = 0x78;
static constexpr uint8_t STARBOARD_OLED_ADDRESS_7BIT = 0x3D;
static constexpr uint8_t STARBOARD_OLED_ADDRESS_U8G2 = 0x7A;
static constexpr uint32_t OLED_REFRESH_MS = 200;
static constexpr uint32_t OLED_RETRY_MS = 2000;
static constexpr uint8_t PORT_ADDRESS = 0xEF;       // Part number 239
static constexpr uint8_t STARBOARD_ADDRESS = 0xF0; // Part number 240
static constexpr uint32_t FRAME_1_MASK = 0xFFFFFF00;
static constexpr uint32_t FRAME_1_BASE = 0x18011700;
static constexpr uint32_t FRAME_2_BASE = 0x18021700;
static constexpr uint32_t STALE_TIMEOUT_MS = 1000;
static constexpr size_t CURRENT_AVERAGE_SAMPLES = 20;

static const char *AP_SSID = "EZkontrol-Dash";
static const char *AP_PASSWORD = "GoldenMotor";

WebServer server(80);
DNSServer dnsServer;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C portOled(U8G2_R0, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C starboardOled(U8G2_R0, U8X8_PIN_NONE);

struct MotorData {
  float voltageV = 0.0f;
  float busCurrentA = 0.0f;
  float phaseCurrentA = 0.0f;
  float averageBusCurrentA = 0.0f;
  float averagePhaseCurrentA = 0.0f;
  float busCurrentSamples[CURRENT_AVERAGE_SAMPLES] = {};
  float phaseCurrentSamples[CURRENT_AVERAGE_SAMPLES] = {};
  float busCurrentSum = 0.0f;
  float phaseCurrentSum = 0.0f;
  uint8_t currentSampleIndex = 0;
  uint8_t currentSampleCount = 0;
  int16_t rpm = 0;
  int16_t controllerTempC = 0;
  int16_t motorTempC = 0;
  uint8_t throttlePercent = 0;
  uint8_t gear = 0;
  bool brake = false;
  uint8_t runMode = 0;
  bool contactorOn = false;
  uint8_t fault1 = 0;
  uint8_t fault2 = 0;
  uint8_t fault3 = 0;
  uint8_t lifeCounter = 0;
  uint32_t frame1At = 0;
  uint32_t frame2At = 0;
  uint32_t frameCount = 0;
};

static MotorData portMotor;
static MotorData starboardMotor;
static bool portOledReady = false;
static bool starboardOledReady = false;
static uint32_t oledLastProbeAt = 0;
static uint32_t portOledLastDrawAt = 0;
static uint32_t starboardOledLastDrawAt = 0;
static const char DASH_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#061019">
<title>EZkontrol Twin Dash</title>
<style>
:root{--bg:#061019;--panel:#0c1b26;--line:#1e3442;--text:#edf8ff;--muted:#7f9aaa;--cyan:#23d9ff;--blue:#6198ff;--green:#4be28b;--amber:#ffbd4a;--red:#ff5570}
*{box-sizing:border-box}html,body{width:100%;min-width:0}body{margin:0;min-height:100vh;min-height:100svh;background:radial-gradient(circle at 50% -20%,#173548 0,#07141e 42%,#040b11 100%);color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;font-variant-numeric:tabular-nums}
.shell{width:100%;max-width:1600px;min-width:0;margin:auto;padding:max(10px,env(safe-area-inset-top)) max(10px,env(safe-area-inset-right)) max(9px,env(safe-area-inset-bottom)) max(10px,env(safe-area-inset-left))}
header{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:8px}.brand{display:flex;align-items:center;gap:9px;min-width:0}.mark{width:38px;height:38px;flex:0 0 auto;border:1px solid #2b5367;border-radius:11px;display:grid;place-items:center;background:linear-gradient(145deg,#112c3c,#09151e);color:var(--cyan);font-weight:900;box-shadow:0 0 24px #23d9ff18}.brand h1{font-size:17px;letter-spacing:.08em;margin:0;white-space:nowrap}.brand p{margin:2px 0 0;color:var(--muted);font-size:10px;letter-spacing:.06em}.head-actions{display:flex;align-items:center;gap:7px}.browser-btn{appearance:none;border:1px solid #276078;border-radius:999px;padding:8px 12px;background:#0b2634;color:var(--cyan);font:inherit;font-size:10px;font-weight:850;letter-spacing:.08em;cursor:pointer;white-space:nowrap}.browser-btn:active{transform:translateY(1px);background:#10384b}.status{display:flex;align-items:center;gap:7px;padding:8px 11px;flex:0 0 auto;border:1px solid #24533d;border-radius:999px;background:#0c261d;color:var(--green);font-size:10px;font-weight:800;letter-spacing:.08em}.dot{width:8px;height:8px;border-radius:50%;background:currentColor;box-shadow:0 0 12px currentColor}
.summary{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:7px;margin-bottom:8px}.summary-card,.panel{background:linear-gradient(145deg,#0e202cdd,#091721ee);border:1px solid var(--line);box-shadow:0 12px 32px #0005,inset 0 1px #ffffff08}.summary-card{border-radius:12px;padding:8px 11px;min-width:0}.label{color:var(--muted);font-size:9px;font-weight:800;letter-spacing:.13em;text-transform:uppercase}.summary-value{display:block;margin-top:3px;font-size:20px;font-weight:850;white-space:nowrap}.suffix{font-size:11px;color:var(--muted);margin-left:3px}
.motors{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.motor{border-radius:16px;padding:10px;min-width:0}.motor-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:5px}.motor-name{display:flex;align-items:center;gap:7px;font-size:14px;font-weight:900;letter-spacing:.1em}.side-dot{width:8px;height:8px;border-radius:50%;background:var(--accent);box-shadow:0 0 12px var(--accent)}.motor-state{font-size:9px;font-weight:850;letter-spacing:.1em;color:var(--amber)}.motor.online .motor-state{color:var(--green)}
.main{display:grid;grid-template-columns:minmax(155px,.82fr) minmax(175px,1.18fr);gap:9px;align-items:center}.gauge-wrap{display:grid;place-items:center}.gauge{--sweep:0deg;width:clamp(155px,17vw,220px);max-width:100%;aspect-ratio:1;border-radius:50%;background:conic-gradient(from 225deg,var(--accent) 0 var(--sweep),#19313f var(--sweep) 270deg,transparent 270deg);position:relative;filter:drop-shadow(0 0 18px color-mix(in srgb,var(--accent) 18%,transparent))}.gauge:before{content:"";position:absolute;inset:13px;border-radius:50%;background:radial-gradient(circle at 50% 36%,#173044,#09151e 68%);border:1px solid #244052}.gauge-content{position:absolute;inset:0;display:grid;place-content:center;text-align:center;z-index:2}.rpm{font-size:clamp(39px,4.6vw,61px);font-weight:850;line-height:.9;letter-spacing:-.07em}.unit{margin-top:6px;color:var(--muted);font-size:10px;letter-spacing:.2em}.signed{margin-top:4px;color:#a5dcec;font-size:10px}
.drive{display:flex;flex-direction:column;gap:8px}.direction{display:grid;grid-template-columns:repeat(3,1fr);gap:5px}.dir{padding:7px 3px;border:1px solid #263e4b;border-radius:8px;text-align:center;color:#5e7684;font-size:11px;font-weight:850;letter-spacing:.08em;background:#08141c}.dir.active{color:#05131a;background:var(--accent);border-color:var(--accent);box-shadow:0 0 14px color-mix(in srgb,var(--accent) 30%,transparent)}.dir.r.active{background:var(--amber);border-color:var(--amber)}.throttle-head{display:flex;justify-content:space-between;align-items:end}.throttle-value{font-size:25px;font-weight:850}.bar{height:13px;border-radius:8px;background:#152a36;padding:3px;border:1px solid #274252;overflow:hidden}.fill{height:100%;width:0;border-radius:5px;background:linear-gradient(90deg,var(--green),var(--accent));box-shadow:0 0 14px #23d9ff55;transition:width .18s linear}.substatus{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:5px}.pill{border:1px solid #203744;border-radius:8px;padding:6px 5px;background:#08151e;min-width:0}.pill strong{display:block;margin-top:2px;font-size:11px;overflow:hidden;text-overflow:ellipsis}
.metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:5px;margin-top:7px}.metric{background:#08151e;border:1px solid #203744;border-radius:9px;padding:6px 7px;min-width:0}.metric .value{display:block;margin-top:2px;font-size:17px;font-weight:820;white-space:nowrap}.temps{grid-column:span 2;display:grid;grid-template-columns:1fr 1fr;gap:5px}.fault-box{margin-top:5px;padding:6px 8px;border:1px solid #203744;border-radius:9px;background:#08151e;display:flex;align-items:center;gap:8px;min-height:30px}.fault-value{display:block;color:var(--green);font-size:10px;font-weight:800;overflow-wrap:anywhere}.motor.fault .fault-value{color:var(--red)}
.chart{margin-top:8px;border-radius:14px;padding:8px 10px;height:112px}.chart-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px}.legend{display:flex;gap:11px}.legend span:before{content:"";display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--c);margin-right:4px}canvas{width:100%;height:79px;display:block}.foot{margin-top:5px;text-align:center;color:#587180;font-size:9px;letter-spacing:.08em}
body.all-offline .status{border-color:#604924;background:#2b210d;color:var(--amber)}
@media(max-width:850px){.summary{grid-template-columns:repeat(2,minmax(0,1fr))}.motors{grid-template-columns:1fr}.gauge{width:min(220px,56vw)}.main{grid-template-columns:minmax(170px,.8fr) 1.2fr}.chart{height:125px}canvas{height:91px}}
@media(max-width:540px){.brand p{display:none}.brand h1{font-size:14px}.mark{width:34px;height:34px}.status{padding:7px 9px}.browser-btn{padding:7px 9px;font-size:0}.browser-btn:after{content:"CHROME";font-size:9px}.main{grid-template-columns:1fr}.gauge{width:min(210px,62vw)}.substatus{grid-template-columns:repeat(2,1fr)}.metrics{grid-template-columns:repeat(2,1fr)}.temps{grid-column:span 2}.motor{padding:9px}.summary-value{font-size:18px}}
@media(min-width:851px) and (max-height:690px){.shell{padding-top:6px;padding-bottom:5px}header{margin-bottom:5px}.mark{width:32px;height:32px}.brand p{display:none}.summary{margin-bottom:5px}.summary-card{padding:5px 9px}.summary-value{font-size:17px}.motor{padding:7px}.gauge{width:clamp(140px,15vw,185px)}.rpm{font-size:43px}.metrics{margin-top:4px}.fault-box{margin-top:4px}.chart{margin-top:5px;height:91px}canvas{height:60px}.foot{margin-top:3px}}
</style>
</head>
<body class="all-offline">
<div class="shell">
<header><div class="brand"><div class="mark">EZ</div><div><h1>TWIN MOTOR DASH</h1><p>GOLDEN MOTOR EZKONTROL · 250 KBPS</p></div></div><div class="head-actions"><button class="browser-btn" type="button" onclick="openChrome()">OPEN IN CHROME</button><div class="status"><span class="dot"></span><span id="busState">WAITING</span></div></div></header>
<section class="summary">
  <div class="summary-card"><span class="label">Bus voltage</span><span class="summary-value"><span id="busVoltage">—</span><span class="suffix">V</span></span></div>
  <div class="summary-card"><span class="label">Combined current</span><span class="summary-value"><span id="totalCurrent">—</span><span class="suffix">A</span></span></div>
  <div class="summary-card"><span class="label">Electrical power</span><span class="summary-value"><span id="totalPower">—</span><span class="suffix">kW</span></span></div>
  <div class="summary-card"><span class="label">CAN frames</span><span class="summary-value" id="totalFrames">0</span></div>
</section>
<main class="motors">
  <section class="panel motor" id="portCard" style="--accent:var(--cyan)">
    <div class="motor-head"><div class="motor-name"><span class="side-dot"></span>PORT</div><span class="motor-state" id="portState">WAITING · EF</span></div>
    <div class="main"><div class="gauge-wrap"><div class="gauge" id="portGauge"><div class="gauge-content"><div class="rpm" id="portRpm">0</div><div class="unit">RPM</div><div class="signed" id="portSigned">SIGNED 0</div></div></div></div><div class="drive"><div class="direction"><div class="dir r" id="portRev">R</div><div class="dir" id="portNeutral">N</div><div class="dir" id="portFwd">FWD</div></div><div><div class="throttle-head"><span class="label">Throttle</span><span class="throttle-value"><span id="portThrottle">0</span><small>%</small></span></div><div class="bar"><div class="fill" id="portFill"></div></div></div><div class="substatus"><div class="pill"><span class="label">Mode</span><strong id="portMode">—</strong></div><div class="pill"><span class="label">Gear</span><strong id="portGear">—</strong></div><div class="pill"><span class="label">Brake</span><strong id="portBrake">OFF</strong></div><div class="pill"><span class="label">Contactor</span><strong id="portContactor">OFF</strong></div></div></div></div>
    <div class="metrics"><div class="metric"><span class="label">Bus current</span><span class="value"><span id="portCurrent">—</span><span class="suffix">A</span></span></div><div class="metric"><span class="label">Phase current</span><span class="value"><span id="portPhase">—</span><span class="suffix">A</span></span></div><div class="metric temps"><div><span class="label">Controller</span><span class="value"><span id="portCtrlTemp">—</span><span class="suffix">°C</span></span></div><div><span class="label">Motor</span><span class="value"><span id="portMotorTemp">—</span><span class="suffix">°C</span></span></div></div></div>
    <div class="fault-box"><span class="label">Status</span><span class="fault-value" id="portFaults">WAITING FOR CAN</span></div>
  </section>
  <section class="panel motor" id="starboardCard" style="--accent:var(--blue)">
    <div class="motor-head"><div class="motor-name"><span class="side-dot"></span>STARBOARD</div><span class="motor-state" id="starboardState">WAITING · F0</span></div>
    <div class="main"><div class="gauge-wrap"><div class="gauge" id="starboardGauge"><div class="gauge-content"><div class="rpm" id="starboardRpm">0</div><div class="unit">RPM</div><div class="signed" id="starboardSigned">SIGNED 0</div></div></div></div><div class="drive"><div class="direction"><div class="dir r" id="starboardRev">R</div><div class="dir" id="starboardNeutral">N</div><div class="dir" id="starboardFwd">FWD</div></div><div><div class="throttle-head"><span class="label">Throttle</span><span class="throttle-value"><span id="starboardThrottle">0</span><small>%</small></span></div><div class="bar"><div class="fill" id="starboardFill"></div></div></div><div class="substatus"><div class="pill"><span class="label">Mode</span><strong id="starboardMode">—</strong></div><div class="pill"><span class="label">Gear</span><strong id="starboardGear">—</strong></div><div class="pill"><span class="label">Brake</span><strong id="starboardBrake">OFF</strong></div><div class="pill"><span class="label">Contactor</span><strong id="starboardContactor">OFF</strong></div></div></div></div>
    <div class="metrics"><div class="metric"><span class="label">Bus current</span><span class="value"><span id="starboardCurrent">—</span><span class="suffix">A</span></span></div><div class="metric"><span class="label">Phase current</span><span class="value"><span id="starboardPhase">—</span><span class="suffix">A</span></span></div><div class="metric temps"><div><span class="label">Controller</span><span class="value"><span id="starboardCtrlTemp">—</span><span class="suffix">°C</span></span></div><div><span class="label">Motor</span><span class="value"><span id="starboardMotorTemp">—</span><span class="suffix">°C</span></span></div></div></div>
    <div class="fault-box"><span class="label">Status</span><span class="fault-value" id="starboardFaults">WAITING FOR CAN</span></div>
  </section>
</main>
<section class="panel chart"><div class="chart-head"><span class="label">RPM · last 30 seconds</span><div class="legend"><span class="label" style="--c:var(--cyan)">PORT</span><span class="label" style="--c:var(--blue)">STARBOARD</span></div></div><canvas id="chart"></canvas></section>
<div class="foot">DISPLAY ONLY · NO MOTOR CONTROL COMMANDS · 192.168.4.1</div>
</div>
<script>
const $=id=>document.getElementById(id),rpmMax=6000,maxPoints=150,hp=[],hs=[];
const canvas=$('chart'),ctx=canvas.getContext('2d');
function openChrome(){const fallback='http://192.168.4.1/';if(/Android/i.test(navigator.userAgent)){location.href='intent://192.168.4.1/#Intent;scheme=http;package=com.android.chrome;S.browser_fallback_url=http%3A%2F%2F192.168.4.1%2F;end'}else{window.open(fallback,'_blank')}}
function direction(prefix,d){['Rev','Neutral','Fwd'].forEach(x=>$(prefix+x).classList.remove('active'));if(d==='REV')$(prefix+'Rev').classList.add('active');else if(d==='FWD')$(prefix+'Fwd').classList.add('active');else $(prefix+'Neutral').classList.add('active')}
function setMotor(prefix,d){const card=$(prefix+'Card');card.classList.toggle('online',d.online);card.classList.toggle('fault',d.faults!=='NONE');$(prefix+'State').textContent=(d.online?'LIVE':'WAITING')+' · '+d.address;$(prefix+'Rpm').textContent=d.absRpm;$(prefix+'Signed').textContent='SIGNED '+d.rpm;$(prefix+'Gauge').style.setProperty('--sweep',Math.min(d.absRpm/rpmMax,1)*270+'deg');$(prefix+'Throttle').textContent=d.throttle;$(prefix+'Fill').style.width=d.throttle+'%';direction(prefix,d.direction);$(prefix+'Mode').textContent=d.mode;$(prefix+'Gear').textContent=d.gear;$(prefix+'Brake').textContent=d.brake?'ON':'OFF';$(prefix+'Contactor').textContent=d.contactor?'ON':'OFF';$(prefix+'Current').textContent=d.online?d.busCurrent.toFixed(0):'—';$(prefix+'Phase').textContent=d.online?d.phaseCurrent.toFixed(0):'—';$(prefix+'CtrlTemp').textContent=d.online?d.controllerTemp:'—';$(prefix+'MotorTemp').textContent=d.online?d.motorTemp:'—';$(prefix+'Faults').textContent=d.online?d.faults:'WAITING FOR CAN'}
function draw(){const dpr=devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;canvas.width=w*dpr;canvas.height=h*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.strokeStyle='#1b3341';ctx.lineWidth=1;for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke()}[[hp,'#23d9ff'],[hs,'#6198ff']].forEach(([a,c])=>{if(a.length<2)return;ctx.beginPath();a.forEach((v,i)=>{const x=i*w/(maxPoints-1),y=h-Math.min(v,rpmMax)*h/rpmMax;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.strokeStyle=c;ctx.lineWidth=2;ctx.shadowColor=c;ctx.shadowBlur=8;ctx.stroke();ctx.shadowBlur=0})}
async function update(){try{const r=await fetch('/api/data',{cache:'no-store'}),d=await r.json(),port=d.port,starboard=d.starboard;setMotor('port',port);setMotor('starboard',starboard);document.body.classList.toggle('all-offline',!port.online&&!starboard.online);$('busState').textContent=port.online&&starboard.online?'BOTH LIVE':(port.online||starboard.online?'ONE MOTOR LIVE':'WAITING');const voltage=port.online?port.voltage:(starboard.online?starboard.voltage:0),current=(port.online?port.busCurrent:0)+(starboard.online?starboard.busCurrent:0),power=voltage*current/1000;$('busVoltage').textContent=d.anyOnline?voltage.toFixed(1):'—';$('totalCurrent').textContent=d.anyOnline?current.toFixed(0):'—';$('totalPower').textContent=d.anyOnline?power.toFixed(1):'—';$('totalFrames').textContent=d.totalFrames;hp.push(port.online?port.absRpm:0);hs.push(starboard.online?starboard.absRpm:0);if(hp.length>maxPoints)hp.shift();if(hs.length>maxPoints)hs.shift();draw()}catch(e){document.body.classList.add('all-offline');$('busState').textContent='DASH LOST'}}
addEventListener('resize',draw);setInterval(update,200);update();
</script>
</body>
</html>
)HTML";

static uint16_t readLE16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

static const char *gearName(uint8_t gear) {
  switch (gear) {
    case 1: return "R";
    case 2: return "N";
    case 3: return "D1";
    case 4: return "D2";
    case 5: return "D3";
    case 6: return "S";
    case 7: return "P";
    default: return "--";
  }
}

static const char *directionName(uint8_t gear) {
  if (gear == 1) return "REV";
  if (gear >= 3 && gear <= 6) return "FWD";
  return "NEUTRAL";
}

static const char *runModeName(uint8_t mode) {
  switch (mode) {
    case 0: return "STOP";
    case 1: return "DRIVE";
    case 2: return "CRUISE";
    case 3: return "EBS";
    case 4: return "HOLD";
    default: return "RESERVED";
  }
}

static bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static void startOLED(U8G2 &display, uint8_t addressU8g2, const char *sideName) {
  display.setI2CAddress(addressU8g2);
  display.setBusClock(400000);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(19, 20, "EZKONTROL DASH");
  display.drawStr((128 - display.getStrWidth(sideName)) / 2, 35, sideName);
  char addressLine[20];
  snprintf(addressLine, sizeof(addressLine), "OLED 0x%02X OK", addressU8g2);
  display.drawStr(25, 50, addressLine);
  display.sendBuffer();
  Serial.printf("%s U8g2 SSD1306 ready at 8-bit address 0x%02X\n",
                sideName, addressU8g2);
}

static void probeOLEDs() {
  oledLastProbeAt = millis();
  if (!portOledReady && i2cDevicePresent(PORT_OLED_ADDRESS_7BIT)) {
    startOLED(portOled, PORT_OLED_ADDRESS_U8G2, "PORT");
    portOledReady = true;
  }
  if (!starboardOledReady && i2cDevicePresent(STARBOARD_OLED_ADDRESS_7BIT)) {
    startOLED(starboardOled, STARBOARD_OLED_ADDRESS_U8G2, "STARBOARD");
    starboardOledReady = true;
  }
}

static void drawMotorOLED(U8G2 &display, const MotorData &motor,
                          const char *header, const char *waitingMessage,
                          uint32_t &lastDrawAt, uint32_t now) {
  if (now - lastDrawAt < OLED_REFRESH_MS) return;
  lastDrawAt = now;

  const uint32_t age1 = motor.frame1At ? now - motor.frame1At : UINT32_MAX;
  const uint32_t age2 = motor.frame2At ? now - motor.frame2At : UINT32_MAX;
  const bool online = age1 <= STALE_TIMEOUT_MS && age2 <= STALE_TIMEOUT_MS;
  const int32_t absoluteRpm = motor.rpm < 0
    ? -static_cast<int32_t>(motor.rpm)
    : motor.rpm;

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 9, header);
  display.drawStr(88, 9, online ? "CAN OK" : "WAIT");

  char rpmText[10];
  snprintf(rpmText, sizeof(rpmText), "%ld", static_cast<long>(absoluteRpm));
  display.setFont(u8g2_font_logisoso24_tn);
  const int rpmX = (128 - display.getStrWidth(rpmText)) / 2;
  display.drawStr(rpmX, 36, rpmText);

  display.setFont(u8g2_font_6x10_tf);
  char statusLine[24];
  snprintf(statusLine, sizeof(statusLine), "%-7s THR %3u%%",
           directionName(motor.gear), motor.throttlePercent);
  display.drawStr(0, 49, statusLine);
  char dataLine[24];
  if (online) {
    snprintf(dataLine, sizeof(dataLine), "%4.1fV %4.1fA M%3dC",
             motor.voltageV, motor.averageBusCurrentA, motor.motorTempC);
  } else {
    snprintf(dataLine, sizeof(dataLine), "%s", waitingMessage);
  }
  display.drawStr(0, 62, dataLine);
  display.sendBuffer();
}

static void serviceOLED() {
  const uint32_t now = millis();
  if ((!portOledReady || !starboardOledReady) &&
      (oledLastProbeAt == 0 || now - oledLastProbeAt >= OLED_RETRY_MS)) {
    probeOLEDs();
  }
  if (portOledReady) {
    drawMotorOLED(portOled, portMotor, "PORT  EF", "WAITING FOR PORT CAN",
                  portOledLastDrawAt, now);
  }
  if (starboardOledReady) {
    drawMotorOLED(starboardOled, starboardMotor, "STBD  F0", "WAITING FOR STBD CAN",
                  starboardOledLastDrawAt, now);
  }
}

static String faultString(const MotorData &motor) {
  String result;
  auto add = [&result](const char *name) {
    if (result.length()) result += '|';
    result += name;
  };

  if (motor.fault1 & 0x01) add("OVERCURRENT");
  if (motor.fault1 & 0x02) add("OVERLOAD");
  if (motor.fault1 & 0x04) add("OVERVOLTAGE");
  if (motor.fault1 & 0x08) add("UNDERVOLTAGE");
  if (motor.fault1 & 0x10) add("CONTROLLER OVERHEAT");
  if (motor.fault1 & 0x20) add("MOTOR OVERHEAT");
  if (motor.fault1 & 0x40) add("MOTOR STALLED");
  if (motor.fault1 & 0x80) add("MOTOR PHASE");
  if (motor.fault2 & 0x01) add("MOTOR SENSOR");
  if (motor.fault2 & 0x02) add("AUX SENSOR");
  if (motor.fault2 & 0x04) add("ENCODER ALIGNMENT");
  if (motor.fault2 & 0x08) add("ANTI-RUNAWAY");
  if (motor.fault2 & 0x10) add("MAIN THROTTLE");
  if (motor.fault2 & 0x20) add("AUX THROTTLE");
  if (motor.fault2 & 0x40) add("PRECHARGE");
  if (motor.fault2 & 0x80) add("CONTACTOR");
  if (motor.fault3 & 0x01) add("POWER STAGE");
  if (motor.fault3 & 0x02) add("CURRENT SENSOR");
  if (motor.fault3 & 0x04) add("AUTOTUNE");
  if (motor.fault3 & 0x08) add("RS485");
  if (motor.fault3 & 0x10) add("CAN");
  if (motor.fault3 & 0x20) add("SOFTWARE");
  return result.length() ? result : String("NONE");
}

static MotorData *motorForAddress(uint8_t address) {
  if (address == PORT_ADDRESS) return &portMotor;
  if (address == STARBOARD_ADDRESS) return &starboardMotor;
  return nullptr;
}

static void resetCurrentAverage(MotorData &motor) {
  motor.averageBusCurrentA = 0.0f;
  motor.averagePhaseCurrentA = 0.0f;
  motor.busCurrentSum = 0.0f;
  motor.phaseCurrentSum = 0.0f;
  motor.currentSampleIndex = 0;
  motor.currentSampleCount = 0;
}

static void addCurrentSample(MotorData &motor, float busCurrent, float phaseCurrent) {
  const uint8_t index = motor.currentSampleIndex;
  if (motor.currentSampleCount == CURRENT_AVERAGE_SAMPLES) {
    motor.busCurrentSum -= motor.busCurrentSamples[index];
    motor.phaseCurrentSum -= motor.phaseCurrentSamples[index];
  } else {
    ++motor.currentSampleCount;
  }

  motor.busCurrentSamples[index] = busCurrent;
  motor.phaseCurrentSamples[index] = phaseCurrent;
  motor.busCurrentSum += busCurrent;
  motor.phaseCurrentSum += phaseCurrent;
  motor.currentSampleIndex = (index + 1) % CURRENT_AVERAGE_SAMPLES;
  motor.averageBusCurrentA = motor.busCurrentSum / motor.currentSampleCount;
  motor.averagePhaseCurrentA = motor.phaseCurrentSum / motor.currentSampleCount;
}

static void decodeFrame1(MotorData &motor, const twai_message_t &message, uint32_t now) {
  motor.voltageV = readLE16(&message.data[0]) * 0.1f;
  motor.busCurrentA = readLE16(&message.data[2]) * 0.1f - 3200.0f;
  motor.phaseCurrentA = readLE16(&message.data[4]) * 0.1f - 3200.0f;
  motor.rpm = static_cast<int32_t>(readLE16(&message.data[6])) - 32000;
  const bool stopped = motor.runMode == 0 && motor.throttlePercent == 0 && abs(motor.rpm) < 50;
  if (stopped) resetCurrentAverage(motor);
  else addCurrentSample(motor, motor.busCurrentA, motor.phaseCurrentA);
  motor.frame1At = now;
  ++motor.frameCount;
}

static void decodeFrame2(MotorData &motor, const twai_message_t &message, uint32_t now) {
  motor.controllerTempC = static_cast<int16_t>(message.data[0]) - 40;
  motor.motorTempC = static_cast<int16_t>(message.data[1]) - 40;
  motor.throttlePercent = message.data[2];
  const uint8_t status = message.data[3];
  motor.gear = status & 0x07;
  motor.brake = (status & 0x08) != 0;
  motor.runMode = (status >> 4) & 0x07;
  motor.contactorOn = (status & 0x80) != 0;
  motor.fault1 = message.data[4];
  motor.fault2 = message.data[5];
  motor.fault3 = message.data[6];
  motor.lifeCounter = (message.data[7] >> 4) & 0x0F;
  if (motor.runMode == 0 && motor.throttlePercent == 0 && abs(motor.rpm) < 50) {
    resetCurrentAverage(motor);
  }
  motor.frame2At = now;
  ++motor.frameCount;
}

static bool motorOnline(const MotorData &motor, uint32_t now, uint32_t &age) {
  const uint32_t age1 = motor.frame1At ? now - motor.frame1At : UINT32_MAX;
  const uint32_t age2 = motor.frame2At ? now - motor.frame2At : UINT32_MAX;
  age = age1 > age2 ? age1 : age2;
  return age <= STALE_TIMEOUT_MS;
}

static bool startCAN() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL
  );
  general.tx_queue_len = 0;
  general.rx_queue_len = 64;
  const twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", DASH_HTML);
}

static String motorJson(const MotorData &motor, const char *address, uint32_t now) {
  uint32_t age = UINT32_MAX;
  const bool online = motorOnline(motor, now, age);
  const int32_t absoluteRpm = motor.rpm < 0 ? -static_cast<int32_t>(motor.rpm) : motor.rpm;
  const String faults = faultString(motor);
  char json[760];
  snprintf(json, sizeof(json),
    "{\"address\":\"%s\",\"online\":%s,\"age\":%lu,\"rpm\":%d,\"absRpm\":%ld,"
    "\"throttle\":%u,\"direction\":\"%s\",\"gear\":\"%s\",\"voltage\":%.1f,"
    "\"busCurrent\":%.1f,\"phaseCurrent\":%.1f,\"controllerTemp\":%d,\"motorTemp\":%d,"
    "\"brake\":%s,\"mode\":\"%s\",\"contactor\":%s,\"faults\":\"%s\",\"frames\":%lu,\"life\":%u}",
    address, online ? "true" : "false", static_cast<unsigned long>(online ? age : 999999),
    motor.rpm, static_cast<long>(absoluteRpm), motor.throttlePercent, directionName(motor.gear),
    gearName(motor.gear), motor.voltageV, motor.averageBusCurrentA, motor.averagePhaseCurrentA,
    motor.controllerTempC, motor.motorTempC, motor.brake ? "true" : "false",
    runModeName(motor.runMode), motor.contactorOn ? "true" : "false", faults.c_str(),
    static_cast<unsigned long>(motor.frameCount), motor.lifeCounter);
  return String(json);
}

static void handleApiData() {
  const uint32_t now = millis();
  uint32_t portAge, starboardAge;
  const bool portOnline = motorOnline(portMotor, now, portAge);
  const bool starboardOnline = motorOnline(starboardMotor, now, starboardAge);
  const bool anyOnline = portOnline || starboardOnline;
  const float voltage = portOnline ? portMotor.voltageV : (starboardOnline ? starboardMotor.voltageV : 0.0f);
  const float current = (portOnline ? portMotor.averageBusCurrentA : 0.0f) +
                        (starboardOnline ? starboardMotor.averageBusCurrentA : 0.0f);
  const float powerKw = voltage * current / 1000.0f;

  String json;
  json.reserve(1800);
  json = "{\"anyOnline\":";
  json += anyOnline ? "true" : "false";
  json += ",\"busVoltage\":" + String(voltage, 1);
  json += ",\"totalCurrent\":" + String(current, 1);
  json += ",\"totalPower\":" + String(powerKw, 2);
  json += ",\"totalFrames\":" + String(portMotor.frameCount + starboardMotor.frameCount);
  json += ",\"port\":" + motorJson(portMotor, "EF", now);
  json += ",\"starboard\":" + motorJson(starboardMotor, "F0", now);
  json += '}';

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

static void startWebDashboard() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(100);

  const IPAddress ip = WiFi.softAPIP();
  dnsServer.start(53, "*", ip);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/generate_204", HTTP_ANY, handleRoot);
  server.on("/hotspot-detect.html", HTTP_ANY, handleRoot);
  server.on("/connecttest.txt", HTTP_ANY, handleRoot);
  server.onNotFound(handleRoot);
  server.begin();

  Serial.printf("Dual Wi-Fi dashboard ready\nSSID: %s\nPassword: %s\nURL: http://%s\n",
                AP_SSID, AP_PASSWORD, ip.toString().c_str());
  Serial.println("Local-only mode: no station Wi-Fi and no internet uploads");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("Golden Motor EZkontrol dual-motor Wi-Fi dashboard");
  Serial.println("Port address EF (239), starboard address F0 (240)");
  Serial.println("Waveshare ESP32-S3-RS485-CAN: TX GPIO15, RX GPIO16");

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  Serial.println("OLED I2C: SDA GPIO1, SCL GPIO2; port 0x78, starboard 0x7A");
  probeOLEDs();

  pinMode(CAN_TX_PIN, OUTPUT);
  digitalWrite(CAN_TX_PIN, HIGH);

  if (!startCAN()) {
    Serial.println("ERROR: CAN initialization failed");
    while (true) delay(1000);
  }

  startWebDashboard();
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  twai_message_t message = {};
  while (twai_receive(&message, 0) == ESP_OK) {
    if (!message.extd || message.rtr || message.data_length_code != 8) continue;
    MotorData *motor = motorForAddress(static_cast<uint8_t>(message.identifier & 0xFF));
    if (!motor) continue;
    const uint32_t frameBase = message.identifier & FRAME_1_MASK;
    const uint32_t now = millis();
    if (frameBase == FRAME_1_BASE) decodeFrame1(*motor, message, now);
    else if (frameBase == FRAME_2_BASE) decodeFrame2(*motor, message, now);
  }

  serviceOLED();

  delay(1);
}
