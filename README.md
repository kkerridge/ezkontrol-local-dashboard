# EZkontrol Local Twin-Motor Dashboard

An offline ESP32-S3 dashboard for two Golden Motor EZkontrol motor
controllers. It displays live motor data on a responsive local web page and on
two optional SSD1306 OLED displays.

No internet connection, account, server or cloud service is required. The
ESP32 creates its own Wi-Fi network and serves the complete dashboard from its
flash memory.

## Features

- Dual Golden Motor EZkontrol controller display
- RPM, throttle, direction, voltage, current, power and temperatures
- Controller status and fault display
- Responsive phone, tablet and desktop web dashboard
- Two optional 128x64 SSD1306 I2C displays
- Local JSON endpoint at `/api/data`
- No internet upload, analytics, tracking or external assets
- Display-only CAN implementation: the transmit queue is disabled and the
  firmware never calls `twai_transmit()`

## Tested hardware

- Waveshare ESP32-S3-RS485-CAN
- Golden Motor EZkontrol C481500 controllers
- Two 128x64 SSD1306 OLED displays using the U8g2 library

The supplied pin configuration is for the **Waveshare ESP32-S3-RS485-CAN**:

| Function | ESP32-S3 pin |
| --- | ---: |
| CAN TX | GPIO15 |
| CAN RX | GPIO16 |
| OLED SDA | GPIO1, J4 pin 1 |
| OLED SCL | GPIO2, J4 pin 2 |

## Motor configuration

Configure the controller part numbers in the Golden Motor software:

| Side | Part number | Address | Meter frame 1 | Meter frame 2 |
| --- | ---: | ---: | ---: | ---: |
| Port | 239 | `0xEF` | `0x180117EF` | `0x180217EF` |
| Starboard | 240 | `0xF0` | `0x180117F0` | `0x180217F0` |

The firmware expects 250 kbit/s extended CAN frames. It can operate with just
one of the two configured controllers connected.

## CAN wiring

The following colours refer to the documented EZkontrol CN2 pigtail:

| EZkontrol | Connect to Waveshare |
| --- | --- |
| Yellow, CAN-H | CAN-H |
| Green, CAN-L | CAN-L |
| Black, CAN-GND | GND |

Use a twisted pair for CAN-H/CAN-L and provide 120-ohm termination at the two
physical ends of the CAN bus. Check the termination resistance and the manual
for your controller revision before energising the system.

> **Important:** Keep the 48 V traction supply away from the ESP32 GPIO, USB,
> I2C and logic-power connections. Work on the wiring only with the propulsion
> system isolated.

## OLED wiring

Both displays share GPIO1 (SDA), GPIO2 (SCL), 3.3 V and GND:

| Display | Normal 7-bit address | U8g2 address notation |
| --- | ---: | ---: |
| Port | `0x3C` | `0x78` |
| Starboard | `0x3D` | `0x7A` |

Many SSD1306 modules default to `0x3C`. The second display must be changed to
`0x3D`, normally by moving its address solder link. The firmware automatically
detects either display, so the web dashboard also works with no OLEDs fitted.

## Arduino IDE setup

1. Install the **ESP32 by Espressif Systems** board package.
2. Install the **U8g2** library by olikraus.
3. Open
   `ezkontrol_local_dashboard/ezkontrol_local_dashboard.ino`.
4. Select **ESP32S3 Dev Module**.
5. Use these tested settings:
   - USB CDC On Boot: Enabled
   - Flash size: 16 MB
   - PSRAM: OPI PSRAM
   - CPU frequency: 240 MHz
6. Connect the board's USB Type-C port and upload the sketch.

The firmware uses only libraries supplied by the ESP32 Arduino core plus U8g2.

## Using the dashboard

After the ESP32 starts, connect a phone, tablet or computer to:

```text
Wi-Fi name: EZkontrol-Dash
Password:    GoldenMotor
Dashboard:   http://192.168.4.1
```

The client device may report **No internet**; that is intentional. Choose the
option to remain connected, then open `http://192.168.4.1` in a browser.

Change `AP_SSID` and `AP_PASSWORD` near the top of the sketch before sharing a
unit outside a controlled environment. WPA2 requires a password of at least
eight characters.

## CAN behaviour and safety

The ESP32 uses normal TWAI mode so it supplies the CAN acknowledgement bit that
some controller setups require to continue broadcasting. The code disables
the transmit queue and contains no motor-control command path. It is therefore
display-only, but it is not electrically silent in the same sense as a
listen-only analyser.

This is an independent community project and is not affiliated with or
endorsed by Golden Motor or Waveshare. Verify all addresses, pin assignments,
termination and decoded values against the documentation for your hardware.

## Licence

Released under the MIT Licence. See [LICENSE](LICENSE).
