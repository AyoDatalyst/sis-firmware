# Smart Irrigation Control System (ESP32 + Firebase)

Firmware for an ESP32-based smart irrigation system, developed as a final year project. The system reads four soil moisture sensors and a water-level sensor, controls a water pump through a relay, and syncs with a Firebase Realtime Database so the pump can be monitored and controlled from a web/mobile app.

## Features

- **AUTO mode:** the pump turns on when 3 or more soil sensors fall to the dry threshold and turns off when the soil is wet enough.
- **MANUAL mode:** the pump is controlled from the app, with a 5-minute safety timeout.
- **Water protection:** the pump is blocked when the reservoir is low and allowed again once it is refilled.
- **Offline-first:** sensing, pump control and the LCD never wait for the network. Wi-Fi reconnects in the background.
- **Dual-core design:** the main loop (core 1) handles sensors, pump and LCD; a separate task (core 0) handles Wi-Fi and Firebase.
- **Saved settings:** mode and threshold are stored in flash and used when offline.
- **16x2 I2C LCD** with start-up screens and rotating status pages.

## Hardware

| Component | Pin / note |
|---|---|
| ESP32 development board | |
| Soil moisture sensors (x4) | GPIO 34, 35, 32, 33 |
| Relay module + water pump | GPIO 4 |
| Ultrasonic sensor (water level) | TRIG: GPIO 19, ECHO: GPIO 23 (use a voltage divider on ECHO, it outputs 5 V) |
| 16x2 I2C LCD | SDA: GPIO 21, SCL: GPIO 22, address `0x27` (try `0x3F` if blank) |

Full parts list and circuit details are in the project report.

## Libraries

Install these from the Arduino Library Manager:

- `ArduinoJson`
- `LiquidCrystal_I2C`

The following come with the ESP32 board package: `WiFi`, `WiFiClientSecure`, `HTTPClient`, `Wire`, `Preferences`.

## Setup

1. Install the Arduino IDE and add ESP32 board support.
2. Open `ogunlekev3.ino` (the Arduino IDE will ask to put it in a folder named `ogunlekev3`; click OK).
3. Replace the placeholders near the top of the file:
   ```cpp
   #define WIFI_SSID       "YOUR_WIFI_NAME"
   #define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
   // DATABASE_URL: https://YOUR-PROJECT-default-rtdb.firebaseio.com
   ```
4. Create a Firebase Realtime Database and set the URL above.
5. Select your ESP32 board and port, then upload.
6. Calibrate the soil sensors by updating the `ADC_DRY` and `ADC_WET` values for your own sensors.

## Database structure

See [`database_structure.json`](database_structure.json).

```
smartIrrigation
  sensors: soil1..soil4, waterLevel
  system:  mode, pump, threshold, waterLow, heartbeat
```

## Security note

Wi-Fi credentials and the Firebase URL in this repository are placeholders. Do not commit real credentials. Set proper Firebase security rules before using the system outside a demo.

## Author

Ogunleke, Final Year Project.
