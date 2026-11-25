# Smart Irrigation System (ESP32 + Firebase)

An IoT smart irrigation controller with a live web/mobile dashboard.
The ESP32 reads 4 soil moisture sensors and a water-tank level sensor, controls the pump, and sends everything to Firebase. The app shows live readings and lets you change the mode, pump and moisture threshold. It also uses cloud weather and a local BMP280 sensor to hold the pump back when rain is expected.

**Project by:** Ogunleke Samson Abidemi
**Supervisor:** Engr. O.A. Omole

## Features

- **Offline-first:** soil logic, pump control and water protection run on the ESP32 even without Wi-Fi. Wi-Fi and Firebase run in the background on core 0.
- **AUTO mode:** pump starts when 3 or more of the 4 sensors are dry (30 % or less) and stops at your threshold.
- **MANUAL mode:** control the pump from the app. It returns to AUTO after 5 minutes for safety.
- **Water protection:** the pump is blocked at 20 % tank level or below and allowed again at 30 %.
- **Cloud weather (Open-Meteo, free, no key):** temperature, humidity, pressure, wind, cloud cover, rain chance and conditions, refreshed every 10 minutes.
- **Rain delay:** the pump is blocked in AUTO and MANUAL when the rain chance for the next 3 hours is 80 % or more, and released below 65 %.
- **Rain confirmation:** while the pump is off during a rain delay, a rise in soil moisture confirms real rain.
- **Local BMP280 sensor (optional):** temperature and pressure. The pressure trend can nudge the rain chance by up to 10 points. If the sensor is missing, it is ignored.
- **Users and roles:** admin and member accounts with login.
- **16x2 LCD** with rotating status pages and a start-up message.

## Hardware

| Part | Pin / address |
|---|---|
| ESP32 | - |
| 4 soil moisture sensors | GPIO 34, 35, 32, 33 |
| Relay (pump) | GPIO 4 |
| Ultrasonic sensor | TRIG GPIO 19, ECHO GPIO 23 (use a voltage divider, echo is 5 V) |
| I2C LCD 16x2 | SDA GPIO 21, SCL GPIO 22, address 0x27 (try 0x3F) |
| BMP280 (HW-611) | SDA GPIO 21, SCL GPIO 22, address 0x76 or 0x77 |

## Files

| File | Purpose |
|---|---|
| `sisweatherv2.ino` | ESP32 firmware |
| `desktop_final_index_login_weather.html` | Web / mobile dashboard (single file) |
| `sis_iot_login_weather.json` | Starting data template for Firebase |

## Setup

### 1. Firebase
1. Create a Firebase project and a **Realtime Database**.
2. Copy the database URL, for example `https://YOUR-PROJECT-ID-default-rtdb.firebaseio.com`.
3. Optional: import `sis_iot_login_weather.json` (Realtime Database, three-dot menu, Import JSON) to create the starting structure.

### 2. ESP32 firmware
1. Install these libraries from the Arduino Library Manager: **ArduinoJson**, **LiquidCrystal I2C**, **Adafruit BMP280 Library** (also installs Adafruit Unified Sensor and BusIO).
2. Open `sisweatherv2.ino` and set your own values:
   - `WIFI_SSID` and `WIFI_PASSWORD`
   - `DATABASE_URL`
   - `WEATHER_LAT` and `WEATHER_LON` (your farm location)
3. Calibrate `ADC_DRY` and `ADC_WET` for your soil sensors, and `TANK_EMPTY_CM` and `TANK_FULL_CM` for your tank.
4. Select your ESP32 board and upload.

### 3. Dashboard
1. Open `desktop_final_index_login_weather.html` in a text editor and set:
   - `HOST` to your database URL
   - `WX_LAT` and `WX_LON` to your farm location
   - `MAIN_ADMIN` (username and password hash, see below)
2. Host the file (GitHub Pages, Firebase Hosting) or load it in your mobile app WebView.

### 4. Create the admin password hash
The password is stored as a SHA-256 hash of `sis|<username in lowercase>|<password>`.

```bash
printf 'sis|admin|YourStrongPassword' | sha256sum
```

Paste the result into `passHash` in `MAIN_ADMIN`. Use a strong password.

## Keep your details private

**Never commit real values for:** Wi-Fi name and password, your Firebase URL, farm coordinates, or any password hash. This repository contains placeholders only (`YOUR_WIFI_NAME`, `YOUR-PROJECT-ID`, and so on). Keep your real copy locally and do not push it.

Firebase security: if your database rules are public, anyone who knows the URL can read and write it. Use Firebase rules or an auth token (`AUTH` in the dashboard, `DATABASE_AUTH_QUERY` in the firmware) for any real deployment.

## License

Add your preferred license here (for example MIT).
