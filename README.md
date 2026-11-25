Part
Pin / address
ESP32
-
4 soil moisture sensors
GPIO 34, 35, 32, 33
Relay (pump)
GPIO 4
Ultrasonic sensor
TRIG GPIO 19, ECHO GPIO 23 (use a voltage divider, echo is 5 V)
I2C LCD 16x2
SDA GPIO 21, SCL GPIO 22, address 0x27 (try 0x3F)
BMP280 (HW-611)
SDA GPIO 21, SCL GPIO 22, address 0x76 or 0x77
Files
File
Purpose
sisweatherv2.ino
ESP32 firmware
desktop_final_index_login_weather.html
Web / mobile dashboard (single file)
sis_iot_login_weather.json
Starting data template for Firebase
Setup
1. Firebase
Create a Firebase project and a Realtime Database.
Copy the database URL, for example https://YOUR-PROJECT-ID-default-rtdb.firebaseio.com.
Optional: import sis_iot_login_weather.json (Realtime Database, three-dot menu, Import JSON) to create the starting structure.
