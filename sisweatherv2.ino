// ================================================================
// SMART IRRIGATION CONTROL SYSTEM - ESP32 + Firebase REST
// Offline-first, FAST version
//
// Core 1 (loop):   soil + water sensors, pump logic, water
//                  protection, LCD. NEVER waits for the network.
// Core 0 (task):   Wi-Fi reconnect + Firebase upload/download.
//
// - Works with or without Wi-Fi (reconnects in the background).
// - One request uploads all sensor data, one request reads all
//   commands (instead of 9 separate requests).
// - HTTPS connection is kept open between requests.
// - Mode and threshold are saved in flash (used when offline).
// - Pump changes made offline are synced when Wi-Fi returns.
//
// NEW in this version
// - Cloud weather (Open-Meteo, free, no key): temperature, humidity,
//   pressure, wind, cloud cover, rain probability, sunny/cloudy/
//   cooling conditions. Fetched every 10 min on core 0.
// - RAIN DELAY: when the rain probability (next 3 h) is 80 % or more,
//   the pump is blocked in AUTO *and* MANUAL mode (and switched off if
//   running). Released again below 65 %.
// - RAIN CONFIRMATION: while the rain delay is active the pump is OFF, so
//   a rise in soil moisture proves that it is really raining. It only sets
//   a "Raining" status; the cloud forecast data is never changed.
// - Optional BMP280 (HW-611, confirmed BMP280): auto-detected at 0x76 / 0x77.
//   Temperature + pressure only (this chip has NO humidity sensor).
//   If it is not found it is ignored completely. If found, it is used
//   to confirm or soften the cloud rain forecast (pressure trend).
// ================================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>
#include <Adafruit_BMP280.h>   // Library Manager: "Adafruit BMP280 Library"

// ================================================================
// WIFI / FIREBASE CONFIGURATION
// ================================================================

#define WIFI_SSID       "YOUR_WIFI_NAME"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

#define DATABASE_URL \
"https://YOUR-PROJECT-ID-default-rtdb.firebaseio.com"

#define DATABASE_AUTH_QUERY ""

const unsigned long WIFI_RETRY_INTERVAL = 15000;   // background retry
const uint16_t FIREBASE_TIMEOUT_MS = 5000;         // per request
const unsigned long FIREBASE_BACKOFF_MS = 5000;    // pause after failure

// ================================================================
// CLOUD WEATHER (Open-Meteo: free, no API key)
// ================================================================
// Location = your farm. Find it by right-clicking your farm in Google Maps.
const double WEATHER_LAT = 0.0;   // YOUR_LATITUDE
const double WEATHER_LON = 0.0;   // YOUR_LONGITUDE

const unsigned long WEATHER_INTERVAL_MS = 10UL * 60UL * 1000UL;  // normal refresh
const unsigned long WEATHER_RETRY_MS    = 60UL * 1000UL;         // retry after a failure
const unsigned long WEATHER_STALE_MS    = 45UL * 60UL * 1000UL;  // older than this = ignored
const uint16_t WEATHER_TIMEOUT_MS       = 7000;

// RAIN DELAY
const int RAIN_DELAY_ON_PERCENT  = 80;   // start delay at or above this
const int RAIN_DELAY_OFF_PERCENT = 65;   // end delay below this (stops flip-flopping)
const int RAIN_LOOKAHEAD_HOURS   = 3;    // highest rain chance in the next N hours (1 to 6)

// RAIN CONFIRMATION (soil moisture rises while the pump is blocked = real rain)
const int RAIN_SOIL_RISE_AVG    = 4;     // average moisture must rise this many points
const int RAIN_SOIL_RISE_SENSOR = 3;     // a sensor counts as "rising" at this many points
const int RAIN_SOIL_MIN_SENSORS = 2;     // how many sensors must be rising
const unsigned long RAIN_SEEN_HOLD_MS = 20UL * 60UL * 1000UL;  // no new rise for this long = rain stopped

// Altitude of the BMP280, used only for the sea-level pressure.
// -9999 = use the elevation that the weather service reports.
const float LOCAL_ALTITUDE_M = -9999.0;

// LOCAL SENSOR (BMP280) tuning
const unsigned long ENV_READ_INTERVAL_MS   = 5000;    // read every 5 s
const unsigned long ENV_RETRY_INTERVAL_MS  = 30000;   // look for the sensor again every 30 s
const unsigned long TREND_SAMPLE_MS        = 600000UL; // one history sample every 10 min
const unsigned long TREND_MIN_SPAN_MS      = 7200000UL;// need 2 h of history for pressure trend
const float RAIN_PRESSURE_TREND_HPA        = 3.0;     // fast fall / rise per 3 h
// (In the tropics the daily pressure "tide" is about +/-1.5 hPa, so
//  only changes of 3 hPa per 3 h are treated as real weather.)

// ================================================================
// PIN DEFINITIONS
// ================================================================

#define SOIL_SENSOR_1  34
#define SOIL_SENSOR_2  35
#define SOIL_SENSOR_3  32
#define SOIL_SENSOR_4  33
#define RELAY_PIN      4

#define ULTRASONIC_TRIG_PIN  19
#define ULTRASONIC_ECHO_PIN  23   // Use a voltage divider! Echo gives 5V.

// I2C LCD: SDA = GPIO 21, SCL = GPIO 22

// ================================================================
// LCD CONFIGURATION
// ================================================================

#define LCD_ADDRESS  0x27   // Try 0x3F if the screen stays blank
#define LCD_COLUMNS  16
#define LCD_ROWS     2

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// ================================================================
// RELAY LOGIC
// ================================================================
// true  = LOW turns relay ON
// false = HIGH turns relay ON

#define RELAY_ACTIVE_LOW false

// ================================================================
// AUTOMATIC CONTROL THRESHOLDS
// ================================================================

#define DRY_THRESHOLD 30
#define WET_THRESHOLD 60

// Average moisture at which an automatically running pump stops.
int threshold = 80;

// ================================================================
// WATER TANK SETTINGS
// ================================================================

const float TANK_EMPTY_CM = 30.0;
const float TANK_FULL_CM  = 5.0;

const int WATER_LOW_PERCENT = 20;      // pump blocked at or below this
const int WATER_RESUME_PERCENT = 30;   // pump allowed again at or above

// ================================================================
// SENSOR CALIBRATION
// ================================================================

const int ADC_DRY[4] = { 2694, 2686, 2678, 2676 };
const int ADC_WET[4] = { 1258, 1244, 1232, 1227 };

// ================================================================
// TIMING
// ================================================================

const unsigned long SENSOR_INTERVAL   = 1000;  // soil read + auto control
const unsigned long WATER_INTERVAL    = 500;   // water level + protection
const unsigned long LCD_PAGE_INTERVAL = 3000;
const unsigned long UPLOAD_INTERVAL   = 1000;  // cloud upload (was 3000 = the ~3 s app delay)
const unsigned long COMMAND_INTERVAL  = 400;   // cloud command check (was 1000)

// MANUAL mode safety: after this time the system goes back to AUTO
// by itself and a running pump is forced OFF (works offline too).
const unsigned long MANUAL_TIMEOUT_MS = 5UL * 60UL * 1000UL;   // 5 minutes

// Start-up message (shown only once, when the board is powered/reset)
const unsigned long WELCOME_MS = 2000;   // screen 1: Welcome!
// screen 2: scrolling title + loading %
const unsigned long SCROLL_STEP_MS = 150;  // time per character shift (bigger = slower)
const unsigned long SCROLL_HOLD_MS = 600;  // pause at the start and at the end
const char STARTUP_SCROLL_TEXT[] =
    "Smart Irrigation System V1.0 Project by: Ogunleke S. A.";

// ================================================================
// GLOBAL VARIABLES
// ================================================================

WiFiClientSecure sslClient;
HTTPClient http;          // global so the connection can be reused
Preferences prefs;

// Owned by the main loop (read by the network task)
int soilMoisture[4] = {0, 0, 0, 0};
volatile bool pumpState = false;
volatile int waterLevel = 0;
volatile bool waterSensorOk = false;
volatile bool waterLow = true;
float waterDistanceCm = 0;
String systemMode = "AUTO";

// MANUAL-mode timeout (main loop only)
unsigned long manualStartedAt = 0;   // when MANUAL mode began

// Mode sync after a MANUAL timeout (main loop -> network task)
volatile uint32_t modeVersion = 0;        // +1 when the device forces AUTO
volatile uint32_t modeSyncedVersion = 0;  // last version sent to cloud

// Pump sync between main loop and network task
volatile uint32_t pumpVersion = 0;        // +1 on every pump change
volatile uint32_t pumpSyncedVersion = 0;  // last version sent to cloud

// Heartbeat: counts up on every sensor upload. The mobile app shows
// the system as OFFLINE if nothing in Firebase changes for 15 s, so
// this keeps it ONLINE even when all sensor values stay the same.
uint32_t heartbeat = 0;

// Commands received from the cloud (written by network task,
// consumed by main loop)
portMUX_TYPE netMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool netFresh = false;
volatile int netMode = -1;        // -1 none, 0 AUTO, 1 MANUAL
volatile int netThreshold = -1;   // -1 none
volatile int netPump = -1;        // -1 none, 0 off, 1 on

// Connectivity state (network task)
volatile bool firebaseReachable = false;
bool wifiWasOnline = false;
unsigned long lastWifiAttempt = 0;
unsigned long firebaseRetryAt = 0;

// Main loop timers
unsigned long lastSensorRead = 0;
unsigned long lastWaterRead = 0;
unsigned long lastLcdPage = 0;
int lcdPage = 0;   // 0 = summary, 1 = soil, 2 = weather, 3 = local sensor (if found), 4 = connection

// LCD status screens stay off until the start-up message is finished.
bool lcdStatusActive = false;
String lcdLast1 = "";
String lcdLast2 = "";

// ================================================================
// WEATHER + LOCAL SENSOR DATA
// ================================================================

struct CloudWeather
{
    bool valid;                 // at least one successful fetch
    unsigned long receivedAt;   // millis() of that fetch
    long epoch;                 // UTC seconds (0 if clock not synced)
    float temp, feels, humidity, pressMsl, pressSurf;
    float windKmh, gustKmh;
    int windDir, cloudCover, code;
    bool isDay;
    float rainNowMm;
    int probNow, probMax;       // probMax = highest in the look-ahead window
    float rainSoonMm;           // rain expected in the look-ahead window
    float tempSoon;             // temperature 3 h from now
    float elevation;
    float dayMax, dayMin;
    int dayProb;
    float daySumMm, uvMax;
    char sunrise[6], sunset[6];
    int hProb[6];
    float hTemp[6];
};

struct LocalEnv
{
    bool available;             // sensor found and giving good data
    uint8_t type;               // 0 none, 1 BMP280
    uint8_t addr;
    float temp, press, pressSea;
    bool trendValid; float trend3h; int trendMinutes;
    bool tTrendValid; float tTrend1h;
    bool diffValid; float tempDiff, pressDiff;
    uint8_t status;             // 0 none, 1 ok, 2 check, 3 suspect, 4 no cloud
    int adj;                    // rain probability adjustment: -10, 0, +10
};

struct TrendSample { float t; float p; unsigned long ms; };

const int ENV_AVG_COUNT = 6;
const int TREND_SIZE = 19;      // 19 x 10 min = 3 h

portMUX_TYPE wxMux = portMUX_INITIALIZER_UNLOCKED;
CloudWeather cloudShared;       // written by network task, read by loop
LocalEnv localShared;           // written by loop, read by network task
volatile bool rainDelay = false;
int rainProbCloud = -1;
int rainProbEff = -1;
uint8_t rainReason = 2;         // 1 disabled, 2 no weather, 3 active, 4 clear
volatile bool rainDelayEnabled = true;
volatile int netRain = -1;      // -1 none, 0 off, 1 on (command from the cloud)

// Rain confirmed by the soil sensors (loop writes, network task reads)
volatile bool rainConfirmed = false;
volatile int soilRise = 0;                 // points above the driest value since the delay began
volatile uint8_t soilCheck = 0;            // 0 not watching, 1 watching, 2 raining, 3 soil saturated (cannot confirm)
int rainBase[4] = {0, 0, 0, 0};
int rainBaseAvg = 0, rainPeakAvg = 0;
bool rainBaseSet = false;
unsigned long rainLastRise = 0;

// Loop-owned copies (used by the LCD)
CloudWeather cloudLoop;
LocalEnv localLoop;
bool cloudFreshLoop = false;

WiFiClientSecure weatherClient;   // separate HTTPS client for the weather service

Adafruit_BMP280 bmp;
uint8_t envType = 0;              // 0 none, 1 BMP280
uint8_t envAddr = 0;
int envFail = 0;
float envT[ENV_AVG_COUNT], envP[ENV_AVG_COUNT];
int envIdx = 0, envCount = 0;
TrendSample trendBuf[TREND_SIZE];
int trendN = 0, trendHead = 0;
unsigned long lastTrendSample = 0;
unsigned long lastEnvRead = 0;
unsigned long lastRainCheck = 0;

// ================================================================
// FUNCTION DECLARATIONS
// ================================================================

bool wifiConnected();
void startWiFi();
void maintainWiFi();
bool firebaseAvailable();
int firebaseRequest(const char *method, const String &path,
                    const String &body, String *out);
void networkTask(void *param);
void uploadSensors();
void pushPumpState();
void fetchCommands();
void applyNetworkCommands();
void checkManualTimeout();
void pushModeState();
bool pumpNeedsSync();
bool modeNeedsSync();

void loadSavedSettings();
void saveMode();
void saveThreshold();

void readSoilSensors();
int convertToMoisturePercentage(int rawValue, int sensorIndex);
float readDistanceCm();
void readWaterLevel();
void updateWaterLock();
void enforceWaterProtection();
void controlPump();
void setPump(bool state);
int countDrySensors();
int getAverageMoisture();

String cleanFirebaseString(String value);
bool variantToBool(JsonVariant v, bool &out);
bool variantToInt(JsonVariant v, int &out);
String padLine(String text);
void updateLcd();
void showWelcomeScreen();
void showScrollingStartup();

// weather / local sensor
String conditionText(int code, bool isDay);
uint8_t i2cReadChipId(uint8_t addr);
bool detectEnvSensor();
bool readEnvOnce(float &t, float &p);
void updateEnvSensor();
void computeLocal(const CloudWeather &c, bool cloudFresh, int cloudProb);
void updateRainDelay();
void enforceRainDelay();
void updateRainConfirm();
const char *outlookText(const CloudWeather &c, const LocalEnv &le);
bool fetchWeather();
bool uploadWeather();
void uploadControl();
void lcdWeatherPage(String &line1, String &line2);
void lcdLocalPage(String &line1, String &line2);

// ================================================================
// SMALL HELPERS
// ================================================================

bool pumpNeedsSync()
{
    return pumpVersion != pumpSyncedVersion;
}

bool modeNeedsSync()
{
    return modeVersion != modeSyncedVersion;
}

String cleanFirebaseString(String value)
{
    value.trim();

    while (
        value.length() >= 2 &&
        value.startsWith("\"") &&
        value.endsWith("\"")
    )
    {
        value = value.substring(1, value.length() - 1);
        value.trim();
    }

    return value;
}

// Accepts true/false, "true"/"false", "on"/"off", "1"/"0",
// including values with extra quotation marks.
bool variantToBool(JsonVariant v, bool &out)
{
    if (v.is<bool>())
    {
        out = v.as<bool>();
        return true;
    }

    if (v.is<const char *>())
    {
        String s = cleanFirebaseString(v.as<String>());
        s.toLowerCase();

        if (s == "true" || s == "1" || s == "on")
        {
            out = true;
            return true;
        }

        if (s == "false" || s == "0" || s == "off")
        {
            out = false;
            return true;
        }
    }

    return false;
}

// Accepts 75, 75.0 and "75" (with extra quotation marks too).
bool variantToInt(JsonVariant v, int &out)
{
    if (v.is<int>() || v.is<long>())
    {
        out = v.as<int>();
        return true;
    }

    if (v.is<float>())
    {
        out = (int)v.as<float>();
        return true;
    }

    if (v.is<const char *>())
    {
        String s = cleanFirebaseString(v.as<String>());

        if (s.length() > 0)
        {
            out = s.toInt();
            return true;
        }
    }

    return false;
}

// ================================================================
// SAVED SETTINGS (SURVIVE REBOOT / POWER LOSS)
// ================================================================

void loadSavedSettings()
{
    prefs.begin("irrigation", false);

    String savedMode = prefs.getString("mode", "AUTO");
    int savedThreshold = prefs.getInt("threshold", 80);

    if (savedMode == "AUTO" || savedMode == "MANUAL")
    {
        systemMode = savedMode;
    }

    threshold = constrain(savedThreshold, 0, 100);
    rainDelayEnabled = prefs.getBool("rainEn", true);

    Serial.print("Loaded saved mode: ");
    Serial.println(systemMode);
    Serial.print("Loaded saved threshold: ");
    Serial.print(threshold);
    Serial.println("%");
}

void saveMode()
{
    prefs.putString("mode", systemMode);
}

void saveThreshold()
{
    prefs.putInt("threshold", threshold);
}

// ================================================================
// LCD
// ================================================================

String padLine(String text)
{
    while ((int)text.length() < LCD_COLUMNS)
    {
        text += ' ';
    }

    return text.substring(0, LCD_COLUMNS);
}

// Only lines whose text changed are written (I2C is slow).
void updateLcd()
{
    if (!lcdStatusActive)
    {
        return;   // start-up message is still on the screen
    }

    String line1;
    String line2;

    if (lcdPage == 0)
    {
        line1 = "S:" + String(getAverageMoisture()) + "% W:";

        if (waterSensorOk)
        {
            line1 += String(waterLevel) + "%";
        }
        else
        {
            line1 += "ERR";
        }

        if (waterLow)
        {
            line2 = "LOW WATER P:" + String(pumpState ? "ON" : "OFF");
        }
        else if (rainDelay && rainConfirmed)
        {
            line2 = "RAINING P:" + String(pumpState ? "ON" : "OFF");
        }
        else if (rainDelay)
        {
            line2 = "RAIN DELAY P:" + String(pumpState ? "ON" : "OFF");
        }
        else
        {
            line2 = systemMode + " Pump:" + (pumpState ? "ON" : "OFF");
        }
    }
    else if (lcdPage == 1)
    {
        line1 = "S1:" + String(soilMoisture[0]) +
                "% S2:" + String(soilMoisture[1]) + "%";

        line2 = "S3:" + String(soilMoisture[2]) +
                "% S4:" + String(soilMoisture[3]) + "%";
    }
    else if (lcdPage == 2)
    {
        lcdWeatherPage(line1, line2);
    }
    else if (lcdPage == 3)
    {
        lcdLocalPage(line1, line2);
    }
    else
    {
        if (wifiConnected())
        {
            line1 = "WiFi: ONLINE";
            line2 = firebaseReachable ? "Cloud: OK" : "Cloud: waiting";
        }
        else
        {
            line1 = "WiFi: OFFLINE";
            line2 = "Local mode";
        }
    }

    line1 = padLine(line1);
    line2 = padLine(line2);

    if (line1 != lcdLast1)
    {
        lcd.setCursor(0, 0);
        lcd.print(line1);
        lcdLast1 = line1;
    }

    if (line2 != lcdLast2)
    {
        lcd.setCursor(0, 1);
        lcd.print(line2);
        lcdLast2 = line2;
    }
}

// ================================================================
// START-UP MESSAGE (runs once in setup(), never again)
// ================================================================

void lcdPrintLine(uint8_t row, const String &text)
{
    lcd.setCursor(0, row);
    lcd.print(padLine(text));
}

// Screen 1 (2 s): Welcome! / Smart Farmer
void showWelcomeScreen()
{
    lcd.clear();
    lcdPrintLine(0, "Welcome!");
    lcdPrintLine(1, "Smart Farmer");
}

// Screen 2 (about 7 s): line 1 scrolls the title right-to-left,
// line 2 shows the loading percentage = how much of the text
// has already scrolled past.
void showScrollingStartup()
{
    const int textLength = strlen(STARTUP_SCROLL_TEXT);
    const int totalSteps = textLength - LCD_COLUMNS;   // shifts needed to show the last letter

    String text = STARTUP_SCROLL_TEXT;

    lcd.clear();

    for (int step = 0; step <= totalSteps; step++)
    {
        int percent = (step * 100) / totalSteps;

        lcdPrintLine(0, text.substring(step, step + LCD_COLUMNS));
        lcdPrintLine(1, String(percent) + "% Loading...");

        if (step == 0 || step == totalSteps)
        {
            delay(SCROLL_HOLD_MS);   // time to read the start / the end
        }

        delay(SCROLL_STEP_MS);
    }
}

// ================================================================
// PUMP
// ================================================================

void setPump(bool state)
{
    pumpState = state;

    if (RELAY_ACTIVE_LOW)
    {
        digitalWrite(RELAY_PIN, state ? LOW : HIGH);
    }
    else
    {
        digitalWrite(RELAY_PIN, state ? HIGH : LOW);
    }

    // Tell the network task there is something to upload.
    pumpVersion = pumpVersion + 1;

    Serial.print("Pump: ");
    Serial.println(state ? "ON" : "OFF");

    updateLcd();
}

// ================================================================
// SOIL SENSORS
// ================================================================

int convertToMoisturePercentage(int rawValue, int sensorIndex)
{
    int percentage = map(
        rawValue,
        ADC_DRY[sensorIndex],
        ADC_WET[sensorIndex],
        0,
        100
    );

    return constrain(percentage, 0, 100);
}

void readSoilSensors()
{
    const int sensorPins[4] =
    {
        SOIL_SENSOR_1,
        SOIL_SENSOR_2,
        SOIL_SENSOR_3,
        SOIL_SENSOR_4
    };

    int raw[4];

    for (int i = 0; i < 4; i++)
    {
        long total = 0;
        const int numberOfSamples = 10;

        for (int j = 0; j < numberOfSamples; j++)
        {
            total += analogRead(sensorPins[i]);
            delay(1);
        }

        raw[i] = total / numberOfSamples;
        soilMoisture[i] = convertToMoisturePercentage(raw[i], i);
    }

    Serial.print("Soil % : ");
    for (int i = 0; i < 4; i++)
    {
        Serial.print(soilMoisture[i]);
        Serial.print("% (");
        Serial.print(raw[i]);
        Serial.print(")  ");
    }
    Serial.println();
}

int countDrySensors()
{
    int drySensorCount = 0;

    for (int i = 0; i < 4; i++)
    {
        if (soilMoisture[i] <= DRY_THRESHOLD)
        {
            drySensorCount++;
        }
    }

    return drySensorCount;
}

int getAverageMoisture()
{
    int sum = 0;

    for (int i = 0; i < 4; i++)
    {
        sum += soilMoisture[i];
    }

    return sum / 4;
}

// ================================================================
// WATER LEVEL
// ================================================================

float readDistanceCm()
{
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    // 15 ms timeout is about 2.5 m, far more than the tank needs.
    unsigned long duration =
        pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 15000);

    if (duration == 0)
    {
        return -1;
    }

    return duration * 0.0343 / 2.0;
}

// Median of 3 readings (removes ripples and noise).
void readWaterLevel()
{
    const int numberOfReadings = 3;
    float readings[numberOfReadings];
    int validCount = 0;

    for (int i = 0; i < numberOfReadings; i++)
    {
        float distance = readDistanceCm();

        if (distance > 0 && distance < 400)
        {
            readings[validCount] = distance;
            validCount++;
        }

        delay(30);
    }

    if (validCount == 0)
    {
        waterSensorOk = false;
        Serial.println("Water level: no echo from ultrasonic sensor.");
        return;
    }

    for (int i = 1; i < validCount; i++)
    {
        float key = readings[i];
        int j = i - 1;

        while (j >= 0 && readings[j] > key)
        {
            readings[j + 1] = readings[j];
            j--;
        }

        readings[j + 1] = key;
    }

    waterDistanceCm = readings[validCount / 2];

    float range = TANK_EMPTY_CM - TANK_FULL_CM;
    float percent =
        (TANK_EMPTY_CM - waterDistanceCm) / range * 100.0;

    waterLevel = constrain((int)round(percent), 0, 100);
    waterSensorOk = true;
}

// Locks the pump when water is low. Unlocks at WATER_RESUME_PERCENT.
// If the sensor fails, the pump stays locked for safety.
void updateWaterLock()
{
    bool oldValue = waterLow;

    if (!waterSensorOk)
    {
        waterLow = true;
    }
    else if (waterLevel <= WATER_LOW_PERCENT)
    {
        waterLow = true;
    }
    else if (waterLevel >= WATER_RESUME_PERCENT)
    {
        waterLow = false;
    }

    if (waterLow != oldValue)
    {
        Serial.println(waterLow
            ? "WATER LOCK: Water is low. Pump is blocked."
            : "WATER LOCK: Water is enough. Pump is allowed.");
    }
}

// Works in AUTO and MANUAL mode, online or offline.
void enforceWaterProtection()
{
    if (waterLow && pumpState)
    {
        Serial.println("WATER LOCK: Turning PUMP OFF (low water).");
        setPump(false);
    }
}

// ================================================================
// AUTOMATIC PUMP CONTROL (fully local, no network needed)
// ================================================================

void controlPump()
{
    if (!pumpState)
    {
        if (countDrySensors() >= 3)
        {
            if (waterLow)
            {
                Serial.println("AUTO MODE: Soil is dry but water is low. Pump blocked.");
            }
            else if (rainDelay)
            {
                Serial.println("AUTO MODE: Soil is dry but RAIN DELAY is active. Pump blocked.");
            }
            else
            {
                Serial.println("AUTO MODE: 3 or more sensors <= 30%. Pump ON");
                setPump(true);
            }
        }
    }
    else
    {
        int averageMoisture = getAverageMoisture();

        if (averageMoisture >= threshold)
        {
            Serial.print("AUTO MODE: Average ");
            Serial.print(averageMoisture);
            Serial.print("% reached threshold ");
            Serial.print(threshold);
            Serial.println("%. Pump OFF");

            setPump(false);
        }
    }
}

// ================================================================
// MANUAL MODE TIMEOUT (fully local, works with or without Wi-Fi)
// ================================================================
// If MANUAL mode lasts MANUAL_TIMEOUT_MS, the pump is forced OFF and
// the system returns to AUTO. The network task then uploads
// mode = "AUTO" and pump = false to Firebase immediately (retried until
// it works), so the app follows. Pressing MANUAL in the app once starts
// a new 5 minute timer.

void checkManualTimeout()
{
    if (systemMode != "MANUAL")
    {
        return;
    }

    if (millis() - manualStartedAt < MANUAL_TIMEOUT_MS)
    {
        return;
    }

    Serial.println("MANUAL MODE: 5 minute timeout. Returning to AUTO.");

    if (pumpState)
    {
        Serial.println("MANUAL MODE: Timeout - forcing PUMP OFF.");
        setPump(false);
    }

    systemMode = "AUTO";
    saveMode();

    // Tell the network task to upload the new mode.
    modeVersion = modeVersion + 1;

    updateLcd();
}

// ================================================================
// APPLY COMMANDS RECEIVED FROM THE CLOUD (main loop)
// ================================================================

void applyNetworkCommands()
{
    int m, t, p, r;

    portENTER_CRITICAL(&netMux);
    if (!netFresh)
    {
        portEXIT_CRITICAL(&netMux);
        return;
    }
    m = netMode;
    t = netThreshold;
    p = netPump;
    r = netRain;
    netFresh = false;
    portEXIT_CRITICAL(&netMux);

    // Mode
    // Ignored while the device's own AUTO (after a MANUAL timeout) is
    // not uploaded yet, so an old cloud value can't override it.
    if ((m == 0 || m == 1) && !modeNeedsSync())
    {
        String requestedMode = (m == 1) ? "MANUAL" : "AUTO";

        if (requestedMode != systemMode)
        {
            Serial.print("Operating mode changed from ");
            Serial.print(systemMode);
            Serial.print(" to ");
            Serial.println(requestedMode);

            systemMode = requestedMode;
            saveMode();

            if (systemMode == "MANUAL")
            {
                manualStartedAt = millis();   // start the 5 minute timer
            }

            if (systemMode == "AUTO")
            {
                controlPump();
            }

            updateLcd();
        }
    }

    // Threshold
    if (t >= 0)
    {
        t = constrain(t, 0, 100);

        if (t != threshold)
        {
            Serial.print("Threshold changed from ");
            Serial.print(threshold);
            Serial.print("% to ");
            Serial.print(t);
            Serial.println("%");

            threshold = t;
            saveThreshold();
        }
    }

    // Rain delay on / off (from the app)
    if (r == 0 || r == 1)
    {
        bool en = (r == 1);

        if (en != rainDelayEnabled)
        {
            rainDelayEnabled = en;
            prefs.putBool("rainEn", en);

            Serial.print("Rain delay feature: ");
            Serial.println(en ? "ENABLED" : "DISABLED");

            updateRainDelay();
            updateLcd();
        }
    }

    // Manual pump command
    // Ignored while a local pump change is still waiting to be uploaded,
    // so an old cloud value can't override it.
    if (systemMode == "MANUAL" && p >= 0 && !pumpNeedsSync())
    {
        bool requested = (p == 1);

        if (requested && waterLow)
        {
            Serial.println("MANUAL MODE: Pump request refused. Water is low.");

            if (pumpState)
            {
                setPump(false);
            }
            else
            {
                // Cloud says ON but the pump is OFF: push the real state.
                pumpVersion = pumpVersion + 1;
            }
        }
        else if (requested && rainDelay)
        {
            Serial.println("MANUAL MODE: Pump request refused. Rain delay is active.");

            if (pumpState)
            {
                setPump(false);
            }
            else
            {
                pumpVersion = pumpVersion + 1;
            }
        }
        else if (requested != pumpState)
        {
            Serial.print("MANUAL MODE: User requested pump ");
            Serial.println(requested ? "ON" : "OFF");

            setPump(requested);
        }
    }
}

// ================================================================
// WIFI (NON-BLOCKING, BACKGROUND RECONNECT) - network task only
// ================================================================

bool wifiConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

void startWiFi()
{
    Serial.println("Wi-Fi: starting connection in the background...");

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastWifiAttempt = millis();
}

void maintainWiFi()
{
    if (wifiConnected())
    {
        if (!wifiWasOnline)
        {
            wifiWasOnline = true;
            firebaseRetryAt = 0;

            Serial.println("Wi-Fi: CONNECTED.");
            Serial.print("ESP32 IP address: ");
            Serial.println(WiFi.localIP());
        }

        return;
    }

    if (wifiWasOnline)
    {
        wifiWasOnline = false;
        firebaseReachable = false;
        lastWifiAttempt = millis();

        // Drop the old secure connection so it is not reused dead.
        sslClient.stop();

        Serial.println("Wi-Fi: connection LOST. Running in offline mode.");
    }

    if (millis() - lastWifiAttempt >= WIFI_RETRY_INTERVAL)
    {
        lastWifiAttempt = millis();

        Serial.println("Wi-Fi: offline, retrying in the background...");

        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
}

bool firebaseAvailable()
{
    return wifiConnected() && millis() >= firebaseRetryAt;
}

// ================================================================
// FIREBASE REQUEST - network task only
// ================================================================
// Returns the HTTP code (200 = OK), or a negative number on a
// connection error. A connection error starts a short back-off.

int firebaseRequest(const char *method, const String &path,
                    const String &body, String *out)
{
    if (!firebaseAvailable())
    {
        return -1000;
    }

    String url = String(DATABASE_URL) + path + ".json";

    if (strlen(DATABASE_AUTH_QUERY) > 0)
    {
        url += DATABASE_AUTH_QUERY;
    }

    unsigned long startTime = millis();

    http.setReuse(true);
    http.setTimeout(FIREBASE_TIMEOUT_MS);
    http.setConnectTimeout(FIREBASE_TIMEOUT_MS);

    if (!http.begin(sslClient, url))
    {
        Serial.println("HTTP begin() failed.");
        return -1001;
    }

    http.addHeader("Content-Type", "application/json");

    int httpCode;

    if (strcmp(method, "GET") == 0)
    {
        httpCode = http.GET();
    }
    else
    {
        httpCode = http.sendRequest(method, body);
    }

    if (httpCode == 200)
    {
        if (out != nullptr)
        {
            *out = http.getString();
        }

        firebaseReachable = true;
    }
    else
    {
        Serial.print("Firebase ");
        Serial.print(method);
        Serial.print(" failed. HTTP code: ");
        Serial.println(httpCode);

        if (httpCode <= 0)
        {
            firebaseReachable = false;
            firebaseRetryAt = millis() + FIREBASE_BACKOFF_MS;
        }
    }

    http.end();

    unsigned long elapsed = millis() - startTime;

    if (elapsed > 800)
    {
        Serial.print("Firebase slow request (");
        Serial.print(method);
        Serial.print("): ");
        Serial.print(elapsed);
        Serial.println(" ms");
    }

    return httpCode;
}

// ================================================================
// CLOUD UPLOAD / DOWNLOAD - network task only
// ================================================================

// ONE request uploads every sensor value (multi-path update).
// system/heartbeat changes on every upload (the app uses it to show
// ONLINE / OFFLINE).
// The pump state is NOT included here on purpose: sending it
// periodically could overwrite a command from the app before the
// device has read it.
void uploadSensors()
{
    String body = "{";

    for (int i = 0; i < 4; i++)
    {
        body += "\"sensors/soil" + String(i + 1) + "\":" +
                String(soilMoisture[i]) + ",";
    }

    // If the tank sensor fails, send 0 so the app shows the pump lock
    // instead of an old, wrong tank level.
    body += "\"sensors/waterLevel\":" +
            String(waterSensorOk ? (int)waterLevel : 0) + ",";

    heartbeat++;
    body += "\"system/heartbeat\":" + String(heartbeat) + ",";

    body += "\"system/waterLow\":";
    body += waterLow ? "true" : "false";
    body += "}";

    firebaseRequest("PATCH", "/smartIrrigation", body, nullptr);
}

// Sent immediately when the pump changes, and retried until it works.
void pushPumpState()
{
    uint32_t version = pumpVersion;
    bool state = pumpState;

    int code = firebaseRequest(
        "PUT",
        "/smartIrrigation/system/pump",
        state ? "true" : "false",
        nullptr
    );

    if (code == 200)
    {
        pumpSyncedVersion = version;

        Serial.print("Firebase pump state: ");
        Serial.println(state ? "ON" : "OFF");
    }
}

// Sent after a MANUAL timeout, and retried until it works.
void pushModeState()
{
    uint32_t version = modeVersion;

    int code = firebaseRequest(
        "PUT",
        "/smartIrrigation/system/mode",
        "\"AUTO\"",
        nullptr
    );

    if (code == 200)
    {
        modeSyncedVersion = version;

        Serial.println("Firebase mode: AUTO");
    }
}

// ONE request reads mode, threshold and pump command together.
void fetchCommands()
{
    String response;

    int code = firebaseRequest(
        "GET",
        "/smartIrrigation/system",
        "",
        &response
    );

    if (code != 200)
    {
        return;
    }

    response.trim();

    if (response.length() == 0 || response == "null")
    {
        return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, response);

    if (err)
    {
        Serial.print("Command parse error: ");
        Serial.println(err.c_str());
        return;
    }

    int m = -1;
    int t = -1;
    int p = -1;
    int r = -1;

    // Mode
    JsonVariant vMode = doc["mode"];

    if (!vMode.isNull() && !modeNeedsSync())
    {
        String s = cleanFirebaseString(vMode.as<String>());
        s.trim();
        s.toUpperCase();

        if (s == "AUTO")
        {
            m = 0;
        }
        else if (s == "MANUAL")
        {
            m = 1;
        }
        else
        {
            Serial.print("Invalid mode received from Firebase: ");
            Serial.println(s);
        }
    }

    // Threshold
    JsonVariant vThreshold = doc["threshold"];

    if (!vThreshold.isNull())
    {
        int value;

        if (variantToInt(vThreshold, value))
        {
            t = value;
        }
        else
        {
            Serial.println("Invalid threshold received from Firebase.");
        }
    }

    // Pump command (skipped if a local pump change is not uploaded yet)
    JsonVariant vPump = doc["pump"];

    if (!vPump.isNull() && !pumpNeedsSync())
    {
        bool value;

        if (variantToBool(vPump, value))
        {
            p = value ? 1 : 0;
        }
        else
        {
            Serial.println("Invalid pump command received from Firebase.");
        }
    }

    // Rain delay on/off switch
    JsonVariant vRain = doc["rainDelayEnabled"];

    if (!vRain.isNull())
    {
        bool value;

        if (variantToBool(vRain, value))
        {
            r = value ? 1 : 0;
        }
    }

    portENTER_CRITICAL(&netMux);
    netMode = m;
    netThreshold = t;
    netPump = p;
    netRain = r;
    netFresh = true;
    portEXIT_CRITICAL(&netMux);
}

// ================================================================
// NETWORK TASK (runs on core 0, separate from the control loop)
// ================================================================

// ================================================================
// WEATHER TEXT HELPERS
// ================================================================

String conditionText(int code, bool isDay)
{
    switch (code)
    {
        case 0:  return isDay ? "Sunny" : "Clear";
        case 1:  return isDay ? "Mostly sunny" : "Mostly clear";
        case 2:  return "Partly cloudy";
        case 3:  return "Cloudy";
        case 45:
        case 48: return "Foggy";
        case 51:
        case 53:
        case 55:
        case 56:
        case 57: return "Drizzle";
        case 61:
        case 63: return "Rain";
        case 65: return "Heavy rain";
        case 66:
        case 67: return "Freezing rain";
        case 71:
        case 73:
        case 75:
        case 77:
        case 85:
        case 86: return "Snow";
        case 80:
        case 81: return "Rain showers";
        case 82: return "Heavy showers";
        case 95: return "Thunderstorm";
        case 96:
        case 99: return "Thunder + hail";
        default: return "Unknown";
    }
}

// "Cooling" / "Warming" / "Steady".
// Cloud forecast: temperature 3 h ahead vs now (+/-1.5 C).
// Local sensor:   what the temperature really did in the last hour (+/-1.0 C).
// If both have an opinion and they disagree, the answer is "Steady".
const char *outlookText(const CloudWeather &c, const LocalEnv &le)
{
    int cdir = 0;

    if (c.valid && !isnan(c.tempSoon) && !isnan(c.temp))
    {
        float d = c.tempSoon - c.temp;

        if (d <= -1.5) cdir = -1;
        else if (d >= 1.5) cdir = 1;
    }

    int ldir = 0;

    if (le.available && le.tTrendValid)
    {
        if (le.tTrend1h <= -1.0) ldir = -1;
        else if (le.tTrend1h >= 1.0) ldir = 1;
    }

    int s;

    if (cdir != 0 && ldir != 0 && cdir != ldir)
    {
        s = 0;
    }
    else
    {
        s = (cdir != 0) ? cdir : ldir;
    }

    return s < 0 ? "Cooling" : (s > 0 ? "Warming" : "Steady");
}

String jnum(float v, int decimals)
{
    if (isnan(v) || isinf(v))
    {
        return "null";
    }

    return String(v, decimals);
}

// ================================================================
// LOCAL SENSOR: BMP280 (HW-611)  -  loop (core 1) only
// ================================================================
// The chip is identified by its ID register (0xD0):
//   0x56 / 0x57 / 0x58 = BMP280 (temperature + pressure, NO humidity)
// Any other ID (for example 0x60 = BME280) is not used.

uint8_t i2cReadChipId(uint8_t addr)
{
    Wire.beginTransmission(addr);
    Wire.write(0xD0);

    if (Wire.endTransmission(false) != 0)
    {
        return 0;   // nothing answers at this address
    }

    if (Wire.requestFrom(addr, (uint8_t)1) != 1)
    {
        return 0;
    }

    return Wire.read();
}

bool detectEnvSensor()
{
    const uint8_t addresses[2] = { 0x76, 0x77 };

    for (int i = 0; i < 2; i++)
    {
        uint8_t id = i2cReadChipId(addresses[i]);

        if (id == 0)
        {
            continue;
        }

        if (id == 0x58 || id == 0x57 || id == 0x56)
        {
            if (bmp.begin(addresses[i], id))
            {
                bmp.setSampling(
                    Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,    // temperature
                    Adafruit_BMP280::SAMPLING_X4,    // pressure
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_1000);

                envType = 1;
                envAddr = addresses[i];
                envFail = 0;
                envIdx = 0;
                envCount = 0;
                trendN = 0;
                trendHead = 0;
                return true;
            }
        }
    }

    envType = 0;
    return false;
}

// One reading with sanity checks (temperature + pressure only).
bool readEnvOnce(float &t, float &p)
{
    if (i2cReadChipId(envAddr) == 0)
    {
        return false;   // sensor stopped answering
    }

    t = bmp.readTemperature();
    p = bmp.readPressure() / 100.0F;

    if (isnan(t) || isnan(p) || t < -40 || t > 85 || p < 300 || p > 1100)
    {
        return false;
    }

    return true;
}

float envAverage(const float *values)
{
    float sum = 0;

    for (int i = 0; i < envCount; i++)
    {
        sum += values[i];
    }

    return envCount > 0 ? sum / envCount : NAN;
}

const TrendSample &trendAt(int k)   // k = 0 is the newest sample
{
    int idx = (trendHead - 1 - k + TREND_SIZE * 2) % TREND_SIZE;
    return trendBuf[idx];
}

void updateEnvSensor()
{
    if (envType == 0)
    {
        if (detectEnvSensor())
        {
            Serial.print("Local sensor found: BMP280");
            Serial.print(" at 0x");
            Serial.println(envAddr, HEX);
        }

        return;
    }

    float t, p;

    if (!readEnvOnce(t, p))
    {
        envFail++;

        if (envFail >= 5)
        {
            Serial.println("Local sensor: lost. Ignoring it until it is found again.");

            envType = 0;
            envCount = 0;
            envIdx = 0;
            trendN = 0;
            trendHead = 0;
        }

        return;
    }

    envFail = 0;

    envT[envIdx] = t;
    envP[envIdx] = p;
    envIdx = (envIdx + 1) % ENV_AVG_COUNT;

    if (envCount < ENV_AVG_COUNT)
    {
        envCount++;
    }

    // One history sample every 10 minutes (for the 3 h pressure trend
    // and the 1 h temperature trend).
    if (envCount >= 3 &&
        (trendN == 0 || millis() - lastTrendSample >= TREND_SAMPLE_MS))
    {
        lastTrendSample = millis();

        trendBuf[trendHead].t = envAverage(envT);
        trendBuf[trendHead].p = envAverage(envP);
        trendBuf[trendHead].ms = millis();
        trendHead = (trendHead + 1) % TREND_SIZE;

        if (trendN < TREND_SIZE)
        {
            trendN++;
        }
    }
}

// ================================================================
// FUSION: cloud forecast + local sensor  -  loop (core 1) only
// ================================================================
// What the local sensor adds to the cloud forecast:
//  1. Sea-level pressure (needs altitude) so it can be compared.
//  2. Pressure trend over 3 h  -> falling fast = rain more likely.
//  3. Temperature trend over 1 h -> confirms "Cooling" / "Warming".
//  4. A health check against the cloud values (bad placement / faulty).
//  5. Rain chance adjustment of -10 / 0 / +10 points (never more).

void computeLocal(const CloudWeather &c, bool cloudFresh, int cloudProb)
{
    LocalEnv le;
    memset(&le, 0, sizeof(le));
    le.temp = le.press = le.pressSea = NAN;
    le.tempDiff = le.pressDiff = NAN;

    if (envType != 0 && envCount >= 3)
    {
        le.available = true;
        le.type = envType;
        le.addr = envAddr;

        float T = envAverage(envT);
        float P = envAverage(envP);

        le.temp = T;
        le.press = P;

        float alt = (LOCAL_ALTITUDE_M > -9000.0)
                        ? LOCAL_ALTITUDE_M
                        : (c.valid && !isnan(c.elevation) ? c.elevation : 0.0);

        le.pressSea = P / pow(1.0 - alt / 44330.0, 5.255);

        // Pressure trend (hPa per 3 h): newest average vs oldest sample.
        if (trendN >= 2)
        {
            const TrendSample &old = trendAt(trendN - 1);
            unsigned long span = millis() - old.ms;

            le.trendMinutes = (int)(span / 60000UL);

            if (span >= TREND_MIN_SPAN_MS)
            {
                le.trend3h = (P - old.p) * (10800000.0F / (float)span);
                le.trendValid = true;
            }
        }

        // Temperature trend (C per hour) from the sample about 1 h ago.
        if (trendN >= 6)
        {
            int k = (trendN > 6) ? 6 : (trendN - 1);
            const TrendSample &ref = trendAt(k);
            unsigned long span = millis() - ref.ms;

            if (span >= 2700000UL)   // at least 45 min
            {
                le.tTrend1h = (T - ref.t) * (3600000.0F / (float)span);
                le.tTrendValid = true;
            }
        }

        // Health check against the cloud values.
        if (!cloudFresh)
        {
            le.status = 4;   // nothing to compare with
        }
        else
        {
            le.diffValid = true;
            le.pressDiff = P - c.pressSurf;
            le.tempDiff = T - c.temp;

            if (fabs(le.pressDiff) > 20.0)
            {
                le.status = 3;   // suspect: ignored for control
            }
            else if (fabs(le.tempDiff) > 6.0)
            {
                le.status = 2;   // check placement (sun / heat / enclosure)
            }
            else
            {
                le.status = 1;
            }
        }

        // Rain probability adjustment.
        bool usable = le.status != 3 && le.trendValid;

        if (usable && cloudProb >= 0)
        {
            bool fastFall = le.trend3h <= -RAIN_PRESSURE_TREND_HPA;
            bool fastRise = le.trend3h >= RAIN_PRESSURE_TREND_HPA;

            if (fastFall && cloudProb >= 60)
            {
                le.adj = 10;
            }
            else if (fastRise && cloudProb < 95)
            {
                le.adj = -10;
            }
        }
    }

    localLoop = le;

    portENTER_CRITICAL(&wxMux);
    localShared = le;
    portEXIT_CRITICAL(&wxMux);
}

void updateRainDelay()
{
    CloudWeather c;

    portENTER_CRITICAL(&wxMux);
    c = cloudShared;
    portEXIT_CRITICAL(&wxMux);

    cloudLoop = c;

    bool fresh = c.valid && (millis() - c.receivedAt < WEATHER_STALE_MS);
    cloudFreshLoop = fresh;

    bool enabled = rainDelayEnabled;
    int cloudProb = (enabled && fresh) ? c.probMax : -1;

    computeLocal(c, fresh, cloudProb);

    bool newState = rainDelay;
    int eff = -1;
    uint8_t reason;

    if (!enabled)
    {
        newState = false;
        reason = 1;
    }
    else if (!fresh)
    {
        // No recent weather: never block irrigation because of old data.
        newState = false;
        reason = 2;
    }
    else
    {
        eff = constrain(cloudProb + localLoop.adj, 0, 100);

        if (!rainDelay && eff >= RAIN_DELAY_ON_PERCENT)
        {
            newState = true;
        }
        else if (rainDelay && eff < RAIN_DELAY_OFF_PERCENT)
        {
            newState = false;
        }

        reason = newState ? 3 : 4;
    }

    bool changed = (newState != rainDelay);

    portENTER_CRITICAL(&wxMux);
    rainDelay = newState;
    rainProbCloud = cloudProb;
    rainProbEff = eff;
    rainReason = reason;
    portEXIT_CRITICAL(&wxMux);

    if (changed)
    {
        Serial.print("RAIN DELAY: ");
        Serial.print(newState ? "ACTIVE" : "ended");
        Serial.print(" (cloud ");
        Serial.print(cloudProb);
        Serial.print("%, local adjustment ");
        Serial.print(localLoop.adj);
        Serial.print(", effective ");
        Serial.print(eff);
        Serial.println("%)");

        updateLcd();
    }
}

// Works in AUTO and MANUAL mode, online or offline.
void enforceRainDelay()
{
    if (rainDelay && pumpState)
    {
        Serial.println("RAIN DELAY: Turning PUMP OFF (rain expected).");
        setPump(false);
    }
}

// Soil moisture confirms real rain. Watched ONLY while the rain delay is
// active and the pump is OFF, so irrigation water can never be mistaken
// for rain. Works offline. It does not change the pump or the forecast.
void updateRainConfirm()
{
    unsigned long now = millis();

    if (!rainDelay)
    {
        rainBaseSet = false;
        soilCheck = 0;

        if (rainConfirmed)
        {
            rainConfirmed = false;
            soilRise = 0;
            Serial.println("RAIN CONFIRM: delay ended, status cleared.");
            updateLcd();
        }

        return;
    }

    if (pumpState)
    {
        rainBaseSet = false;
        return;
    }

    int avg = getAverageMoisture();

    if (!rainBaseSet)
    {
        for (int i = 0; i < 4; i++) rainBase[i] = soilMoisture[i];

        rainBaseAvg = avg;
        rainPeakAvg = avg;
        rainLastRise = now;
        rainBaseSet = true;
        soilRise = 0;
        soilCheck = (avg > 100 - RAIN_SOIL_RISE_AVG) ? 3 : 1;
        return;
    }

    // The baseline follows the driest value seen (soil slowly drying).
    for (int i = 0; i < 4; i++)
    {
        if (soilMoisture[i] < rainBase[i]) rainBase[i] = soilMoisture[i];
    }

    if (avg < rainBaseAvg) rainBaseAvg = avg;

    int rising = 0;

    for (int i = 0; i < 4; i++)
    {
        if (soilMoisture[i] - rainBase[i] >= RAIN_SOIL_RISE_SENSOR) rising++;
    }

    soilRise = avg - rainBaseAvg;

    if (avg > rainPeakAvg)
    {
        rainPeakAvg = avg;
        rainLastRise = now;
    }

    if (!rainConfirmed)
    {
        if (soilRise >= RAIN_SOIL_RISE_AVG && rising >= RAIN_SOIL_MIN_SENSORS)
        {
            rainConfirmed = true;
            rainLastRise = now;

            Serial.print("RAIN CONFIRMED: soil moisture rose ");
            Serial.print(soilRise);
            Serial.println(" points while the pump was off.");

            updateLcd();
        }
    }
    else if (now - rainLastRise >= RAIN_SEEN_HOLD_MS)
    {
        rainConfirmed = false;
        rainBaseSet = false;   // a new rise is needed to confirm again

        Serial.println("RAIN CONFIRM: moisture stopped rising, rain has stopped.");
        updateLcd();
    }

    // Soil at about 100 % cannot rise any more, so rain cannot be confirmed.
    soilCheck = rainConfirmed ? 2 : ((rainBaseAvg > 100 - RAIN_SOIL_RISE_AVG) ? 3 : 1);
}

// ================================================================
// LCD: weather screens
// ================================================================

void lcdWeatherPage(String &line1, String &line2)
{
    if (!cloudFreshLoop)
    {
        line1 = "Weather: N/A";
        line2 = wifiConnected() ? "Waiting data..." : "Offline";
        return;
    }

    int prob = (rainProbEff >= 0) ? rainProbEff : cloudLoop.probMax;

    line1 = "Rain:" + String(prob) + "% " +
            String((int)round(cloudLoop.temp)) + "C";

    if (!rainDelayEnabled)
    {
        line2 = "Delay feature OFF";
    }
    else if (rainDelay && rainConfirmed)
    {
        line2 = "Raining (soil)";
    }
    else if (rainDelay)
    {
        line2 = "RAIN DELAY: ON";
    }
    else
    {
        line2 = conditionText(cloudLoop.code, cloudLoop.isDay);
    }
}

void lcdLocalPage(String &line1, String &line2)
{
    if (envType == 0 || !localLoop.available)
    {
        line1 = "Local sensor:";
        line2 = "Not found";
        return;
    }

    line1 = "Temp: " + String(localLoop.temp, 1) + "C";
    line2 = "Press: " + String((int)round(localLoop.press)) + " hPa";
}

// ================================================================
// CLOUD WEATHER FETCH - network task only
// ================================================================
// ONE request to Open-Meteo returns everything. The answer is filtered
// while it is parsed, so only the needed fields use memory.

bool fetchWeather()
{
    if (!wifiConnected())
    {
        return false;
    }

    String url = "https://api.open-meteo.com/v1/forecast?latitude=" +
                 String(WEATHER_LAT, 6) + "&longitude=" + String(WEATHER_LON, 6) +
                 "&current=temperature_2m,relative_humidity_2m,apparent_temperature,"
                 "is_day,precipitation,weather_code,cloud_cover,pressure_msl,"
                 "surface_pressure,wind_speed_10m,wind_direction_10m,wind_gusts_10m"
                 "&hourly=temperature_2m,precipitation_probability,precipitation"
                 "&forecast_hours=6"
                 "&daily=temperature_2m_max,temperature_2m_min,"
                 "precipitation_probability_max,precipitation_sum,uv_index_max,"
                 "sunrise,sunset&forecast_days=1&timezone=auto";

    // Free memory for the second secure connection.
    sslClient.stop();

    HTTPClient wh;
    wh.setReuse(false);
    wh.setTimeout(WEATHER_TIMEOUT_MS);
    wh.setConnectTimeout(WEATHER_TIMEOUT_MS);

    if (!wh.begin(weatherClient, url))
    {
        Serial.println("Weather: HTTP begin() failed.");
        return false;
    }

    int code = wh.GET();

    if (code != 200)
    {
        Serial.print("Weather: HTTP error ");
        Serial.println(code);
        wh.end();
        weatherClient.stop();
        return false;
    }

    String payload = wh.getString();
    wh.end();
    weatherClient.stop();

    StaticJsonDocument<768> filter;
    filter["elevation"] = true;

    const char *curKeys[] = {
        "temperature_2m", "relative_humidity_2m", "apparent_temperature",
        "is_day", "precipitation", "weather_code", "cloud_cover",
        "pressure_msl", "surface_pressure", "wind_speed_10m",
        "wind_direction_10m", "wind_gusts_10m"
    };

    for (size_t i = 0; i < sizeof(curKeys) / sizeof(curKeys[0]); i++)
    {
        filter["current"][curKeys[i]] = true;
    }

    filter["hourly"]["temperature_2m"] = true;
    filter["hourly"]["precipitation_probability"] = true;
    filter["hourly"]["precipitation"] = true;

    const char *dayKeys[] = {
        "temperature_2m_max", "temperature_2m_min",
        "precipitation_probability_max", "precipitation_sum",
        "uv_index_max", "sunrise", "sunset"
    };

    for (size_t i = 0; i < sizeof(dayKeys) / sizeof(dayKeys[0]); i++)
    {
        filter["daily"][dayKeys[i]] = true;
    }

    StaticJsonDocument<3072> doc;
    DeserializationError err =
        deserializeJson(doc, payload, DeserializationOption::Filter(filter));

    if (err)
    {
        Serial.print("Weather: parse error: ");
        Serial.println(err.c_str());
        return false;
    }

    JsonObject cur = doc["current"];

    if (cur.isNull() || cur["temperature_2m"].isNull())
    {
        Serial.println("Weather: no current data in the answer.");
        return false;
    }

    CloudWeather c;
    memset(&c, 0, sizeof(c));

    c.temp = cur["temperature_2m"] | NAN;
    c.feels = cur["apparent_temperature"] | NAN;
    c.humidity = cur["relative_humidity_2m"] | NAN;
    c.pressMsl = cur["pressure_msl"] | NAN;
    c.pressSurf = cur["surface_pressure"] | NAN;
    c.windKmh = cur["wind_speed_10m"] | NAN;
    c.gustKmh = cur["wind_gusts_10m"] | NAN;
    c.windDir = cur["wind_direction_10m"] | 0;
    c.cloudCover = cur["cloud_cover"] | 0;
    c.code = cur["weather_code"] | 0;
    c.isDay = (cur["is_day"] | 1) == 1;
    c.rainNowMm = cur["precipitation"] | 0.0F;
    c.elevation = doc["elevation"] | 0.0F;

    JsonArray hp = doc["hourly"]["precipitation_probability"];
    JsonArray ht = doc["hourly"]["temperature_2m"];
    JsonArray hr = doc["hourly"]["precipitation"];

    int lookahead = constrain(RAIN_LOOKAHEAD_HOURS, 1, 6);

    c.probMax = 0;
    c.rainSoonMm = 0;

    for (int i = 0; i < 6; i++)
    {
        c.hProb[i] = hp[i] | -1;
        c.hTemp[i] = ht[i] | NAN;

        if (i < lookahead)
        {
            if (c.hProb[i] > c.probMax)
            {
                c.probMax = c.hProb[i];
            }

            c.rainSoonMm += (float)(hr[i] | 0.0F);
        }
    }

    c.probNow = c.hProb[0] < 0 ? 0 : c.hProb[0];
    c.tempSoon = c.hTemp[3];

    JsonObject day = doc["daily"];

    c.dayMax = day["temperature_2m_max"][0] | NAN;
    c.dayMin = day["temperature_2m_min"][0] | NAN;
    c.dayProb = day["precipitation_probability_max"][0] | 0;
    c.daySumMm = day["precipitation_sum"][0] | 0.0F;
    c.uvMax = day["uv_index_max"][0] | 0.0F;

    // "2026-11-24T06:38" -> "06:38"
    const char *sr = day["sunrise"][0] | "";
    const char *ss = day["sunset"][0] | "";

    if (strlen(sr) >= 16) { strncpy(c.sunrise, sr + 11, 5); c.sunrise[5] = 0; }
    if (strlen(ss) >= 16) { strncpy(c.sunset, ss + 11, 5); c.sunset[5] = 0; }

    time_t nowEpoch = time(nullptr);
    c.epoch = (nowEpoch > 1700000000) ? (long)nowEpoch : 0;

    c.valid = true;
    c.receivedAt = millis();

    portENTER_CRITICAL(&wxMux);
    cloudShared = c;
    portEXIT_CRITICAL(&wxMux);

    Serial.print("Weather OK: ");
    Serial.print(c.temp, 1);
    Serial.print("C, RH ");
    Serial.print(c.humidity, 0);
    Serial.print("%, ");
    Serial.print(c.pressMsl, 1);
    Serial.print(" hPa, rain chance (next ");
    Serial.print(lookahead);
    Serial.print(" h) ");
    Serial.print(c.probMax);
    Serial.print("%, ");
    Serial.println(conditionText(c.code, c.isDay));

    return true;
}

uint32_t weatherCount = 0;

// Sends the cloud weather to Firebase (one PUT). The app shows it.
bool uploadWeather()
{
    CloudWeather c;

    portENTER_CRITICAL(&wxMux);
    c = cloudShared;
    portEXIT_CRITICAL(&wxMux);

    if (!c.valid)
    {
        return false;
    }

    weatherCount++;

    String b = "{";
    b += "\"temperature\":" + jnum(c.temp, 1) + ",";
    b += "\"feelsLike\":" + jnum(c.feels, 1) + ",";
    b += "\"humidity\":" + jnum(c.humidity, 0) + ",";
    b += "\"pressure\":" + jnum(c.pressMsl, 1) + ",";
    b += "\"surfacePressure\":" + jnum(c.pressSurf, 1) + ",";
    b += "\"windSpeed\":" + jnum(c.windKmh, 0) + ",";
    b += "\"windGust\":" + jnum(c.gustKmh, 0) + ",";
    b += "\"windDir\":" + String(c.windDir) + ",";
    b += "\"cloudCover\":" + String(c.cloudCover) + ",";
    b += "\"rainNow\":" + jnum(c.rainNowMm, 1) + ",";
    b += "\"rainProb\":" + String(c.probMax) + ",";
    b += "\"rainProbNow\":" + String(c.probNow) + ",";
    b += "\"rainNext\":" + jnum(c.rainSoonMm, 1) + ",";
    b += "\"lookaheadHours\":" + String(constrain(RAIN_LOOKAHEAD_HOURS, 1, 6)) + ",";
    b += "\"code\":" + String(c.code) + ",";
    b += "\"isDay\":" + String(c.isDay ? "true" : "false") + ",";
    b += "\"condition\":\"" + conditionText(c.code, c.isDay) + "\",";
    b += "\"tempSoon\":" + jnum(c.tempSoon, 1) + ",";
    b += "\"elevation\":" + jnum(c.elevation, 0) + ",";
    b += "\"dayMax\":" + jnum(c.dayMax, 1) + ",";
    b += "\"dayMin\":" + jnum(c.dayMin, 1) + ",";
    b += "\"dayRainProb\":" + String(c.dayProb) + ",";
    b += "\"dayRainSum\":" + jnum(c.daySumMm, 1) + ",";
    b += "\"uvMax\":" + jnum(c.uvMax, 1) + ",";
    b += "\"sunrise\":\"" + String(c.sunrise) + "\",";
    b += "\"sunset\":\"" + String(c.sunset) + "\",";

    b += "\"hProb\":[";
    for (int i = 0; i < 6; i++)
    {
        b += String(c.hProb[i] < 0 ? 0 : c.hProb[i]);
        if (i < 5) b += ",";
    }
    b += "],\"hTemp\":[";
    for (int i = 0; i < 6; i++)
    {
        b += jnum(c.hTemp[i], 1);
        if (i < 5) b += ",";
    }
    b += "],";

    b += "\"updated\":" + String(c.epoch) + ",";
    b += "\"count\":" + String(weatherCount);
    b += "}";

    int code = firebaseRequest("PUT", "/smartIrrigation/weather/cloud", b, nullptr);

    return code == 200;
}

// Every 5 s: rain-delay status + local sensor values (multi-path update).
void uploadControl()
{
    CloudWeather c;
    LocalEnv le;
    bool rd;
    int pc, pe;
    uint8_t rr;

    portENTER_CRITICAL(&wxMux);
    c = cloudShared;
    le = localShared;
    rd = rainDelay;
    pc = rainProbCloud;
    pe = rainProbEff;
    rr = rainReason;
    portEXIT_CRITICAL(&wxMux);

    String b = "{";

    b += "\"weather/control/rainDelay\":";
    b += rd ? "true" : "false";
    b += ",\"weather/control/enabled\":";
    b += rainDelayEnabled ? "true" : "false";
    b += ",\"weather/control/probCloud\":" + (pc >= 0 ? String(pc) : String("null"));
    b += ",\"weather/control/probEffective\":" + (pe >= 0 ? String(pe) : String("null"));
    b += ",\"weather/control/adjust\":" + String(le.adj);
    b += ",\"weather/control/reason\":" + String(rr);
    b += ",\"weather/control/onAt\":" + String(RAIN_DELAY_ON_PERCENT);
    b += ",\"weather/control/offAt\":" + String(RAIN_DELAY_OFF_PERCENT);
    b += ",\"weather/control/outlook\":\"" + String(outlookText(c, le)) + "\"";
    b += ",\"weather/control/rainConfirmed\":";
    b += rainConfirmed ? "true" : "false";
    b += ",\"weather/control/soilRise\":" + String((int)soilRise);
    b += ",\"weather/control/soilCheck\":" + String((int)soilCheck);

    b += ",\"local/available\":";
    b += le.available ? "true" : "false";
    b += ",\"local/type\":\"";
    b += le.type == 1 ? "BMP280" : "none";
    b += "\"";
    // The BMP280 has no humidity sensor: clear any old humidity values.
    b += ",\"local/hasHumidity\":null,\"local/humidity\":null,\"local/dewPoint\":null";

    if (le.available)
    {
        const char *st[] = { "none", "ok", "check", "suspect", "no_cloud" };

        b += ",\"local/address\":\"0x" + String(le.addr, HEX) + "\"";
        b += ",\"local/temperature\":" + jnum(le.temp, 1);
        b += ",\"local/pressure\":" + jnum(le.press, 1);
        b += ",\"local/seaLevelPressure\":" + jnum(le.pressSea, 1);
        b += ",\"local/pressureTrend3h\":" + (le.trendValid ? jnum(le.trend3h, 1) : String("null"));
        b += ",\"local/trendMinutes\":" + String(le.trendMinutes);
        b += ",\"local/tempTrend1h\":" + (le.tTrendValid ? jnum(le.tTrend1h, 1) : String("null"));
        b += ",\"local/tempDiff\":" + (le.diffValid ? jnum(le.tempDiff, 1) : String("null"));
        b += ",\"local/pressDiff\":" + (le.diffValid ? jnum(le.pressDiff, 1) : String("null"));
        b += ",\"local/status\":\"" + String(st[le.status < 5 ? le.status : 0]) + "\"";
        b += ",\"local/adjust\":" + String(le.adj);
    }
    else
    {
        // Sensor not found: clear old values so the app shows "not detected".
        const char *keys[] = {
            "address", "temperature", "pressure", "seaLevelPressure",
            "pressureTrend3h", "trendMinutes", "tempTrend1h",
            "tempDiff", "pressDiff", "status", "adjust"
        };

        for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
        {
            b += ",\"local/" + String(keys[i]) + "\":null";
        }
    }

    b += "}";

    firebaseRequest("PATCH", "/smartIrrigation", b, nullptr);
}

// ================================================================
// NETWORK TASK (runs on core 0, separate from the control loop)
// ================================================================

void networkTask(void *param)
{
    unsigned long lastUpload = 0;
    unsigned long lastFetch = 0;
    unsigned long lastControl = 0;
    unsigned long nextWeatherAt = 0;
    bool weatherScheduled = false;
    bool weatherUploadPending = false;
    bool clockStarted = false;

    for (;;)
    {
        maintainWiFi();

        if (wifiConnected())
        {
            // Internet clock (only used to time-stamp the weather).
            if (!clockStarted)
            {
                clockStarted = true;
                configTime(0, 0, "pool.ntp.org", "time.google.com");
            }

            // First weather request 8 s after Wi-Fi is up.
            if (!weatherScheduled)
            {
                weatherScheduled = true;
                nextWeatherAt = millis() + 8000;
            }
        }

        if (firebaseAvailable())
        {
            // 1. Pump change goes out immediately.
            if (pumpNeedsSync())
            {
                pushPumpState();
            }

            // 1b. Mode forced back to AUTO (manual timeout) goes out too.
            if (modeNeedsSync() && firebaseAvailable())
            {
                pushModeState();
            }

            // 2. Periodic sensor upload.
            if (millis() - lastUpload >= UPLOAD_INTERVAL)
            {
                lastUpload = millis();
                uploadSensors();
            }

            // 3. Periodic command check.
            if (firebaseAvailable() &&
                millis() - lastFetch >= COMMAND_INTERVAL)
            {
                lastFetch = millis();
                fetchCommands();
            }

            // 4. Rain-delay status + local BMP280 values (every 5 s).
            if (firebaseAvailable() &&
                millis() - lastControl >= 5000)
            {
                lastControl = millis();
                uploadControl();
            }

            // 5. Send a freshly downloaded weather report to Firebase.
            if (weatherUploadPending && firebaseAvailable())
            {
                if (uploadWeather())
                {
                    weatherUploadPending = false;
                }
            }
        }

        // 6. Cloud weather download (every 10 min; a few seconds each time).
        if (weatherScheduled && wifiConnected() && millis() >= nextWeatherAt)
        {
            bool ok = fetchWeather();

            nextWeatherAt = millis() + (ok ? WEATHER_INTERVAL_MS : WEATHER_RETRY_MS);

            if (ok)
            {
                weatherUploadPending = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" SMART IRRIGATION CONTROL SYSTEM (REST)");
    Serial.println("     OFFLINE-FIRST / FAST VERSION");
    Serial.println("========================================");

    // LCD + start-up screen 1 (Welcome!)
    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    showWelcomeScreen();
    unsigned long welcomeStart = millis();

    // Ultrasonic sensor pins
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

    // Relay: always start with the pump OFF.
    pinMode(RELAY_PIN, OUTPUT);
    setPump(false);

    // ADC configuration
    analogReadResolution(12);

    analogSetPinAttenuation(SOIL_SENSOR_1, ADC_11db);
    analogSetPinAttenuation(SOIL_SENSOR_2, ADC_11db);
    analogSetPinAttenuation(SOIL_SENSOR_3, ADC_11db);
    analogSetPinAttenuation(SOIL_SENSOR_4, ADC_11db);

    // Last known settings (used until the cloud can be reached).
    loadSavedSettings();
    manualStartedAt = millis();   // used only if the saved mode is MANUAL

    // Local weather sensor (BMP280). OPTIONAL: if it is not
    // found at 0x76 / 0x77 it is ignored and everything else runs as normal.
    if (detectEnvSensor())
    {
        Serial.print("Local sensor found: BMP280");
        Serial.print(" at 0x");
        Serial.println(envAddr, HEX);
    }
    else
    {
        Serial.println("Local sensor: BMP280 not found (ignored, will keep looking).");
    }

    // First readings, before the pump can be used and before the
    // network task starts (so no zero values are uploaded).
    readSoilSensors();
    readWaterLevel();
    waterLow = (!waterSensorOk || waterLevel <= WATER_LOW_PERCENT);

    // Firebase HTTPS client
    sslClient.setInsecure();
    weatherClient.setInsecure();

    // Wi-Fi + Firebase run in their own task on core 0.
    // They start now so the connection is ready while the
    // start-up message is still showing.
    startWiFi();

    xTaskCreatePinnedToCore(
        networkTask,
        "networkTask",
        20480,
        NULL,
        1,
        NULL,
        0
    );

    // Keep "Welcome!" on the screen for the full 2 seconds.
    while (millis() - welcomeStart < WELCOME_MS)
    {
        delay(10);
    }

    // Start-up screen 2 (5 s): scrolling title + loading %
    showScrollingStartup();

    // Start-up message done: hand the LCD over to the normal pages.
    lcd.clear();
    lcdLast1 = "";
    lcdLast2 = "";
    lcdStatusActive = true;
    lcdPage = 0;
    lastLcdPage = millis();
    updateLcd();

    Serial.println("System startup complete (Wi-Fi connects in background).");
}

// ================================================================
// MAIN LOOP - never waits for the network
// ================================================================

void loop()
{
    unsigned long now = millis();

    // Local BMP280: read every 5 s if found, look for it every 30 s if not
    if (now - lastEnvRead >= (envType != 0 ? ENV_READ_INTERVAL_MS : ENV_RETRY_INTERVAL_MS))
    {
        lastEnvRead = now;
        updateEnvSensor();
    }

    // Weather + local sensor -> rain delay decision (1 s, never uses the network)
    if (now - lastRainCheck >= 1000)
    {
        lastRainCheck = now;
        updateRainDelay();
        enforceRainDelay();
    }

    // Soil sensors + automatic pump control
    if (now - lastSensorRead >= SENSOR_INTERVAL)
    {
        lastSensorRead = now;

        readSoilSensors();
        updateRainConfirm();

        if (systemMode == "AUTO")
        {
            controlPump();
        }
    }

    // Water level + protection (AUTO and MANUAL)
    if (now - lastWaterRead >= WATER_INTERVAL)
    {
        lastWaterRead = now;

        readWaterLevel();
        updateWaterLock();
        enforceWaterProtection();
        updateLcd();
    }

    // Commands from the cloud (mode, threshold, manual pump)
    applyNetworkCommands();

    // MANUAL mode safety timeout (5 minutes)
    checkManualTimeout();

    // Rotate LCD pages
    if (now - lastLcdPage >= LCD_PAGE_INTERVAL)
    {
        lastLcdPage = now;
        lcdPage = (lcdPage + 1) % 5;

        if (lcdPage == 3 && envType == 0)
        {
            lcdPage = 4;   // no local sensor: skip its screen
        }

        updateLcd();
    }
}
