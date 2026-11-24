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
// ================================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ================================================================
// WIFI / FIREBASE CONFIGURATION
// ================================================================

#define WIFI_SSID       "YOUR_WIFI_NAME"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

#define DATABASE_URL \
"https://YOUR-PROJECT-default-rtdb.firebaseio.com"

#define DATABASE_AUTH_QUERY ""

const unsigned long WIFI_RETRY_INTERVAL = 15000;   // background retry
const uint16_t FIREBASE_TIMEOUT_MS = 5000;         // per request
const unsigned long FIREBASE_BACKOFF_MS = 5000;    // pause after failure

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
int lcdPage = 0;   // 0 = summary, 1 = soil sensors, 2 = connection

// LCD status screens stay off until the start-up message is finished.
bool lcdStatusActive = false;
String lcdLast1 = "";
String lcdLast2 = "";

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
    int m, t, p;

    portENTER_CRITICAL(&netMux);
    if (!netFresh)
    {
        portEXIT_CRITICAL(&netMux);
        return;
    }
    m = netMode;
    t = netThreshold;
    p = netPump;
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

    StaticJsonDocument<384> doc;
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

    portENTER_CRITICAL(&netMux);
    netMode = m;
    netThreshold = t;
    netPump = p;
    netFresh = true;
    portEXIT_CRITICAL(&netMux);
}

// ================================================================
// NETWORK TASK (runs on core 0, separate from the control loop)
// ================================================================

void networkTask(void *param)
{
    unsigned long lastUpload = 0;
    unsigned long lastFetch = 0;

    for (;;)
    {
        maintainWiFi();

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

    // First readings, before the pump can be used and before the
    // network task starts (so no zero values are uploaded).
    readSoilSensors();
    readWaterLevel();
    waterLow = (!waterSensorOk || waterLevel <= WATER_LOW_PERCENT);

    // Firebase HTTPS client
    sslClient.setInsecure();

    // Wi-Fi + Firebase run in their own task on core 0.
    // They start now so the connection is ready while the
    // start-up message is still showing.
    startWiFi();

    xTaskCreatePinnedToCore(
        networkTask,
        "networkTask",
        12288,
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

    // Soil sensors + automatic pump control
    if (now - lastSensorRead >= SENSOR_INTERVAL)
    {
        lastSensorRead = now;

        readSoilSensors();

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
        lcdPage = (lcdPage + 1) % 3;
        updateLcd();
    }
}
