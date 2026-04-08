/*
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║       REAL-TIME ACCIDENT ALERT TRACKING SYSTEM                   ║
 * ║       ESP32 + GPS NEO-7M + GSM SIM900A + GY-61 + LCD I2C        ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 *
 * REQUIRED LIBRARIES (install via Arduino Library Manager):
 *   1. Firebase Arduino Client Library for ESP8266 and ESP32  (Mobizt)
 *   2. TinyGPS++                                              (Mikal Hart)
 *   3. LiquidCrystal I2C                                      (Frank de Brabander)
 *
 * ─── WIRING GUIDE ───────────────────────────────────────────────────
 *
 *  GPS NEO-7M
 *    VCC  → 3.3V
 *    GND  → GND
 *    TX   → GPIO 16  (ESP32 RX2)
 *    RX   → GPIO 17  (ESP32 TX2)
 *
 *  GSM SIM900A  (needs 2A external 4V supply — do NOT use USB 5V)
 *    VCC  → External 4V / 3.7V Li-Ion
 *    GND  → GND (common with ESP32)
 *    TX   → GPIO 26  (ESP32 RX)
 *    RX   → GPIO 27  (ESP32 TX)
 *
 *  GY-61 Accelerometer (ADXL335 analog)
 *    VCC  → 3.3V
 *    GND  → GND
 *    XOUT → GPIO 34  (ADC input only)
 *    YOUT → GPIO 35  (ADC input only)
 *    ZOUT → GPIO 32  (ADC input only)
 *
 *  LCD 16×2 I2C
 *    VCC  → 5V
 *    GND  → GND
 *    SDA  → GPIO 21
 *    SCL  → GPIO 22
 *    (Default I2C address 0x27; change to 0x3F if LCD stays blank)
 *
 *  Onboard LED → GPIO 2  (built-in)
 *
 * ─── FIREBASE DB STRUCTURE ──────────────────────────────────────────
 *
 *  vehicle_001/
 *    lat          : float
 *    lon          : float
 *    speed_kmh    : float
 *    satellites   : int
 *    gps_valid    : bool
 *    timestamp    : int   (Unix seconds, millis()/1000 approximation)
 *    online       : bool
 *    accident     : bool
 *    impact       : float (net m/s² deviation)
 *    jerk         : float (m/s³)
 *    acc_ts       : int   (timestamp of last accident)
 *    accel/
 *      ax : float
 *      ay : float
 *      az : float
 *      net: float
 *      jerk: float
 *
 *  accidents/
 *    <auto_key>/
 *      lat, lon, timestamp, impact, jerk, speed_kmh, source
 */

// ═══════════════════════════════════════════════════════════════════
//  INCLUDES
// ═══════════════════════════════════════════════════════════════════
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>    // Comes with the Firebase library
#include <addons/RTDBHelper.h>     // Comes with the Firebase library
#include <TinyGPS++.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ═══════════════════════════════════════════════════════════════════
//  ▼▼▼  USER CONFIGURATION — EDIT BEFORE UPLOADING  ▼▼▼
// ═══════════════════════════════════════════════════════════════════

// WiFi
#define WIFI_SSID         "hotspot"
#define WIFI_PASSWORD     "myhotspot"

// Firebase — go to Firebase Console → Project Settings → Service Accounts
//            → Database secrets → Show → copy the secret string
#define FIREBASE_HOST     "https://vehicle-accident-tra-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH     "Qp0o8GFxk8DPoDbgTOf5d1wTVm4o1JZbF6v9YylS"

// Device & alert
#define DEVICE_ID         "vehicle_001"          // Must match dashboard DEVICE_ID
#define ALERT_PHONE       "+917868911490"        // SMS recipient (E.164 format)

// ═══════════════════════════════════════════════════════════════════
//  HARDWARE PINS
// ═══════════════════════════════════════════════════════════════════
#define LED_PIN           2

// GPS NEO-7M  — Hardware Serial2
#define GPS_RX            16
#define GPS_TX            17
#define GPS_BAUD          9600

// GSM SIM900A — Hardware Serial1 (custom pins)
#define GSM_RX            26
#define GSM_TX            27
#define GSM_BAUD          9600

// GY-61 Accelerometer (analog, input-only pins)
#define ACCEL_X           34
#define ACCEL_Y           35
#define ACCEL_Z           32

// ═══════════════════════════════════════════════════════════════════
//  TUNABLE PARAMETERS
// ═══════════════════════════════════════════════════════════════════

// Calibration: how many samples to average for idle baseline
#define CAL_SAMPLES       150

// Accident trigger: how far any axis must deviate from idle (ADC counts)
// ESP32 ADC = 12-bit (0-4095), ~3.3V reference
// GY-61 at 3.3V: ~330 mV/g → ~410 ADC counts/g
// 500 counts ≈ 1.2 g — tweak up if too sensitive, down if not sensitive enough
#define ACCEL_THRESHOLD   500

// Cooldown between consecutive accident triggers (ms)
#define ACCIDENT_COOLDOWN 12000UL   // 12 seconds

// Firebase push interval (ms) — 2 s keeps costs low
#define FB_INTERVAL       2000UL

// Heartbeat to mark device online (ms)
#define HB_INTERVAL       6000UL

// LCD backlight stays on (always on in this firmware)
#define LCD_ADDR          0x27     // Change to 0x3F if needed
#define LCD_COLS          16
#define LCD_ROWS          2

// ═══════════════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════════════
HardwareSerial    gpsSerial(2);          // UART2
HardwareSerial    gsmSerial(1);          // UART1

TinyGPSPlus       gps;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

FirebaseData      fbdo;
FirebaseAuth      auth;
FirebaseConfig    config;

// ═══════════════════════════════════════════════════════════════════
//  RUNTIME STATE
// ═══════════════════════════════════════════════════════════════════
// Accelerometer idle baseline (ADC counts)
int    idleX = 2048, idleY = 2048, idleZ = 2048;

// Current GPS data
bool   gpsValid    = false;
float  curLat      = 0.0f, curLon = 0.0f;
float  curSpeed    = 0.0f;
int    curSats     = 0;

// Accident state
bool           accidentActive = false;
bool           smsSentThisCycle = false;
unsigned long  lastAccidentTime = 0;

// Timing
unsigned long  lastFbPush  = 0;
unsigned long  lastHb      = 0;

// Jerk calculation
float  prevNetAccel = 0.0f;
unsigned long prevNetTime = 0;

// LCD cache (avoid flicker from constant rewrites)
String cachedL0 = "@@INIT@@";
String cachedL1 = "@@INIT@@";

// GSM init success flag
bool gsmReady = false;

// Firebase connection flag
bool fbConnected = false;

// ═══════════════════════════════════════════════════════════════════
//  LCD HELPER — only redraws when content changes
// ═══════════════════════════════════════════════════════════════════
void lcdShow(const String& l0, const String& l1) {
  if (l0 == cachedL0 && l1 == cachedL1) return;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(l0.substring(0, LCD_COLS));
  lcd.setCursor(0, 1);
  lcd.print(l1.substring(0, LCD_COLS));
  cachedL0 = l0;
  cachedL1 = l1;
}

// ═══════════════════════════════════════════════════════════════════
//  GSM — AT command helper
// ═══════════════════════════════════════════════════════════════════
String gsmSendAT(const String& cmd, unsigned long waitMs = 1500) {
  while (gsmSerial.available()) gsmSerial.read(); // flush
  gsmSerial.println(cmd);
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < waitMs) {
    while (gsmSerial.available()) {
      char c = (char)gsmSerial.read();
      resp += c;
    }
  }
  Serial.print("[GSM] CMD: " + cmd + "  RESP: ");
  Serial.println(resp);
  return resp;
}

// ═══════════════════════════════════════════════════════════════════
//  GSM — Initialise module
// ═══════════════════════════════════════════════════════════════════
bool gsmInit() {
  lcdShow("GSM Init...", "Please wait");
  Serial.println("[GSM] Initialising SIM900A...");

  // Wait for module to boot
  delay(3000);

  String r = gsmSendAT("AT", 2000);
  if (r.indexOf("OK") == -1) {
    // Try once more (module may need extra time)
    delay(2000);
    r = gsmSendAT("AT", 2000);
    if (r.indexOf("OK") == -1) {
      Serial.println("[GSM] No response — check wiring/power");
      lcdShow("GSM: No Resp!", "Check wiring");
      delay(2000);
      return false;
    }
  }

  gsmSendAT("ATE0",          1000);   // Echo off
  gsmSendAT("AT+CMGF=1",     1000);   // SMS text mode
  gsmSendAT("AT+CNMI=2,2,0,0,0", 1000); // Forward incoming SMS to serial

  Serial.println("[GSM] Ready");
  lcdShow("GSM Ready", "SIM900A OK");
  delay(1000);
  return true;
}

// ═══════════════════════════════════════════════════════════════════
//  GSM — Send SMS accident alert
// ═══════════════════════════════════════════════════════════════════
void sendAccidentSMS(float lat, float lon) {
  Serial.println("[GSM] Sending accident SMS to " + String(ALERT_PHONE));
  lcdShow("Sending Alert!", String(ALERT_PHONE).substring(0, LCD_COLS));

  String mapUrl = "";
  if (lat != 0.0f || lon != 0.0f) {
    mapUrl = "https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6);
  } else {
    mapUrl = "GPS fix unavailable";
  }

  String msg = "ACCIDENT DETECTED!\n"
               "Vehicle: " + String(DEVICE_ID) + "\n"
               "Location: " + mapUrl + "\n"
               "Lat: " + String(lat, 6) + "\n"
               "Lon: " + String(lon, 6) + "\n"
               "Please respond immediately!";

  // Set text mode
  gsmSendAT("AT+CMGF=1", 1000);
  delay(300);

  // Open SMS entry
  gsmSerial.println("AT+CMGS=\"" + String(ALERT_PHONE) + "\"");
  delay(800);

  // Write message body
  gsmSerial.print(msg);
  delay(300);

  // Send with Ctrl+Z (ASCII 26)
  gsmSerial.write(26);
  delay(4000); // Wait for confirmation

  Serial.println("[GSM] SMS dispatched");
  lcdShow("SMS Sent!", "Alert delivered");
  delay(1500);
}

// ═══════════════════════════════════════════════════════════════════
//  ACCELEROMETER — Idle calibration
// ═══════════════════════════════════════════════════════════════════
void calibrateAccelerometer() {
  Serial.println("[ACCEL] Calibrating idle baseline — keep board STILL");
  lcdShow("Calibrating...", "Do not move!");
  delay(1000); // Extra settle time

  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sumX += analogRead(ACCEL_X);
    sumY += analogRead(ACCEL_Y);
    sumZ += analogRead(ACCEL_Z);
    delay(15);
  }
  idleX = (int)(sumX / CAL_SAMPLES);
  idleY = (int)(sumY / CAL_SAMPLES);
  idleZ = (int)(sumZ / CAL_SAMPLES);

  Serial.printf("[ACCEL] Baseline  X=%d  Y=%d  Z=%d\n", idleX, idleY, idleZ);
  lcdShow("Accel Ready!", "X" + String(idleX) + " Y" + String(idleY));
  delay(1500);
}

// ═══════════════════════════════════════════════════════════════════
//  FIREBASE — Push sensor data
// ═══════════════════════════════════════════════════════════════════
void pushToFirebase(float ax, float ay, float az, float net, float jerk, bool accident) {
  if (!Firebase.ready()) return;

  String base = "/" + String(DEVICE_ID);

  // Build main JSON payload
  FirebaseJson data;
  data.set("lat",       curLat);
  data.set("lon",       curLon);
  data.set("speed_kmh", curSpeed);
  data.set("satellites", curSats);
  data.set("gps_valid", gpsValid);
  data.set("timestamp", (int)(millis() / 1000));
  data.set("online",    true);
  data.set("accident",  accident);
  data.set("impact",    net);
  data.set("jerk",      jerk);

  if (accident) {
    data.set("acc_ts", (int)(millis() / 1000));
  }

  // Nested accel sub-object
  FirebaseJson accelObj;
  accelObj.set("ax",   ax);
  accelObj.set("ay",   ay);
  accelObj.set("az",   az);
  accelObj.set("net",  net);
  accelObj.set("jerk", jerk);
  data.set("accel", accelObj);

  // Update main vehicle node
  if (!Firebase.RTDB.updateNode(&fbdo, base.c_str(), &data)) {
    Serial.println("[FB] Update failed: " + fbdo.errorReason());
  }

  // Push to accident log on first trigger
  if (accident && !smsSentThisCycle) {
    FirebaseJson logEntry;
    logEntry.set("lat",       curLat);
    logEntry.set("lon",       curLon);
    logEntry.set("timestamp", (int)(millis() / 1000));
    logEntry.set("impact",    net);
    logEntry.set("jerk",      jerk);
    logEntry.set("speed_kmh", curSpeed);
    logEntry.set("source",    "gsm");
    Firebase.RTDB.pushJSON(&fbdo, "/accidents", &logEntry);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] Accident Tracker starting...");

  // ── LED ──
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ── LCD ──
  Wire.begin(); // SDA=21, SCL=22 by default
  lcd.init();
  lcd.backlight();
  lcdShow("Accident Tracker", "Booting v1.0...");
  delay(1200);

  // ── GPS Serial ──
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] Serial started on UART2");
  lcdShow("GPS NEO-7M", "Initialised");
  delay(800);

  // ── GSM Serial ──
  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(500);
  gsmReady = gsmInit();

  // ── Accelerometer calibration ──
  calibrateAccelerometer();

  // ── WiFi ──
  lcdShow("WiFi Connecting", String(WIFI_SSID).substring(0, LCD_COLS));
  Serial.print("[WIFI] Connecting to " + String(WIFI_SSID));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    // Blink LED while connecting
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  digitalWrite(LED_PIN, LOW);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected! IP: " + WiFi.localIP().toString());
    lcdShow("WiFi Connected", WiFi.localIP().toString());
  } else {
    Serial.println("\n[WIFI] Failed — offline mode");
    lcdShow("WiFi FAILED", "Offline Mode");
  }
  delay(1200);

  // ── Firebase ──
  if (WiFi.status() == WL_CONNECTED) {
    config.database_url    = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;
    config.token_status_callback = tokenStatusCallback; // from TokenHelper.h

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    lcdShow("Firebase Init", "Please wait...");
    Serial.println("[FB] Initialising Firebase...");
    delay(2000);
  }

  lcdShow("System Ready!", "Monitoring...");
  Serial.println("[BOOT] All systems go!");

  // Slow triple-blink = ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }

  prevNetTime = millis();
}

// ═══════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ══ 1. FEED GPS DATA ════════════════════════════════════════════
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    gpsValid = true;
    curLat   = (float)gps.location.lat();
    curLon   = (float)gps.location.lng();
    curSpeed = (float)gps.speed.kmph();
    curSats  = (int)gps.satellites.value();
  } else {
    gpsValid = false;
    if (gps.satellites.isValid()) {
      curSats = (int)gps.satellites.value();
    }
  }

  // ══ 2. READ ACCELEROMETER ═══════════════════════════════════════
  int rawX = analogRead(ACCEL_X);
  int rawY = analogRead(ACCEL_Y);
  int rawZ = analogRead(ACCEL_Z);

  // Deviation from idle baseline (signed ADC counts)
  int dX = rawX - idleX;
  int dY = rawY - idleY;
  int dZ = rawZ - idleZ;

  // Convert to m/s²
  // GY-61 (ADXL335) at 3.3V: sensitivity ≈ 330 mV/g
  // ADC LSB = 3300 mV / 4095 = 0.806 mV → counts per g = 330/0.806 ≈ 410
  const float COUNTS_PER_G = 410.0f;
  const float G_TO_MS2     = 9.81f;

  float ax = (float)dX / COUNTS_PER_G * G_TO_MS2;
  float ay = (float)dY / COUNTS_PER_G * G_TO_MS2;
  float az = (float)dZ / COUNTS_PER_G * G_TO_MS2;

  // Net acceleration magnitude (m/s²)
  float net = sqrtf(ax*ax + ay*ay + az*az);

  // Jerk = rate of change of net acceleration (m/s³)
  float dt   = (now - prevNetTime) / 1000.0f; // seconds
  float jerk = (dt > 0.0f) ? fabsf(net - prevNetAccel) / dt : 0.0f;
  prevNetAccel = net;
  prevNetTime  = now;

  // ══ 3. ACCIDENT DETECTION ═══════════════════════════════════════
  bool axisTriggered = (abs(dX) > ACCEL_THRESHOLD ||
                        abs(dY) > ACCEL_THRESHOLD ||
                        abs(dZ) > ACCEL_THRESHOLD);

  bool cooledDown = (now - lastAccidentTime > ACCIDENT_COOLDOWN);

  if (axisTriggered && cooledDown) {
    // ── NEW ACCIDENT DETECTED ────────────────────────────────────
    accidentActive    = true;
    smsSentThisCycle  = false;
    lastAccidentTime  = now;

    Serial.printf("[ACCIDENT] dX=%d dY=%d dZ=%d  net=%.2f m/s²  jerk=%.2f\n",
                  dX, dY, dZ, net, jerk);

    // LCD — top priority display
    if (gpsValid) {
      lcdShow("!! ACCIDENT !!", String(curLat, 4) + "," + String(curLon, 4));
    } else {
      lcdShow("!! ACCIDENT !!", "No GPS Fix Yet");
    }

    // Fast-blink LED
    for (int i = 0; i < 8; i++) {
      digitalWrite(LED_PIN, HIGH); delay(80);
      digitalWrite(LED_PIN, LOW);  delay(80);
    }

    // Push immediately to Firebase (before SMS delay)
    pushToFirebase(ax, ay, az, net, jerk, true);

    // Send GSM SMS
    if (gsmReady) {
      sendAccidentSMS(curLat, curLon);
      smsSentThisCycle = true;
    } else {
      Serial.println("[GSM] Skipped — module not ready");
    }

  } else if (accidentActive && (now - lastAccidentTime > ACCIDENT_COOLDOWN)) {
    // ── ACCIDENT STATE CLEARED ───────────────────────────────────
    accidentActive   = false;
    smsSentThisCycle = false;
    Serial.println("[ACCIDENT] State cleared");
    lcdShow("System Ready!", "Monitoring...");
  }

  // ══ 4. PERIODIC FIREBASE PUSH ═══════════════════════════════════
  if (WiFi.status() == WL_CONNECTED && (now - lastFbPush >= FB_INTERVAL)) {
    lastFbPush = now;
    pushToFirebase(ax, ay, az, net, jerk, accidentActive);

    // Quick LED pulse on successful push
    digitalWrite(LED_PIN, HIGH); delay(40); digitalWrite(LED_PIN, LOW);
  }

  // ══ 5. HEARTBEAT (keep device "online" in Firebase) ═════════════
  if (WiFi.status() == WL_CONNECTED && (now - lastHb >= HB_INTERVAL)) {
    lastHb = now;
    Firebase.RTDB.setBool(&fbdo, ("/" + String(DEVICE_ID) + "/online").c_str(), true);
  }

  // ══ 6. LCD — NORMAL OPERATING DISPLAY ═══════════════════════════
  if (!accidentActive) {
    String line0, line1;

    if (gpsValid) {
      // Show GPS coords on line 0
      line0 = "L:" + String(curLat, 4) + (curLat >= 0 ? "N" : "S");
      // Show speed + satellites on line 1
      line1 = String(curSpeed, 1) + "km/h S:" + String(curSats);
    } else {
      line0 = "GPS Searching..";
      line1 = "Sats: " + String(curSats) + "  WiFi:" + (WiFi.status() == WL_CONNECTED ? "Y" : "N");
    }

    lcdShow(line0, line1);
  }

  // ══ 7. WIFI RECONNECT ════════════════════════════════════════════
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastReconnect = 0;
    if (now - lastReconnect > 30000UL) {
      lastReconnect = now;
      Serial.println("[WIFI] Reconnecting...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  delay(50); // 20 Hz main loop
}
