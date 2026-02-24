//  LIBRARY 
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <Wire.h>
#include "esp_task_wdt.h"
#include "HX711.h"

// WIFI CONFIG 
const char* WIFI_SSID     = "E38";
const char* WIFI_PASSWORD = "E38Gacor1";

//  MQTT CONFIG 
const char* MQTT_SERVER  = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

//  MQTT TOPIC 
const char* TOPIC_V1 = "agv/raenaldiAS/vpin/V1";
const char* TOPIC_V2 = "agv/raenaldiAS/vpin/V2";
const char* TOPIC_V3 = "agv/raenaldiAS/vpin/V3";
const char* TOPIC_V4 = "agv/raenaldiAS/vpin/V4";
const char* TOPIC_V5 = "agv/raenaldiAS/vpin/V5";

//PIN LINE SENSOR TCRT5000
const int LINE_LEFT   = 34;
const int LINE_CENTER = 35;
const int LINE_RIGHT  = 32;

//PIN TB6612FNG #1 (Motor KIRI) 
const int MOTOR_STBY = 33;
const int PWMA_L     = 25;
const int AIN1_L     = 26;
const int AIN2_L     = 27;
const int PWMB_L     = 14;
const int BIN1_L     = 12;
const int BIN2_L     = 13;

// PIN TB6612FNG #2 (Motor KANAN) 
const int PWMA_R     = 15;
const int AIN1_R     = 19;
const int AIN2_R     = 17;
const int PWMB_R     = 16;
const int BIN1_R     = 23;
const int BIN2_R     = 4;

//HX711 
const int   HX711_DOUT         = 2;
const int   HX711_SCK          = 18;
const float CALIBRATION_FACTOR = 1000.0f;
HX711 scale;
float currentWeightKg = 0.0f;

// KONSTAN LOGIKA 
const int   SPEED_STRAIGHT            = 90;
const int   SPEED_TURN                = 50;
const int   SPEED_SEARCH              = 40;
const unsigned long AUTO_STOP_TIMEOUT = 300000UL;
const unsigned long TURN_HOLD_MS      = 250UL; // durasi tahan belok

// BATTERY ESTIMATION 
unsigned long systemStartTime    = 0;
unsigned long totalMotorOnTime   = 0;
unsigned long lastMotorOnTime    = 0;
bool          motorPreviouslyOn  = false;

const unsigned long IDLE_BATTERY_LIFE_MS  = 7200000UL;
const unsigned long MOTOR_BATTERY_LIFE_MS = 3600000UL;

//  GLOBAL OBJECT 
WiFiClient        espClient;
PubSubClient      mqttClient(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences       preferences;

// STATE 
String  agvStatus          = "STOPPED";
bool    startRequested     = false;
bool    lcdBacklightOn     = false;
int     lastBatteryPercent = 100;

enum LastDir { DIR_STRAIGHT, DIR_LEFT, DIR_RIGHT };
LastDir lastDirection = DIR_STRAIGHT;


// MOTOR CONTROL 


void stopMotorPhysical() {
  ledcWrite(PWMA_L, 0);
  ledcWrite(PWMB_L, 0);
  ledcWrite(PWMA_R, 0);
  ledcWrite(PWMB_R, 0);
  digitalWrite(MOTOR_STBY, LOW);
  digitalWrite(AIN1_L, LOW); digitalWrite(AIN2_L, LOW);
  digitalWrite(BIN1_L, LOW); digitalWrite(BIN2_L, LOW);
  digitalWrite(AIN1_R, LOW); digitalWrite(AIN2_R, LOW);
  digitalWrite(BIN1_R, LOW); digitalWrite(BIN2_R, LOW);
}

void setTB6612Channel(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
  } else if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, -speed);
  }
}

void setMotor(int speedLeft, int speedRight) {
  if (speedLeft == 0 && speedRight == 0) {
    stopMotorPhysical();
    return;
  }
  digitalWrite(MOTOR_STBY, HIGH);
  setTB6612Channel(AIN1_L, AIN2_L, PWMA_L, speedLeft);
  setTB6612Channel(BIN1_L, BIN2_L, PWMB_L, speedLeft);
  setTB6612Channel(AIN1_R, AIN2_R, PWMA_R, speedRight);
  setTB6612Channel(BIN1_R, BIN2_R, PWMB_R, speedRight);
}


// GERAK LINE FOLLOWER 


void goStraight()  { lastDirection = DIR_STRAIGHT; setMotor(-SPEED_STRAIGHT, -SPEED_STRAIGHT); }
void turnLeft()    { lastDirection = DIR_LEFT;     setMotor(SPEED_TURN, -SPEED_TURN); }
void turnRight()   { lastDirection = DIR_RIGHT;    setMotor(-SPEED_TURN, SPEED_TURN); }
void searchLeft()  { setMotor(SPEED_SEARCH, -SPEED_SEARCH); }
void searchRight() { setMotor(-SPEED_SEARCH, SPEED_SEARCH); }

void updateMotion() {
  static unsigned long startTime   = 0;
  static unsigned long turnEndTime = 0;
  static bool          isTurning   = false;

  if (!startRequested) {
    stopMotorPhysical();
    startTime = 0;
    isTurning = false;
    return;
  }

  if (startTime == 0) startTime = millis();
  else if (millis() - startTime > AUTO_STOP_TIMEOUT) {
    Serial.println("⚠️ Auto-stop timeout (5 min)");
    startRequested = false;
    agvStatus = "STOPPED";
    stopMotorPhysical();
    mqttClient.publish(TOPIC_V4, agvStatus.c_str(), true);
    return;
  }

  // Kalau sedang dalam mode belok, tunggu selesai dulu
  if (isTurning) {
    if (millis() < turnEndTime) return;
    isTurning = false;
  }

  bool leftOn   = (digitalRead(LINE_LEFT)   == LOW);
  bool centerOn = (digitalRead(LINE_CENTER) == LOW);
  bool rightOn  = (digitalRead(LINE_RIGHT)  == LOW);

  if (centerOn && !leftOn && !rightOn) {
    goStraight();
    agvStatus = "RUNNING";
  }
  else if (leftOn && !rightOn) {
    turnLeft();
    isTurning   = true;
    turnEndTime = millis() + TURN_HOLD_MS;
    agvStatus   = "RUNNING";
  }
  else if (rightOn && !leftOn) {
    turnRight();
    isTurning   = true;
    turnEndTime = millis() + TURN_HOLD_MS;
    agvStatus   = "RUNNING";
  }
  else if (!leftOn && !centerOn && !rightOn) {
    if      (lastDirection == DIR_LEFT)  searchRight();
    else if (lastDirection == DIR_RIGHT) searchLeft();
    else                                 searchLeft();
    agvStatus = "RUNNING";
  }
  else {
    goStraight();
    agvStatus = "RUNNING";
  }
}


//HX711 

void updateWeight() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 500) return;
  lastRead = millis();
  if (!scale.is_ready()) return;
  float w = scale.get_units(3);
  if (w < 0) w = 0;
  currentWeightKg = w;

  char buf[16];
  dtostrf(currentWeightKg, 1, 2, buf);
  mqttClient.publish(TOPIC_V1, buf, true);
}


// LCD 


bool lcdOK = false;

bool initLCD() {
  Wire.begin(21, 22);
  Wire.beginTransmission(0x27);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("❌ LCD not found at 0x27!");
    Wire.beginTransmission(0x27);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("✅ LCD found at 0x3F, UBAH address di kode!");
    } else {
      Serial.println("❌ LCD not found, disable LCD");
      return false;
    }
  } else {
    Serial.println("✅ LCD found at 0x27");
  }
  lcd.init();
  lcd.backlight();
  lcdBacklightOn = true;
  return true;
}

void updateLcd() {
  if (!lcdOK) return;

  static unsigned long lastLcdUpdate = 0;
  if (millis() - lastLcdUpdate < 500) return;
  lastLcdUpdate = millis();

  // Baris 1: Status AGV
  lcd.setCursor(0, 0);
  if (agvStatus == "RUNNING") {
    lcd.print("Status: RUNNING ");
  } else {
    lcd.print("Status: STOPPED ");
  }

  // Baris 2: Berat dari HX711
  lcd.setCursor(0, 1);
  if (currentWeightKg > 0.05f) {
    char buf[8];
    dtostrf(currentWeightKg, 5, 2, buf);
    lcd.print("Beban:");
    lcd.print(buf);
    lcd.print("kg ");
  } else {
    lcd.print("Beban: 0.00 kg  ");
  }
}


// WIFI
void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500); Serial.print("."); retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi Failed!");
    Serial.println("⚠️ Running in OFFLINE mode.");
  }
}


//MQTT 


void publishStatus() {
  mqttClient.publish(TOPIC_V4, agvStatus.c_str(), true);
  Serial.println("[MQTT] Status: " + agvStatus);
}

void stopMotorCommanded() {
  stopMotorPhysical();
  agvStatus = "STOPPED";
  publishStatus();
}

void startMotor() {
  startRequested = true;
  agvStatus = "RUNNING";
  publishStatus();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length == 0 || length > 100) { Serial.println("Invalid payload length"); return; }
  char buffer[101];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';
  String msg = String(buffer);
  msg.trim(); msg.toUpperCase();
  Serial.print("MQTT Received: Topic="); Serial.print(topic);
  Serial.print(" Msg="); Serial.println(msg);

  if (String(topic) == TOPIC_V5) {
    if (msg == "START") {
      Serial.println("→ START command");
      startMotor();
    } else if (msg == "STOP") {
      Serial.println("→ STOP command");
      startRequested = false;
      stopMotorCommanded();
    } else if (msg == "RESET_BATTERY") {
      Serial.println("→ RESET_BATTERY command");
      totalMotorOnTime = 0;
      systemStartTime  = millis();
      preferences.begin("agv-battery", false);
      preferences.putULong("motorTime", 0);
      preferences.putULong("totalTime", 0);
      preferences.end();
      Serial.println("✅ Battery reset to 100%");
    } else {
      Serial.println("→ Unknown command: " + msg);
    }
  }
}

void mqttReconnect() {
  static unsigned long lastAttempt = 0;
  unsigned long now = millis();
  if (now - lastAttempt < 5000) return;
  lastAttempt = now;
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;
  String clientId = "ESP32-AGV-" + String(random(0xffff), HEX);
  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected!");
    mqttClient.subscribe(TOPIC_V5);
    publishStatus();
  } else {
    Serial.print("failed, rc="); Serial.print(mqttClient.state());
    Serial.println(" retry in 5s");
  }
}


// BATTERY 
int estimateBatteryPercent() {
  unsigned long currentTime   = millis();
  unsigned long totalIdleTime = currentTime - systemStartTime - totalMotorOnTime;
  float idleDrain  = (float)totalIdleTime    / IDLE_BATTERY_LIFE_MS  * 100.0f;
  float motorDrain = (float)totalMotorOnTime / MOTOR_BATTERY_LIFE_MS * 100.0f;
  int percent = 100 - (int)(idleDrain + motorDrain);
  if (percent < 0)   percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

void updateMotorRuntime() {
  bool motorCurrentlyOn = startRequested;
  if (motorCurrentlyOn && !motorPreviouslyOn) {
    lastMotorOnTime   = millis();
    motorPreviouslyOn = true;
  } else if (!motorCurrentlyOn && motorPreviouslyOn) {
    totalMotorOnTime += (millis() - lastMotorOnTime);
    motorPreviouslyOn = false;
  }
}

void publishSensors() {
  char buf[16];
  updateMotorRuntime();

  lastBatteryPercent = estimateBatteryPercent();
  snprintf(buf, sizeof(buf), "%d", lastBatteryPercent);
  mqttClient.publish(TOPIC_V3, buf, true);

  Serial.print("Battery: "); Serial.print(lastBatteryPercent);
  Serial.print("% (Idle: ");
  Serial.print((millis() - systemStartTime - totalMotorOnTime) / 60000);
  Serial.print("m, Motor: "); Serial.print(totalMotorOnTime / 60000);
  Serial.println("m)");

  if      (lastBatteryPercent <= 20 && lastBatteryPercent > 10) Serial.println("⚠️ Battery LOW (20%)");
  else if (lastBatteryPercent <= 10)                            Serial.println("🔴 Battery CRITICAL (10%)!");

  publishStatus();
}


// SETUP 


void setup() {
  Serial.begin(115200);
  randomSeed(micros());

  pinMode(LINE_LEFT,   INPUT);
  pinMode(LINE_CENTER, INPUT);
  pinMode(LINE_RIGHT,  INPUT);

  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(AIN1_L, OUTPUT); pinMode(AIN2_L, OUTPUT);
  pinMode(BIN1_L, OUTPUT); pinMode(BIN2_L, OUTPUT);
  pinMode(AIN1_R, OUTPUT); pinMode(AIN2_R, OUTPUT);
  pinMode(BIN1_R, OUTPUT); pinMode(BIN2_R, OUTPUT);

  ledcAttach(PWMA_L, 1000, 8);
  ledcAttach(PWMB_L, 1000, 8);
  ledcAttach(PWMA_R, 1000, 8);
  ledcAttach(PWMB_R, 1000, 8);

  stopMotorPhysical();
  agvStatus = "STOPPED";

  // HX711
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();
  Serial.println("✅ HX711 siap");

  lcdOK = initLCD();
  if (!lcdOK) Serial.println("⚠️ Running without LCD");

  preferences.begin("agv-battery", false);
  totalMotorOnTime = preferences.getULong("motorTime", 0);
  unsigned long savedTotalTime = preferences.getULong("totalTime", 0);
  preferences.end();
  systemStartTime = millis() - savedTotalTime;

  Serial.print("Restored battery state: ");
  Serial.print(estimateBatteryPercent());
  Serial.println("%");

  setupWifi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(15);
  mqttClient.setSocketTimeout(10);
  mqttClient.setCallback(mqttCallback);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = 10000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  Serial.println("✅ Setup selesai!");
  Serial.println("HX711: DOUT=2, SCK=18");
  Serial.println("LCD I2C (0x27): SDA=21, SCL=22");
  Serial.println("TCRT5000: L=34, C=35, R=32");
  Serial.println("MQTT CMD (V5): START | STOP | RESET_BATTERY");
}


void loop() {
  esp_task_wdt_reset();
  mqttReconnect();
  mqttClient.loop();

  updateWeight();
  updateMotion();
  updateLcd();

  static unsigned long lastPublish = 0;
  if (millis() - lastPublish > 1000) {
    lastPublish = millis();
    publishSensors();
  }

  static unsigned long lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 10000) {
    lastHeapCheck = millis();
    Serial.print("Free heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
    if (ESP.getFreeHeap() < 50000) Serial.println("⚠️ LOW MEMORY WARNING!");
  }

  static unsigned long lastSave = 0;
  if (millis() - lastSave > 60000) {
    lastSave = millis();
    preferences.begin("agv-battery", false);
    preferences.putULong("motorTime", totalMotorOnTime);
    preferences.putULong("totalTime", millis() - systemStartTime);
    preferences.end();
  }
}