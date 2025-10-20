/******************** BLYNK & WIFI (tùy chọn) ********************/
#define BLYNK_TEMPLATE_ID "TMPL6qg3TLSTn"
#define BLYNK_TEMPLATE_NAME "SmartHome"
#define BLYNK_AUTH_TOKEN "KCmSBRvI1nJEo1MN4elCn-Ab3JwTD6E7"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

/******************** DHT11 ********************/
#include <DHT.h>
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/******************** SERVO ********************/
#include <ESP32Servo.h>   // === ADD

// === Servo (chân 21) ===
const int SERVO_PIN = 21;
const int SERVO_MIN_US = 500;    // chỉnh theo servo nếu cần (500–700)
const int SERVO_MAX_US = 2500;   // (2300–2500)
Servo windowServo;

// Trạng thái báo động để giữ servo mở thêm 1 lúc sau khi hết cảnh báo
bool alarmActive = false;
unsigned long lastAlarmChange = 0;
const unsigned long ALARM_HOLD_MS = 5000;  

// Trạng thái/cài đặt theo người dùng
bool windowManualDesired = false;  // mong muốn từ V15 (0=0°, 1=90°)
int  windowAngleNow = 0;           // góc hiện tại để log

// ====== Quạt ======
const int fanPin = 14;
const int freq = 25000;
const int res = 8;

/******************** GAS & FLAME ********************/
const uint8_t PIN_MQ2_A0 = 35;       // MQ-2: analog
const uint8_t PIN_FLAME_DO = 23;     // Cảm biến lửa
const int GAS_ADC_THRESHOLD = 2000;
const unsigned long NOTIFY_COOLDOWN_MS = 60000;
unsigned long lastGasNotifyAt = 0;
unsigned long lastFlameNotifyAt = 0;

/******************** CHÂN LED RGB ********************/
#define RED_NGU_2 25
#define GREEN_NGU_2 26
#define BLUE_NGU_2 27

/******************** BUZZER ********************/
const uint8_t PIN_BUZZER = 12;

/******************** PHÒNG KHÁCH ********************/
const uint8_t PIN_SWITCH_1 = 16;
const uint8_t LED_KHACH_1 = 4;
// PATCH: tuỳ chọn active-LOW cho relay/LED phòng khách
const bool LED_KHACH_1_ACTIVE_LOW = false; // đổi true nếu phần cứng active-LOW

/******************** PHÒNG NGỦ 1 (RGB) ********************/
const uint8_t PIN_SWITCH_2 = 17;
const uint8_t RED_NGU_1 = 5;
const uint8_t GREEN_NGU_1 = 18;
const uint8_t BLUE_NGU_1 = 19;

/******************** WIFI ********************/
char ssid[] = "honda";
char pass[] = "11111111";

/******************** TRẠNG THÁI ********************/
BlynkTimer timer;

/* Phòng khách */
bool ledState = false;
unsigned long lastDebounceTime1 = 0;
const unsigned long debounceDelay = 30;
int lastStableSwitch1 = HIGH;
int lastReadSwitch1 = HIGH;

/* Phòng ngủ 1 (RGB) */
bool rgbOn = false;
uint8_t redValue = 255, greenValue = 255, blueValue = 255;
unsigned long lastDebounceTime2 = 0;
int lastStableSwitch2 = HIGH;
int lastReadSwitch2 = HIGH;

/******************** LEDC ********************/
const uint32_t LED_FREQ = 5000;
const uint8_t LED_RES = 8;
// RGB1
const int CH_RED = 0;
const int CH_GREEN = 4;
const int CH_BLUE = 8;
// RGB2
const int CH2_RED = 1;
const int CH2_GREEN = 5;
const int CH2_BLUE = 9;
// PATCH: kênh riêng cho quạt
const int CH_FAN = 2;

/******************** LED VỆ SINH ********************/
const uint8_t LED_PHU = 32;
const uint8_t PIN_SWITCH_3 = 33;
bool ledPhuState = false;
unsigned long lastDebounceTime3 = 0;
int lastStableSwitch3 = HIGH;
int lastReadSwitch3 = HIGH;

/******************** LED BẾP ********************/
const uint8_t LED_BEP = 15;
const uint8_t PIN_SWITCH_BEP = 22;
bool ledBepState = false;
unsigned long lastDebounceTimeBep = 0;

int lastStableSwitchBep = HIGH;
int lastReadSwitchBep = HIGH;

// >>> ADD: FAN STATE + debounce (D2)
int fanLevel = 0;                          // 0..3
const unsigned long FAN_DEBOUNCE_MS = 40;
int lastReadFan = LOW, lastStableFan = LOW;
unsigned long lastDebounceFan = 0;

void servoInit() {
  ESP32PWM::allocateTimer(3);           // đảm bảo servo dùng timer 3
  windowServo.setPeriodHertz(50);                        // 50Hz tiêu chuẩn
  windowServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  windowServo.write(0);
  Serial.println("[SERVO] Init @ GPIO21 -> 0 deg (Closed)");
}

// Ghi góc ngay lập tức (0..180)
void setWindowAngle(int angle) {
  if (angle < 0)   angle = 0;
  if (angle > 90) angle = 90;
  windowServo.write(angle);
  windowAngleNow = angle;
}


/******************** HÀM BÁO CHÁY ********************/
uint16_t readADC_Averaged(uint8_t pin, uint8_t samples = 16, uint16_t delayMs = 2) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(delayMs);
  }
  return sum / samples;
}

void readMQ2AndFlame() {
  // 1) Đọc cảm biến
  uint16_t adc = readADC_Averaged(PIN_MQ2_A0);   // Lấy trung bình nhiều mẫu cho MQ-2 (ADC 0..4095)
  bool gasAlarm   = (adc >= GAS_ADC_THRESHOLD);  // Vượt ngưỡng → coi là có gas
  int  flameRaw   = digitalRead(PIN_FLAME_DO);   // DO của cảm biến lửa: thường LOW = có lửa
  bool flameDetected = (flameRaw == LOW);

  // 2) Đẩy dữ liệu lên Blynk (nếu đang online)
  if (Blynk.connected()) {
    Blynk.virtualWrite(V8, adc);                       // Biểu đồ/Value: ADC MQ-2
    Serial.printf("[Blynk] send V8=%d\n", adc);
    Blynk.virtualWrite(V10, gasAlarm ? 1 : 0);         // LED/Indicator: Gas alarm
    Blynk.virtualWrite(V9,  flameDetected ? 1 : 0);    // LED/Indicator: Flame alarm
  }

  // 3) Buzzer: có gas hoặc lửa → kêu
  if (gasAlarm || flameDetected) digitalWrite(PIN_BUZZER, HIGH);
  else                           digitalWrite(PIN_BUZZER, LOW);

  // 4) Gửi logEvent có cooldown
  unsigned long now = millis();
  if (gasAlarm && (now - lastGasNotifyAt > NOTIFY_COOLDOWN_MS)) {
    if (Blynk.connected()) Blynk.logEvent("gas_alert", "Phát hiện khí gas trong nhà!!!");
    lastGasNotifyAt = now;
  }
  if (flameDetected && (now - lastFlameNotifyAt > NOTIFY_COOLDOWN_MS)) {
    if (Blynk.connected()) Blynk.logEvent("flame_alert", "Phát hiện lửa trong nhà!!!");
    lastFlameNotifyAt = now;
  }

  // 5) Điều khiển SERVO cửa sổ:
  //    - Có GAS hoặc LỬA: lập tức đặt 90°
  //    - Hết báo động: chờ thêm ALARM_HOLD_MS rồi trả về theo mong muốn của người dùng (V15)
  bool hasAlarm = gasAlarm || flameDetected;

  if (hasAlarm) {
    if (!alarmActive) {
      alarmActive = true;
      lastAlarmChange = now;
      Serial.println("[SERVO] ALARM ON");
    }
    // Mở cửa sổ ngay để thoát khí/khói
    setWindowAngle(90);                           // <<< QUAY NGAY 90°
    if (Blynk.connected()) Blynk.virtualWrite(V15, 1);  // Đồng bộ công tắc app = MỞ
  } else {
    // Chỉ khi trước đó đang báo động và đã qua thời gian giữ mới trả về manual
    if (alarmActive && (now - lastAlarmChange > ALARM_HOLD_MS)) {
      alarmActive = false;
      lastAlarmChange = now;

      // Trả về theo mong muốn người dùng: 0 -> 0°, 1 -> 90°
      setWindowAngle(windowManualDesired ? 90 : 0);
      if (Blynk.connected()) Blynk.virtualWrite(V15, windowManualDesired ? 1 : 0);

      Serial.printf("[SERVO] ALARM CLEAR -> manual %s\n",
                    windowManualDesired ? "OPEN(90)" : "CLOSE(0)");
    }
  }

  // 6) Log debug tổng hợp
  Serial.printf("[MQ2] ADC=%d gas=%s | [Flame] DO=%d fire=%s | alarm=%d\n",
                adc, gasAlarm ? "YES" : "NO",
                flameRaw, flameDetected ? "YES" : "NO",
                alarmActive ? 1 : 0);
}

BLYNK_WRITE(V15) {
  int v = param.asInt();
  windowManualDesired = (v == 1);

  if (alarmActive) {
    // đang báo động → giữ 90°, ghi đè lại V15=1
    setWindowAngle(90);
    if (Blynk.connected()) Blynk.virtualWrite(V15, 1);
    return;
  }
  setWindowAngle(windowManualDesired ? 90 : 0);
}

/******************** BẾP ********************/
void applyOutputBep(bool on) {
  ledBepState = on;
  digitalWrite(LED_BEP, on ? HIGH : LOW);
  if (Blynk.connected()) Blynk.virtualWrite(V2, on ? 1 : 0);
}
void toggleOutputBep() { applyOutputBep(!ledBepState); }

void readSwitch_Bep() {
  int reading = digitalRead(PIN_SWITCH_BEP);
  if (reading != lastReadSwitchBep) {
    lastDebounceTimeBep = millis();
    lastReadSwitchBep = reading;
  }
  if (millis() - lastDebounceTimeBep > debounceDelay) {
    if (reading != lastStableSwitchBep) {
      lastStableSwitchBep = reading;
      if (reading == LOW) toggleOutputBep();
    }
  }
}

/******************** NHÀ VỆ SINH ********************/
void applyOutputPhu(bool on) {
  ledPhuState = on;
  digitalWrite(LED_PHU, on ? HIGH : LOW);
  if (Blynk.connected()) Blynk.virtualWrite(V1, on ? 1 : 0);
}
void toggleOutputPhu() { applyOutputPhu(!ledPhuState); }

void readSwitch_Phu() {
  int reading = digitalRead(PIN_SWITCH_3);
  if (reading != lastReadSwitch3) {
    lastDebounceTime3 = millis();
    lastReadSwitch3 = reading;
  }
  if (millis() - lastDebounceTime3 > debounceDelay) {
    if (reading != lastStableSwitch3) {
      lastStableSwitch3 = reading;
      if (reading == LOW) toggleOutputPhu();
    }
  }
}

/******************** RGB PHÒNG NGỦ 2 ********************/
inline void writeRGB2(uint8_t r, uint8_t g, uint8_t b) {
  ledcWriteChannel(CH2_RED, r);
  ledcWriteChannel(CH2_GREEN, g);
  ledcWriteChannel(CH2_BLUE, b);
}
void setRGB2(int r, int g, int b) { writeRGB2(r, g, b); }

void setFanColor(int level) {
  switch (level) {
    case 0: setRGB2(0,   0,   0);   break; // off
    case 1: setRGB2(255, 255, 0);   break; // yellow
    case 2: setRGB2(0,   255, 0);   break; // green
    case 3: setRGB2(255, 0,   0);   break; // red
  }
}

// PATCH: quạt dùng kênh riêng (không trùng RGB1)
void setupFanPWM() {
  ledcAttachChannel(fanPin, freq, res, CH_FAN);
  ledcWriteChannel(CH_FAN, 0);
}

// >>> ADD: nhảy nấc & đọc công tắc D2 (INPUT_PULLDOWN, nhấn = HIGH)
void fanNextLevel() {
  int nl = (fanLevel + 1) & 0x03;   // 0..3
  setFanLevel(nl);
  if (Blynk.connected()) Blynk.virtualWrite(V7, nl); // báo về app để đồng bộ
}

void readSwitch_Fan() {
  int reading = digitalRead(2);                     // D2 = GPIO2
  if (reading != lastReadFan) {
    lastDebounceFan = millis();
    lastReadFan = reading;
  }
  if (millis() - lastDebounceFan > FAN_DEBOUNCE_MS) {
    if (reading != lastStableFan) {
      lastStableFan = reading;
      // INPUT_PULLDOWN: nhấn = HIGH (cạnh lên)
      if (reading == HIGH) {
        fanNextLevel();
      }
    }
  }
}

void setFanLevel(int level) {
  if (level < 0) level = 0;
  if (level > 3) level = 3;

  static const int dutyMap[4] = {0, 200, 220, 255};
  int duty = dutyMap[level];

  // Ghi PWM quạt vào kênh riêng
  ledcWriteChannel(CH_FAN, duty);

  fanLevel = level; 
  Serial.printf("Fan level %d -> duty %d\n", level, duty);
  // Đổi màu RGB2 theo cấp quạt
  setFanColor(level);

  Serial.printf("Fan level %d -> duty %d\n", level, duty);
}

BLYNK_WRITE(V7) {
  int level = param.asInt();
  if (level < 0) level = 0;
  if (level > 3) level = 3;
  setFanLevel(level);
}

/******************** DHT ********************/
bool readDHTStable(float &t, float &h) {
  float tt = dht.readTemperature();
  float hh = dht.readHumidity();
  if (isnan(tt) || isnan(hh)) {
    delay(60);
    tt = dht.readTemperature();
    hh = dht.readHumidity();
  }
  if (isnan(tt) || isnan(hh)) return false;
  if (tt < -10 || tt > 80 || hh < 0 || hh > 100) return false;
  t = tt; h = hh;
  return true;
}

void readAndSendDHT() {
  float t, h;
  if (readDHTStable(t, h)) {
    Serial.printf("[DHT] T=%.1f °C H=%.1f %%\n", t, h);
    if (Blynk.connected()) {
      Blynk.virtualWrite(V4, t);
      Blynk.virtualWrite(V5, h);
    }
  } else {
    Serial.println("[DHT] Read failed!");
  }
}

/******************** TIỆN ÍCH ********************/
void applyOutput(bool on) {
  ledState = on;
  int lvl = on
      ? (LED_KHACH_1_ACTIVE_LOW ? LOW  : HIGH)
      : (LED_KHACH_1_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(LED_KHACH_1, lvl);
  if (Blynk.connected()) Blynk.virtualWrite(V0, on ? 1 : 0);
}
void toggleOutput() { applyOutput(!ledState); }

inline void writeRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWriteChannel(CH_RED, r);
  ledcWriteChannel(CH_GREEN, g);
  ledcWriteChannel(CH_BLUE, b);
}
void applyRGB() {
  if (rgbOn) writeRGB(redValue, greenValue, blueValue);
  else writeRGB(0, 0, 0);
  if (Blynk.connected()) Blynk.virtualWrite(V6, rgbOn ? 1 : 0);
}
void setRGBColor(uint8_t r, uint8_t g, uint8_t b) {
  redValue = r; greenValue = g; blueValue = b;
  if (rgbOn) applyRGB();
  if (Blynk.connected()) {
    Blynk.virtualWrite(V3, String(redValue)+","+String(greenValue)+","+String(blueValue));
  }
}
void toggleRGB() { rgbOn = !rgbOn; applyRGB(); }

/******************** CÔNG TẮC ********************/
void readSwitch_Khach() {
  int reading = digitalRead(PIN_SWITCH_1);
  if (reading != lastReadSwitch1) {
    lastDebounceTime1 = millis();
    lastReadSwitch1 = reading;
  }
  if (millis() - lastDebounceTime1 > debounceDelay) {
    if (reading != lastStableSwitch1) {
      lastStableSwitch1 = reading;
      if (reading == LOW) toggleOutput();
    }
  }
}
void readSwitch_Ngu1() {
  int reading = digitalRead(PIN_SWITCH_2);
  if (reading != lastReadSwitch2) {
    lastDebounceTime2 = millis();
    lastReadSwitch2 = reading;
  }
  if (millis() - lastDebounceTime2 > debounceDelay) {
    if (reading != lastStableSwitch2) {
      lastStableSwitch2 = reading;
      if (reading == LOW) toggleRGB();
    }
  }
}

/******************** BLYNK HANDLERS ********************/
BLYNK_WRITE(V0) { applyOutput(param.asInt() == 1); }
BLYNK_WRITE(V1) { applyOutputPhu(param.asInt() == 1); }
BLYNK_WRITE(V2) { applyOutputBep(param.asInt() == 1); }
BLYNK_WRITE(V6) { rgbOn = (param.asInt() == 1); applyRGB(); }
BLYNK_WRITE(V3) {
  if (param.getLength() >= 3) {
    setRGBColor(param[0].asInt(), param[1].asInt(), param[2].asInt());
  } else {
    String s = param.asStr();
    if (s.length()==7 && s[0]=='#') {
      uint8_t r = strtoul(s.substring(1,3).c_str(), NULL, 16);
      uint8_t g = strtoul(s.substring(3,5).c_str(), NULL, 16);
      uint8_t b = strtoul(s.substring(5,7).c_str(), NULL, 16);
      setRGBColor(r,g,b);
    }
  }
}
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V6);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V7); 
  Blynk.syncVirtual(V15);   // đồng bộ công tắc servo
}

/******************** RECONNECT ********************/
void tryReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] reconnect...");
    WiFi.begin(ssid, pass);
    return;
  }
  static bool printedIP = false;
  if (!printedIP) {
    Serial.printf("[WiFi] Connected. IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    printedIP = true;
  }
  if (!Blynk.connected()) {
    Serial.println("[Blynk] trying connect...");
    Blynk.connect(3000); // tăng lên 3s cho ổn định
  }
}

/******************** SETUP ********************/
void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT_PULLDOWN);   // D2 = GPIO2, công tắc kéo lên 3V3 (nhấn = HIGH)

  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_FLAME_DO, INPUT);
  analogSetPinAttenuation(PIN_MQ2_A0, ADC_11db);
  analogReadResolution(12);

  pinMode(LED_BEP, OUTPUT);
  pinMode(PIN_SWITCH_BEP, INPUT_PULLUP);
  applyOutputBep(false);

  pinMode(LED_PHU, OUTPUT);
  pinMode(PIN_SWITCH_3, INPUT_PULLUP);
  applyOutputPhu(false);

  // PATCH: fan PWM kênh riêng
  setupFanPWM();
  // RGB2
  ledcAttachChannel(RED_NGU_2,   LED_FREQ, LED_RES, CH2_RED);
  ledcAttachChannel(GREEN_NGU_2, LED_FREQ, LED_RES, CH2_GREEN);
  ledcAttachChannel(BLUE_NGU_2,  LED_FREQ, LED_RES, CH2_BLUE);
  setFanColor(0); // RGB2 off lúc đầu

  pinMode(LED_KHACH_1, OUTPUT);
  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  applyOutput(false);

  pinMode(PIN_SWITCH_2, INPUT_PULLUP);
  // RGB1
  ledcAttachChannel(RED_NGU_1,   LED_FREQ, LED_RES, CH_RED);
  ledcAttachChannel(GREEN_NGU_1, LED_FREQ, LED_RES, CH_GREEN);
  ledcAttachChannel(BLUE_NGU_1,  LED_FREQ, LED_RES, CH_BLUE);
  setRGBColor(255, 255, 255); rgbOn = false; applyRGB();

  servoInit();

  dht.begin(); delay(1500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.printf("[WiFi] begin %s\n", ssid);

  Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 80);


  timer.setInterval(3000, readAndSendDHT);
  timer.setInterval(10, readSwitch_Khach);
  timer.setInterval(10, readSwitch_Ngu1);
  timer.setInterval(5000, tryReconnect);
  timer.setInterval(10, readSwitch_Phu);
  timer.setInterval(10, readSwitch_Bep);
  timer.setInterval(10, readSwitch_Fan);
  timer.setInterval(1000, readMQ2AndFlame);
}

/******************** LOOP ********************/
void loop() {
  // PATCH: luôn chạy Blynk.run(), thư viện tự xử lý khi offline
  Blynk.run();
  timer.run();
}
