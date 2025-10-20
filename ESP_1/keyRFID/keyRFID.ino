#define BLYNK_TEMPLATE_ID "TMPL6qg3TLSTn"
#define BLYNK_TEMPLATE_NAME "SmartHome"
#define BLYNK_AUTH_TOKEN "aDZGCZAqqiNBLVKFmusTrz-NrP_2uba0"

// Blynk debug ra Serial
#define BLYNK_PRINT Serial

#include <Wire.h>
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

// ==== BLYNK ====
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ===== WIFI =====
char ssid[] = "honda";
char pass[] = "11111111";

// ==== LCD I2C ====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Keypad I2C (PCF8574) ====
#define I2C_ADDR 0x20
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {4, 5, 6, 7}; // PCF8574 P4-P7
byte colPins[COLS] = {0, 1, 2, 3}; // PCF8574 P0-P3
Keypad_I2C myKeypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2C_ADDR);

// ==== RFID RC522 ====
// Giữ RST ở GPIO4
#define SS_PIN   5
#define RST_PIN  4
#define SCK_PIN  18
#define MISO_PIN 19
#define MOSI_PIN 23
MFRC522 rfid(SS_PIN, RST_PIN);

/***** Rain sensor + Servo *****/
#define RAIN_DO_PIN 26    // cảm biến mưa (digital)
#define SERVO_RAIN  25    // servo che mưa (GPIO25)

// Tham số lọc & chống rung (ms)
const uint16_t SAMPLE_PERIOD_MS = 20;    // chu kỳ lấy mẫu nhanh
const uint16_t WET_CONFIRM_MS   = 800;   // ướt liên tục ≥ 0.8s => "mưa"
const uint16_t DRY_CONFIRM_MS   = 1500;  // khô liên tục ≥ 1.5s => "hết mưa"

// Biến trạng thái mưa
bool rainState = false;          // false = khô, true = đang mưa (đã xác nhận)
uint32_t wetAccumMs = 0;         // tích luỹ thời gian ướt liên tục
uint32_t dryAccumMs = 0;         // tích luỹ thời gian khô liên tục
uint32_t lastSampleAt = 0;
uint32_t lastServoChangeAt = 0;


// ==== Buzzer ====
#define P_BUZZER 13

// ==== Servo (tách 2 đối tượng) ====
#define SERVO_DOOR 15    // servo mở khoá cửa
Servo servoDoor;         // servo cửa
Servo servoRain;         // servo mưa
const uint16_t SERVO_HOLD_MS = 0;

/***** Light sensor + Servo (curtain2) *****/
#define LDR_DO_PIN       33     // DO của cảm biến ánh sáng (theo phần cứng bạn nói)
#define SERVO_LIGHT_PIN  32     // servo rèm theo ánh sáng

Servo servoLight;
bool lightState = false;        // false=thấp/tối, true=sáng (đã xác nhận)
bool curtainAuto = true;   // true = auto theo cảm biến, false = manual (bằng tay)
int curtainPos = 0;        // lưu vị trí hiện tại 0/1

uint32_t lastLightSampleAt = 0;
uint32_t lightHighAccumMs = 0;  // tích lũy thời gian HIGH (sáng)
uint32_t lightLowAccumMs  = 0;  // tích lũy thời gian LOW (tối)
uint32_t lastLightServoChangeAt = 0;

// Tham số lọc & chống rung cho cảm biến ánh sáng
const uint16_t LIGHT_SAMPLE_MS   = 20;   // chu kỳ lấy mẫu
const uint16_t LIGHT_HIGH_MS     = 200;  // HIGH liên tục ≥ 0.2s → “sáng”
const uint16_t LIGHT_LOW_MS      = 400;  // LOW  liên tục ≥ 0.4s → “tối”
const uint16_t LIGHT_SERVO_HOLD  = 800;  // giữ vị trí tối thiểu 0.8s trước khi đổi


// ==== PIR + ĐÈN HIÊN ====
#define PIR_PIN    27    // cảm biến PIR (HC-SR501 OUT)
#define LED_HIEN   14    // LED đèn hiên (qua điện trở 220–330Ω)

bool porchLedOn = false;
unsigned long pirOnTime = 0;
const unsigned long PORCH_DURATION = 5000; // 5 giây

// ==== Mật khẩu keypad ====
String correctPassword = "123A";
String input_pass = "";

// ==== UID RFID hợp lệ ====
const byte validUID[] = {0xF3, 0xC8, 0x7F, 0xF5};
const byte validLen = 4;

bool checkUID(MFRC522::Uid &uid) {
  if (uid.size != validLen) return false;
  for (byte i = 0; i < validLen; i++) {
    if (uid.uidByte[i] != validUID[i]) return false;
  }
  return true;
}

//TIME_OUT
bool timedOut = true;

// ==== Bảo mật ====
int failCount = 0;
bool locked = false;
unsigned long lockStart = 0;
const unsigned long lockDuration = 30000; // 30s

// ==== Timeout hiển thị mặc định ====
unsigned long lastActionTime = 0;
const unsigned long idleTimeout = 10000; // 10s

// ==== Timeout cho còi ====
const unsigned long p_buzzer_duration = 5000;

// === Manual qua Blynk V11 + trạng thái thực tế servo ===
int manualRainPos = 0;   // 0 => 0°, 1 => 90° (mong muốn của bạn)
int effectivePos   = 0;  // 0/1 = vị trí servo thực tế đang ở

// ==== Trạng thái mạng (non-blocking) + hàng đợi thông báo ====
unsigned long lastNetCheck = 0;
const unsigned long netCheckInterval = 500; // 0.5s
bool prevWifi = false;
bool prevBlynk = false;
bool firstBlynkTry = true;
unsigned long lastBlynkTry = 0;

// Hàng đợi gửi bù khi online
bool pendingPirOn = false;
bool pendingPirOff = false;

void applyLightServo(int pos, const char* reason) {
  int angle = pos ? 180 : 0;
  servoLight.write(angle);
  lastLightServoChangeAt = millis();
  curtainPos = pos;
  Serial.printf("[Curtain GPIO32] -> %d° (%s)\n", angle, reason);

  if (Blynk.connected()) {
    Blynk.virtualWrite(V13, curtainPos); // cập nhật nút V13 trên app
  }
}

void updateLightServoAccurate() {

  if (!curtainAuto) return;   // nếu đang manual thì bỏ qua cảm biến

  uint32_t now = millis();
  if (now - lastLightSampleAt < LIGHT_SAMPLE_MS) return;
  lastLightSampleAt = now;

  // Nhiều module LM393: đủ SÁNG -> DO = HIGH
  bool rawHigh = (digitalRead(LDR_DO_PIN) == HIGH);

  if (rawHigh) {
    lightHighAccumMs += LIGHT_SAMPLE_MS;
    lightLowAccumMs = 0;
    if (!lightState &&
        lightHighAccumMs >= LIGHT_HIGH_MS &&
        (now - lastLightServoChangeAt >= LIGHT_SERVO_HOLD)) {
      lightState = true;                // xác nhận SÁNG
      applyLightServo(1, "LIGHT auto HIGH");
    }
  } else {
    lightLowAccumMs += LIGHT_SAMPLE_MS;
    lightHighAccumMs = 0;
    if (lightState &&
        lightLowAccumMs >= LIGHT_LOW_MS &&
        (now - lastLightServoChangeAt >= LIGHT_SERVO_HOLD)) {
      lightState = false;               // xác nhận TỐI
      applyLightServo(0, "LIGHT auto LOW");
    }
  }
}



void applyServo(int pos, const char* reason) {
  int angle = pos ? 90 : 0;
  servoRain.write(angle);
  lastServoChangeAt = millis();
  effectivePos = pos;

  const char* statusText = pos ? "Đang phơi" : "Đã thu vào";

  Serial.printf("[Servo] -> %d° (%s)\n", angle, reason);
  if (Blynk.connected()) {
    // Đồng bộ widget theo TRẠNG THÁI THỰC TẾ
    Blynk.virtualWrite(V11, effectivePos);
    Blynk.virtualWrite(V12, statusText); 
  }
}

// V13: Điều khiển tay (khi đang manual)
BLYNK_WRITE(V13) {
  if (curtainAuto) {
    Serial.println("[V13] Bị bỏ qua vì đang ở AUTO");
    Blynk.virtualWrite(V13, curtainPos); // giữ trạng thái hiện tại
    return;
  }
  int v = param.asInt();  // 0 hoặc 1
  applyLightServo(v, "V13 manual");
}

// V14: Bật/tắt AUTO
BLYNK_WRITE(V14) {
  curtainAuto = (param.asInt() == 1);

  if (curtainAuto) {
    // Vừa bật AUTO -> quyết định NGAY theo cảm biến hiện tại
    bool rawHigh = (digitalRead(LDR_DO_PIN) == HIGH); // LM393: HIGH = sáng
    int target = rawHigh ? 1 : 0;                     // 1 = mở, 0 = đóng
    applyLightServo(target, "AUTO immediate by sensor");
  } else {
    // Vừa tắt AUTO -> vào MANUAL, GIỮ nguyên vị trí đang có
    if (Blynk.connected()) Blynk.virtualWrite(V13, curtainPos);
  }

  if (Blynk.connected()) Blynk.virtualWrite(V14, curtainAuto ? 1 : 0);
}



void showWelcome() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome");
}
void showConnectingWiFiOnce() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Dang ket noi WF");
  delay(3000);
  lcd.setCursor(0,1); lcd.print("(offline OK)");
  delay(3000);
  showWelcome();
}
void showBlynkOKOnce() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Blynk OK");
  lcd.setCursor(0,1); lcd.print("Ready");
  delay(1000);
  showWelcome();
}

void p_buzzer() {
  unsigned long elapsed = millis() - lockStart;
  if (locked && elapsed < p_buzzer_duration) digitalWrite(P_BUZZER, HIGH);
  else digitalWrite(P_BUZZER, LOW);
}

void drawPasswordMasked() {
  lcd.setCursor(0, 0); lcd.print("Nhap mat khau:");
  lcd.setCursor(0, 1);
  size_t L = input_pass.length();
  for (size_t i = 0; i < L && i < 16; i++) lcd.print('*');
  for (size_t i = L; i < 16; i++) lcd.print(' ');
}

void unlock() {
  Serial.println("✅ Mo khoa (Servo 90°)!");
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Mo khoa thanh ");
  lcd.setCursor(0, 1); lcd.print("cong");
  servoDoor.write(90);
  delay(2000);
  servoDoor.write(0);
  failCount = 0;
  lastActionTime = millis();
  showWelcome();
}

BLYNK_WRITE(V11) {
  int v = param.asInt();
  Serial.printf("[V11] got=%d  rainState=%d  dt=%lu ms\n",
                v, rainState, millis() - lastServoChangeAt);   // 0 hoặc 1
  manualRainPos = v;       // lưu mong muốn
  if (!rainState) {
    applyServo(manualRainPos, "V11 manual");
  } else {
    // Đang mưa -> vẫn ép 90°, chỉ đồng bộ UI cho rõ
    Serial.println("[V11] Nhan manual nhung DANG MUA -> override 90°");
    if (Blynk.connected()) Blynk.virtualWrite(V11, 1);
  }
}

BLYNK_CONNECTED() {
  // Lấy giá trị đã lưu, rồi đẩy trạng thái THỰC TẾ để UI khớp
  Blynk.syncVirtual(V11);
  Blynk.virtualWrite(V11, effectivePos);
  Blynk.virtualWrite(V12, effectivePos ? "Đang phơi" : "Đã thu vào");
  Blynk.virtualWrite(V14, curtainAuto ? 1 : 0);
  Blynk.virtualWrite(V13, curtainPos);

}


void updateNetwork() {
  unsigned long now = millis();
  if (now - lastNetCheck < netCheckInterval) return;
  lastNetCheck = now;

  bool wifi = (WiFi.status() == WL_CONNECTED);

  // Kết nối Blynk: lần đầu 5s, sau đó 1s mỗi 5s (không chặn dài)
  if (wifi && !Blynk.connected()) {
    if (firstBlynkTry) {
      Serial.println("[NET] First connect Blynk 5s");
      Blynk.connect(5000);
      firstBlynkTry = false;
      lastBlynkTry = now;
    } else if (now - lastBlynkTry >= 5000) {
      Serial.println("[NET] Retry Blynk 1s");
      Blynk.connect(1000);
      lastBlynkTry = now;
    }
  }

  bool bk = Blynk.connected();

  if (wifi != prevWifi) {
    if (wifi) {
      Serial.println("[NET] WiFi connected");
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("WiFi OK");
      lcd.setCursor(0,1); lcd.print("Ket noi Blynk...");
      delay(4000);
    } else {
      Serial.println("[NET] WiFi disconnected -> offline");
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("WiFi ERROR!");
      lcd.setCursor(0,1); lcd.print("Offline mode");
      delay(4000);
      showWelcome();
    }
    prevWifi = wifi;
  }

  if (bk != prevBlynk) {
    if (bk) {
      showBlynkOKOnce();
    } 
    prevBlynk = bk;
  }
}

// Đọc & lọc cảm biến mưa, điều khiển servo với hysteresis thời gian
void updateRainServoAccurate() {
  uint32_t now = millis();
  if (now - lastSampleAt < SAMPLE_PERIOD_MS) return;
  lastSampleAt = now;

  bool rawWet = (digitalRead(RAIN_DO_PIN) == LOW); // LM393: ướt -> LOW

  if (rawWet) {
    wetAccumMs += SAMPLE_PERIOD_MS;
    dryAccumMs  = 0;
    if (!rainState && wetAccumMs >= WET_CONFIRM_MS && (now - lastServoChangeAt >= SERVO_HOLD_MS)) {
      rainState = true;
      // MƯA: ép 90°
      applyServo(1, "RAIN auto");
      Serial.println("🌧️ CO MUA -> override 90°");
    }
  } else {
    dryAccumMs += SAMPLE_PERIOD_MS;
    wetAccumMs  = 0;
    if (rainState && dryAccumMs >= DRY_CONFIRM_MS && (now - lastServoChangeAt >= SERVO_HOLD_MS)) {
      rainState = false;
      // HẾT MƯA: trả về manual V11
      applyServo(manualRainPos, "DRY restore manual");
      Serial.printf("☀️ HET MUA -> ve %d° theo V11\n", manualRainPos ? 90 : 0);
    }
  }
}


void setup() {
  Serial.begin(115200);

  // Rain sensor + servo mưa
  pinMode(RAIN_DO_PIN, INPUT);            // nếu module cần kéo lên thì dùng INPUT_PULLUP
  servoRain.attach(SERVO_RAIN, 500, 2400);
  servoRain.write(0);
  lastServoChangeAt = millis();

  // Buzzer
  pinMode(P_BUZZER, OUTPUT);
  digitalWrite(P_BUZZER, LOW);

    // NEW: Light sensor + servo ở GPIO32
  pinMode(LDR_DO_PIN, INPUT);                     // đa số module đã có pull-up trên board
  servoLight.attach(SERVO_LIGHT_PIN, 500, 2400);  // biên xung 500–2400us
  servoLight.write(0);                            // mặc định 0°
  lastLightServoChangeAt = millis();

  // I2C / LCD
  Wire.begin(21, 22);
  Wire.setClock(100000);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Khoi dong...");
  delay(1000);

  // Keypad
  myKeypad.begin();

  // RFID
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  rfid.PCD_Init();
  delay(50);
  rfid.PCD_DumpVersionToSerial();

  // Servo cửa
  servoDoor.attach(SERVO_DOOR, 500, 2400);
  servoDoor.write(0);

  // PIR + LED
  pinMode(PIR_PIN, INPUT);        // HC-SR501: OUT HIGH khi phát hiện
  pinMode(LED_HIEN, OUTPUT);
  digitalWrite(LED_HIEN, LOW);

  // WiFi + Blynk non-blocking
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN); // connect trong updateNetwork()

  showConnectingWiFiOnce();
  lastActionTime = millis();
}


void loop() {
  // Mạng & Blynk
  updateNetwork();
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.run();
  }

  // ===== PIR: bật LED 5s + gửi thông báo =====
  int pirVal = digitalRead(PIR_PIN);
  if (!porchLedOn && pirVal == HIGH) {
    porchLedOn = true;
    pirOnTime = millis();
    digitalWrite(LED_HIEN, HIGH);

    Serial.println("PIR: Phat hien -> bat den hien 5s");
    if (Blynk.connected()) {
      Blynk.logEvent("pir_noti", "Phat hien vat the, den hien da bat ");
    } else {
      pendingPirOn = true; // hàng đợi nếu đang offline
    }
  }

  if (porchLedOn && (millis() - pirOnTime >= PORCH_DURATION)) {
    digitalWrite(LED_HIEN, LOW);
    porchLedOn = false;
    Serial.println("PIR: Het 5s -> tat den hien");
    if (Blynk.connected()) {
      Blynk.logEvent("pir_noti", "den hien da tat");
    } else {
      pendingPirOff = true; // hàng đợi nếu đang offline
    }
  }

  // ===== Khóa tạm nếu quá nhiều sai =====
  if (locked) {
    unsigned long elapsed = millis() - lockStart;
    if (elapsed >= lockDuration) {
      locked = false;
      failCount = 0;
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("System unlocked");
      Serial.println("🔓 Het khoa, co the nhap lai.");
      delay(700);
      showWelcome();
    } else {
      p_buzzer();
      int remain = (lockDuration - elapsed) / 1000;
      lcd.setCursor(0, 0); lcd.print("System locked   ");
      lcd.setCursor(0, 1); lcd.print("Wait ");
      lcd.print(remain); lcd.print("s     ");
      delay(200);
    }
    return;
  }

  // ===== Keypad =====
  char key = myKeypad.getKey();
  if (key) {
    timedOut = false;
    lastActionTime = millis();
    if (key == '#') {
      Serial.print("Mat khau da nhap: ");
      Serial.println(input_pass);

      if (input_pass == correctPassword) {
        Serial.println("✅ Mat khau dung!");
        unlock();
      } else {
        Serial.println("❌ Sai mat khau!");
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Sai mat khau!");
        delay(1000);
        lcd.setCursor(0, 0); lcd.print("Vui long nhap   ");
        lcd.setCursor(0, 1); lcd.print("lai mat khau!   ");\
        delay(1000);
        failCount++;
      }
      input_pass = "";
    }
    else if (key == '*') {
      input_pass = "";
      Serial.println("Da xoa. Nhap lai...");
      lcd.setCursor(0, 0); lcd.print("Nhap lai...      ");
      lcd.setCursor(0, 1); lcd.print("                ");
      delay(1000);
    }
    else {
      input_pass += key;
      Serial.print("Dang nhap: ");
      for (size_t i = 0; i < input_pass.length(); i++) Serial.print('*');
      Serial.println();
      drawPasswordMasked();
    }
  }

  // ===== RFID =====
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    lastActionTime = millis();
    timedOut = false;
    Serial.print("UID the: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) Serial.print("0");
      Serial.print(rfid.uid.uidByte[i], HEX);
      if (i < rfid.uid.size - 1) Serial.print(" ");
    }
    Serial.println();

    if (checkUID(rfid.uid)) {
      Serial.println("✅ UID hop le!");
      unlock();
    } else {
      Serial.println("❌ UID khong hop le!");
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("UID sai!");
      delay(1000);
      lcd.setCursor(0, 0); lcd.print("Vui long quet    ");
      lcd.setCursor(0, 1); lcd.print("lai the!         ");
      delay(1000);
      failCount++;
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }

  // Nếu sai quá nhiều → khóa
  if (failCount >= 5) {
    locked = true;
    lockStart = millis();
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("System locked!");
    Serial.println("🚨 Sai qua 5 lan -> khoa 30s!");
    if (Blynk.connected()) {
      Blynk.logEvent("wrong_pass", "Sai qua 5 lan, he thong da bi khoa!");
    }
  }

  // ===== Timeout 10s (Keypad + RFID) – hiện thông báo 1.5s rồi về Welcome =====
  static bool timeoutShowing = false;
  static unsigned long timeoutShownAt = 0;

  if (!locked) {
    unsigned long now = millis();

    // Kích hoạt đúng 1 lần khi quá idleTimeout và CHƯA “chốt” timeout
    if (!timedOut && !timeoutShowing && (now - lastActionTime > idleTimeout)) {
      input_pass = "";
      if (failCount > 0) failCount = 0;

      Serial.println("Timeout -> clear input & reset failCount");
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("Het thoi gian");
      lcd.setCursor(0,1); lcd.print("Nhap lai tu dau");
      delay(1000);

      timeoutShowing = true;
      timeoutShownAt = now;
      timedOut = true;               //
    }

    // Giữ thông báo 1.5s rồi về màn hình Welcome
    if (timeoutShowing && (now - timeoutShownAt > 1500)) {
      showWelcome();
      timeoutShowing = false;        // đã tắt thông báo, nhưng timedOut vẫn TRUE
    }
  }

  // Cập nhật servo mưa (lọc thời gian)
  updateRainServoAccurate();

  updateLightServoAccurate();

  delay(10);
}
