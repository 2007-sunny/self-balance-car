// ================================================================
//  雙輪自平衡小車 v2 — 含 LCD 操作面板、旋轉編碼器、單層 PID
//
//  硬體：Arduino Nano + MPU6050 + LCD 1602 I2C + A4988×2
//        + HC-05 藍牙 + 旋轉編碼器 (EC11)
//
//  架構：
//    • 互補濾波姿態解算
//    • 單層 PID（角度控制）
//    • Timer1 CTC + 梯形加減速（防失步）
//    • EEPROM 魔術數字保護（防垃圾值）
//    • LCD 狀態機選單
//    • 藍牙指令僅在平衡模式下啟用
//
//  作者保留區：請依實際車體調整 base_angle 與 dir_sign
// ================================================================

// ────────────────────────────────────────────────────────────────
//  函式庫
// ────────────────────────────────────────────────────────────────
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// ────────────────────────────────────────────────────────────────
//  區塊一：腳位定義
// ────────────────────────────────────────────────────────────────
// 馬達
#define STEP_A 2   // 左馬達 STEP（PD2，Timer1 ISR 內直接操作暫存器）
#define DIR_A 3    // 左馬達 DIR
#define STEP_B 4   // 右馬達 STEP（PD4）
#define DIR_B 5    // 右馬達 DIR（程式自動取反補償對稱安裝）
#define MOTOR_EN 6 // A4988 ENABLE 共用（LOW=啟動，HIGH=斷電）

// 旋轉編碼器（當數位腳使用的類比腳）
#define ENC_CLK A0
#define ENC_DT A1
#define ENC_SW A2

// 藍牙 HC-05
#define BT_RX 10 // Nano 接收 ← HC-05 TXD
#define BT_TX 11 // Nano 發送 → HC-05 RXD（需接分壓）

// ────────────────────────────────────────────────────────────────
//  區塊二：系統常數
// ────────────────────────────────────────────────────────────────
// EEPROM
#define EEPROM_MAGIC_0 0xAC // 改為 0xAC 強制重置 EEPROM
#define EEPROM_MAGIC_1 0xCE
#define EEPROM_ADDR_MAGIC 0 // 2 bytes
#define EEPROM_ADDR_KP 2    // float 4 bytes
#define EEPROM_ADDR_KI 6
#define EEPROM_ADDR_KD 10
#define EEPROM_ADDR_BASE 18
#define EEPROM_ADDR_DIR 22       // 1 byte (值為 1 或 255 代表 -1)
#define EEPROM_ADDR_CALIB 23     // 1 byte (0xAB = 已校正)
#define EEPROM_ADDR_MAX_SPEED 24 // float 4 bytes

// 物理常數
#define MPU_ADDR 0x68
#define LCD_ADDR 0x27
#define GYRO_SCALE (1.0f / 131.0f)  // ±250 dps
#define ACC_SCALE (1.0f / 16384.0f) // ±2g
#define RAD2DEG 57.2957795f
#define CF_ALPHA 0.98f // 互補濾波係數
#define LOOP_DT 0.005f    // 200Hz = 5ms
#define LOOP_DT_US 5000UL // 200Hz = 5ms

// 安全與限制
#define FALL_ANGLE 45.0f        // 跌倒判定（度）
#define CALIB_TILT_THRESH 15.0f // 傾倒校正觸發閾值（度）
#define MAX_ANGLE_CMD 8.0f      // 外環輸出目標角度上限（度）

// 梯形加減速
// 每個 5ms 週期，允許速度改變的最大量（steps/s per loop）
#define MAX_ACCEL_STEP 25.0f

// Timer1：20 kHz 基底頻率（Prescaler=1，OCR1A=799）
// 步進脈衝頻率 = 20000 / period_ticks
#define TIMER1_TOP 799

// 藍牙 LCD 提示持續時間
#define BT_MSG_DURATION 2000UL // ms
#define TELEMETRY_INTERVAL_MS 50UL // 20Hz，適合 HC-05 9600 baud

// PID 外環降頻（每 N 次內環執行一次外環）
#define OUTER_DIV 4 // 200Hz / 4 = 50Hz

// 預設 PID 值（EEPROM 首次寫入）
// 針對步進馬達 (0~10000 steps/s)，參數需放大
#define DEF_KP 30.0f
#define DEF_KI 1.0f
#define DEF_KD 3.0f
#define DEF_BASE 0.0f
#define DEF_MAX_SPEED 1050.0f

// ────────────────────────────────────────────────────────────────
//  區塊三：狀態機枚舉
// ────────────────────────────────────────────────────────────────
enum SystemState {
  STATE_MENU,
  STATE_BALANCE,
  STATE_MPU_MENU,
  STATE_MPU_ANGLE,
  STATE_MPU_BASE,
  STATE_MPU_CALIB,
  STATE_BT_MENU,
  STATE_BT_PING,
  STATE_BT_MONITOR,
  STATE_MOTOR_TEST,
  STATE_LINEAR_TEST,
  STATE_PID_MENU,
  STATE_PID_EDIT,
  STATE_MAX_SPEED_EDIT
};

// ────────────────────────────────────────────────────────────────
//  區塊四：全域物件與變數
// ────────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
SoftwareSerial BTSerial(BT_RX, BT_TX);

// --- 狀態機 ---
SystemState sysState = STATE_MENU;
SystemState prevState = STATE_MENU;
int menuIndex = 0;        // 主選單游標
int subIndex = 0;         // 子選單游標
bool stateChanged = true; // 狀態剛切換旗標（觸發重繪）

// --- IMU ---
float currentAngle = 0.0f;  // 互補濾波後傾角
float gyroRate = 0.0f;      // 陀螺儀角速度（°/s）
float accelAngles[3] = {0}; // Ax, Ay, Az 三軸加速度計算角度（診斷用）
int8_t dir_sign = 1;        // 方向符號（傾倒校正結果）

// --- 控制 ---
float base_angle = DEF_BASE;   // 平衡補償角度
float motorSpeedCmd = 0.0f;    // PID 輸出速度指令（浮點）
float motorSpeedActual = 0.0f; // 梯形加減速後的實際速度（開迴路估算）
float angleIntegral = 0.0f;    // 角度積分
float prevAngleErr = 0.0f;
float moveOffset = 0.0f; // 藍牙遙控角度偏移
bool isFallen = false;

// PID 參數與限制（由 EEPROM 載入）
float Kp, Ki, Kd;
float max_speed;

// --- Timer1 ---
volatile int periodA = 0; // 0 = 停止
volatile int periodB = 0;
volatile int counterA = 0;
volatile int counterB = 0;

// --- 旋轉編碼器 ---
int encLastCLK = HIGH;
int encDelta = 0; // +1 / -1 / 0
bool encPressed = false;
bool encLongPress = false;
unsigned long encPressStart = 0;
#define LONG_PRESS_MS 800UL
#define DEBOUNCE_MS 5UL
unsigned long lastDebounce = 0;

// --- 藍牙 ---
String btBuffer = "";
String btLastMsg = "";        // 最後一筆收到的訊息（回覆模式用）
unsigned long btMsgTimer = 0; // 藍牙提示倒數（ms，0=不顯示）
String btOverlayMsg = "";     // 要顯示的提示字串
bool telemetryEnabled = false;
unsigned long telemetryLastMs = 0;

// --- LCD 更新節流 ---
unsigned long lcdLastUpdate = 0;
#define LCD_UPDATE_INTERVAL 150UL // ms（避免 I2C 過於頻繁）
bool lcdNeedsRecover = false;
unsigned long lcdLastRecover = 0;
#define LCD_RECOVER_INTERVAL 10000UL // ms（非平衡模式定期重置 LCD 狀態）
#define I2C_TIMEOUT_US 3000UL        // 避免 LCD/MPU6050 拉住 I2C 造成主程式卡死

// --- 控制迴圈計時 ---
unsigned long lastLoopUs = 0;

// ────────────────────────────────────────────────────────────────
//  區塊五：EEPROM 管理
// ────────────────────────────────────────────────────────────────
void eepromWriteFloat(int addr, float val) { EEPROM.put(addr, val); }

float eepromReadFloat(int addr) {
  float val;
  EEPROM.get(addr, val);
  return val;
}

void eepromWriteDefaults() {
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_0);
  EEPROM.write(EEPROM_ADDR_MAGIC + 1, EEPROM_MAGIC_1);
  eepromWriteFloat(EEPROM_ADDR_KP, DEF_KP);
  eepromWriteFloat(EEPROM_ADDR_KI, DEF_KI);
  eepromWriteFloat(EEPROM_ADDR_KD, DEF_KD);
  eepromWriteFloat(EEPROM_ADDR_BASE, DEF_BASE);
  eepromWriteFloat(EEPROM_ADDR_MAX_SPEED, DEF_MAX_SPEED);
  EEPROM.write(EEPROM_ADDR_DIR, 1);
  EEPROM.write(EEPROM_ADDR_CALIB, 0x00);
}

void eepromLoad() {
  // 魔術數字驗證
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC_0 ||
      EEPROM.read(EEPROM_ADDR_MAGIC + 1) != EEPROM_MAGIC_1) {
    // 首次開機或 EEPROM 損壞 → 寫入預設值
    eepromWriteDefaults();
  }
  Kp = eepromReadFloat(EEPROM_ADDR_KP);
  Ki = eepromReadFloat(EEPROM_ADDR_KI);
  Kd = eepromReadFloat(EEPROM_ADDR_KD);
  base_angle = eepromReadFloat(EEPROM_ADDR_BASE);
  max_speed = eepromReadFloat(EEPROM_ADDR_MAX_SPEED);
  if (isnan(max_speed) || max_speed < 100.0f || max_speed > 10000.0f) {
    max_speed = DEF_MAX_SPEED; // 針對舊版 EEPROM 防呆
  }
  uint8_t ds = EEPROM.read(EEPROM_ADDR_DIR);
  dir_sign = (ds == 255) ? -1 : 1;
}

void eepromSavePID() {
  eepromWriteFloat(EEPROM_ADDR_KP, Kp);
  eepromWriteFloat(EEPROM_ADDR_KI, Ki);
  eepromWriteFloat(EEPROM_ADDR_KD, Kd);
}

void eepromSaveBase() { eepromWriteFloat(EEPROM_ADDR_BASE, base_angle); }

void eepromSaveMaxSpeed() {
  eepromWriteFloat(EEPROM_ADDR_MAX_SPEED, max_speed);
}

void eepromSaveCalib(int8_t sign) {
  EEPROM.write(EEPROM_ADDR_DIR, (sign == -1) ? 255 : 1);
  EEPROM.write(EEPROM_ADDR_CALIB, 0xAB);
}

// ────────────────────────────────────────────────────────────────
//  區塊六：MPU6050
// ────────────────────────────────────────────────────────────────
void i2cInit() {
  Wire.begin();
  Wire.setClock(100000); // 降為 100kHz 以避免平價 LCD 模組當機/亂碼
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
#endif
}

bool i2cTimedOut() {
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    lcdNeedsRecover = true;
    return true;
  }
#endif
  return false;
}

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); // 喚醒
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03); // DLPF 44Hz
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00); // 陀螺儀 ±250 dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00); // 加速度計 ±2g
  Wire.endTransmission();
}

// 讀取並執行互補濾波
// 同時計算三軸診斷角度（accelAngles[]）
void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (i2cTimedOut() || Wire.available() < 14) {
    lcdNeedsRecover = true;
    return;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read(); // 溫度略過
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read(); // Gy 略過
  Wire.read();
  Wire.read(); // Gz 略過

  float ax = rawAx * ACC_SCALE;
  float ay = rawAy * ACC_SCALE;
  float az = rawAz * ACC_SCALE;
  gyroRate = rawGx * GYRO_SCALE * dir_sign;

  // 三軸加速度計角度（診斷視窗用）
  // Ax：以 Y-Z 平面為參考的俯仰角
  accelAngles[0] = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
  // Ay：以 X-Z 平面為參考的橫滾角
  accelAngles[1] = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD2DEG;
  // Az：以 X-Y 平面為參考（偏航，加速度計無法準確量測，僅供參考）
  accelAngles[2] = atan2f(sqrtf(ax * ax + ay * ay), az) * RAD2DEG;

  // 主傾角：車子前後傾（以 atan2(ax, az) 為基底，依實際安裝調整）
  float accAngle = atan2f(ax * dir_sign, az) * RAD2DEG;

  // 互補濾波
  currentAngle = CF_ALPHA * (currentAngle + gyroRate * LOOP_DT) +
                 (1.0f - CF_ALPHA) * accAngle;
}

// 只讀加速度計角度（初始化用，不更新濾波器）
float mpuReadAccAngle() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 6, true);
  if (i2cTimedOut() || Wire.available() < 6) {
    lcdNeedsRecover = true;
    return currentAngle;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  float ax = rawAx * ACC_SCALE;
  (void)rawAy;
  float az = rawAz * ACC_SCALE;
  return atan2f(ax * dir_sign, az) * RAD2DEG;
}

// ────────────────────────────────────────────────────────────────
//  區塊七：Timer1 — CTC 模式，20kHz 基底，ISR 產生 STEP 脈衝
// ────────────────────────────────────────────────────────────────
void timer1Init() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = TIMER1_TOP;      // 16MHz / 1 / 800 = 20kHz
  TCCR1B |= (1 << WGM12);  // CTC 模式
  TCCR1B |= (1 << CS10);   // Prescaler = 1
  TIMSK1 |= (1 << OCIE1A); // 開啟比較中斷
  interrupts();
}

// 速度（steps/s）→ 定時器 period_ticks
// period_ticks = 20000 / speed_steps_per_sec
// 限制範圍 [2, 5000]，0 = 停止
inline int speedToTicks(float speed) {
  if (fabsf(speed) < 2.0f)
    return 0;
  int t = (int)(20000.0f / fabsf(speed));
  return constrain(t, 2, 5000);
}

ISR(TIMER1_COMPA_vect) {
  bool pulseA = false;
  bool pulseB = false;

  // 馬達 A（D2 = PD2）
  if (periodA > 0) {
    if (++counterA >= periodA) {
      counterA = 0;
      pulseA = true;
    }
  }
  // 馬達 B（D4 = PD4）
  if (periodB > 0) {
    if (++counterB >= periodB) {
      counterB = 0;
      pulseB = true;
    }
  }

  if (pulseA)
    PORTD |= (1 << PD2);
  if (pulseB)
    PORTD |= (1 << PD4);
  if (pulseA || pulseB) {
    delayMicroseconds(2); // A4988 STEP high time needs >= 1us.
    if (pulseA)
      PORTD &= ~(1 << PD2);
    if (pulseB)
      PORTD &= ~(1 << PD4);
  }
}

// 設定馬達速度（含梯形加減速）
// 呼叫於主迴圈，每 5ms 一次
void motorSetSpeed(float target) {
  // 梯形加減速：每次最多改變 MAX_ACCEL_STEP
  if (motorSpeedActual < target) {
    motorSpeedActual = min(motorSpeedActual + MAX_ACCEL_STEP, target);
  } else if (motorSpeedActual > target) {
    motorSpeedActual = max(motorSpeedActual - MAX_ACCEL_STEP, target);
  }

  float spd = motorSpeedActual;

  if (fabsf(spd) < 2.0f || isFallen) {
    noInterrupts();
    periodA = 0;
    periodB = 0;
    interrupts();
    return;
  }

  bool fwd = (spd > 0);
  // 對稱安裝補償：反轉馬達方向
  digitalWrite(DIR_A, fwd ? LOW : HIGH);
  digitalWrite(DIR_B, fwd ? HIGH : LOW);

  int ticks = speedToTicks(spd);
  noInterrupts();
  periodA = ticks;
  periodB = ticks;
  interrupts();
}

void motorStop() {
  noInterrupts();
  periodA = 0;
  periodB = 0;
  interrupts();
  motorSpeedActual = 0;
  motorSpeedCmd = 0;
}

void motorEmergencyStop() {
  motorStop();
  angleIntegral = 0;
  moveOffset = 0;
  prevAngleErr = 0;
  digitalWrite(MOTOR_EN, HIGH);
}

// ────────────────────────────────────────────────────────────────
//  區塊八：旋轉編碼器
// ────────────────────────────────────────────────────────────────
void encoderUpdate() {
  encDelta = 0;
  encPressed = false;
  encLongPress = false;

  // 旋轉方向偵測（CLK 下降緣觸發）
  int clk = digitalRead(ENC_CLK);
  if (clk == LOW && encLastCLK == HIGH) {
    if (digitalRead(ENC_DT) == HIGH) {
      encDelta = +1; // 順時針
    } else {
      encDelta = -1; // 逆時針
    }
  }
  encLastCLK = clk;

  // 按下偵測（含防彈跳）
  int sw = digitalRead(ENC_SW);
  unsigned long now = millis();
  if (sw == LOW) {
    if (encPressStart == 0)
      encPressStart = now;
    if ((now - encPressStart) >= LONG_PRESS_MS) {
      encLongPress = true;
    }
  } else {
    if (encPressStart > 0) {
      if ((now - encPressStart) >= DEBOUNCE_MS &&
          (now - encPressStart) < LONG_PRESS_MS) {
        encPressed = true; // 短按
      }
      encPressStart = 0;
    }
  }
}

// ────────────────────────────────────────────────────────────────
//  區塊九：藍牙指令解析
//  格式：
//    AP=15.0\n → Kp_in      BT 調整後 LCD 顯示 2 秒提示
//    AD=0.6\n  → Kd_in
//    GO=2.0\n  → moveOffset（前進/後退）
//    STOP\n    → moveOffset = 0
//    LOG=1\n   → 開啟藍牙 telemetry CSV
//    LOG=0\n   → 關閉藍牙 telemetry CSV
// ────────────────────────────────────────────────────────────────
void telemetryPrintHeader() {
  BTSerial.println(F("H,ms,angle_cdeg,gyro_dps10,cmd_sps,act_sps,move_cdeg,fallen"));
}

void telemetryUpdate() {
  if (!telemetryEnabled)
    return;

  unsigned long now = millis();
  if (now - telemetryLastMs < TELEMETRY_INTERVAL_MS)
    return;
  telemetryLastMs = now;

  BTSerial.print(F("D,"));
  BTSerial.print(now);
  BTSerial.print(',');
  BTSerial.print((long)(currentAngle * 100.0f));
  BTSerial.print(',');
  BTSerial.print((long)(gyroRate * 10.0f));
  BTSerial.print(',');
  BTSerial.print((long)motorSpeedCmd);
  BTSerial.print(',');
  BTSerial.print((long)motorSpeedActual);
  BTSerial.print(',');
  BTSerial.print((long)(moveOffset * 100.0f));
  BTSerial.print(',');
  BTSerial.println(isFallen ? 1 : 0);
}

void bluetoothProcess(bool balanceMode) {
  while (BTSerial.available()) {
    char c = BTSerial.read();
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() == 0)
        continue;
      String cmd = btBuffer;
      btBuffer = "";

      // 藍牙回覆模式記錄
      btLastMsg = cmd;

      if (cmd == "LOG=1") {
        telemetryEnabled = true;
        telemetryLastMs = 0;
        telemetryPrintHeader();
        BTSerial.println(F("OK:LOG=1"));
        continue;
      }
      if (cmd == "LOG=0") {
        telemetryEnabled = false;
        BTSerial.println(F("OK:LOG=0"));
        continue;
      }

      if (!balanceMode)
        continue; // 非平衡模式不處理控制指令

      bool paramChanged = false;
      String paramName = "";
      float paramVal = 0;

      if (cmd == "STOP") {
        moveOffset = 0;
        btOverlayMsg = "BT: STOP";
        btMsgTimer = millis() + BT_MSG_DURATION;
        BTSerial.println("OK:STOP");
        continue;
      }
      if (cmd.startsWith("GO=")) {
        moveOffset = constrain(cmd.substring(3).toFloat(), -MAX_ANGLE_CMD,
                               MAX_ANGLE_CMD);
        btOverlayMsg = "BT:GO=" + String(moveOffset, 1);
        btMsgTimer = millis() + BT_MSG_DURATION;
        BTSerial.println("OK:GO");
        continue;
      }
      if (cmd.startsWith("AP=")) {
        Kp = cmd.substring(3).toFloat();
        paramName = "Kp=";
        paramVal = Kp;
        paramChanged = true;
      } else if (cmd.startsWith("AI=")) {
        Ki = cmd.substring(3).toFloat();
        paramName = "Ki=";
        paramVal = Ki;
        paramChanged = true;
      } else if (cmd.startsWith("AD=")) {
        Kd = cmd.substring(3).toFloat();
        paramName = "Kd=";
        paramVal = Kd;
        paramChanged = true;
      }

      if (paramChanged) {
        btOverlayMsg = "BT:" + paramName + String(paramVal, 2);
        btMsgTimer = millis() + BT_MSG_DURATION;
        BTSerial.println("OK:RAM " + paramName + String(paramVal, 2));
      }
    } else {
      if (btBuffer.length() < 24)
        btBuffer += c;
    }
  }
}

// ────────────────────────────────────────────────────────────────
//  區塊十：LCD 工具函式
// ────────────────────────────────────────────────────────────────
void lcdRecover() {
  i2cInit();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdNeedsRecover = false;
  lcdLastRecover = millis();
  stateChanged = true;
}

void lcdPrint(const char *line0, const char *line1) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void lcdPrint(const String &line0, const String &line1) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line1.substring(0, 16));
}

// 顯示選單項目（顯示 > 游標 + 當前項 + 下一項預覽）
const char *mainMenuItems[] = {
    "1.Start Balance", "2.MPU6050 Setup", "3.Bluetooth Tst", "4.Motor Test",
    "5.PID Params",    "6.Linear Test",   "7.Max Speed Adj"};
const int MAIN_MENU_COUNT = 7;

const char *mpuMenuItems[] = {"2-1.Angle View", "2-2.Base Adjust",
                              "2-3.Tilt Calib"};

const char *btMenuItems[] = {"3-1.Ping Test", "3-2.Monitor"};

const char *pidParamNames[] = {"Kp", "Ki", "Kd"};
const int PID_PARAM_COUNT = 3;
int pidSelectIndex = 0; // 選哪個 PID 參數
float *pidParamPtrs[] = {&Kp, &Ki, &Kd};

void lcdShowMenu(const char **items, int count, int idx) {
  lcd.clear();
  // 第一行：當前項（加 > 游標）
  String line0 = String("> ") + items[idx];
  lcd.setCursor(0, 0);
  lcd.print(line0.substring(0, 16));
  // 第二行：下一項預覽
  if (idx + 1 < count) {
    String line1 = String("  ") + items[idx + 1];
    lcd.setCursor(0, 1);
    lcd.print(line1.substring(0, 16));
  }
}

// ────────────────────────────────────────────────────────────────
//  區塊十一：單層 PID 計算（只在 BALANCE 模式呼叫）
// ────────────────────────────────────────────────────────────────
void pidUpdate() {
  float currentTilt = currentAngle - base_angle;

  // 斷電保護
  if (fabsf(currentTilt) > FALL_ANGLE) {
    motorEmergencyStop();
    isFallen = true;
    sysState = STATE_MENU;
    stateChanged = true;
    return;
  }

  // 單層 PID (200Hz)
  float targetAngle = moveOffset;
  float angleErr = targetAngle - currentTilt;

  angleIntegral += angleErr * LOOP_DT;
  angleIntegral = constrain(angleIntegral, -1000.0f, 1000.0f);

  float dErr = (angleErr - prevAngleErr) / LOOP_DT;
  prevAngleErr = angleErr;

  motorSpeedCmd = Kp * angleErr + Ki * angleIntegral + Kd * dErr;
  motorSpeedCmd = constrain(motorSpeedCmd, -max_speed, max_speed);

  motorSetSpeed(motorSpeedCmd);
}

// ────────────────────────────────────────────────────────────────
//  區塊十二：馬達測試（同步阻塞式，僅在 MENU 模式使用）
// ────────────────────────────────────────────────────────────────
// 以指定 steps/s 速度轉指定步數（阻塞式，僅供測試）
void motorTestRun(int stepsPerSec, long steps, bool forward) {
  digitalWrite(DIR_A, forward ? LOW : HIGH);
  digitalWrite(DIR_B, forward ? HIGH : LOW);
  unsigned long stepInterval = 1000000UL / stepsPerSec; // µs
  for (long i = 0; i < steps; i++) {
    PORTD |= (1 << PD2) | (1 << PD4);
    delayMicroseconds(2);
    PORTD &= ~(1 << PD2) & ~(1 << PD4);
    delayMicroseconds(stepInterval - 2);
  }
}

// NEMA17 1/16 微步，1 圈 = 200 × 16 = 3200 步
#define STEPS_PER_REV 3200
void motorTestSequence(int testSpeed) {
  // 前轉 1 圈
  lcdPrint("Motor Test", "Forward 1 rev...");
  motorTestRun(testSpeed, STEPS_PER_REV, true);
  // 停頓 0.5 秒
  lcdPrint("Motor Test", "Pausing...");
  delay(500);
  // 後轉 1 圈
  lcdPrint("Motor Test", "Backward 1rev..");
  motorTestRun(testSpeed, STEPS_PER_REV, false);
  lcdPrint("Motor Test", "Done! Press btn");
}

// ────────────────────────────────────────────────────────────────
//  區塊十三：傾倒方向校正（兩步校正法）
// ────────────────────────────────────────────────────────────────
enum CalibStep { CALIB_WAIT_FORWARD, CALIB_WAIT_BACKWARD, CALIB_DONE };
CalibStep calibStep = CALIB_WAIT_FORWARD;
float calibFwdAngle = 0;
float calibBwdAngle = 0;

void calibUpdate() {
  // 使用已更新的 currentAngle（mpuRead 已呼叫）
  switch (calibStep) {
  case CALIB_WAIT_FORWARD:
    if (stateChanged) {
      lcdPrint("Tilt FORWARD", "> 15deg, wait..");
      stateChanged = false;
    }
    if (fabsf(currentAngle) > CALIB_TILT_THRESH) {
      calibFwdAngle = currentAngle;
      calibStep = CALIB_WAIT_BACKWARD;
      delay(1000); // 讓使用者扶正
      lcdPrint("Tilt BACKWARD", "> 15deg, wait..");
    }
    break;

  case CALIB_WAIT_BACKWARD:
    if (fabsf(currentAngle) > CALIB_TILT_THRESH) {
      calibBwdAngle = currentAngle;
      // 判斷方向符號
      // 前傾角 > 後傾角 → 符號正確（+1）；反之取反
      dir_sign = (calibFwdAngle > calibBwdAngle) ? 1 : -1;
      // 計算平衡基準角（兩個傾倒方向的中點，理論上為 0）
      base_angle = (calibFwdAngle + calibBwdAngle) / 2.0f;
      eepromSaveCalib(dir_sign);
      eepromSaveBase();
      calibStep = CALIB_DONE;
      lcdPrint("Calibrated!",
               "dir=" + String(dir_sign) + " base=" + String(base_angle, 1));
      delay(2000);
      sysState = STATE_MPU_MENU;
      stateChanged = true;
      calibStep = CALIB_WAIT_FORWARD; // 重置供下次使用
    }
    break;

  default:
    break;
  }
}

// ────────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // GPIO
  pinMode(STEP_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(STEP_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH); // 馬達先斷電

  // 編碼器
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  encLastCLK = digitalRead(ENC_CLK);

  // I2C
  i2cInit();

  // LCD
  lcd.init();
  lcd.backlight();
  lcdPrint("Initializing...", "Please wait...");

  // EEPROM（含魔術數字驗證）
  eepromLoad();

  // MPU6050
  mpuInit();
  delay(500);
  // 預熱：讀取 100 次讓互補濾波收斂
  for (int i = 0; i < 100; i++) {
    mpuRead();
    delay(5);
  }

  // 藍牙
  BTSerial.begin(9600);

  // Timer1（背景脈衝，但馬達仍斷電）
  timer1Init();

  // 進入主選單
  sysState = STATE_MENU;
  stateChanged = true;
  menuIndex = 0;
  lcdPrint("Ready!", "");
  delay(500);

  lastLoopUs = micros();
}

// ────────────────────────────────────────────────────────────────
//  Loop — 狀態機主體
// ────────────────────────────────────────────────────────────────
void loop() {
  // ── 精確 5ms 節拍（平衡模式與角度測試共用） ──────────────
  bool tickReady = false;
  if (sysState == STATE_BALANCE || sysState == STATE_MPU_ANGLE) {
    if (micros() - lastLoopUs >= LOOP_DT_US) {
      lastLoopUs += LOOP_DT_US;
      tickReady = true;
    } else {
      return; // 尚未到下一個 tick，直接返回
    }
  }

  // ── 編碼器輪詢 ────────────────────────────────
  encoderUpdate();

  // ── 藍牙處理 ──────────────────────────────────
  bluetoothProcess(sysState == STATE_BALANCE);

  // ══════════════════════════════════════════════
  //  狀態機
  // ══════════════════════════════════════════════
  switch (sysState) {

  // ──────────────────────────────────────────────
  //  主選單
  // ──────────────────────────────────────────────
  case STATE_MENU: {
    if (stateChanged) {
      digitalWrite(MOTOR_EN, HIGH); // 確保馬達斷電
      motorStop();
      isFallen = false;
      lcdShowMenu(mainMenuItems, MAIN_MENU_COUNT, menuIndex);
      stateChanged = false;
    }
    if (encDelta != 0) {
      menuIndex = (menuIndex + encDelta + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;
      lcdShowMenu(mainMenuItems, MAIN_MENU_COUNT, menuIndex);
    }
    if (encPressed) {
      stateChanged = true;
      switch (menuIndex) {
      case 0: // 開始平衡
        angleIntegral = 0;
        prevAngleErr = 0;
        motorSpeedActual = 0;
        moveOffset = 0;
        currentAngle = mpuReadAccAngle();
        digitalWrite(MOTOR_EN, LOW);
        sysState = STATE_BALANCE;
        lastLoopUs = micros();
        break;
      case 1:
        subIndex = 0;
        sysState = STATE_MPU_MENU;
        break;
      case 2:
        subIndex = 0;
        sysState = STATE_BT_MENU;
        break;
      case 3:
        sysState = STATE_MOTOR_TEST;
        break;
      case 4:
        pidSelectIndex = 0;
        sysState = STATE_PID_MENU;
        break;
      case 5:
        sysState = STATE_LINEAR_TEST;
        break;
      case 6:
        sysState = STATE_MAX_SPEED_EDIT;
        break;
      }
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  平衡模式
  // ──────────────────────────────────────────────
  case STATE_BALANCE: {
    if (stateChanged) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Balancing...");
      stateChanged = false;
    }
    if (!tickReady)
      break;

    // IMU 讀取 + PID
    mpuRead();
    pidUpdate(); // 內部處理跌倒保護與狀態切換
    telemetryUpdate();

    // 按下編碼器退出平衡模式
    if (encPressed) {
      motorEmergencyStop();
      sysState = STATE_MENU;
      stateChanged = true;
      break;
    }

    // LCD 顯示（節流）
    unsigned long now = millis();
    if (now - lcdLastUpdate >= LCD_UPDATE_INTERVAL) {
      lcdLastUpdate = now;
      // 藍牙提示覆蓋
      if (btMsgTimer > 0 && now < btMsgTimer) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(btOverlayMsg.substring(0, 16));
      } else {
        btMsgTimer = 0;
        // 正常顯示：傾角 + 馬達速度
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Ang:");
        lcd.print(currentAngle, 1);
        lcd.print(" B:");
        lcd.print(base_angle, 1);
        lcd.setCursor(0, 1);
        lcd.print("Spd:");
        lcd.print(motorSpeedActual, 0);
        lcd.print(" O:");
        lcd.print(moveOffset, 1);
      }
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  MPU6050 子選單
  // ──────────────────────────────────────────────
  case STATE_MPU_MENU: {
    if (stateChanged) {
      lcdShowMenu(mpuMenuItems, 3, subIndex);
      stateChanged = false;
    }
    if (encDelta != 0) {
      subIndex = (subIndex + encDelta + 3) % 3;
      lcdShowMenu(mpuMenuItems, 3, subIndex);
    }
    if (encPressed) {
      stateChanged = true;
      switch (subIndex) {
      case 0:
        sysState = STATE_MPU_ANGLE;
        lastLoopUs = micros();
        break;
      case 1:
        sysState = STATE_MPU_BASE;
        break;
      case 2:
        sysState = STATE_MPU_CALIB;
        calibStep = CALIB_WAIT_FORWARD;
        break;
      }
    }
    if (encLongPress) {
      sysState = STATE_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  2-1 角度診斷視窗
  //  第一行：Ax=XX.X Ay=XX.X
  //  第二行：Az=XX.X F=XX.X
  // ──────────────────────────────────────────────
  case STATE_MPU_ANGLE: {
    if (!tickReady)
      break;

    mpuRead();
    unsigned long now = millis();
    if (now - lcdLastUpdate >= LCD_UPDATE_INTERVAL) {
      lcdLastUpdate = now;
      // 第一行：Ax Ay
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Ax:");
      String axStr = String(accelAngles[0], 1);
      lcd.print(axStr);
      lcd.print(" Ay:");
      String ayStr = String(accelAngles[1], 1);
      lcd.print(ayStr);
      // 第二行：Az + 融合角 F
      lcd.setCursor(0, 1);
      lcd.print("Az:");
      lcd.print(String(accelAngles[2], 1));
      lcd.print(" F:");
      lcd.print(String(currentAngle, 1));
    }
    if (encPressed || encLongPress) {
      sysState = STATE_MPU_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  2-2 平衡腳調整（旋鈕 ±0.1°，按下儲存）
  // ──────────────────────────────────────────────
  case STATE_MPU_BASE: {
    if (stateChanged) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Base Angle Adj");
      lcd.setCursor(0, 1);
      lcd.print("Val:");
      lcd.print(base_angle, 2);
      stateChanged = false;
    }
    if (encDelta != 0) {
      base_angle += encDelta * 0.1f;
      base_angle = constrain(base_angle, -10.0f, 10.0f);
      lcd.setCursor(0, 1);
      lcd.print("Val:");
      lcd.print(String(base_angle, 2) + "   ");
    }
    if (encPressed) {
      eepromSaveBase();
      lcdPrint("Base Saved!", String(base_angle, 2) + " deg");
      delay(1200);
      sysState = STATE_MPU_MENU;
      stateChanged = true;
    }
    if (encLongPress) {
      sysState = STATE_MPU_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  2-3 傾倒方向校正（兩步校正法）
  // ──────────────────────────────────────────────
  case STATE_MPU_CALIB: {
    mpuRead();
    calibUpdate(); // 狀態流轉於 calibUpdate 內部處理
    if (encLongPress) {
      calibStep = CALIB_WAIT_FORWARD;
      sysState = STATE_MPU_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  藍牙子選單
  // ──────────────────────────────────────────────
  case STATE_BT_MENU: {
    if (stateChanged) {
      lcdShowMenu(btMenuItems, 2, subIndex);
      stateChanged = false;
    }
    if (encDelta != 0) {
      subIndex = (subIndex + encDelta + 2) % 2;
      lcdShowMenu(btMenuItems, 2, subIndex);
    }
    if (encPressed) {
      stateChanged = true;
      sysState = (subIndex == 0) ? STATE_BT_PING : STATE_BT_MONITOR;
    }
    if (encLongPress) {
      sysState = STATE_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  3-1 藍牙 Ping 測試
  // ──────────────────────────────────────────────
  case STATE_BT_PING: {
    if (stateChanged) {
      lcdPrint("BT Ping Test", "Press to send");
      stateChanged = false;
    }
    if (encPressed) {
      btLastMsg = "";
      BTSerial.println("PING");
      lcdPrint("Sent: PING", "Waiting...");
      unsigned long deadline = millis() + 2000;
      while (millis() < deadline) {
        bluetoothProcess(false);
        if (btLastMsg.length() > 0)
          break;
        delay(10);
      }
      if (btLastMsg.length() > 0) {
        lcdPrint("ACK received:", btLastMsg.substring(0, 16));
      } else {
        lcdPrint("No Response", "Check BT module");
      }
      delay(2500);
      stateChanged = true;
    }
    if (encLongPress) {
      sysState = STATE_BT_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  3-2 藍牙回覆監視模式
  // ──────────────────────────────────────────────
  case STATE_BT_MONITOR: {
    if (stateChanged) {
      lcdPrint("BT Monitor", "(long:exit)");
      stateChanged = false;
    }
    bluetoothProcess(false);
    unsigned long now = millis();
    if (now - lcdLastUpdate >= LCD_UPDATE_INTERVAL) {
      lcdLastUpdate = now;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Recv:");
      lcd.setCursor(0, 1);
      if (btLastMsg.length() > 0) {
        lcd.print(btLastMsg.substring(0, 16));
      } else {
        lcd.print("(no msg yet)");
      }
    }
    if (encLongPress) {
      sysState = STATE_BT_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  4. 馬達測試
  // ──────────────────────────────────────────────
  case STATE_MOTOR_TEST: {
    static int testSpeedLevel = 1; // 0=慢 1=中 2=快
    static bool testDone = false;
    const int testSpeeds[] = {400, 800, 1600};
    const char *speedNames[] = {"Slow", "Med ", "Fast"};

    if (stateChanged) {
      testDone = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Motor Test");
      lcd.setCursor(0, 1);
      lcd.print("Spd:");
      lcd.print(speedNames[testSpeedLevel]);
      lcd.print(" Btn=Run");
      stateChanged = false;
    }

    if (!testDone) {
      // 旋鈕調速（測試前）
      if (encDelta != 0) {
        testSpeedLevel = constrain(testSpeedLevel + encDelta, 0, 2);
        lcd.setCursor(0, 1);
        lcd.print("Spd:");
        lcd.print(speedNames[testSpeedLevel]);
        lcd.print(" Btn=Run");
      }
      if (encPressed) {
        digitalWrite(MOTOR_EN, LOW);
        timer1Init(); // 確保 Timer1 在正確狀態
        motorTestSequence(testSpeeds[testSpeedLevel]);
        digitalWrite(MOTOR_EN, HIGH);
        testDone = true;
      }
    } else {
      if (encPressed || encLongPress) {
        sysState = STATE_MENU;
        stateChanged = true;
        testDone = false;
      }
    }
    if (!testDone && encLongPress) {
      sysState = STATE_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  5. 線性調速測試 (0~10000 steps/s)
  // ──────────────────────────────────────────────
  case STATE_LINEAR_TEST: {
    static float linearTestSpeed = 0.0f;
    if (stateChanged) {
      linearTestSpeed = 0.0f;
      motorSpeedActual = 0.0f;
      digitalWrite(MOTOR_EN, LOW); // 啟動馬達
      timer1Init();                // 確保 Timer1 啟動
      noInterrupts();
      periodA = 0;
      periodB = 0;
      interrupts();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Linear Spd Test");
      lcd.setCursor(0, 1);
      lcd.print("Spd:0        ");
      stateChanged = false;
    }

    if (encDelta != 0) {
      linearTestSpeed += encDelta * 100.0f; // 每次增減 100
      linearTestSpeed =
          constrain(linearTestSpeed, 0.0f, 10000.0f); // 範圍 0~10000

      motorSpeedActual = linearTestSpeed;
      if (linearTestSpeed < 2.0f) {
        noInterrupts();
        periodA = 0;
        periodB = 0;
        interrupts();
      } else {
        digitalWrite(DIR_A, LOW); // 固定前進（已反轉）
        digitalWrite(DIR_B, HIGH);
        int ticks = speedToTicks(linearTestSpeed);
        noInterrupts();
        periodA = ticks;
        periodB = ticks;
        interrupts();
      }

      lcd.setCursor(0, 1);
      lcd.print("Spd:");
      lcd.print((long)linearTestSpeed);
      lcd.print("      "); // 清除尾部多餘字元
    }

    if (encPressed || encLongPress) {
      motorStop();
      digitalWrite(MOTOR_EN, HIGH); // 關閉馬達
      sysState = STATE_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  6. PID 參數選擇選單
  // ──────────────────────────────────────────────
  case STATE_PID_MENU: {
    if (stateChanged) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("> ");
      lcd.print(pidParamNames[pidSelectIndex]);
      lcd.print("=");
      lcd.print(*pidParamPtrs[pidSelectIndex], 2);
      lcd.setCursor(0, 1);
      lcd.print("Btn=Edit LLong=Back");
      stateChanged = false;
    }
    if (encDelta != 0) {
      pidSelectIndex =
          (pidSelectIndex + encDelta + PID_PARAM_COUNT) % PID_PARAM_COUNT;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("> ");
      lcd.print(pidParamNames[pidSelectIndex]);
      lcd.print("=");
      lcd.print(*pidParamPtrs[pidSelectIndex], 2);
      lcd.setCursor(0, 1);
      lcd.print("Btn=Edit LLong=Back");
    }
    if (encPressed) {
      sysState = STATE_PID_EDIT;
      stateChanged = true;
    }
    if (encLongPress) {
      sysState = STATE_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  5. PID 數值編輯（Ki 步進 0.01，其他 0.1）
  // ──────────────────────────────────────────────
  case STATE_PID_EDIT: {
    if (stateChanged) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Edit:");
      lcd.print(pidParamNames[pidSelectIndex]);
      lcd.setCursor(0, 1);
      lcd.print("Val:");
      lcd.print(*pidParamPtrs[pidSelectIndex], 2);
      stateChanged = false;
    }
    if (encDelta != 0) {
      float stepSize = (pidSelectIndex == 1) ? 0.1f : 0.1f;
      *pidParamPtrs[pidSelectIndex] += encDelta * stepSize;
      // 防止負值（PID 參數不應為負）
      if (*pidParamPtrs[pidSelectIndex] < 0.0f)
        *pidParamPtrs[pidSelectIndex] = 0.0f;
      lcd.setCursor(0, 1);
      lcd.print("Val:");
      lcd.print(String(*pidParamPtrs[pidSelectIndex], 2) + "   ");
    }
    if (encPressed) {
      eepromSavePID();
      String savedMsg = String(pidParamNames[pidSelectIndex]) + "=" +
                        String(*pidParamPtrs[pidSelectIndex], 2);
      lcdPrint("Saved!", savedMsg);
      delay(1200);
      sysState = STATE_PID_MENU;
      stateChanged = true;
    }
    if (encLongPress) {
      sysState = STATE_PID_MENU;
      stateChanged = true;
    }
    break;
  }

  // ──────────────────────────────────────────────
  //  7. 最大速限編輯（±100 每格，按下儲存）
  // ──────────────────────────────────────────────
  case STATE_MAX_SPEED_EDIT: {
    if (stateChanged) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Max Speed Adj");
      lcd.setCursor(0, 1);
      lcd.print("Val:");
      lcd.print(max_speed, 0);
      stateChanged = false;
    }
    if (encDelta != 0) {
      max_speed += encDelta * 100.0f;
      max_speed =
          constrain(max_speed, 100.0f, 10000.0f); // 限制在 100~10000 之間
      lcd.setCursor(0, 1);
      lcd.print("Val:");
      lcd.print((long)max_speed);
      lcd.print("    "); // 清除尾部多餘字元
    }
    if (encPressed) {
      eepromSaveMaxSpeed();
      lcdPrint("Saved!", "MaxSpd=" + String((long)max_speed));
      delay(1200);
      sysState = STATE_MENU;
      stateChanged = true;
    }
    if (encLongPress) {
      sysState = STATE_MENU;
      stateChanged = true;
    }
    break;
  }

  default:
    sysState = STATE_MENU;
    stateChanged = true;
    break;
  }
  // ── 非平衡模式的 tick 更新 ──────────────────
  if (sysState != STATE_BALANCE && sysState != STATE_MPU_ANGLE)
    lastLoopUs = micros();

  if (i2cTimedOut())
    lcdNeedsRecover = true;

  if (sysState != STATE_BALANCE &&
      (lcdNeedsRecover || millis() - lcdLastRecover >= LCD_RECOVER_INTERVAL)) {
    lcdRecover();
  }
}
