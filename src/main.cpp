#include <Arduino.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>

// Compatibility shim: DabbleESP32 dùng hàm ledcAttachChannel, chưa có trên core ESP32 cũ.
#ifdef ARDUINO_ARCH_ESP32
  #if !defined(ledcAttachChannel)
  static inline void ledcAttachChannel(uint8_t pin, double freq, uint8_t resolution, uint8_t channel) {
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(pin, channel);
  }
  #endif
#endif

#include <DabbleESP32.h>

// ==================== CẤU HÌNH CHÂN GPIO (THEO SCHEMATIC) ====================
// Motor Driver TB6612FNG
// Motor A = bánh trái, Motor B = bánh phải
#define AIN1  25   // D25 → TB6612 AIN1 (pin 14)
#define AIN2  33   // D33 → TB6612 AIN2 (pin 15)
#define PWMA  32   // D32 → TB6612 PWMA (pin 16)

#define BIN1  26   // D26 → TB6612 BIN1 (pin 12)
#define BIN2  27   // D27 → TB6612 BIN2 (pin 11)
#define PWMB  14   // D14 → TB6612 PWMB (pin 10)

// Cảm biến dò line TCRT5000 (5 mắt, bên phải ESP32)
#define S1    18   // D18 → TCRT5000 S1
#define S2     5   // D5  → TCRT5000 S2
#define S3    17   // TX2 → TCRT5000 S3
#define S4    16   // RX2 → TCRT5000 S4
#define S5     4   // D4  → TCRT5000 S5

// Servo cơ cấu gắp (Càng gắp trước-sau, 4 servo)
#define SERVO_LIFT_LEFT    23   // D23 → Servo nâng/hạ tay trái
#define SERVO_LIFT_RIGHT   22   // D22 → Servo nâng/hạ tay phải
#define SERVO_GRIP_LEFT    21   // D21 → Servo mở/gắp tay trái
#define SERVO_GRIP_RIGHT   19   // D19 → Servo mở/gắp tay phải

// ==================== PWM CONFIG ====================
#define PWM_FREQ      20000   // Tần số PWM động cơ (20kHz, giảm tiếng ồn)
#define PWM_RES       8       // Độ phân giải 8-bit (0-255)
#define PWM_LEFT_CH   0       // LEDC channel cho motor trái
#define PWM_RIGHT_CH  1       // LEDC channel cho motor phải

// LEDC channels cho servo
#define SERVO_CH_LIFT_L  2
#define SERVO_CH_LIFT_R  3
#define SERVO_CH_GRIP_L  4
#define SERVO_CH_GRIP_R  5

// ==================== BIẾN TOÀN CỤC ====================
#define ROBOT_NAME "ROBOT_001" 

// ---- PID ----
float Kp = 18.0;
float Ki = 0.0;
float Kd = 12.0;

float lineError = 0;
float previous_error = 0;
float I_sum = 0;
int lastDirection = 1;  // 1=phải, -1=trái (hướng cuối cùng rõ ràng)

int base_speed = 100;
int max_speed  = 255;

// ---- Trạng thái dò line ----
bool lineStarted    = false;   // Bắt đầu chạy sau vạch 11111 lần 1
bool lineStopped    = false;   // Dừng khi gặp vạch 11111 lần 2
bool lastAllBlack   = false;
int  detectCount    = 0;

// ---- Chế độ hoạt động ----
enum RobotMode {
  MODE_AUTO_LINE_FOLLOW,   // Tự động dò line (PID)
  MODE_MANUAL_GRIPPER      // Thủ công điều khiển (Dabble GamePad)
};
RobotMode currentMode = MODE_MANUAL_GRIPPER;

// ---- Góc servo đặt trước ----
const int goc_nang      = 0;  // Nâng = 0°
const int goc_ha        = 110;   // Hạ   = 110°
const int goc_mo_cang   = 90;  // Mở   = 90°
const int goc_dong_cang = 0;   // Đóng = 0°

// ---- Servo positions ----
int liftLeftPosition  = goc_nang;         // Khởi tạo: nâng 
int liftRightPosition = goc_nang;
int gripLeftPosition  = goc_dong_cang;    // Khởi tạo: đóng
int gripRightPosition = goc_dong_cang;

// ---- Toggle state (chẵn=90°, lẻ=0°) ----
bool liftLeftState  = false;  // false = đang ở goc_nang (0°)
bool liftRightState = false;
bool gripLeftState  = false;  // false = đang ở goc_mo_cang (90°) → lần nhấn đầu sẽ đóng
bool gripRightState = false;

// ---- Edge detection (phát hiện cạnh lên của nút) ----
bool lastTriangle = false;
bool lastCross    = false;
bool lastSquare   = false;
bool lastCircle   = false;

// ---- Debounce cho nút servo (chống rung) ----
unsigned long lastToggleLiftL = 0;
unsigned long lastToggleLiftR = 0;
unsigned long lastToggleGripL = 0;
unsigned long lastToggleGripR = 0;
const unsigned long DEBOUNCE_MS = 300;  // 300ms chống rung
const int SERVO_STEP_DELAY = 10;        // ms giữa mỗi bước 1° cho servo nâng/hạ

// ---- BLE ----
bool wasConnected = false;
bool modeChanged  = false;

// ==================== MOTOR TB6612FNG ====================
void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  ledcWrite(PWM_LEFT_CH, speed);
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  ledcWrite(PWM_RIGHT_CH, speed);
}

void stopMotors() {
  ledcWrite(PWM_LEFT_CH, 0);
  ledcWrite(PWM_RIGHT_CH, 0);
}

void moveForward(int speed) {
  setMotorLeft(speed);
  setMotorRight(speed);
}

void moveBackward(int speed) {
  setMotorLeft(-speed);
  setMotorRight(-speed);
}

void turnLeft(int speed) {
  setMotorLeft(-speed);
  setMotorRight(speed);
}

void turnRight(int speed) {
  setMotorLeft(speed);
  setMotorRight(-speed);
}

// ==================== SERVO ====================
void writeServoAngle(int channel, int angle) {
  int duty = map(angle, 0, 180, 1638, 8192);
  ledcWrite(channel, duty);
}

void smoothServo(int channel, int &currentPos, int targetAngle, int stepDelay) {
  targetAngle = constrain(targetAngle, 0, 180);
  int step = (targetAngle > currentPos) ? 1 : -1;
  while (currentPos != targetAngle) {
    currentPos += step;
    writeServoAngle(channel, currentPos);
    delay(stepDelay);
  }
}

void setLiftLeft(int angle) {
  smoothServo(SERVO_CH_LIFT_L, liftLeftPosition, angle, SERVO_STEP_DELAY);
}

void setLiftRight(int angle) {
  smoothServo(SERVO_CH_LIFT_R, liftRightPosition, angle, SERVO_STEP_DELAY);
}

void setGripLeft(int angle) {
  gripLeftPosition = constrain(angle, 0, 180);
  writeServoAngle(SERVO_CH_GRIP_L, gripLeftPosition);
}

void setGripRight(int angle) {
  gripRightPosition = constrain(angle, 0, 180);
  writeServoAngle(SERVO_CH_GRIP_R, gripRightPosition);
}

// ==================== ĐỌC CẢM BIẾN DÒ LINE ====================
void readLine() {
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // In serial chỉ khi cảm biến thay đổi
  static int ps1=-1, ps2=-1, ps3=-1, ps4=-1, ps5=-1;
  if (s1!=ps1 || s2!=ps2 || s3!=ps3 || s4!=ps4 || s5!=ps5) {
    Serial.printf("[RAW] S1=%d S2=%d S3=%d S4=%d S5=%d\n", s1, s2, s3, s4, s5);
    ps1=s1; ps2=s2; ps3=s3; ps4=s4; ps5=s5;
  }

  // Đảo bit: 0=đen → 1=active, 1=trắng → 0=inactive
  int v1 = !s1;  // Vị trí +4 (cực phải)
  int v2 = !s2;  // Vị trí +2
  int v3 = !s3;  // Vị trí  0 (giữa)
  int v4 = !s4;  // Vị trí -2
  int v5 = !s5;  // Vị trí -4 (cực trái)

  int activeCount = v1 + v2 + v3 + v4 + v5;

  if (activeCount > 0) {
    // Trung bình có trọng số: tính vị trí vạch
    float weightedSum = v1*4.0 + v2*2.0 + v3*0.0 + v4*(-2.0) + v5*(-4.0);
    lineError = weightedSum / activeCount;
    // Lưu hướng rõ ràng khi error đủ lớn
    if (lineError > 0.5) lastDirection = 1;
    else if (lineError < -0.5) lastDirection = -1;
  } else {
    // Mất line hoàn toàn → quay theo hướng cuối cùng để tìm lại
    lineError = lastDirection * 5;
  }

  // In lại khi lineError thay đổi
  static float prevPrintError = -999;
  if (lineError != prevPrintError) {
    Serial.printf("[SENSOR] S1=%d S2=%d S3=%d S4=%d S5=%d | Error=%.1f\n", s1, s2, s3, s4, s5, lineError);
    prevPrintError = lineError;
  }
}

// ==================== PID ====================
void computePID() {
  float P = lineError;
  I_sum += lineError;
  I_sum = constrain(I_sum, -50, 50);
  float D = lineError - previous_error;

  float PID_value = Kp * P + Ki * I_sum + Kd * D;

  int left_speed  = base_speed - (int)PID_value;
  int right_speed = base_speed + (int)PID_value;

  left_speed  = constrain(left_speed,  0, max_speed);
  right_speed = constrain(right_speed, 0, max_speed);

  setMotorLeft(left_speed);
  setMotorRight(right_speed);

  // In PID chỉ khi error thay đổi
  static float prevPidError = -999;
  if (lineError != prevPidError) {
    Serial.printf("[PID] Err=%.0f P=%.1f I=%.1f D=%.1f PID=%.1f | L=%d R=%d\n",
      lineError, Kp*P, Ki*I_sum, Kd*D, PID_value, left_speed, right_speed);
    prevPidError = lineError;
  }

  previous_error = lineError;
}

// ==================== RESET PID STATE ====================
void resetLineFollow() {
  lineError      = 0;
  previous_error = 0;
  I_sum          = 0;
  lastDirection  = 1;
}

// ==================== XỬ LÝ GAMEPAD DABBLE (BLE) ====================
void processGamepadManual() {
  // Di chuyển bằng D-Pad
  if (GamePad.isUpPressed()) {
    moveForward(150);
  } else if (GamePad.isDownPressed()) {
    moveBackward(150);
  } else if (GamePad.isLeftPressed()) {
    turnLeft(130);
  } else if (GamePad.isRightPressed()) {
    turnRight(130);
  } else {
    stopMotors();
  }

  // △ : Toggle Nâng/Hạ servo TRÁI (nhấn 1 lần = nâng 90°, nhấn lần 2 = hạ 0°, ...)
  unsigned long now = millis();
  bool curTriangle = GamePad.isTrianglePressed();
  if (curTriangle && !lastTriangle && (now - lastToggleLiftL > DEBOUNCE_MS)) {
    lastToggleLiftL = now;
    liftLeftState = !liftLeftState;
    setLiftLeft(liftLeftState ? goc_nang : goc_ha);
    Serial.printf("[SERVO] Lift L -> %d°\n", liftLeftState ? goc_nang : goc_ha);
  }
  lastTriangle = curTriangle;

  // ✕ : Toggle Nâng/Hạ servo PHẢI
  bool curCross = GamePad.isCrossPressed();
  if (curCross && !lastCross && (now - lastToggleLiftR > DEBOUNCE_MS)) {
    lastToggleLiftR = now;
    liftRightState = !liftRightState;
    setLiftRight(liftRightState ? goc_nang : goc_ha);
    Serial.printf("[SERVO] Lift R -> %d°\n", liftRightState ? goc_nang : goc_ha);
  }
  lastCross = curCross;

  // □ : Toggle Mở/Đóng servo TRÁI
  bool curSquare = GamePad.isSquarePressed();
  if (curSquare && !lastSquare && (now - lastToggleGripL > DEBOUNCE_MS)) {
    lastToggleGripL = now;
    gripLeftState = !gripLeftState;
    setGripLeft(gripLeftState ? goc_dong_cang : goc_mo_cang);
    Serial.printf("[SERVO] Grip L -> %d°\n", gripLeftState ? goc_dong_cang : goc_mo_cang);
  }
  lastSquare = curSquare;

  // ○ : Toggle Mở/Đóng servo PHẢI
  bool curCircle = GamePad.isCirclePressed();
  if (curCircle && !lastCircle && (now - lastToggleGripR > DEBOUNCE_MS)) {
    lastToggleGripR = now;
    gripRightState = !gripRightState;
    setGripRight(gripRightState ? goc_dong_cang : goc_mo_cang);
    Serial.printf("[SERVO] Grip R -> %d°\n", gripRightState ? goc_dong_cang : goc_mo_cang);
  }
  lastCircle = curCircle;
}

// ==================== XỬ LÝ CHUYỂN CHẾ ĐỘ ====================
void checkModeSwitch() {
  if (GamePad.isSelectPressed() && !modeChanged) {
    modeChanged = true;
    if (currentMode == MODE_AUTO_LINE_FOLLOW) {
      currentMode = MODE_MANUAL_GRIPPER;
      stopMotors();
      Serial.println("[MODE] -> MANUAL GRIPPER");
    } else {
      currentMode = MODE_AUTO_LINE_FOLLOW;
      resetLineFollow();
      stopMotors();
      Serial.println("[MODE] -> AUTO LINE FOLLOW");
    }
  } else if (!GamePad.isSelectPressed()) {
    modeChanged = false;
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  ROBOT DO LINE + GRIPPER (TB6612FNG)");
  Serial.println("========================================");

  // GPIO động cơ (TB6612FNG direction pins)
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // GPIO cảm biến dò line (TCRT5000)
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // PWM động cơ (TB6612FNG PWMA/PWMB)
  ledcSetup(PWM_LEFT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_LEFT_CH);
  ledcSetup(PWM_RIGHT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, PWM_RIGHT_CH);

  // PWM Servo (50Hz, 16-bit resolution)
  ledcSetup(SERVO_CH_LIFT_L, 50, 16);
  ledcSetup(SERVO_CH_LIFT_R, 50, 16);
  ledcSetup(SERVO_CH_GRIP_L, 50, 16);
  ledcSetup(SERVO_CH_GRIP_R, 50, 16);
  ledcAttachPin(SERVO_LIFT_LEFT,  SERVO_CH_LIFT_L);
  ledcAttachPin(SERVO_LIFT_RIGHT, SERVO_CH_LIFT_R);
  ledcAttachPin(SERVO_GRIP_LEFT,  SERVO_CH_GRIP_L);
  ledcAttachPin(SERVO_GRIP_RIGHT, SERVO_CH_GRIP_R);

  // Vị trí mặc định servo
  setLiftLeft(goc_nang);
  setLiftRight(goc_nang);
  setGripLeft(goc_dong_cang);
  setGripRight(goc_dong_cang);

  Serial.println("[INIT] GPIO + PWM OK");

  // BLE Dabble — khởi tạo trước khi set TX Power
  Serial.printf("[DEBUG] About to set BLE name: %s\n", ROBOT_NAME);
  Dabble.begin(ROBOT_NAME);
  Serial.printf("[INIT] BLE Dabble: %s\n", ROBOT_NAME);

  // Tăng TX Power BLE SAU khi init (để setting có hiệu lực)
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,  ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);

  Serial.println("\n[HUONG DAN]");
  Serial.println("  D-Pad: Di chuyen  |  SELECT: Chuyen che do");
  Serial.println("  Triangle: Servo nang/ha TRAI");
  Serial.println("  Cross:    Servo nang/ha PHAI");
  Serial.println("  Square:   Servo mo/gap TRAI");
  Serial.println("  Circle:   Servo mo/gap PHAI");
  Serial.printf( "  Mode hien tai: %s\n",
    currentMode == MODE_AUTO_LINE_FOLLOW ? "AUTO LINE FOLLOW" : "MANUAL GRIPPER");
  Serial.println("========================================\n");
}

// ==================== LOOP ====================
void loop() {
  Dabble.processInput();

  // Kiểm tra kết nối BLE
  if (Dabble.isAppConnected()) {
    if (!wasConnected) {
      Serial.println("[BLE] Connected");
      wasConnected = true;
    }
    // Luôn kiểm tra nút SELECT để chuyển chế độ
    checkModeSwitch();
  } else {
    if (wasConnected) {
      Serial.println("[BLE] Disconnected");
      stopMotors();
      wasConnected = false;
    }
  }

  // Xử lý theo chế độ
  if (currentMode == MODE_AUTO_LINE_FOLLOW) {
    readLine();
    computePID();
  } else if (currentMode == MODE_MANUAL_GRIPPER) {
    if (Dabble.isAppConnected()) {
      processGamepadManual();
    } else {
      stopMotors();
    }
  }

  delay(5);  // 5ms loop cho phản hồi PID nhanh
}