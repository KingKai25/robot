#include <Arduino.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>

// Compatibility shim: DabbleESP32 dùng hàm ledcAttachChannel, chưa có trên core ESP32 mới.
#ifdef ARDUINO_ARCH_ESP32
static inline void ledcAttachChannel(uint8_t pin, double freq, uint8_t resolution, uint8_t channel) {
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
}
#endif

#include <DabbleESP32.h>

// ==================== CẤU HÌNH CHÂN GPIO ====================
// Động cơ bánh xe (Motor Driver - L298N hoặc tương tự)
#define MOTOR_LEFT_FWD    25   // Động cơ trái tiến
#define MOTOR_LEFT_BWD    26   // Động cơ trái lùi
#define MOTOR_RIGHT_FWD   27   // Động cơ phải tiến
#define MOTOR_RIGHT_BWD   14   // Động cơ phải lùi
#define MOTOR_LEFT_PWM    32   // PWM điều khiển tốc độ trái
#define MOTOR_RIGHT_PWM   33   // PWM điều khiển tốc độ phải

// Servo cơ cấu gắp (Gripper)
#define SERVO_ARM     19   // Servo cánh tay nâng/hạ
#define SERVO_GRIPPER 21   // Servo kẹp/thả

// ==================== BIẾN TOÀN CỤC ====================

// Tên robot (thay đổi thành số khác nếu có nhiều robot cùng thi)
#define ROBOT_NAME "ROBOT_001"

// Chế độ hoạt động
enum RobotMode {
  MODE_AUTO_LINE_FOLLOW,   // Tự động dò line
  MODE_MANUAL_GRIPPER      // Thủ công điều khiển gripper/arm
};
RobotMode currentMode = MODE_AUTO_LINE_FOLLOW;

// Vị trí servo
int gripperPosition = 90;    // 90 = mở, 0 = đóng
int armPosition = 90;        // Vị trí cánh tay nâng/hạ

bool wasConnected = false;
bool modeChanged = false;    // Flag để tránh xử lý SELECT liên tục

// ==================== HÀM ĐIỀU KHIỂN ĐỘNG CƠ ====================
void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  Serial.println("[MOTOR] Dừng hẳn");
}

void moveForward(int speed = 200) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  ledcWrite(0, speed);
  ledcWrite(1, speed);
  Serial.printf("[MOTOR] Tiến - Tốc độ: %d\n", speed);
}

void moveBackward(int speed = 200) {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  ledcWrite(0, speed);
  ledcWrite(1, speed);
  Serial.printf("[MOTOR] Lùi - Tốc độ: %d\n", speed);
}

void turnLeft(int speed = 180) {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  ledcWrite(0, speed);
  ledcWrite(1, speed);
  Serial.printf("[MOTOR] Rẽ trái - Tốc độ: %d\n", speed);
}

void turnRight(int speed = 180) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  ledcWrite(0, speed);
  ledcWrite(1, speed);
  Serial.printf("[MOTOR] Rẽ phải - Tốc độ: %d\n", speed);
}

// ==================== HÀM ĐIỀU KHIỂN SERVO ====================
void setGripper(int angle) {
  gripperPosition = constrain(angle, 0, 180);
  int pulseWidth = map(gripperPosition, 0, 180, 500, 2500);
  ledcWrite(2, pulseWidth);
  Serial.printf("[SERVO] Gripper: %d độ (PWM: %d)\n", gripperPosition, pulseWidth);
}

void setArm(int angle) {
  armPosition = constrain(angle, 0, 180);
  int pulseWidth = map(armPosition, 0, 180, 500, 2500);
  ledcWrite(3, pulseWidth);
  Serial.printf("[SERVO] Arm: %d độ (PWM: %d)\n", armPosition, pulseWidth);
}


// ==================== XỬ LÝ GAMEPAD DABBLE (BLE) ====================
void processGamepadDabble() {
  // Di chuyển bằng D-Pad
  if (GamePad.isUpPressed()) {
    moveForward(220);
  } else if (GamePad.isDownPressed()) {
    moveBackward(220);
  } else if (GamePad.isLeftPressed()) {
    turnLeft(200);
  } else if (GamePad.isRightPressed()) {
    turnRight(200);
  } else {
    stopMotors();
  }

  // Gắp / Thả
  if (GamePad.isTrianglePressed()) {
    setGripper(0);
    Serial.println("[GRIPPER] ▲ Gắp");
  }
  if (GamePad.isCrossPressed()) {
    setGripper(90);
    Serial.println("[GRIPPER] ✕ Thả");
  }

  // Nâng / Hạ tay
  if (GamePad.isSquarePressed()) {
    armPosition = constrain(armPosition + 5, 0, 180);
    setArm(armPosition);
  }
  if (GamePad.isCirclePressed()) {
    armPosition = constrain(armPosition - 5, 0, 180);
    setArm(armPosition);
  }

  // Chuyển chế độ (SELECT button)
  if (GamePad.isSelectPressed() && !modeChanged) {
    modeChanged = true;
    if (currentMode == MODE_AUTO_LINE_FOLLOW) {
      currentMode = MODE_MANUAL_GRIPPER;
      Serial.println("[MODE] 🎮 CHUYỂN SANG: MANUAL GRIPPER/ARM");
    } else {
      currentMode = MODE_AUTO_LINE_FOLLOW;
      Serial.println("[MODE] 🤖 CHUYỂN SANG: AUTO LINE FOLLOW");
    }
  } else if (!GamePad.isSelectPressed()) {
    modeChanged = false;  // Reset flag khi thả nút
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("   ROBOT ĐIỀU KHIỂN GẮPER VỚI DABBLE");
  Serial.println("========================================");

  // GPIO động cơ
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);

  // PWM Servo
  ledcSetup(2, 50, 16);
  ledcSetup(3, 50, 16);
  ledcAttachPin(SERVO_GRIPPER, 2);
  ledcAttachPin(SERVO_ARM, 3);

  setGripper(90);
  setArm(90);

  Serial.println("[INIT] ✅ GPIO OK");

  // Tăng TX Power BLE để tránh nhiễu trong thi đấu
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);     // ADV công suất cao
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);    // SCAN công suất cao
  
  // BLE Dabble
  Dabble.begin(ROBOT_NAME);
  Serial.printf("[INIT] ✅ BLE Dabble sẵn sàng (Tên: %s)\n", ROBOT_NAME);
  Serial.println("[INIT] ✅ TX Power tối đa để chống nhiễu");

  Serial.println("\n📱 HƯỚNG DẪN KẾT NỐI:");
  Serial.println("  1. Mở Dabble app trên điện thoại");
  Serial.println("  2. Vào module GamePad -> chọn Connect");
  Serial.printf("  3. Chọn thiết bị tên %s (BLE)\n", ROBOT_NAME);
  Serial.println("  4. Dùng các nút để điều khiển");
  Serial.println("\n🎮 ĐIỀU KHIỂN GAMEPAD:");
  Serial.println("  ← → ↑ ↓ : Di chuyển robot");
  Serial.println("  △ (Triangle) : Gắp vật");
  Serial.println("  ✕ (Cross) : Thả vật");
  Serial.println("  □ (Square) : Nâng cánh tay");
  Serial.println("  ○ (Circle) : Hạ cánh tay");
  Serial.println("  START : Reset tất cả");
  Serial.println("========================================\n");
}

void loop() {
  Dabble.processInput();

  if (Dabble.isAppConnected()) {
    if (!wasConnected) {
      Serial.println("[BLE] ✅ Đã kết nối Dabble");
      wasConnected = true;
    }
    processGamepadDabble();
  } else {
    if (wasConnected) {
      Serial.println("[BLE] ❌ Mất kết nối Dabble");
      stopMotors();
      wasConnected = false;
    }
  }

  delay(10);  // Giảm từ 20ms → 10ms để phản ứng nhanh hơn, chống nhiễu tốt hơn
}
