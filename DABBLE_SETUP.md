# Hướng Dẫn Kết Nối Robot với Dabble App

## 🔧 Cấu Hình Phần Cứng

### Kết Nối Bluetooth Module (HC-05 hoặc HM-10)
```
ESP32 Pin 16 (RX1) --> Bluetooth TX
ESP32 Pin 17 (TX1) --> Bluetooth RX
ESP32 GND ----------> Bluetooth GND
ESP32 5V ------------> Bluetooth VCC (nếu cần)
```

### Kết Nối Motor & Servo
- **Động cơ trái:**
  - GPIO 25: FWD
  - GPIO 26: BWD
  - GPIO 32: PWM
  
- **Động cơ phải:**
  - GPIO 27: FWD
  - GPIO 14: BWD
  - GPIO 33: PWM

- **Servo Gripper (kẹp):**
  - GPIO 21: PWM
  - 90° = Mở
  - 0° = Đóng

- **Servo Arm (cánh tay):**
  - GPIO 19: PWM
  - Điều khiển nâng hạ

## 📱 Cài Đặt Dabble App

1. **Tải Dabble App:**
   - iOS: App Store
   - Android: Google Play Store

2. **Ghép Nối Bluetooth:**
   - Bật Bluetooth trên điện thoại
   - Mở Dabble App
   - Chọn "Gamepad" từ danh sách
   - Chọn module Bluetooth (HC-05/HM-10)
   - Mã PIN mặc định: `1234` hoặc `0000`

3. **Baud Rate:**
   - Đảm bảo module Bluetooth được cấu hình với baud rate **9600**
   - Nếu dùng HC-05: AT+UART=9600,0,0

## 🎮 Điều Khiển GamePad

### Nút Di Chuyển (Trái)
```
      ⬆️  (UP)
    ⬅️  ➡️  
      ⬇️  (DOWN)
```

### Nút Chức Năng (Phải)
```
      ▲ (TRIANGLE) = Gắp vật
   ◀   ▶
      ○ (CIRCLE) = Hạ cánh tay

  □ (SQUARE) = Nâng cánh tay
  ✕ (CROSS) = Thả vật
```

### Nút Bổ Sung
- **START:** Reset tất cả servo về vị trí mặc định (90°)
- **SELECT:** Chưa được lập trình

## 🔄 Quy Trình Làm Việc

1. **Khởi Động:**
   ```
   - Cấp điện cho ESP32
   - Bật Bluetooth module
   - Mở Dabble App
   - Kết nối Bluetooth
   ```

2. **Điều Khiển:**
   - Di chuyển robot bằng nút D-Pad (trái)
   - Gắp thả vật bằng Triangle/Cross (phải)
   - Nâng hạ cánh tay bằng Square/Circle

3. **An Toàn:**
   - Nếu không nhận lệnh trong 500ms, robot sẽ tự dừng
   - Nhấn START để reset servo

## 🔧 Cách Hiệu Chỉnh

### Điều Chỉnh Góc Gripper
Sửa trong [src/main.cpp](src/main.cpp):
```cpp
// 90° = mở, 0° = đóng
// Thay đổi giá trị này để điều chỉnh độ gắp
```

### Điều Chỉnh Tốc Độ Motor
```cpp
moveForward(220);   // Thay 220 bằng giá trị khác (0-255)
turnLeft(200);      // Thay 200 để điều chỉnh tốc độ rẽ
```

### Điều Chỉnh Độ Cao Arm
```cpp
armPosition = constrain(armPosition + 5, 0, 180);
// Thay 5 bằng giá trị khác để tăng/giảm độ thay đổi
```

## 📊 Giám Sát Serial

Để xem log debug:
```bash
platformio device monitor --port COM3 --baud 115200
```

Sẽ hiển thị các thông điệp như:
```
[MOVE] ⬆️ Tiến
[GRIPPER] ▲ 🔒 Gắp vật
[ARM] □ ⬆️ Nâng cánh tay
```

## ⚠️ Khắc Phục Sự Cố

### Robot không di chuyển
- Kiểm tra kết nối Bluetooth
- Kiểm tra pin motor có điện không
- Kiểm tra các chân GPIO

### Servo không hoạt động
- Kiểm tra chân PWM có kết nối đúng không
- Kiểm tra servo có điện không
- Kiểm tra giá trị góc có nằm trong khoảng 0-180 không

### Dabble không kết nối
- Kiểm tra baud rate Bluetooth (9600)
- Kiểm tra chân RX=16, TX=17
- Cài đặt lại module Bluetooth

## 📝 Ghi Chú

- **Chế độ:** Chỉ hỗ trợ chế độ thủ công qua Dabble
- **Timeout:** 500ms không nhận lệnh = tự dừng
- **Servo:** Mức PWM từ 500µs đến 2500µs (0-180°)

---

Tác giả: Robot Control System v2.0
Ngày: 2024
