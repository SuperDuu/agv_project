# ESP32 Sensor Hub & Firebase Gateway

Thư mục này chứa mã nguồn Arduino C++ cho mạch **ESP32**, đóng vai trò là một "Bộ não phụ" cực kỳ mạnh mẽ. Nó vừa làm trạm trung chuyển (Gateway) dữ liệu với Firebase RTDB, vừa đóng vai trò là **Sensor Hub** đọc các cảm biến I2C (IMU BNO055 và Siêu âm VL53L5CX) rồi truyền về vi điều khiển trung tâm STM32.

---

## 1. Tính Năng Nổi Bật

### 🌐 Kiến trúc Đa luồng (FreeRTOS Dual-Core)
Để giải quyết bài toán nút thắt cổ chai khi kết nối mạng, phần mềm được chia làm 2 luồng độc lập:
- **Core 0 (Task riêng biệt):** Chuyên trách xử lý kết nối WiFi và giao tiếp HTTP với Firebase. Hiện tượng "Lag" mạng sẽ không bao giờ làm treo hệ thống.
- **Core 1 (Loop chính):** Chuyên trách xử lý giao tiếp RS485 tốc độ siêu cao với STM32 và đọc các cảm biến I2C (BNO055, VL53L5CX) theo thời gian thực.

### ⚙️ Thư viện Cảm biến Tích hợp (Local Libraries)
- **BNO055 (IMU):** Bộ thư viện C++ được port trực tiếp từ STM32 sang, sử dụng chuẩn `Wire.h` của Arduino.
- **VL53L5CX (Siêu âm ToF):** Bộ thư viện cao cấp của SparkFun với 64 vùng quét (8x8). Đã được tinh chỉnh `I2C_BUFFER_SIZE = 128` riêng cho ESP32 để giảm thời gian nạp 90KB Firmware từ 2.5 giây xuống còn **0.6 giây**! Toàn bộ source code nằm sẵn trong thư mục `src/`.

### 📡 Giao thức Master-Slave RS485 (9 Bytes)
Sử dụng chuẩn gói tin nhị phân Custom Protocol để giao tiếp với STM32:
- **STM32 -> ESP32:** `[0xAA] [0x55] [0x63] [0x01] [Node_MSB] [Node_LSB] [Checksum]`
- **ESP32 -> STM32:** `[0xAA] [0x55] [0x63] [0x01] [Yaw_H] [Yaw_L] [Mã_Node] [Mã_H] [Checksum]`
Trong đó `Mã_Node` và `Mã_H` là các lệnh động được ESP32 bóc tách từ chuỗi `NxxHyy` trên Firebase để ra lệnh cho STM32 sinh quỹ đạo mới.

### 🌐 Web Server Cấu Hình (Smart Config)
Tự động phát WiFi `AGV_Config` nếu mất mạng. Thông qua IP `192.168.4.1`, người dùng có thể cấu hình WiFi và Firebase dễ dàng.

---

## 2. Sơ Đồ Đấu Nối (Wiring Guide)

Sử dụng cổng **Hardware Serial 2** (UART2) của ESP32 để giao tiếp RS485 với STM32 (Baudrate: `115200`):

| Chân ESP32 | Chức năng | Ghi chú |
|------------|-----------|---------|
| **GPIO 17 (TX2)** | TX của Max485 | Gắn vào chân DI của IC Max485 |
| **GPIO 16 (RX2)** | RX của Max485 | Gắn vào chân RO của IC Max485 |
| **GPIO 21 (SDA)** | I2C SDA | Nối song song cảm biến BNO055 và VL53L5CX |
| **GPIO 22 (SCL)** | I2C SCL | Nối song song cảm biến BNO055 và VL53L5CX |
| **GND** | GND chung | Bắt buộc phải nối chung Mass hệ thống |

---

## 3. Hướng Dẫn Cài Đặt Môi Trường (Build & Flash)

1. Mở phần mềm **Arduino IDE**.
2. Cài đặt thư viện giao tiếp Firebase:
   - Vào **Sketch** -> **Include Library** -> **Manage Libraries...**
   - Tìm từ khóa `Firebase ESP32 Client`.
   - Cài đặt thư viện của tác giả **Mobizt** (Phiên bản mới nhất).
3. Đảm bảo cấu hình Web dùng **API Key** thay vì Secret Token cũ.
4. Bấm nút **Upload** để nạp code. 
*(Các thư viện cảm biến đã được gói gọn trong thư mục `src/`, trình biên dịch sẽ tự động gom chúng lại, bạn không cần cài thêm thư viện cảm biến nào khác!)*
