# ESP32 Firebase Gateway cho Hệ Thống AGV

Thư mục này chứa mã nguồn Arduino C++ cho mạch **ESP32**, đóng vai trò làm trạm trung chuyển (Gateway) dữ liệu không dây giữa nền tảng Cloud (Firebase RTDB) và vi điều khiển trung tâm STM32.

---

## 1. Tính Năng Nổi Bật

### Web Server Cấu Hình (Smart Config)
Không cần cắm cáp nạp lại code mỗi khi thay đổi mạng WiFi hoặc đổi dự án Firebase. Mạch ESP32 tự động cấp phát một giao diện Web Server nội bộ (Captive Portal) để người dùng cài đặt:
- **Kích hoạt thủ công:** Nhấn giữ nút **BOOT (GPIO 0)** khi vừa cấp nguồn cho ESP32 để ép mạch phát WiFi tên `AGV_Config`.
- **Kích hoạt tự động (Auto-Fallback):** Nếu mạch đang chạy bình thường nhưng WiFi cũ bị tắt, đổi mật khẩu hoặc mất sóng quá 15 giây, mạch sẽ tự động bật chế độ phát WiFi cấu hình.
- Các thông số (SSID, Password, Firebase Host, Secret Token) được lưu vĩnh viễn vào bộ nhớ **NVS (Non-Volatile Storage)** của ESP32 qua thư viện `Preferences.h`.

### Cơ Chế Bắt Tay UART (ACK Handshake)
Để đảm bảo tín hiệu truyền xuống STM32 không bao giờ bị mất trong môi trường nhiễu sóng từ trường của động cơ:
- Khi nhận lệnh mới từ Firebase (Ví dụ: `N03H2`), ESP32 sẽ nén thành chuỗi `N03H2\n` và gửi xuống STM32 qua cổng UART.
- ESP32 sẽ **liên tục gửi lại** lệnh này mỗi 1 giây cho đến khi STM32 đọc thành công và phản hồi ngược lại chữ `OK\n` hoặc `ACK\n`.
- Khi nhận được ACK, ESP32 sẽ dừng gửi và chuyển về trạng thái chờ lệnh mới từ mây.

---

## 2. Sơ Đồ Đấu Nối (Wiring Guide)

Sử dụng cổng **Hardware Serial 2** (UART2) của ESP32 để giao tiếp với STM32 (Baudrate: `115200`):

| Chân ESP32 | Kết nối với STM32 | Ghi chú |
|------------|--------------------|---------|
| **TX2 (GPIO 17)** | Chân **RX** của STM32 | Chân truyền tín hiệu đi |
| **RX2 (GPIO 16)** | Chân **TX** của STM32 | Chân nhận ACK báo về |
| **GND** | Chân **GND** của STM32 | Bắt buộc phải nối chung Mass |

---

## 3. Hướng Dẫn Sử Dụng Giao Diện Web

1. Khi ESP32 vào chế độ cấu hình, lấy điện thoại/laptop kết nối vào WiFi:
   - **Tên WiFi:** `AGV_Config`
   - **Mật khẩu:** `12345678`
2. Mở trình duyệt Web (Chrome/Safari) và truy cập vào địa chỉ IP: **`192.168.4.1`**
3. Một giao diện thẻ (Card UI) sẽ hiện ra:
   - Danh sách các mạng WiFi xung quanh sẽ được quét tự động, bạn chỉ việc chọn mạng nhà xưởng.
   - Nhập mật khẩu WiFi (Nếu có).
   - Nhập Host của Firebase (Bỏ chữ `https://` và dấu `/` ở cuối).
   - Nhập mã Secret Token (Auth).
4. Bấm **Lưu & Khởi Động Lại**. Mạch sẽ tự khởi động lại và kết nối vào hệ thống.

---

## 4. Hướng Dẫn Cài Đặt Môi Trường (Build & Flash)

1. Mở phần mềm **Arduino IDE**.
2. Cài đặt thư viện giao tiếp Firebase:
   - Vào **Sketch** -> **Include Library** -> **Manage Libraries...** (hoặc `Ctrl + Shift + I`).
   - Tìm từ khóa `Firebase ESP32 Client`.
   - Cài đặt thư viện của tác giả **Mobizt** (Phiên bản mới nhất).
3. Cắm cáp USB vào ESP32, chọn Board là **ESP32 Dev Module**.
4. Bấm nút **Upload** để nạp code. 
*(Lưu ý: Bạn chỉ cần nạp 1 lần duy nhất trong suốt vòng đời dự án. Sau này mọi thứ đều cấu hình qua Web).*
