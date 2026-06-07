#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <FirebaseESP32.h> // Cài đặt thư viện "Firebase ESP32 Client" của Mobizt

// ==========================================
// CẤU HÌNH PHẦN CỨNG & MẶC ĐỊNH
// ==========================================
#define BOOT_BUTTON_PIN 0 // Chân nút BOOT trên ESP32 (Thường là GPIO 0)
#define AP_SSID "AGV_Config"
#define AP_PASS "12345678"

// Sử dụng Hardware Serial 2 của ESP32 để giao tiếp STM32
#define RXD2 16
#define TXD2 17
#define UART_BAUDRATE 115200

#define RS485_ADDR 0x63  // Địa chỉ 99 cho Master-Slave IMU

// ==========================================
// KHAI BÁO BIẾN TOÀN CỤC
// ==========================================
Preferences preferences; // Thư viện NVS thay thế EEPROM
WebServer server(80);    // Khởi tạo WebServer ở cổng 80

TaskHandle_t FirebaseTaskHandle; // Task đa luồng cho Firebase

// Biến lưu trữ cấu hình mạng (Sẽ được nạp từ bộ nhớ)
String wifi_ssid = "";
String wifi_pass = "";
String fb_host = "";
String fb_auth = "";

// Biến Firebase
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

String currentCommand = ""; 
bool isAckReceived = true;  
unsigned long lastSendTime = 0; 
unsigned long lastCheckTime = 0;
const int FIREBASE_CHECK_INTERVAL = 2000; 
bool isConfigMode = false; // Cờ kiểm tra đang ở chế độ Web hay chế độ Chạy

// Biến cho giao thức Master-Slave IMU (RS485)
uint16_t current_node = 0xFFFF;
float current_yaw = 0.0f; // Góc mô phỏng BNO055
uint8_t rs485_rx_buffer[10];
uint8_t rs485_rx_index = 0;
unsigned long last_rs485_rx_time = 0;

// Hàm tính XOR Checksum cho khung nhị phân
uint8_t calculate_checksum(uint8_t *data, uint8_t start, uint8_t end) {
  uint8_t cs = 0;
  for (uint8_t i = start; i <= end; i++) cs ^= data[i];
  return cs;
}

// Xử lý gói tin nhị phân nhận từ STM32
void parse_rs485_frame() {
  if (rs485_rx_index == 7) {
    if (rs485_rx_buffer[0] == 0xAA && rs485_rx_buffer[1] == 0x55) {
      if (rs485_rx_buffer[2] == RS485_ADDR) {
        if (calculate_checksum(rs485_rx_buffer, 2, 5) == rs485_rx_buffer[6]) {
          // STM32 yêu cầu lấy Yaw và gửi Node hiện tại
          if (rs485_rx_buffer[3] == 0x01) {
            current_node = (rs485_rx_buffer[4] << 8) | rs485_rx_buffer[5];
            
            // TODO: Ở đây bạn đọc góc thực tế từ BNO055
            current_yaw += 0.5f; // Chạy giả lập góc
            if (current_yaw > 360.0f) current_yaw -= 360.0f;

            uint8_t tx_frame[7];
            tx_frame[0] = 0xAA; tx_frame[1] = 0x55;
            tx_frame[2] = RS485_ADDR; tx_frame[3] = 0x01;
            int16_t yaw_int = (int16_t)(current_yaw * 10.0f);
            tx_frame[4] = (yaw_int >> 8) & 0xFF;
            tx_frame[5] = yaw_int & 0xFF;
            tx_frame[6] = calculate_checksum(tx_frame, 2, 5);
            
            Serial2.write(tx_frame, 7);
          }
        }
      }
    }
  }
  rs485_rx_index = 0;
}

// ==========================================
// GIAO DIỆN WEB (HTML + CSS CƠ BẢN)
// ==========================================
// Dùng Raw String Literal (R"=====(...)=====") để viết HTML ngay trong code C++ mà không cần escape ký tự
const char* HTML_CONTENT = R"=====(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cấu Hình AGV Gateway</title>
<style>
  body { font-family: Arial, sans-serif; padding: 20px; background-color: #f0f2f5; }
  .card { background: white; padding: 25px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); max-width: 400px; margin: auto; }
  h2 { text-align: center; color: #333; }
  label { font-weight: bold; font-size: 14px; display: block; margin-top: 15px; }
  input, select { width: 100%; padding: 10px; margin-top: 5px; border: 1px solid #ccc; border-radius: 5px; box-sizing: border-box; font-size: 16px; }
  button { width: 100%; padding: 12px; margin-top: 20px; background-color: #007bff; color: white; border: none; border-radius: 5px; font-size: 16px; font-weight: bold; cursor: pointer; transition: 0.3s; }
  button:hover { background-color: #0056b3; }
</style>
</head><body>
<div class="card">
  <h2>AGV Gateway Setup</h2>
  <form action="/save" method="POST">
    <label>Tên WiFi (Bấm để chọn):</label>
    <select name="ssid">%SSID_OPTIONS%</select>
    
    <label>Mật khẩu WiFi:</label>
    <input type="password" name="pass" value="%WIFI_PASS%" placeholder="Để trống nếu WiFi không có pass">
    
    <label>Firebase Host (Bỏ https:// và /):</label>
    <input type="text" name="fb_host" value="%FB_HOST%" required placeholder="VD: agv-test.firebaseio.com">
    
    <label>Firebase Auth (Secret Token):</label>
    <input type="text" name="fb_auth" value="%FB_AUTH%" required>
    
    <button type="submit">Lưu & Khởi Động Lại</button>
  </form>
</div>
</body></html>
)=====";

// ==========================================
// CÁC HÀM XỬ LÝ LƯU TRỮ (NVS PREFERENCES)
// ==========================================
void loadConfig() {
  preferences.begin("agv_config", true); // Tham số true = Read-only mode
  wifi_ssid = preferences.getString("ssid", "");
  wifi_pass = preferences.getString("pass", "");
  fb_host = preferences.getString("fb_host", "");
  fb_auth = preferences.getString("fb_auth", "");
  preferences.end();
}

void saveConfig(String s, String p, String h, String a) {
  preferences.begin("agv_config", false); // Tham số false = Read/Write mode
  preferences.putString("ssid", s);
  preferences.putString("pass", p);
  preferences.putString("fb_host", h);
  preferences.putString("fb_auth", a);
  preferences.end();
}

// ==========================================
// CÁC HÀM XỬ LÝ WEBSERVER
// ==========================================
void handleRoot() {
  String html = HTML_CONTENT;
  
  // Quét danh sách WiFi xung quanh
  int n = WiFi.scanNetworks();
  String options = "";
  if (n == 0) {
    options = "<option value=''>Không tìm thấy WiFi nào!</option>";
  } else {
    for (int i = 0; i < n; ++i) {
      String ssidName = WiFi.SSID(i);
      options += "<option value='" + ssidName + "'";
      if (ssidName == wifi_ssid) options += " selected"; // Bôi đen sẵn WiFi cũ nếu trùng
      options += ">" + ssidName + " (" + String(WiFi.RSSI(i)) + "dBm)</option>";
    }
  }
  
  // Trộn dữ liệu vào HTML (Template Engine cơ bản)
  html.replace("%SSID_OPTIONS%", options);
  html.replace("%WIFI_PASS%", wifi_pass);
  html.replace("%FB_HOST%", fb_host);
  html.replace("%FB_AUTH%", fb_auth);
  
  server.send(200, "text/html", html);
}

void handleSave() {
  String new_ssid = server.arg("ssid");
  String new_pass = server.arg("pass");
  String new_host = server.arg("fb_host");
  String new_auth = server.arg("fb_auth");
  
  // Lưu vào bộ nhớ Flash
  saveConfig(new_ssid, new_pass, new_host, new_auth);
  
  // Phản hồi cho trình duyệt
  String successPage = "<html><body style='font-family:Arial;text-align:center;margin-top:50px;'>"
                       "<h2>Đã lưu cấu hình thành công!</h2>"
                       "<p>Mạch ESP32 đang khởi động lại. Vui lòng tắt WiFi này và chờ xe AGV kết nối mạng.</p>"
                       "</body></html>";
  server.send(200, "text/html", successPage);
  
  // Đợi 2 giây cho web kịp load rồi reset board
  delay(2000);
  ESP.restart();
}

void enterConfigMode() {
  isConfigMode = true;
  Serial.println("\n>>> VAO CHE DO CAU HINH WIFI & FIREBASE <<<");
  
  // Cài đặt làm Trạm phát (Access Point)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  Serial.print("Da phat WiFi ten: "); Serial.println(AP_SSID);
  Serial.print("Mat khau WiFi   : "); Serial.println(AP_PASS);
  Serial.print("Truy cap Web qua IP: "); Serial.println(WiFi.softAPIP());
  
  // Khai báo đường dẫn Web
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
}

// ==========================================
// TASK ĐA LUỒNG: CHẠY FIREBASE TRÊN CORE 0
// ==========================================
void FirebaseTask(void *pvParameters) {
  for (;;) {
    if (!isConfigMode && WiFi.status() == WL_CONNECTED) {
      if (millis() - lastCheckTime >= FIREBASE_CHECK_INTERVAL) {
        lastCheckTime = millis();

        if (Firebase.ready()) {
          if (Firebase.getString(firebaseData, "/AGV_01/TargetCommand")) {
            if (firebaseData.dataType() == "string") {
              String newCommand = firebaseData.stringData();
              if (newCommand != currentCommand && newCommand != "") {
                currentCommand = newCommand;
                isAckReceived = false;
                Serial.print("Nhan duoc lenh moi tu Firebase: ");
                Serial.println(currentCommand);
              }
            }
          }
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Nhường CPU 10ms để Watchdog không reset
  }
}

// ==========================================
// CHƯƠNG TRÌNH CHÍNH
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, RXD2, TXD2);
  
  // Kích hoạt điện trở nội kéo lên (Pull-up) cho nút BOOT
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  delay(100); 

  // Lấy dữ liệu cũ từ bộ nhớ
  loadConfig();
  
  // KIỂM TRA NÚT BOOT: 
  // Nếu đè nút BOOT lúc cấp nguồn -> Trạng thái chân 0 sẽ là LOW
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    Serial.println("Phat hien nut BOOT duoc nhan giu!");
    enterConfigMode();
  } else {
    // Nếu không ấn nút BOOT, kiểm tra xem đã từng cài WiFi chưa
    if (wifi_ssid == "") {
      Serial.println("Chua co thong tin WiFi (Mach moi). Tu dong vao che do Setup!");
      enterConfigMode();
    } else {
      Serial.print("Dang ket noi WiFi: "); Serial.println(wifi_ssid);
      
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
      
      unsigned long startAttemptTime = millis();
      // Thử kết nối trong 15 giây (Timeout)
      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
        Serial.print(".");
        delay(500);
      }
      
      // AUTO-FALLBACK: Nếu rớt mạng hoặc pass sai -> Tự bật Web Config
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nLoi: Khong the ket noi WiFi (Sai Pass hoac Mat mang)! Tu dong chuyen sang che do cau hinh.");
        enterConfigMode();
      } else {
        Serial.println("\nWiFi da ket noi thanh cong!");
        Serial.print("IP Address: "); Serial.println(WiFi.localIP());
        
        // 4. BẮT ĐẦU CHẠY FIREBASE
        config.host = fb_host;
        config.signer.tokens.legacy_token = fb_auth.c_str(); // Cần ép kiểu String sang const char*
        
        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true); // Tự động reconnect nếu rớt mạng trong lúc đang chạy
      }
    }
  }

  // Khởi tạo Task Firebase chạy riêng biệt trên Core 0 (Core 1 để dành cho Loop chính xử lý RS485)
  xTaskCreatePinnedToCore(
    FirebaseTask,   // Hàm chạy Task
    "FirebaseTask", // Tên Task
    8192,           // Kích thước Stack (Firebase cần nhiều RAM)
    NULL,           // Tham số
    1,              // Độ ưu tiên
    &FirebaseTaskHandle, 
    0               // Chạy trên Core 0 (Core xử lý WiFi mặc định)
  );
}

void loop() {
  // NẾU ĐANG Ở CHẾ ĐỘ CẤU HÌNH -> CHỈ CHẠY WEBSERVER
  if (isConfigMode) {
    server.handleClient();
    return; // Dừng lại ở đây
  }

  // ==========================================
  // LOGIC RS485 & FIREBASE COMMAND
  // ==========================================
  // Đoạn đọc Firebase đã được chuyển sang Core 0 (FirebaseTask) để không làm kẹt vòng lặp này!

  // CƠ CHẾ GỬI LẠI UART (CHỜ ACK TỪ STM32)
  if (!isAckReceived && (millis() - lastSendTime >= 1000)) {
    lastSendTime = millis();
    Serial2.print(currentCommand);
    Serial2.print("\n");
    Serial.println("Dang gui lenh xuong STM32... (Cho ACK)");
  }

  // ==========================================
  // XỬ LÝ NHẬN DATA RS485 (MASTER-SLAVE + FIREBASE ACK)
  // ==========================================
  // ĐỌC PHẢN HỒI TỪ STM32 (Dạng chuỗi ASCII hoặc Khung nhị phân)
  while (Serial2.available() > 0) {
    // Để tránh kẹt vòng lặp do dùng readStringUntil, chúng ta đọc từng byte
    // và phân biệt đâu là chuỗi ACK, đâu là gói tin nhị phân 0xAA 0x55.
    
    uint8_t b = Serial2.read();
    
    // Xử lý gói nhị phân (Polling 50ms)
    if (millis() - last_rs485_rx_time > 10) rs485_rx_index = 0;
    if (rs485_rx_index < sizeof(rs485_rx_buffer)) {
      rs485_rx_buffer[rs485_rx_index++] = b;
    }
    last_rs485_rx_time = millis();
    if (rs485_rx_index == 7) {
      parse_rs485_frame();
    }
    
    // (Tạm thời bỏ khối check ASCII "OK/ACK" vì nó sẽ xung đột với gói nhị phân)
    // Nếu bạn muốn giữ lại Firebase Command, nên gộp lệnh Firebase vào 
    // gói truyền nhị phân từ STM32 để đồng bộ 100%.
  }
}
