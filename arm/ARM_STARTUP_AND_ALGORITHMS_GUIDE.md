# HƯỚNG DẪN KHỞI ĐỘNG, KIẾN TRÚC CODE VÀ THUẬT TOÁN HỆ THỐNG CÁNH TAY ROBOT KÉP
### *(Dual-Arm System Startup, Code Architecture & Algorithms Guide)*

Tài liệu này cung cấp thông tin chi tiết từ quy trình khởi động hệ thống hai cánh tay robot kép (Left & Right 6-DOF Arms) cho đến thiết kế phần mềm, cách biên dịch (Build), chạy ứng dụng (Run) trên máy tính bằng NetBeans, cấu trúc truyền thông và các thuật toán điều khiển vòng kín chạy trong firmware. Tài liệu được thiết kế trực quan, dễ hiểu cho cả quản lý (sếp), kỹ sư và nhân viên vận hành.

---

## PHẦN I: KIẾN TRÚC HỆ THỐNG & TỔ CHỨC CODE

### 1. Sơ Đồ Kết Nối Hệ Thống (System Topology)
Hệ thống gồm hai cánh tay robot 6 trục đối xứng (Left & Right) được điều khiển độc lập bởi 2 bo mạch xử lý STM32F446VET6 riêng biệt (Arm Slave). Cả hai mạch cùng kết nối chung vào một đường truyền vật lý RS485 dạng bus, nối về máy tính PC thông qua duy nhất một bộ chuyển đổi USB-to-RS485.

```mermaid
flowchart TD
    PC_App[Java PC App / GUI] <-->|Serial COM - USB-to-RS485| RS485_Bus{Bus Truyền Thông RS485}
    RS485_Bus <-->|Nhận gói L:xxx| Arm_L[STM32F4 Left Arm]
    RS485_Bus <-->|Nhận gói R:xxx| Arm_R[STM32F4 Right Arm]
    PC_App <-->|UDP Port 5010/5011| MoveIt[ROS2 MoveIt 2 Docker]
```

* **Phân biệt lệnh**: Cả hai tay robot đều nghe thấy tất cả các dữ liệu truyền từ PC. Tuy nhiên, cánh tay bên trái chỉ thực thi các gói tin có tiền tố `L:`, cánh tay bên phải chỉ thực thi các gói tin có tiền tố `R:`.

---

### 2. Tổ Chức Thư Mục Mã Nguồn (Firmware Directory)
Mã nguồn điều khiển cánh tay robot nằm trong hai thư mục `arm_firmware_left` (cánh tay trái) và `arm_firmware_right` (cánh tay phải). Cả hai dự án có cấu trúc mô-đun tương đương nhau:

```
arm/
├── arm_firmware_left/ & arm_firmware_right/
│   └── Core/Src/
│       ├── main.c           : Khởi chạy ngắt nhận UART, phân tích cú pháp chuỗi điều khiển (Parser).
│       ├── joint_control.c  : Thuật toán điều khiển PID vòng kín Cascade (Vị trí & Vận tốc).
│       ├── servo.c          : Driver điều chế xung PWM (500us - 2500us), quy đổi góc qua tỷ số truyền.
│       ├── encoder.c        : Đọc phản hồi xung góc khớp thực tế từ Encoder.
│       └── pid.c            : Lớp thuật toán PID tổng quát cho vòng lặp điều khiển.
```

---

## PHẦN II: CHI TIẾT CÁC THUẬT TOÁN ĐIỀU KHIỂN & ĐỊNH VẠNG TRUYỀN THÔNG

### 1. Thuật Toán Điều Khiển Vòng Kín Cascade PID (Cascade Position-Velocity Control)
Để đảm bảo cánh tay di chuyển êm ái, bám sát quỹ đạo mà không bị rung giật hoặc quá dòng động cơ, chúng ta áp dụng thuật toán **Cascade PID** (Vòng điều khiển xếp chồng lồng kép) trong `joint_control.c` ở tần số 100Hz ($\Delta t = 0.01\text{s}$):

```mermaid
flowchart LR
    TargetPos[Vị trí Mục tiêu] -->|Vòng Vị trí PID_pos| PID_Pos[PID Vị trí]
    PID_Pos -->|Vận tốc Mục tiêu| PID_Vel[PID Vận tốc]
    FeedbackPos[Encoder Ticks] --> PID_Pos
    FeedbackVel[Vận tốc Thực tế] --> PID_Vel
    PID_Vel -->|Tốc độ thay đổi Servo| Integrator[Tích phân Góc]
    Integrator -->|Xung PWM| Servo[Động cơ Servo]
```

#### Luồng xử lý chi tiết cho mỗi khớp (Joint):
1. **Đọc phản hồi vị trí**: Hàm `Encoder_Get_Ticks()` đọc giá trị hiện tại của Encoder (đơn vị: Ticks).
2. **Tính toán vận tốc thực tế**: Sử dụng sai phân vị trí chia cho thời gian lấy mẫu $\Delta t$:
   $$Vel_{\text{thực}} = \frac{Pos_{\text{hiện\_tại}} - Pos_{\text{trước\_đó}}}{\Delta t}$$
3. **Vòng điều khiển Vị trí (Vòng ngoài)**: So sánh Vị trí mục tiêu và Vị trí thực tế. Đầu ra của vòng ngoài là Vận tốc mục tiêu ($Vel_{\text{mục\_tiêu}}$):
   $$Vel_{\text{mục\_tiêu}} = K_{p\_pos} \times (Pos_{\text{mục\_tiêu}} - Pos_{\text{thực}})$$
4. **Vòng điều khiển Vận tốc (Vòng trong)**: So sánh Vận tốc mục tiêu với Vận tốc thực tế. Đầu ra là tốc độ thay đổi góc của servo ($Rate_{\text{servo}}$):
   $$Rate_{\text{servo}} = K_{p\_vel} \times (Vel_{\text{mục\_tiêu}} - Vel_{\text{thực}}) + K_{i\_vel} \times \int (Vel_{\text{mục\_tiêu}} - Vel_{\text{thực}}) \, dt$$
5. **Tích phân đầu ra**: Lệnh điều khiển góc servo được tích lũy liên tục để tạo chuyển động mượt mà:
   $$\theta_{\text{servo\_mới}} = \theta_{\text{servo\_cũ}} + Rate_{\text{servo}} \times \Delta t$$
6. **Chống quá đà (Saturation Clamping)**: Góc lệnh điều khiển được giới hạn chặt chẽ theo góc quay vật lý và cấu trúc cơ học của từng khớp (ví dụ khớp có hộp số được chặn từ $0^\circ$ đến $192.86^\circ$ để tránh gãy cơ cấu).

---

### 2. Thuật Toán Phân Tích Chuỗi UART & XOR Checksum (Communication Protocol)
PC gửi lệnh điều khiển xuống dưới dạng chuỗi Text (ASCII) kết thúc bằng ký tự ngắt dòng `\n` hoặc `\r` để dễ dàng gỡ lỗi và chống nhiễu đường truyền.

#### Định dạng khung truyền:
`[Tiền_tố][dq0],[dq1],[dq2],[dq3],[dq4],[dq5]*[XOR_Checksum]\n`
* `Tiền_tố`: `R:` hoặc `L:`
* `dq0` đến `dq5`: Góc khớp mục tiêu tương đối nhân với 100 (`độ * 100`) để loại bỏ dấu phẩy số thực giúp truyền tải nhanh hơn.
* `*`: Ký tự phân tách mã checksum.
* `XOR_Checksum`: 2 ký tự mã Hex viết hoa.

#### Thuật toán tính toán Checksum (XOR Logic):
Mã Checksum là kết quả phép XOR từng byte của chuỗi ký tự đứng trước dấu `*`.
```java
// Phía PC (Java App) sinh Checksum:
private String addChecksum(String str) {
    int sum = 0;
    for (int i = 0; i < str.length(); i++) {
        sum ^= str.charAt(i); // XOR từng ký tự
    }
    return str + "*" + String.format("%02X", sum & 0xFF) + "\n";
}
```
Tại vi điều khiển STM32F4, hàm `HAL_UART_RxCpltCallback` nhận từng ký tự. Khi gặp `\n` hoặc `\r`, nó dừng nhận, tính toán XOR chuỗi ký tự nhận được, so sánh với mã mã Hex sau dấu `*`. Nếu trùng khớp 100% mới tiến hành bóc tách góc và thực thi điều khiển, loại bỏ hoàn toàn các khung truyền bị nhiễu điện áp trên đường truyền RS485.

#### Ánh xạ góc tương đối từ PC sang góc tuyệt đối của Servo:
Do cơ cấu lắp ráp góc của Servo thực tế bị lệch so với mô hình động học Robot, firmware thực hiện ánh xạ tuyến tính trong ngắt nhận:
* Khớp 0: $\theta_{\text{servo}} = -\theta_{\text{khớp}} + 96.43^\circ$
* Khớp 1: $\theta_{\text{servo}} = -\theta_{\text{khớp}} + 90.00^\circ$
* Khớp 2: $\theta_{\text{servo}} = \theta_{\text{khớp}} + 35.00^\circ$
* Khớp 3: $\theta_{\text{servo}} = 65.00^\circ - \theta_{\text{khớp}}$
* Khớp 4: $\theta_{\text{servo}} = -\theta_{\text{khớp}} + 90.00^\circ$
* Khớp 5: $\theta_{\text{servo}} = \theta_{\text{khớp}}$

---

### 3. Thuật Toán Tỷ Lệ Hộp Số Cơ Khí (Gearbox Scaling)
Để nhân mô-men xoắn giúp cánh tay khỏe hơn, các khớp chuyển động sử dụng các bộ truyền bánh răng giảm tốc có tỷ số truyền cơ học đặc biệt. Firmware trong `servo.c` tự động tính toán bù tỷ số truyền trước khi xuất xung PWM:
* **Khớp 0, 1, 2**: Sử dụng hộp số **tỷ số truyền 5:7** (Servo cần xoay 7 độ để khớp xoay 5 độ).
  $$\theta_{\text{servo\_thực}} = \theta_{\text{khớp}} \times \frac{7.0}{5.0}$$
* **Khớp 4, 6**: Sử dụng hộp số **tỷ số truyền 2:3** (Servo cần xoay 3 độ để khớp xoay 2 độ).
  $$\theta_{\text{servo\_thực}} = \theta_{\text{khớp}} \times \frac{3.0}{2.0}$$
* **Khớp 3, 5**: Tỷ lệ 1:1, không đổi góc.

Sau khi quy đổi góc khớp ra góc quay vật lý thực tế của trục Servo, driver ánh xạ tuyến tính sang độ rộng xung PWM cấp cho Servo (500us tương ứng $0^\circ$ và 2500us tương ứng góc tối đa $180^\circ$ hoặc $270^\circ$).

---

### 4. Thuật Thuật Toán Kiểm Soát Lực Kẹp Vật Thể (Force Control Gripper)
Khớp kẹp vật (Gripper) sử dụng động cơ có tích hợp cảm biến dòng điện thông qua ADC 12-bit trên vi điều khiển STM32F4:
1. Khi nhận lệnh đóng kẹp `"R:GRIP"` hoặc `"L:GRIP"`, động cơ kẹp bắt đầu chạy khép lại.
2. Vi điều khiển liên tục đọc giá trị dòng điện động cơ thông qua bộ ADC.
3. Khi ngón kẹp chạm vào vật thể, dòng điện động cơ sẽ tăng vọt do bị kẹt cơ học.
4. Ngay khi giá trị dòng điện ADC vượt quá ngưỡng an toàn được lập trình, hệ thống lập tức dừng động cơ kẹp và chuyển sang trạng thái duy trì lực (Force Hold), tránh làm móp méo hỏng vật thể hoặc cháy cuộn dây của động cơ kẹp.

---

## PHẦN III: HƯỚNG DẪN BIÊN DỊCH VÀ CHẠY JAVA PC APP

Ứng dụng PC điều khiển cánh tay là một project Java Swing thuần túy chạy trên cơ chế Ant Build, đã được tích hợp đầy đủ file thiết lập để mở trực tiếp trong **NetBeans IDE**.

### 1. Chuẩn Bị Môi Trường
* **JDK (Java Development Kit)**: Cần cài đặt **JDK 17** hoặc mới hơn.
* **IDE**: Khuyên dùng **Apache NetBeans IDE (phiên bản 17 trở lên)**.
* **Python (Cho tay cầm PS5)**: Python 3 và thư viện `pygame` (cài đặt qua pip).

---

### 2. Hướng Dẫn Sử Dụng NetBeans IDE (Khuyên Dùng)

#### Bước 1: Mở Project trong NetBeans
1. Khởi động **NetBeans IDE**.
2. Trên thanh menu, chọn `File` -> `Open Project...` (hoặc phím tắt `Ctrl + Shift + O`).
3. Điều hướng tới đường dẫn: `/home/du/Desktop/agv_project/arm/java_app/arm`.
4. NetBeans sẽ tự động nhận diện thư mục này là một project Java hợp lệ (hiển thị biểu tượng cốc cà phê Java kèm tên `arm`). Nhấn **Open Project**.

#### Bước 2: Quản lý Thư Viện (Libraries)
* NetBeans tự động nhận diện thư viện giao tiếp nối tiếp nằm trong thư mục `lib/jSerialComm-2.10.4.jar`. Không cần cấu hình tay.

#### Bước 3: Biên Dịch (Clean and Build)
* Để biên dịch toàn bộ mã nguồn Java:
  * Click chuột phải vào tên project `arm` ở cây thư mục bên trái -> Chọn **Clean and Build** (hoặc click vào biểu tượng Cái Búa và Cái Chổi trên thanh công cụ).
  * NetBeans sẽ biên dịch các file `.java` từ thư mục `src` sang file `.class` trong `build/classes`, đồng thời đóng gói thành một file JAR hoàn chỉnh đặt tại thư mục `dist/arm.jar`.

#### Bước 4: Chạy Ứng Dụng (Run Project)
* Click chuột phải vào project `arm` -> Chọn **Run** (hoặc click vào nút **Play màu xanh** trên thanh công cụ).
* NetBeans sẽ khởi chạy class chính `app.App` và hiển thị giao diện đồ họa điều khiển 2 cánh tay robot.

---

### 3. Hướng Dẫn Chạy Bằng Script Dòng Lệnh (Không Cần NetBeans)

Nếu không sử dụng NetBeans, bạn có thể tự biên dịch và chạy bằng các file script đã viết sẵn:

#### Trên Ubuntu (Linux):
Mở terminal tại thư mục `/home/du/Desktop/agv_project/arm/java_app/arm` và chạy:
```bash
# 1. Tạo môi trường ảo Python và cài đặt pygame cho PS5 controller
python3 -m venv .venv
source .venv/bin/activate
pip install -r scripts/requirements.txt

# 2. Chạy ứng dụng Java (script tự biên dịch và khởi chạy)
./run.sh
```

#### Trên Windows:
Mở Command Prompt (cmd) tại thư mục `arm\java_app\arm` và chạy:
```cmd
:: 1. Cài đặt thư viện Python hỗ trợ tay cầm
python -m pip install -r scripts\requirements.txt

:: 2. Chạy script Java
run.bat
```

---

### 4. Biên Dịch Bộ Giải Động Học Ngược C++ (JNI Solver - Tùy Chọn)
Để tăng tốc độ tính toán động học ngược (IK) bằng mã C++ hiệu năng cao thay cho thuật toán chạy trong Java:
1. Đảm bảo máy tính đã cài đặt trình biên dịch C++ (`g++` trên Linux hoặc MSVC/MinGW trên Windows) và biến môi trường `JAVA_HOME` đã trỏ đúng tới JDK 17.
2. Chạy file script build JNI trong thư mục dự án:
   * **Ubuntu**: `python3 cpp_jni/build_jni.py`
   * **Windows**: `python cpp_jni\build_jni.py`
3. Quá trình biên dịch sẽ tạo ra file thư viện động:
   * `lib/libkinematics_jni.so` (trên Ubuntu) hoặc `lib\kinematics_jni.dll` (trên Windows).
4. Khi phát hiện các file này trong thư mục `lib`, ứng dụng Java sẽ tự động kích hoạt bộ giải thuật động học JNI tốc độ cao.

---

## PHẦN IV: QUY TRÌNH VẬN HÀNH CHẠY THỬ THỰC TẾ

### Bước 1: Kết Nối Phần Cứng
1. Kết nối bộ nguồn DC 12V-24V cấp điện cho hệ thống hai cánh tay robot và mạch driver logic.
2. Cắm cáp truyền thông RS485 của cả hai mạch cánh tay vào đầu chuyển USB-to-RS485.
3. Cắm đầu USB-to-RS485 vào máy tính PC điều khiển.

### Bước 2: Khởi Chạy Kết Nối
1. Khởi động ứng dụng PC qua NetBeans IDE (Run) hoặc chạy trực tiếp bằng file `./run.sh` / `run.bat`.
2. Trên GUI của ứng dụng: Chọn đúng cổng COM của USB-to-RS485, baudrate **115200**, rồi nhấn **Connect**.

### Bước 3: Đưa Cánh Tay Về Vị Trí Home
1. Trên giao diện, thiết lập tất cả các thanh trượt góc khớp 6 trục của hai tay về giá trị `0` độ.
2. Cánh tay sẽ tự động di chuyển về tư thế Home cơ học (tư thế thẳng đứng). Kiểm tra sự ăn khớp răng để đảm bảo cánh tay thẳng đứng hoàn toàn.

### Bước 4: Điều Khiển Quỹ Đạo
* **Điều khiển thủ công**: Kéo các thanh trượt góc khớp (Forward Kinematics - FK) hoặc nhập tọa độ gắp thả XYZ (Inverse Kinematics - IK) trực tiếp trên GUI của Java App.
* **Điều khiển qua tay cầm PS5**: Khởi chạy script Python chuyển tiếp tín hiệu tay cầm, cắm tay cầm PS5 vào cổng USB của PC để điều khiển cánh tay gắp nhả vật tư trực quan.
* **Tự động hóa qua ROS2**: Khởi động Docker Compose chứa ROS2 MoveIt 2 để truyền nhận quỹ đạo không va chạm qua UDP cục bộ về Java App.

---

## PHẦN V: CƠ CHẾ BẢO VỆ & AN TOÀN TRONG CODE
1. **Chặn biên mềm (Software Clamping)**: Trong file `joint_control.c` và `servo.c`, tất cả các lệnh góc đều được chạy qua bộ lọc giới hạn tối đa/tối thiểu để chống xung đột phần cứng gây hỏng động cơ hoặc gãy cơ học cánh tay.
2. **Khóa dòng kẹp Gripper**: Thuật toán kiểm soát lực kẹp tự động ngắt PWM và khóa cơ ở chế độ Force Hold khi ADC dòng điện quá tải để bảo vệ động cơ kẹp không bị cháy.
3. **Mất kết nối khẩn cấp**: Nếu mạch cánh tay không nhận được dữ liệu điều khiển từ PC qua cổng UART sau 1 khoảng thời gian timeout nhất định, nó tự động ngắt xung kích để xả các động cơ servo về trạng thái không tải (Idle), giảm thiểu rủi ro va chạm mất kiểm soát.
