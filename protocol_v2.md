# AGV/ARM Protocol V2.1

Protocol V2.1 là giao thức nhị phân dùng chung cho:

- `PC app -> HC12 -> ESP32 gateway`
- `ESP32 gateway -> AGV main (STM32)`
- `AGV main -> arm_left / arm_right`

Mục tiêu:

- một frame duy nhất cho tất cả node
- dễ route qua `main`
- dễ thêm ACK, timeout, retry khi cần
- không truyền `float` trên đường dây
- góc khớp dùng `int16 = degree * 100`
- **Little-Endian toàn bộ** (native cho STM32/ESP32)
- **LEN đặt trước CMD/SEQ** để DMA pre-allocate payload ngay lập tức
- **arm_id xóa khỏi payload** — DEST header là định danh duy nhất
- **max_delta_x100** trong mỗi frame joint command để bảo vệ cơ cấu hình thang

## 1. Frame format

Tất cả frame đều có dạng:

```text
+--------+--------+------+------+---------+---------+------+------+---------+---------+--------+--------+
| SOF1   | SOF2   | DEST | SRC  | LEN_L   | LEN_H   | CMD  | SEQ  | PAYLOAD | CRC_L   | CRC_H  |        |
+--------+--------+------+------+---------+---------+------+------+---------+---------+--------+--------+
| 0xAA   | 0x55   | 1B   | 1B   | 1B      | 1B      | 1B   | 1B   | N byte  | 1B      | 1B     |        |
+--------+--------+------+------+---------+---------+------+------+---------+---------+--------+--------+
```

Byte offset:

```text
[0]  SOF1   = 0xAA
[1]  SOF2   = 0x55
[2]  DEST
[3]  SRC
[4]  LEN_L  (byte thấp của payload_len, Little-Endian)
[5]  LEN_H  (byte cao của payload_len, Little-Endian)
[6]  CMD
[7]  SEQ
[8..8+N-1] PAYLOAD
[8+N]   CRC_L
[8+N+1] CRC_H
```

Số byte cố định trước payload: `8`

Số byte CRC: `2`

Tổng frame length:

```text
frame_len = 10 + payload_len
```

> **So với V2:** LEN được chuyển lên **trước** CMD và SEQ. Khi State Machine đọc được LEN tại index 6, nó cấu hình ngay DMA nhận payload mà không cần đợi thêm byte nào.

## 2. Byte order

**Tất cả** trường `int16`, `uint16`, `int32`, `uint32` trong payload và CRC16 đều dùng **Little-Endian**.

```text
value = 0x1234 -> [0x34, 0x12]
LEN   = 22     -> [0x16, 0x00]
CRC   = 0xABCD -> [0xCD, 0xAB]
```

Lý do chuyển sang Little-Endian:
- ARM Cortex-M (STM32) và Xtensa LX7 (ESP32) đều là LE native.
- Big-Endian (V2 cũ) buộc mọi node phải swap byte thủ công mỗi lần đọc — lãng phí chu kỳ ở 50 Hz.

Java: `ByteBuffer.order(ByteOrder.LITTLE_ENDIAN)` — không đổi logic, chỉ đổi order flag.

## 3. CRC

Dùng:

- `CRC-16/CCITT-FALSE`
- polynomial: `0x1021`
- init: `0xFFFF`
- refin: `false`
- refout: `false`
- xorout: `0x0000`

CRC tính trên các byte:

```text
DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD...
```

Tức là từ `frame[2]` đến `frame[8 + payload_len - 1]`, tổng `6 + payload_len` byte.

Không tính trên:

- `SOF1`
- `SOF2`
- hai byte `CRC_L`, `CRC_H`

## 4. Node address

```text
0x01 = AGV main
0x02 = Arm left
0x03 = Arm right
0x10 = ESP32 gateway
0x20 = PC app
0x7F = Broadcast
```

## 5. Command ID

```text
0x01 = Sensor report
0x10 = AGV command
0x11 = Sync request
0x20 = Arm joint command
0x21 = Arm gripper command
0x30 = ACK
0x31 = NACK
0x40 = Heartbeat
0x50 = AGV status report    (AGV main -> PC, chỉ trạng thái di chuyển)
0x51 = Arm status report    (Arm slave -> AGV main -> PC)
```

> **So với V2:** CMD 0x50 tách thành 2: `0x50` báo trạng thái AGV (node, lỗi, flag), `0x51` báo trạng thái Arm slave (6 khớp + gripper). Tránh payload quá tải khi cần báo cáo tổng hợp.

## 6. Sequence number

- `SEQ` tăng tuần tự `0 -> 255 -> 0`
- dùng để:
  - bỏ frame trùng
  - bỏ frame cũ
  - map với ACK/NACK

## 7. Joint angle encoding

Tất cả góc khớp dùng:

```text
int16_t angle_x100 = round(angle_deg * 100.0)
angle_deg = angle_x100 / 100.0
```

Ví dụ:

```text
35.12 deg -> 3512
-7.50 deg -> -750
0.00 deg  -> 0
```

Range `int16`:

```text
-327.68 deg -> +327.67 deg
```

Đủ cho toàn bộ khớp hiện tại.

## 8. Payload definitions

### 8.1 Sensor report (`CMD = 0x01`)

Hướng:

- `ESP32 gateway -> AGV main`

Payload (8 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| yaw_x100      | int16  | IMU yaw (deg * 100) LE             |
| obstacle_mm   | uint16 | Khoảng cách vật cản LE             |
| target_node   | uint16 | Node đích nhận từ App/Firebase LE  |
| h_cmd         | uint8  | Lệnh nâng/hạ/phụ                  |
| sensor_flags  | uint8  | bitmask trạng thái cảm biến        |
+---------------+--------+------------------------------------+
```

`sensor_flags`:

```text
bit0 = IMU valid
bit1 = VL53 valid
bit2 = new target node
bit3-7 = reserved
```

### 8.2 AGV command (`CMD = 0x10`)

Hướng:

- `PC/ESP32 -> AGV main`

Payload (4 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| target_node   | uint16 | Node đích LE                       |
| move_mode     | uint8  | chế độ chạy                        |
| command_flags | uint8  | start/stop/reset/...               |
+---------------+--------+------------------------------------+
```

### 8.3 Sync request (`CMD = 0x11`)

Hướng:

- `AGV main -> ESP32 gateway`

Payload (4 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| current_node  | uint16 | Node hiện tại của AGV LE           |
| is_arrived    | uint8  | 0/1                                |
| reserved      | uint8  | = 0                                |
+---------------+--------+------------------------------------+
```

### 8.4 Arm joint command (`CMD = 0x20`)

Hướng:

- `PC/ESP32 -> AGV main`
- `AGV main -> Arm left/right`

Payload (**22 bytes**, tất cả multi-byte LE):

```text
+-------------------+--------+------------------------------------------+
| Field             | Type   | Meaning                                  |
+-------------------+--------+------------------------------------------+
| motion_mode       | uint8  | 0=hold, 1=abs, 2=rel, 3=home, 4=estop   |
| arm_flags         | uint8  | bitmask (xem bên dưới)                  |
| q1_x100           | int16  | joint 1 (deg * 100) LE                  |
| q2_x100           | int16  | joint 2 LE                               |
| q3_x100           | int16  | joint 3 LE                               |
| q4_x100           | int16  | joint 4 LE                               |
| q5_x100           | int16  | joint 5 LE                               |
| q6_x100           | int16  | joint 6 LE                               |
| move_time_ms      | uint16 | thời gian nội suy mong muốn LE          |
| max_delta_x100    | uint16 | Δθ tối đa cho phép mỗi khớp (deg*100)  |
+-------------------+--------+------------------------------------------+
```

> **So với V2:** `arm_id` đã bị **xóa**. DEST trong header (`0x02`/`0x03`) là nguồn định danh duy nhất. `max_delta_x100` **mới** — Slave drop frame nếu bất kỳ khớp nào vượt ngưỡng này.

`motion_mode`:

```text
0 = hold current
1 = absolute joint command
2 = relative joint command
3 = home
4 = estop
```

`arm_flags`:

```text
bit0 = payload valid
bit1 = gripper state included separately
bit2 = synchronous with AGV step
bit3-7 = reserved
```

`max_delta_x100` — Cơ chế bảo vệ Δθ:

```text
Slave kiểm tra: if |q_new[i] - q_last[i]| > max_delta_x100 -> DROP frame + giữ nguyên vị trí
Giá trị 0 = không giới hạn (CHỈ dùng khi test)

Khuyến nghị theo tần số:
  50 Hz -> max_delta_x100 = 300  (3.00 deg/frame ~ 150 deg/s)
  20 Hz -> max_delta_x100 = 750  (7.50 deg/frame ~ 150 deg/s)
```

### 8.5 Arm gripper command (`CMD = 0x21`)

Payload (4 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| grip_action   | uint8  | 0=release, 1=grip, 2=toggle        |
| reserved[3]   | 3B     | = 0                                |
+---------------+--------+------------------------------------+
```

> **So với V2:** `arm_id` và `grip_force` đã bị **xóa**. Gọn hơn, không xung đột với DEST.

### 8.6 ACK (`CMD = 0x30`)

Payload (4 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| acked_cmd     | uint8  | CMD đang được xác nhận             |
| acked_seq     | uint8  | SEQ đang được xác nhận             |
| status        | uint8  | 0=ok, 1=busy, 2=queued             |
| reserved      | uint8  | = 0                                |
+---------------+--------+------------------------------------+
```

### 8.7 NACK (`CMD = 0x31`)

Payload (4 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| nack_cmd      | uint8  | CMD bị từ chối                     |
| nack_seq      | uint8  | SEQ bị từ chối                     |
| error_code    | uint8  | mã lỗi                             |
| reserved      | uint8  | = 0                                |
+---------------+--------+------------------------------------+
```

`error_code`:

```text
1 = bad length
2 = bad CRC
3 = bad address
4 = unsupported command
5 = invalid payload
6 = joint delta exceeded   <- MỚI: Δθ > max_delta_x100
7 = busy
```

### 8.8 Heartbeat (`CMD = 0x40`)

Payload (8 bytes):

```text
+---------------+--------+------------------------------------+
| Field         | Type   | Meaning                            |
+---------------+--------+------------------------------------+
| uptime_ms     | uint32 | thời gian sống LE                  |
| state         | uint8  | trạng thái node                    |
| reserved[3]   | 3B     | = 0                                |
+---------------+--------+------------------------------------+
```

### 8.9 AGV status report (`CMD = 0x50`)

Hướng: `AGV main -> PC app`

Chỉ báo trạng thái di chuyển của AGV main.

Payload (8 bytes):

```text
+---------------+--------+--------------------------------------------+
| Field         | Type   | Meaning                                    |
+---------------+--------+--------------------------------------------+
| node_state    | uint8  | 0=idle, 1=running, 2=error, 3=estop       |
| error_code    | uint8  | mã lỗi AGV                                |
| current_node  | uint16 | node hiện tại LE                          |
| target_node   | uint16 | node đích đang nhắm tới LE                |
| agv_flags     | uint8  | arrived/charging/door_open...             |
| reserved      | uint8  | = 0                                       |
+---------------+--------+--------------------------------------------+
```

> **So với V2:** CMD 0x50 cũ gộp cả AGV state và arm joint feedback — payload 11 byte không đủ chứa 2 tay. Nay tách riêng.

### 8.10 Arm status report (`CMD = 0x51`) — MỚI

Hướng: `Arm slave -> AGV main -> PC app`

AGV main forward nguyên frame, giữ `SRC` của Slave để PC biết tay nào.

Payload (16 bytes):

```text
+-------------------+--------+--------------------------------------------+
| Field             | Type   | Meaning                                    |
+-------------------+--------+--------------------------------------------+
| arm_state         | uint8  | 0=idle, 1=moving, 2=error, 3=estop        |
| error_code        | uint8  | mã lỗi arm                                |
| q1_x100           | int16  | joint 1 feedback LE                       |
| q2_x100           | int16  | joint 2 feedback LE                       |
| q3_x100           | int16  | joint 3 feedback LE                       |
| q4_x100           | int16  | joint 4 feedback LE                       |
| q5_x100           | int16  | joint 5 feedback LE                       |
| q6_x100           | int16  | joint 6 feedback LE                       |
| gripper_state     | uint8  | 0=open, 1=closed, 2=moving                |
| reserved          | uint8  | = 0                                       |
+-------------------+--------+--------------------------------------------+
```

Khi PC App cần trạng thái tổng hợp: AGV main gửi 1 frame `0x50` + 2 frame `0x51` (SRC=0x02 và SRC=0x03) riêng biệt.

## 9. Routing rule

### 9.1 ESP32 gateway

- nhận lệnh từ PC app qua HC12
- đóng gói thành Protocol V2.1
- gửi về `DEST = 0x01` nếu là lệnh điều phối tổng
- hoặc gửi trực tiếp `DEST = 0x02/0x03` cho arm command

### 9.2 AGV main — Quy tắc vàng

```text
DEST = 0x01 -> Xử lý nội bộ
DEST = 0x02 -> Forward nguyên frame ra RS485 -> Arm Left
DEST = 0x03 -> Forward nguyên frame ra RS485 -> Arm Right
DEST = 0x7F -> Xử lý nội bộ + Broadcast RS485
```

**AGV main KHÔNG đọc `arm_id` trong payload — trường này đã bị xóa.**

### 9.3 Arm slave

- chỉ xử lý frame có `DEST` đúng địa chỉ của mình
- bỏ qua frame của node khác
- với CMD 0x20: kiểm tra Δθ trước khi execute

## 10. Reliability policy

| Frame | ACK bắt buộc | Ghi chú |
|-------|-------------|---------|
| AGV command (0x10) | Có | Timeout 100ms, retry 3 lần |
| Arm joint stream (0x20) | Không | Drop frame cũ theo SEQ + Δθ guard tại Slave |
| Arm gripper (0x21) | Có | Timeout 100ms, retry 3 lần |
| Estop (motion_mode=4) | Có | Timeout 50ms, retry 5 lần |
| Heartbeat (0x40) | Không | — |
| Status 0x50/0x51 | Không | Chu kỳ 200ms |

Timeout đề nghị:

```text
ESP32 -> main  : 100 ms
main -> arm    : 100 ms
heartbeat      : 500 ms
```

## 11. State machine parser (firmware)

Vị trí `LEN` trước `CMD/SEQ` cho phép DMA pre-allocation:

```text
WAIT_SOF1    -> byte == 0xAA
WAIT_SOF2    -> byte == 0x55
WAIT_DEST    -> lưu dest
WAIT_SRC     -> lưu src
WAIT_LEN_L   -> lưu len_low
WAIT_LEN_H   -> len = len_low | (len_high << 8)
               -> configure DMA: nhận (len + 2) byte tiếp theo
WAIT_CMD     -> lưu cmd          [index == 6]
WAIT_SEQ     -> lưu seq          [index == 7]
RECV_PAYLOAD -> nhận len byte
WAIT_CRC_L   -> lưu crc_low
WAIT_CRC_H   -> verify CRC -> dispatch
```

Trong firmware:

- parser nên là state machine byte-by-byte
- không parse bằng chuỗi
- bỏ qua frame sai CRC
- không block `while(1)` để đợi đủ byte
- dùng DMA/IT + ring buffer khi cần

## 12. Ví dụ frame hoàn chỉnh

Gửi lệnh joint command cho `arm_right` tại 50 Hz:

```text
Header:
  SOF1    = 0xAA
  SOF2    = 0x55
  DEST    = 0x03         (Arm Right)
  SRC     = 0x20         (PC App)
  LEN_L   = 0x16         (22 bytes, LE)
  LEN_H   = 0x00
  CMD     = 0x20
  SEQ     = 0x15

Payload (22 bytes, Little-Endian):
  motion_mode    = 0x01              (absolute)
  arm_flags      = 0x01              (payload valid)
  q1_x100        = 0xB8 0x0D        (3512 = 35.12 deg)
  q2_x100        = 0xE8 0x03        (1000 = 10.00 deg)
  q3_x100        = 0xDA 0xFD        (-550 = -5.50 deg)
  q4_x100        = 0x00 0x00        (0)
  q5_x100        = 0x89 0x17        (6025 = 60.25 deg)
  q6_x100        = 0x28 0x23        (9000 = 90.00 deg)
  move_time_ms   = 0x2C 0x01        (300 ms)
  max_delta_x100 = 0x2C 0x01        (300 = 3.00 deg/frame)

CRC = [CRC_L, CRC_H]  (CRC-16/CCITT-FALSE, Little-Endian)
```

## 13. Migration plan

### Phase 1 — STM32 Arm Slave Firmware ✅

- `arm_protocol.h`: header tự chứa cho STM32F4 arm slave
- `arm_firmware_right/main.c`: thay parser text `R:` bằng V2.1 SM
  - Δθ guard: drop frame nếu |q_new - q_last| > max_delta_x100
  - `arm_q_last[6]` khởi tạo khớp với vị trí home servo

### Phase 2 — STM32 AGV Main Firmware ✅

- `agv_link_protocol.h`: viết lại hoàn toàn cho V2.1
- `esp32_hub.c`: parser migrated sang V2.1
  - ReadS16LE/ReadU16LE thay BE
  - expected_len tính tại index==6 (LEN tại offset 4:5)
  - ArmJointCommand: 18B→22B, arm_id removed, dest forwarded

### Phase 3 — ESP32 Gateway ✅

- `esp32_gateway.ino`:
  - write_u16_le/read_u16_le thay BE helpers
  - send_proto_frame: LEN tại [4:5], CMD tại [6], SEQ tại [7]
  - send_arm_command_frame: arm_id xóa, payload 18B→22B, max_delta_x100=300
  - process_main_uart: expected_len tính tại index==6

### Phase 4 — Java PC App ✅

- `UartManager.java`:
  - `buildArmJointFrame(dest, qActuator_deg)`: ByteBuffer.LITTLE_ENDIAN
  - CRC-16/CCITT-FALSE Java implementation
  - `sendBytes(byte[])` cho binary frame
  - `sendData(String)` giữ lại cho ESTOP/config
- `MainFrame.java`:
  - `sendJointsToUart()`: text → binary V2.1 frame
  - DEST drives arm selection (0x02 Left / 0x03 Right)
  - `frameHexSummary()`: display `"DEST=03 SEQ=xx | q=[...]"`

## 14. So sánh V2 vs V2.1

| Điểm | Protocol V2 | Protocol V2.1 |
|------|------------|---------------|
| Byte order | Big-Endian | **Little-Endian** |
| Tên trường LEN | `LEN_L`(cao)/`LEN_H`(thấp) — ngược nghĩa | `LEN` uint16 LE chuẩn |
| Vị trí LEN | Sau CMD, SEQ | **Trước** CMD, SEQ |
| `arm_id` trong payload | Có — dư thừa, xung đột tiềm ẩn | **Xóa** |
| Bảo vệ Δθ | Không có | `max_delta_x100` + Slave enforce |
| Status Report | `0x50` gộp AGV+Arm (11B, không đủ) | `0x50` AGV (8B) + `0x51` Arm (16B) |
| NACK error 0x06 | Không có | `JOINT_DELTA_EXCEEDED` |
| Payload `0x20` | 18 bytes | 22 bytes |
| Payload `0x21` | 6 bytes | 4 bytes |
| Header bytes | 8 | 8 (không đổi tổng) |
