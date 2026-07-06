# AGV/ARM Protocol V2

Protocol V2 la giao thuc nhi phan dung chung cho:

- `PC app -> HC12 -> ESP32 gateway`
- `ESP32 gateway -> AGV main (STM32)`
- `AGV main -> arm_left / arm_right`

Muc tieu:

- mot frame duy nhat cho tat ca node
- de route qua `main`
- de them ACK, timeout, retry khi can
- khong truyen `float` tren duong day
- goc khop dung `int16 = degree * 100`

## 1. Frame format

Tat ca frame deu co dang:

```text
+--------+--------+------+------+------+------+---------+---------+--------+--------+
| SOF1   | SOF2   | DEST | SRC  | CMD  | SEQ  | LEN_L   | LEN_H   | PAYLOAD| CRC16 |
+--------+--------+------+------+------+------+---------+---------+--------+--------+
| 0xAA   | 0x55   | 1B   | 1B   | 1B   | 1B   | 1B      | 1B      | N byte | 2B    |
+--------+--------+------+------+------+------+---------+---------+--------+--------+
```

So byte co dinh truoc payload: `8`

So byte CRC: `2`

Tong frame length:

```text
frame_len = 8 + payload_len + 2
```

## 2. Byte order

- cac truong `int16`, `uint16`, `int32`, `uint32` trong payload: `big-endian`
- `LEN`: `LEN_L` la byte cao, `LEN_H` la byte thap
- `CRC16`: `big-endian`

Vi du:

```text
value = 0x1234 -> [0x12, 0x34]
```

## 3. CRC

Dung:

- `CRC-16/CCITT-FALSE`
- polynomial: `0x1021`
- init: `0xFFFF`
- refin: `false`
- refout: `false`
- xorout: `0x0000`

CRC tinh tren cac byte:

```text
DEST, SRC, CMD, SEQ, LEN_L, LEN_H, PAYLOAD...
```

Khong tinh tren:

- `SOF1`
- `SOF2`
- hai byte `CRC16`

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
0x30 = Ack
0x31 = Nack
0x40 = Heartbeat
0x50 = Status report
```

## 6. Sequence number

- `SEQ` tang tuan tu `0 -> 255 -> 0`
- dung de:
  - bo frame trung
  - bo frame cu
  - map voi ACK/NACK

## 7. Joint angle encoding

Tat ca goc khop dung:

```text
int16_t angle_x100 = round(angle_deg * 100.0)
angle_deg = angle_x100 / 100.0
```

Vi du:

```text
35.12 deg -> 3512
-7.50 deg -> -750
0.00 deg -> 0
```

Range `int16`:

```text
-327.68 deg -> +327.67 deg
```

Du cho toan bo khop hien tai.

## 8. Payload definitions

### 8.1 Sensor report (`CMD = 0x01`)

Huong:

- `ESP32 gateway -> AGV main`

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| yaw_x100          | int16  | IMU yaw (deg * 100)              |
| obstacle_mm       | uint16 | Khoang cach vat can              |
| target_node       | uint16 | Node dich nhan tu app/Firebase   |
| h_cmd             | uint8  | Lenh nang/ha/phu                 |
| sensor_flags      | uint8  | bitmask trang thai cam bien      |
+-------------------+--------+----------------------------------+
```

`sensor_flags`:

```text
bit0 = IMU valid
bit1 = VL53 valid
bit2 = new target node
bit3 = reserved
bit4 = reserved
bit5 = reserved
bit6 = reserved
bit7 = reserved
```

### 8.2 AGV command (`CMD = 0x10`)

Huong:

- `PC/ESP32 -> AGV main`

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| target_node       | uint16 | Node dich                        |
| move_mode         | uint8  | che do chay                      |
| command_flags     | uint8  | start/stop/reset/...             |
+-------------------+--------+----------------------------------+
```

### 8.2.1 Sync request (`CMD = 0x11`)

Huong:

- `AGV main -> ESP32 gateway`

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| current_node      | uint16 | Node hien tai cua AGV            |
| is_arrived        | uint8  | 0/1                              |
| reserved          | uint8  | de 0                             |
+-------------------+--------+----------------------------------+
```

### 8.3 Arm joint command (`CMD = 0x20`)

Huong:

- `PC/ESP32 -> AGV main`
- `AGV main -> Arm left/right`

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| arm_id            | uint8  | 0=left, 1=right                  |
| motion_mode       | uint8  | abs/rel/home/hold                |
| q1_x100           | int16  | joint 1 angle                    |
| q2_x100           | int16  | joint 2 angle                    |
| q3_x100           | int16  | joint 3 angle                    |
| q4_x100           | int16  | joint 4 angle                    |
| q5_x100           | int16  | joint 5 angle                    |
| q6_x100           | int16  | joint 6 angle                    |
| move_time_ms      | uint16 | thoi gian noi suy mong muon      |
| arm_flags         | uint8  | emergency/valid/grip-follow...   |
| reserved          | uint8  | de 0                             |
+-------------------+--------+----------------------------------+
```

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
bit3 = reserved
bit4 = reserved
bit5 = reserved
bit6 = reserved
bit7 = reserved
```

### 8.4 Arm gripper command (`CMD = 0x21`)

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| arm_id            | uint8  | 0=left, 1=right                  |
| grip_action       | uint8  | 0=release, 1=grip, 2=toggle      |
| grip_force        | uint16 | tuy chon                         |
| reserved          | uint16 | de 0                             |
+-------------------+--------+----------------------------------+
```

### 8.5 Ack (`CMD = 0x30`)

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| acked_cmd         | uint8  | CMD dang duoc xac nhan           |
| acked_seq         | uint8  | SEQ dang duoc xac nhan           |
| status            | uint8  | 0=ok, 1=busy, 2=queued           |
| reserved          | uint8  | de 0                             |
+-------------------+--------+----------------------------------+
```

### 8.6 Nack (`CMD = 0x31`)

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| nack_cmd          | uint8  | CMD bi tu choi                   |
| nack_seq          | uint8  | SEQ bi tu choi                   |
| error_code        | uint8  | ma loi                           |
| reserved          | uint8  | de 0                             |
+-------------------+--------+----------------------------------+
```

`error_code`:

```text
1 = bad length
2 = bad crc
3 = bad address
4 = unsupported command
5 = invalid payload
6 = out of range
7 = busy
```

### 8.7 Heartbeat (`CMD = 0x40`)

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| uptime_ms         | uint32 | thoi gian song                   |
| state             | uint8  | trang thai node                  |
| reserved[3]       | 3B     | de 0                             |
+-------------------+--------+----------------------------------+
```

### 8.8 Status report (`CMD = 0x50`)

Payload:

```text
+-------------------+--------+----------------------------------+
| Field             | Type   | Meaning                          |
+-------------------+--------+----------------------------------+
| node_state        | uint8  | idle/running/error               |
| error_code        | uint8  | loi hien tai                     |
| current_node      | uint16 | dung cho AGV main                |
| q1_x100           | int16  | dung cho arm board               |
| q2_x100           | int16  | dung cho arm board               |
| q3_x100           | int16  | dung cho arm board               |
| q4_x100           | int16  | dung cho arm board               |
| q5_x100           | int16  | dung cho arm board               |
| q6_x100           | int16  | dung cho arm board               |
+-------------------+--------+----------------------------------+
```

## 9. Routing rule

### 9.1 ESP32 gateway

- nhan lenh tu PC app qua HC12
- dong goi thanh Protocol V2
- gui ve `DEST = 0x01` neu la lenh dieu phoi tong
- hoac gui truc tiep `DEST = 0x02/0x03` neu sau nay muon

### 9.2 AGV main

- nhan moi frame tu `ESP32`
- neu `DEST = 0x01` thi xu ly noi bo
- neu `DEST = 0x02` hoac `0x03` thi forward sang `RS485_0`
- neu la lenh tong hop, `main` duoc phep tach thanh:
  - AGV command noi bo
  - arm_left command
  - arm_right command

### 9.3 Arm slave

- chi xu ly frame co `DEST` dung dia chi cua minh
- bo qua frame cua node khac

## 10. Reliability policy

- frame command quan trong:
  - `AGV command`
  - `arm home`
  - `gripper`
  - `estop`
  Can `ACK`

- frame stream lien tuc:
  - `arm joint command` chu ky 20-50 Hz
  Khong bat buoc ACK tung frame
  Chi giu `SEQ` moi nhat

- neu `SEQ` cu hon frame da xu ly:
  - bo frame

- timeout de nghi:
  - `ESP32 -> main`: `100 ms`
  - `main -> arm`: `100 ms`
  - `heartbeat`: `500 ms`

## 11. Migration plan

### Phase 1

- giu logic cu dang text de test nhanh
- `ESP32` va `main` song song ho tro text + Protocol V2

### Phase 2

- Java app gui Protocol V2
- `ESP32` chi bridge frame
- `main` route arm frame theo `DEST`

### Phase 3

- arm_left/right bo parser `L:` / `R:`
- chuyen sang parser Protocol V2 day du

## 12. Example

Vi du gui lenh cho `arm_right`:

- `DEST = 0x03`
- `SRC = 0x20`
- `CMD = 0x20`
- `SEQ = 0x15`
- payload:
  - `arm_id = 1`
  - `motion_mode = 1`
  - `q1 = 35.12 -> 3512`
  - `q2 = 10.00 -> 1000`
  - `q3 = -5.50 -> -550`
  - `q4 = 0.00 -> 0`
  - `q5 = 60.25 -> 6025`
  - `q6 = 90.00 -> 9000`
  - `move_time_ms = 300`

## 13. Implementation note

Trong firmware:

- parser nen la state machine byte-by-byte
- khong parse bang chuoi
- bo qua frame sai CRC
- khong block `while(1)` de doi du byte
- dung DMA/IT + ring buffer neu can

Trong Java:

- encode `float -> int16 x100`
- dung `round`, khong cast truc tiep
