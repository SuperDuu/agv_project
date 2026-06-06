#include "hmi_modbus.h"
#include <string.h>
#include "agv_routing.h"

uint16_t hmi_registers[HMI_REG_COUNT] = {0};
HMI_HandleTypeDef h_hmi;

// Biến lưu trạng thái cũ để bắt sự kiện thay đổi
static uint16_t prev_dest_node = 0xFFFF;
static uint16_t prev_command = 0;

// Các biến từ main.c
extern volatile uint16_t current_node;
extern volatile uint16_t destination_node;
extern uint16_t current_path[];
extern uint16_t path_length;
extern volatile uint8_t agv_run_mode;
extern bool agv_follow_line_enable;
extern void Route_Recalculate(void); // Cần định nghĩa hoặc dùng flag trong main

// Tính mã CRC16 cho Modbus RTU
static uint16_t Modbus_CalcCRC(uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Khởi tạo UART DMA/IT cho Modbus
void HMI_Init(UART_HandleTypeDef *huart, uint8_t slave_address) {
    h_hmi.huart = huart;
    h_hmi.slave_address = slave_address;
    h_hmi.rx_index = 0;
    h_hmi.frame_ready = false;
    
    // Khởi động nhận DMA hoặc ngắt (sử dụng chế độ ngắt nhàn rỗi - IDLE LINE nếu có)
    HAL_UARTEx_ReceiveToIdle_IT(h_hmi.huart, h_hmi.rx_buffer, sizeof(h_hmi.rx_buffer));
}

// Callback khi UART nhận được dữ liệu (Gọi từ HAL_UARTEx_RxEventCallback trong main)
void HMI_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == h_hmi.huart->Instance) {
        h_hmi.rx_index = Size;
        h_hmi.frame_ready = true;
        
        // Khởi động lại vòng nhận
        HAL_UARTEx_ReceiveToIdle_IT(h_hmi.huart, h_hmi.rx_buffer, sizeof(h_hmi.rx_buffer));
    }
}

// Xử lý gói tin Modbus (Gọi liên tục trong while(1))
void HMI_Process(void) {
    if (!h_hmi.frame_ready) return;
    h_hmi.frame_ready = false;

    uint16_t len = h_hmi.rx_index;
    if (len < 8) return; // Khung Modbus RTU tối thiểu 8 bytes

    // Kiểm tra Slave Address
    if (h_hmi.rx_buffer[0] != h_hmi.slave_address) return;

    // Kiểm tra CRC
    uint16_t received_crc = (h_hmi.rx_buffer[len - 1] << 8) | h_hmi.rx_buffer[len - 2];
    uint16_t calc_crc = Modbus_CalcCRC(h_hmi.rx_buffer, len - 2);
    if (received_crc != calc_crc) return;

    uint8_t function_code = h_hmi.rx_buffer[1];
    uint16_t start_addr = (h_hmi.rx_buffer[2] << 8) | h_hmi.rx_buffer[3];
    uint16_t reg_count = (h_hmi.rx_buffer[4] << 8) | h_hmi.rx_buffer[5];

    // Chống ghi đè ngoài mảng
    if (start_addr + reg_count > HMI_REG_COUNT) return;

    uint16_t tx_len = 0;

    // Lệnh 0x03: Đọc thanh ghi Holding (Read Holding Registers)
    if (function_code == 0x03) {
        h_hmi.tx_buffer[0] = h_hmi.slave_address;
        h_hmi.tx_buffer[1] = 0x03;
        h_hmi.tx_buffer[2] = reg_count * 2; // Số byte trả về
        
        uint16_t byte_idx = 3;
        for (uint16_t i = 0; i < reg_count; i++) {
            h_hmi.tx_buffer[byte_idx++] = (hmi_registers[start_addr + i] >> 8) & 0xFF;
            h_hmi.tx_buffer[byte_idx++] = hmi_registers[start_addr + i] & 0xFF;
        }
        
        uint16_t crc = Modbus_CalcCRC(h_hmi.tx_buffer, byte_idx);
        h_hmi.tx_buffer[byte_idx++] = crc & 0xFF;
        h_hmi.tx_buffer[byte_idx++] = (crc >> 8) & 0xFF;
        
        tx_len = byte_idx;
    }
    // Lệnh 0x06: Ghi 1 thanh ghi (Write Single Register)
    else if (function_code == 0x06) {
        uint16_t write_val = (h_hmi.rx_buffer[4] << 8) | h_hmi.rx_buffer[5];
        hmi_registers[start_addr] = write_val;
        
        // Echo lại toàn bộ lệnh 0x06
        memcpy(h_hmi.tx_buffer, h_hmi.rx_buffer, len);
        tx_len = len;
    }
    // Lệnh 0x10: Ghi nhiều thanh ghi (Write Multiple Registers)
    else if (function_code == 0x10) {
        uint8_t byte_count = h_hmi.rx_buffer[6];
        if (byte_count != reg_count * 2) return;
        
        uint16_t data_idx = 7;
        for (uint16_t i = 0; i < reg_count; i++) {
            hmi_registers[start_addr + i] = (h_hmi.rx_buffer[data_idx] << 8) | h_hmi.rx_buffer[data_idx + 1];
            data_idx += 2;
        }
        
        // Trả lời lệnh 0x10
        h_hmi.tx_buffer[0] = h_hmi.slave_address;
        h_hmi.tx_buffer[1] = 0x10;
        h_hmi.tx_buffer[2] = h_hmi.rx_buffer[2];
        h_hmi.tx_buffer[3] = h_hmi.rx_buffer[3];
        h_hmi.tx_buffer[4] = h_hmi.rx_buffer[4];
        h_hmi.tx_buffer[5] = h_hmi.rx_buffer[5];
        
        uint16_t crc = Modbus_CalcCRC(h_hmi.tx_buffer, 6);
        h_hmi.tx_buffer[6] = crc & 0xFF;
        h_hmi.tx_buffer[7] = (crc >> 8) & 0xFF;
        tx_len = 8;
    }

    if (tx_len > 0) {
        // Nếu dùng vi mạch MAX485 thì set cờ DE lên cao ở đây, truyền xong hạ xuống
        HAL_UART_Transmit(h_hmi.huart, h_hmi.tx_buffer, tx_len, 100);
    }
}

// Đồng bộ dữ liệu HMI với hệ thống
void HMI_SyncData(void) {
    // 1. STM32 -> HMI (Cập nhật dữ liệu từ biến vào mảng)
    hmi_registers[REG_AGV_MODE] = agv_run_mode;
    hmi_registers[REG_CURRENT_NODE] = current_node;
    hmi_registers[REG_PATH_LENGTH] = path_length;
    hmi_registers[REG_AGV_STATUS] = agv_follow_line_enable ? 1 : 0;
    
    for (int i = 0; i < path_length && i < 20; i++) {
        hmi_registers[REG_PATH_START + i] = current_path[i];
    }

    // 2. HMI -> STM32 (Lắng nghe lệnh từ màn hình)
    if (hmi_registers[REG_DEST_NODE] != prev_dest_node) {
        prev_dest_node = hmi_registers[REG_DEST_NODE];
        destination_node = prev_dest_node;
        
        // Cập nhật lại đường đi ngay lập tức
        extern AGV_Map_t factory_map;
        bool found = Routing_Dijkstra(&factory_map, current_node, destination_node, current_path, (uint16_t *)&path_length);
        if (found) {
            extern volatile uint16_t path_index;
            path_index = 0;
        }
    }

    if (hmi_registers[REG_COMMAND] != prev_command) {
        prev_command = hmi_registers[REG_COMMAND];
        if (prev_command == 1) {
            agv_follow_line_enable = true; // Chạy
        } else if (prev_command == 2) {
            agv_follow_line_enable = false; // Dừng
        }
        hmi_registers[REG_COMMAND] = 0; // Xóa lệnh sau khi thực hiện
        prev_command = 0;
    }
}
