#ifndef HMI_MODBUS_H
#define HMI_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* --- Modbus RTU Register Map Configuration --- */
#define HMI_REG_COUNT 50

// Register Addresses (0-indexed offset for Modbus 4x Holding Registers)
#define REG_AGV_MODE      0x0000  // R/W: Current Run Mode (1-7)
#define REG_COMMAND       0x0001  // W  : 1=Start, 2=Stop, 3=Clear Error
#define REG_CURRENT_NODE  0x0002  // R  : Current Node ID
#define REG_NEXT_NODE     0x0003  // R  : Next Target Node ID
#define REG_DEST_NODE     0x0004  // R/W: Destination Node ID (Set from HMI)
#define REG_AGV_STATUS    0x0005  // R  : 0=Idle, 1=Running, 2=Error
#define REG_PATH_LENGTH   0x0006  // R  : Number of nodes in current path
#define REG_INDICATOR     0x0007  // R  : 0=Off, 1=Turning (Blink Green), 2=Error (Blink Red)
#define REG_PATH_START    0x0008  // R  : Array of up to 20 nodes for path (0x0008 - 0x001B)

/* --- Modbus Communication Structure --- */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[256];
    uint8_t tx_buffer[256];
    uint16_t rx_index;
    bool frame_ready;
    uint32_t last_rx_time;
    uint8_t slave_address;
} HMI_HandleTypeDef;

extern uint16_t hmi_registers[HMI_REG_COUNT];
extern HMI_HandleTypeDef h_hmi;

/* --- API Functions --- */
void HMI_Init(UART_HandleTypeDef *huart, uint8_t slave_address);
void HMI_Process(void);
void HMI_SyncData(void);
void HMI_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* HMI_MODBUS_H */
