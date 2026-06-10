#ifndef __QR50_READER_H__
#define __QR50_READER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* 1. Enum for Status */
typedef enum {
    QR50_OK = 0,
    QR50_TIMEOUT,
    QR50_ERROR
} QR50_Status_t;

/* 2. Struct for Configuration */
typedef struct {
    uint8_t Address;
    uint32_t Baudrate;
    bool ActiveUpload;
} QR50_Config_t;

/* 3. Struct for Data */
#define QR50_MAX_DATA_LEN 256

typedef struct {
    uint8_t Data_Buffer[QR50_MAX_DATA_LEN];
    uint16_t Data_Length;
    volatile bool New_Data_Flag;
} QR50_Data_t;

/* 4. Overarching Handler Struct */
typedef struct {
    UART_HandleTypeDef *huart;
    QR50_Config_t Config;
    QR50_Data_t Data;
    QR50_Status_t Status;
} QR50_Handler_t;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* 5. Function Prototypes */
void QR50_Init(QR50_Handler_t *handler, UART_HandleTypeDef *huart, uint8_t addr);
void QR50_ParseData(QR50_Handler_t *handler, uint8_t *raw_buffer, uint16_t length);

/* USER CODE BEGIN Prototypes */
typedef struct {
    uint32_t bit_count;
    uint64_t data_buffer;
    uint32_t last_bit_time;
    volatile bool new_data_ready;
    uint32_t final_card_id;
} Wiegand_HandleTypeDef;

void Wiegand_Init(Wiegand_HandleTypeDef *hwg);
void Wiegand_ProcessBit(Wiegand_HandleTypeDef *hwg, uint8_t bit);
void Wiegand_ProcessLoop(Wiegand_HandleTypeDef *hwg);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __QR50_READER_H__ */
