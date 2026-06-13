#ifndef __AGV_EEPROM_H
#define __AGV_EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Địa chỉ Sector 127 của Bank 2 (Sector cuối cùng của Flash 2MB) */
#define EEPROM_START_ADDRESS    0x081FE000UL
#define EEPROM_SECTOR_SIZE      8192UL  // 8KB
#define EEPROM_END_ADDRESS      (EEPROM_START_ADDRESS + EEPROM_SECTOR_SIZE)
#define EEPROM_MAGIC_WORD       0xABCD1234UL
#define EEPROM_EMPTY_WORD       0xFFFFFFFFUL

#define FLASH_SECTOR_EEPROM     127U

typedef struct {
    uint16_t current_node;
    uint8_t current_heading;
    uint8_t reserved1;
    uint32_t magic_word; // 0xABCD1234
    uint32_t reserved2;
    uint32_t reserved3;
} __attribute__((aligned(16))) EEPROM_Record_t;

void EEPROM_Init(void);
void EEPROM_SaveState(uint16_t node, uint8_t heading);
bool EEPROM_LoadState(uint16_t *node, uint8_t *heading);

#ifdef __cplusplus
}
#endif

#endif /* __AGV_EEPROM_H */
