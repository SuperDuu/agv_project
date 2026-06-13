#include "agv_eeprom.h"
#include <string.h>

/* Xóa Sector 127 của Bank 2 */
static void EEPROM_EraseSector(void) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    HAL_FLASH_Unlock();
    
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks = FLASH_BANK_2;
    EraseInitStruct.Sector = FLASH_SECTOR_EEPROM;
    EraseInitStruct.NbSectors = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    
    HAL_FLASH_Lock();
}

void EEPROM_Init(void) {
    // Không cần làm gì đặc biệt, hàm LoadState sẽ tự lo việc kiểm tra
}

void EEPROM_SaveState(uint16_t node, uint8_t heading) {
    EEPROM_Record_t record = {0};
    record.current_node = node;
    record.current_heading = heading;
    record.magic_word = EEPROM_MAGIC_WORD;
    record.reserved1 = 0;
    record.reserved2 = 0;
    record.reserved3 = 0;

    uint32_t address = EEPROM_START_ADDRESS;
    bool found_empty = false;

    // Tìm ô trống đầu tiên (16 bytes = 4 words)
    while (address < EEPROM_END_ADDRESS) {
        // Đọc word thứ 2 chứa magic_word (vì struct là: node/heading (word 0), magic (word 1))
        // Wait, the struct alignment:
        // current_node (16), current_heading(8), reserved1(8) -> 32 bits (word 0)
        // magic_word (32) -> 32 bits (word 1)
        // reserved2 (32) -> 32 bits (word 2)
        // reserved3 (32) -> 32 bits (word 3)
        uint32_t *pData = (uint32_t *)address;
        
        // Cả 4 word đều phải là 0xFFFFFFFF thì mới thực sự trống
        if (pData[0] == EEPROM_EMPTY_WORD && pData[1] == EEPROM_EMPTY_WORD && 
            pData[2] == EEPROM_EMPTY_WORD && pData[3] == EEPROM_EMPTY_WORD) {
            found_empty = true;
            break;
        }
        address += sizeof(EEPROM_Record_t); // Tịnh tiến 16 bytes
    }

    if (!found_empty) {
        // Đã ghi đầy Sector -> Xóa và ghi lại từ đầu
        EEPROM_EraseSector();
        address = EEPROM_START_ADDRESS;
    }

    // Ghi 16 bytes (Quad-Word)
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, (uint32_t)&record);
    HAL_FLASH_Lock();
}

bool EEPROM_LoadState(uint16_t *node, uint8_t *heading) {
    uint32_t address = EEPROM_END_ADDRESS - sizeof(EEPROM_Record_t);
    bool found = false;

    // Quét từ cuối Sector về đầu để tìm bản ghi mới nhất có Magic Word
    while (address >= EEPROM_START_ADDRESS) {
        EEPROM_Record_t *record = (EEPROM_Record_t *)address;
        if (record->magic_word == EEPROM_MAGIC_WORD) {
            *node = record->current_node;
            *heading = record->current_heading;
            found = true;
            break;
        }
        // Tránh vòng lặp vô hạn do address là uint32_t
        if (address == EEPROM_START_ADDRESS) break;
        address -= sizeof(EEPROM_Record_t);
    }

    return found;
}
