#ifndef LS7366R_H
#define LS7366R_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

// ASSUMPTION: SPI handle is hspi3 for LS7366R (PB3/4/5 is typically SPI3)
// TUNE: Change this if using a different SPI instance
extern SPI_HandleTypeDef hspi3;
#define LS7366R_SPI_HANDLE &hspi3

// Pin definitions
#define ENC_CS1  1
#define ENC_CS2  2
#define ENC_CS3  3
#define ENC_CS4  4

// Register commands (OPC + RS)
#define LS_WRITE_MDR0  0x88
#define LS_WRITE_MDR1  0x90
#define LS_WRITE_DTR   0x98
#define LS_READ_CNTR   0x60
#define LS_READ_OTR    0x68
#define LS_READ_STR    0x70
#define LS_LOAD_OTR    0xE8  // Transfer CNTR to OTR
#define LS_LOAD_CNTR   0xE0  // Transfer DTR to CNTR
#define LS_CLEAR_CNTR  0x20
#define LS_CLEAR_STR   0x30

// Recommended MDR0 config for Hall signals: x4 quadrature, free running, no index, filter off
// Change to non-quadrature (A=clk, B=dir) to test if Hall signals reach the chip
#define LS_MDR0_QUAD_X4_FREE 0x00

// Recommended MDR1 config: 4-byte counter, enable counting, no flags
// TUNE: Change counter byte length if needed
#define LS_MDR1_4BYTE_EN     0x00

typedef struct {
    int32_t count_e1;
    int32_t count_e2;
    int32_t count_e3;
    int32_t count_e4;
    float vel_e1;
    float vel_e2;
    float vel_e3;
    float vel_e4;
    uint32_t last_update;
} LS7366R_EncoderData_t;

// Function prototypes
uint32_t LS7366R_TestSPI(uint8_t csPin);
void LS7366R_Init(uint8_t csPin, uint8_t mdr0, uint8_t mdr1);
void LS7366R_InitAll(void);
int32_t LS7366R_ReadCounter(uint8_t csPin);
void LS7366R_ClearCounter(uint8_t csPin);
uint8_t LS7366R_ReadStatus(uint8_t csPin);
void LS7366R_LoadOTR(uint8_t csPin);
int32_t LS7366R_ReadOTR(uint8_t csPin);
float LS7366R_GetVelocity(uint8_t csPin, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif // LS7366R_H
