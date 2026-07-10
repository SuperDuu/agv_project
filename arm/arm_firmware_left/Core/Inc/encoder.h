#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define NUM_ENCODERS 5

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t last_counter_value;
    int32_t accumulated_count;
    uint8_t is_32bit;
} Encoder_t;

extern Encoder_t encoders[NUM_ENCODERS];

void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_Get_Ticks(uint8_t index);
void Encoder_Reset(uint8_t index);
int32_t Encoder_Read_Raw(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
