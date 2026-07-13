#include "encoder.h"

// Extern declarations of the timer handles defined in main.c
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
// extern TIM_HandleTypeDef htim5;

// Global array of encoder tracker instances
Encoder_t encoders[NUM_ENCODERS];

void Encoder_Init(void) {
    // Configure the 4 encoders: TIM1, TIM2, TIM3, TIM4
    // TIM2 is 32-bit timer on STM32F446, the rest are 16-bit
    encoders[0] = (Encoder_t){&htim1, 0, 0, 0}; // TIM1 (16-bit)
    encoders[1] = (Encoder_t){&htim2, 0, 0, 1}; // TIM2 (32-bit)
    encoders[2] = (Encoder_t){&htim3, 0, 0, 0}; // TIM3 (16-bit)
    encoders[3] = (Encoder_t){&htim4, 0, 0, 0}; // TIM4 (16-bit)
    // encoders[4] = (Encoder_t){&htim5, 0, 0, 1}; // TIM5 (32-bit)

    // Start all encoder interfaces
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    // HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    // Initialize last counter values and counts
    for (int i = 0; i < NUM_ENCODERS; i++) {
        encoders[i].last_counter_value = __HAL_TIM_GET_COUNTER(encoders[i].htim);
        encoders[i].accumulated_count = 0;
    }
}

void Encoder_Update(void) {
    for (int i = 0; i < NUM_ENCODERS; i++) {
        int32_t current_counter = __HAL_TIM_GET_COUNTER(encoders[i].htim);
        int32_t diff = 0;
        if (encoders[i].is_32bit) {
            // 32-bit difference casting
            diff = (int32_t)(current_counter - encoders[i].last_counter_value);
        } else {
            // 16-bit difference casting to handle overflow and underflow wrap-around automatically
            diff = (int16_t)(current_counter - encoders[i].last_counter_value);
        }
        encoders[i].accumulated_count += diff;
        encoders[i].last_counter_value = current_counter;
    }
}

int32_t Encoder_Get_Ticks(uint8_t index) {
    if (index >= NUM_ENCODERS) return 0;
    
    Encoder_Update(); // Ensure we have the latest counts
    
    // Return raw accumulated ticks without division to preserve full encoder resolution
    return encoders[index].accumulated_count;
}

void Encoder_Reset(uint8_t index) {
    if (index >= NUM_ENCODERS) return;
    
    __HAL_TIM_SET_COUNTER(encoders[index].htim, 0);
    encoders[index].last_counter_value = 0;
    encoders[index].accumulated_count = 0;
}

// Đọc encoder đơn giản - trả về giá trị counter trực tiếp
int32_t Encoder_Read_Raw(uint8_t index) {
    if (index >= NUM_ENCODERS) return 0;
    return (int32_t)__HAL_TIM_GET_COUNTER(encoders[index].htim);
}
