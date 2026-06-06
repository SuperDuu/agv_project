/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    agv_control.c
 * @brief   This file provides code for AGV high-level control and line
 * following.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "agv_control.h"
#include <stdbool.h>
#include <stddef.h>

extern volatile bool agv_follow_line_enable;
extern volatile bool is_at_intersection;
extern volatile uint32_t intersection_time;
extern volatile uint32_t last_leave_intersection_time;

volatile AGV_RunMode_t agv_run_mode = MODE_7_DEBUG_NO_QR;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

float Delta_t = 0.01f;

void AGV_Init(AGV_HandleTypeDef *hagv, Motor_HandleTypeDef *m_left,
              Motor_HandleTypeDef *m_right, LineSensor_HandleTypeDef *l_sensor,
              Pid_data *pid, float base_speed) {
  if (hagv == NULL)
    return;

  hagv->motor_left = m_left;
  hagv->motor_right = m_right;
  hagv->line_sensor = l_sensor;
  hagv->pid_controller = pid;
  hagv->base_speed = base_speed;
  hagv->direction = 1;
  hagv->current_error = 0.0f;
}

static float AGV_ComputePID(Pid_data *pid, float setpoint) {
  if (pid == NULL)
    return 0.0f;

  pid->er = setpoint - pid->gtht;
  pid->i += pid->er * Delta_t;
  pid->d = (pid->er - pid->pre_er) / Delta_t;

  float output = (pid->Kp * pid->er) + (pid->Kd * pid->d) + (pid->Ki * pid->i);
  pid->pre_er = pid->er;

  return output;
}

float AGV_GetLineError(uint16_t line_value, float current_error) {
  switch (line_value) {
  case 0xFC3F: return 0.0f;

  // Left deviation 4 units
  case 0xF87F: return -1.0f;
  case 0xF0FF: return -1.5f;
  case 0xE1FF: return -2.0f;
  case 0xC3FF: return -2.5f;
  case 0x87FF: return -3.0f;
  case 0x0FFF: return -3.5f;

  // Left deviation 3 units
  case 0xFC7F: return -1.0f;
  case 0xF8FF: return -1.5f;
  case 0xF1FF: return -2.0f;
  case 0xE3FF: return -2.5f;
  case 0xC7FF: return -3.0f;
  case 0x8FFF: return -3.5f;
  case 0x1FFF: return -4.0f;

  // Left deviation 5 units
  case 0xF07F: return -1.0f;
  case 0xE0FF: return -1.5f;
  case 0xC1FF: return -2.0f;
  case 0x83FF: return -2.5f;
  case 0x07FF: return -3.0f;

  // Right deviation 4 units
  case 0xFE1F: return 1.0f;
  case 0xFF0F: return 1.5f;
  case 0xFF87: return 2.0f;
  case 0xFFC3: return 2.5f;
  case 0xFFE1: return 3.0f;
  case 0xFFF0: return 3.5f;

  // Right deviation 3 units
  case 0xFE3F: return 1.0f;
  case 0xFF1F: return 1.5f;
  case 0xFF8F: return 2.0f;
  case 0xFFC7: return 2.5f;
  case 0xFFE3: return 3.0f;
  case 0xFFF1: return 3.5f;
  case 0xFFF8: return 4.0f;

  // Right deviation 5 units
  case 0xFE0F: return 1.0f;
  case 0xFF07: return 1.5f;
  case 0xFF83: return 2.0f;
  case 0xFFC1: return 2.5f;
  case 0xFFE0: return 3.0f;

  default:
    return current_error;
  }
}

void AGV_FollowLine(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL)
    return;

  static uint32_t lost_line_time = 0;
  uint16_t line_val = LineSensor_Read(hagv->line_sensor);

  // Stop condition: all 1s (Pull-up disconnected) or all 0s (lost line)
  if (line_val == 0xFFFF || line_val == 0x0000) {
    if (lost_line_time == 0)
      lost_line_time = HAL_GetTick();

    if (HAL_GetTick() - lost_line_time > 1000) {
      if (agv_run_mode == MODE_4_FULL_RUN) {
        agv_follow_line_enable = false;
      }
      AGV_Stop(hagv);
    }
    return;
  } else {
    lost_line_time = 0;
  }

  // Intersection check: Bit 15 and Bit 0
  if (agv_run_mode != MODE_1_LINE_ONLY &&
      agv_run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
    if (((line_val & 0x8001) != 0x8001) &&
        (HAL_GetTick() - last_leave_intersection_time > 800)) {
      AGV_Stop(hagv);
      agv_follow_line_enable = false;
      is_at_intersection = true;
      intersection_time = HAL_GetTick();
      lost_line_time = 0;
      return;
    }
  }

  hagv->current_error = AGV_GetLineError(line_val, hagv->current_error);
  hagv->pid_controller->gtht = hagv->current_error;
  float output = AGV_ComputePID(hagv->pid_controller, 0.0f);

  if (hagv->direction == 1) {
    int16_t speed_l = (int16_t)(hagv->base_speed - output);
    int16_t speed_r = (int16_t)(hagv->base_speed + output);

    if (speed_l > 999) speed_l = 999;
    if (speed_r > 999) speed_r = 999;
    if (speed_l < -300) speed_l = -300;
    if (speed_r < -300) speed_r = -300;

    if (agv_run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
      Motor_SetSpeed(hagv->motor_left, speed_l);
      Motor_SetSpeed(hagv->motor_right, speed_r);
    }
  } else if (hagv->direction == -1) {
    // Đảo dấu logic PID khi đi lùi (do cảm biến nằm ở mũi xe nhưng xe lùi)
    int16_t speed_l = (int16_t)(-hagv->base_speed + output);
    int16_t speed_r = (int16_t)(-hagv->base_speed - output);

    if (speed_l < -999) speed_l = -999;
    if (speed_r < -999) speed_r = -999;
    if (speed_l > 300) speed_l = 300;
    if (speed_r > 300) speed_r = 300;

    if (agv_run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
      Motor_SetSpeed(hagv->motor_left, speed_l);
      Motor_SetSpeed(hagv->motor_right, speed_r);
    }
  }
}

void AGV_Stop(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL)
    return;
  Motor_Stop(hagv->motor_left);
  Motor_Stop(hagv->motor_right);
}

// Mask for 4 center sensors (bits 9-6): 0x03C0 = 0000001111000000
#define CENTER_MASK 0x03C0

static void Turn_Time_Based(AGV_HandleTypeDef *hagv, int16_t speed_l,
                            int16_t speed_r, uint32_t total_time) {
  bool center_found = false;
  uint32_t start = HAL_GetTick();

  // Cấp xung mạnh (Kick-start) 700PWM trong 80ms để thắng lực ma sát tĩnh lúc xe đang đứng im
  int16_t kick_l = (speed_l > 0) ? 700 : -700;
  int16_t kick_r = (speed_r > 0) ? 700 : -700;
  Motor_SetSpeed(hagv->motor_left, kick_l);
  Motor_SetSpeed(hagv->motor_right, kick_r);
  HAL_Delay(80);

  // Trả về tốc độ quay chậm (calib_speed = 150) để xe quay mượt mà, dò vạch không bị văng lố
  Motor_SetSpeed(hagv->motor_left, speed_l);
  Motor_SetSpeed(hagv->motor_right, speed_r);

  while (HAL_GetTick() - start < total_time) {
    // Blind turn time: wait 1500ms before checking center sensors to clear original line
    if (HAL_GetTick() - start > 1500) {
      uint16_t val = LineSensor_Read(hagv->line_sensor);
      if ((val & CENTER_MASK) != CENTER_MASK) {
        center_found = true;
        break;
      }
    }
    HAL_Delay(5);
  }

  // Phase 2: If line not found, continue turning for up to 800ms
  if (!center_found) {
    while (HAL_GetTick() - start < total_time + 800) {
      uint16_t val = LineSensor_Read(hagv->line_sensor);
      if ((val & CENTER_MASK) != CENTER_MASK) {
        break;
      }
      HAL_Delay(5);
    }
  }

  AGV_Stop(hagv);
  HAL_Delay(200);
}

void AGV_TurnLeft(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  extern volatile uint32_t calib_time_turn_90;
  extern volatile int16_t calib_speed;

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(1000);

  AGV_Stop(hagv);
  HAL_Delay(300);

  Turn_Time_Based(hagv, -calib_speed, calib_speed, calib_time_turn_90);
}

void AGV_TurnRight(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  extern volatile uint32_t calib_time_turn_90;
  extern volatile int16_t calib_speed;

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(1000);

  AGV_Stop(hagv);
  HAL_Delay(300);

  Turn_Time_Based(hagv, calib_speed, -calib_speed, calib_time_turn_90);
}



/* USER CODE END 1 */
