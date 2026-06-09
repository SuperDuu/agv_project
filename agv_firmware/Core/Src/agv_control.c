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
#include "esp32_hub.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

AGV_State_t agv_state = {.run_mode = MODE_7_DEBUG_NO_QR,
                         .indicator_state = 0,
                         .follow_line_enable = false,
                         .is_at_intersection = false,
                         .intersection_time = 0,
                         .last_leave_intersection_time = 0,
                         .current_node = 0,
                         .destination_node = 0,
                         .need_recalculate_path = false,
                         .last_qr_time = 0,
                         .path_index = 0};

AGV_Config_t agv_config = {.time_offset = 400, // Reduced from 600 because of higher speed
                           .time_forward = 2000,
                           .time_turn_90 = 3100,
                           .time_turn_180 = 6200,
                           .turn_speed = 150, // Keep turn speed moderate to avoid overshoot
                           .base_speed = 250}; // Increased from 150 to 250
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
  hagv->current_speed = 0.0f;
  hagv->direction = 1;
  hagv->current_error = 0.0f;
}

static float AGV_ComputePID(Pid_data *pid, float setpoint) {
  if (pid == NULL)
    return 0.0f;

  pid->error = setpoint - pid->current_val;
  pid->i += pid->error * Delta_t;
  pid->d = (pid->error - pid->prev_error) / Delta_t;

  float output =
      (pid->Kp * pid->error) + (pid->Kd * pid->d) + (pid->Ki * pid->i);
  pid->prev_error = pid->error;

  return output;
}

float AGV_GetLineError(uint16_t line_value, float current_error) {
  switch (line_value) {
  case 0xFC3F:
    return 0.0f;

  // Left deviation 4 units
  case 0xF87F:
    return -1.0f;
  case 0xF0FF:
    return -1.5f;
  case 0xE1FF:
    return -2.0f;
  case 0xC3FF:
    return -2.5f;
  case 0x87FF:
    return -3.0f;
  case 0x0FFF:
    return -3.5f;

  // Left deviation 3 units
  case 0xFC7F:
    return -1.0f;
  case 0xF8FF:
    return -1.5f;
  case 0xF1FF:
    return -2.0f;
  case 0xE3FF:
    return -2.5f;
  case 0xC7FF:
    return -3.0f;
  case 0x8FFF:
    return -3.5f;
  case 0x1FFF:
    return -4.0f;

  // Left deviation 5 units
  case 0xF07F:
    return -1.0f;
  case 0xE0FF:
    return -1.5f;
  case 0xC1FF:
    return -2.0f;
  case 0x83FF:
    return -2.5f;
  case 0x07FF:
    return -3.0f;

  // Right deviation 4 units
  case 0xFE1F:
    return 1.0f;
  case 0xFF0F:
    return 1.5f;
  case 0xFF87:
    return 2.0f;
  case 0xFFC3:
    return 2.5f;
  case 0xFFE1:
    return 3.0f;
  case 0xFFF0:
    return 3.5f;

  // Right deviation 3 units
  case 0xFE3F:
    return 1.0f;
  case 0xFF1F:
    return 1.5f;
  case 0xFF8F:
    return 2.0f;
  case 0xFFC7:
    return 2.5f;
  case 0xFFE3:
    return 3.0f;
  case 0xFFF1:
    return 3.5f;
  case 0xFFF8:
    return 4.0f;

  // Right deviation 5 units
  case 0xFE0F:
    return 1.0f;
  case 0xFF07:
    return 1.5f;
  case 0xFF83:
    return 2.0f;
  case 0xFFC1:
    return 2.5f;
  case 0xFFE0:
    return 3.0f;

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
      if (agv_state.run_mode == MODE_4_FULL_RUN) {
        agv_state.follow_line_enable = false;
        agv_state.indicator_state = 3; // Lỗi mất line (State 3: Error)
      }
      AGV_Stop(hagv);
    }
    return;
  } else {
    lost_line_time = 0;
  }

  // Intersection check: Bit 15 and Bit 0
  if (agv_state.run_mode != MODE_1_LINE_ONLY &&
      agv_state.run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
    if (((line_val & 0x8001) != 0x8001) &&
        (HAL_GetTick() - agv_state.last_leave_intersection_time >
         AGV_LINE_RECOVERY_TIME)) {
      AGV_Stop(hagv);
      agv_state.follow_line_enable = false;
      agv_state.is_at_intersection = true;
      agv_state.intersection_time = HAL_GetTick();
      lost_line_time = 0;
      return;
    }
  }

  hagv->current_error = AGV_GetLineError(line_val, hagv->current_error);
  hagv->pid_controller->current_val = hagv->current_error;
  float output = AGV_ComputePID(hagv->pid_controller, 0.0f);

  // Ramping (Tăng/Giảm tốc mềm mại)
  float accel_step = 1.5f; // Chỉnh gia tốc tại đây (1.5 * 100Hz = 150 PWM/s)
  if (hagv->current_speed < hagv->base_speed) {
    hagv->current_speed += accel_step;
    if (hagv->current_speed > hagv->base_speed) hagv->current_speed = hagv->base_speed;
  } else if (hagv->current_speed > hagv->base_speed) {
    hagv->current_speed -= accel_step;
    if (hagv->current_speed < hagv->base_speed) hagv->current_speed = hagv->base_speed;
  }

  if (hagv->direction == 1) {
    int16_t speed_l = (int16_t)(hagv->current_speed - output);
    int16_t speed_r = (int16_t)(hagv->current_speed + output);

    if (speed_l > 999)
      speed_l = 999;
    if (speed_r > 999)
      speed_r = 999;
    if (speed_l < -300)
      speed_l = -300;
    if (speed_r < -300)
      speed_r = -300;

    if (agv_state.run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
      Motor_SetSpeed(hagv->motor_left, speed_l);
      Motor_SetSpeed(hagv->motor_right, speed_r);
    }
  } else if (hagv->direction == -1) {
    // Đảo dấu logic PID khi đi lùi (do cảm biến nằm ở mũi xe nhưng xe lùi)
    int16_t speed_l = (int16_t)(-hagv->current_speed + output);
    int16_t speed_r = (int16_t)(-hagv->current_speed - output);

    if (speed_l < -999)
      speed_l = -999;
    if (speed_r < -999)
      speed_r = -999;
    if (speed_l > 300)
      speed_l = 300;
    if (speed_r > 300)
      speed_r = 300;

    if (agv_state.run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
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
  hagv->current_speed = 0.0f; // Reset speed for ramping when resuming
}

// Mask for 4 center sensors (bits 9-6): 0x03C0 = 0000001111000000
#define CENTER_MASK 0x03C0

static void Turn_Time_Based(AGV_HandleTypeDef *hagv, int16_t speed_l,
                            int16_t speed_r, uint32_t total_time) {
  bool center_found = false;
  uint32_t start = HAL_GetTick();

  agv_state.indicator_state = 2; // State 2: Turning

  Motor_SetSpeed(hagv->motor_left, speed_l);
  Motor_SetSpeed(hagv->motor_right, speed_r);

  extern void HMI_Process(void);

  while (HAL_GetTick() - start < total_time) {
    HMI_Process(); // Keep Modbus alive during blocking turn
    // Blind turn time before checking center sensors to clear original line
    if (HAL_GetTick() - start > AGV_TURN_BLIND_TIME) {
      uint16_t val = LineSensor_Read(hagv->line_sensor);
      if ((val & CENTER_MASK) != CENTER_MASK) {
        center_found = true;
        break;
      }
    }
    HAL_Delay(5);
  }

  // Phase 2: If line not found, continue turning until line is reacquired.
  if (!center_found) {
    while (1) {
      HMI_Process(); // Keep Modbus alive during blocking turn
      uint16_t val = LineSensor_Read(hagv->line_sensor);
      if ((val & CENTER_MASK) != CENTER_MASK) {
        center_found = true;
        break;
      }
      HAL_Delay(5);
    }
  }

  AGV_Stop(hagv);
  agv_state.indicator_state = 0; // Turn complete
  HAL_Delay(120);
}

void AGV_TurnLeft(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_state.run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(700);

  AGV_Stop(hagv);
  HAL_Delay(500);

  Turn_Time_Based(hagv, -agv_config.turn_speed, agv_config.turn_speed,
                  agv_config.time_turn_90);
}

void AGV_TurnRight(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_state.run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(700);

  AGV_Stop(hagv);
  HAL_Delay(500);

  Turn_Time_Based(hagv, agv_config.turn_speed, -agv_config.turn_speed,
                  agv_config.time_turn_90);
}

// void AGV_Turn180(AGV_HandleTypeDef *hagv) {
//   if (hagv == NULL || agv_state.run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
//     return;
//
//   AGV_Stop(hagv);
//   HAL_Delay(300);
//
//   // 1. Quay mù 45 độ sang phải (bằng nửa thời gian quay 90 độ)
//   uint32_t time_45 = agv_config.time_turn_90 / 2;
//   Motor_SetSpeed(hagv->motor_left, 700); // Kick-start
//   Motor_SetSpeed(hagv->motor_right, -700);
//   HAL_Delay(80);
//   Motor_SetSpeed(hagv->motor_left, agv_config.turn_speed);
//   Motor_SetSpeed(hagv->motor_right, -agv_config.turn_speed);
//   HAL_Delay(time_45 - 80);
//
//   AGV_Stop(hagv);
//   HAL_Delay(300);
//
//   // 2. Lùi thẳng lại 1 giây (kéo xe về hướng Tây Nam để né tường phải và
//   tường
//   // trước)
//   Motor_SetSpeed(hagv->motor_left, -hagv->base_speed);
//   Motor_SetSpeed(hagv->motor_right, -hagv->base_speed);
//   HAL_Delay(1000);
//
//   AGV_Stop(hagv);
//   HAL_Delay(300);
//
//   // 3. Chọn chiều quay (phải) để quay tiếp 135 độ còn lại và dò vạch
//   uint32_t time_135 = agv_config.time_turn_180 - time_45;
//   Turn_Time_Based(hagv, agv_config.turn_speed, -agv_config.turn_speed,
//                   time_135);
// }

static bool Turn_FindCenterLine(AGV_HandleTypeDef *hagv) {
  uint16_t val = LineSensor_Read(hagv->line_sensor);
  return (val & CENTER_MASK) != CENTER_MASK;
}

static void Turn_IMU_Based(AGV_HandleTypeDef *hagv, float target_angle,
                           int16_t speed_l, int16_t speed_r, bool search_line) {
  uint32_t start_time = HAL_GetTick();
  float start_yaw = ESP32_GetSafeData().Yaw;
  bool line_search_enabled = search_line && (target_angle > 0.0f);
  bool line_found = false;
  bool target_reached = false;

  agv_state.indicator_state = 2;

  Motor_SetSpeed(hagv->motor_left, speed_l);
  Motor_SetSpeed(hagv->motor_right, speed_r);

  extern void HMI_Process(void);
  static uint32_t last_esp_req = 0;
  uint32_t timeout_limit = (target_angle > 120.0f) ? 8000 : 5500;

  while (1) {
    HMI_Process();

    if (HAL_GetTick() - last_esp_req > 50) {
      last_esp_req = HAL_GetTick();
      ESP32_RequestData(agv_state.current_node);
    }

    float current_yaw = ESP32_GetSafeData().Yaw;
    float diff = fabs(current_yaw - start_yaw);
    while (diff > 180.0f)
      diff = 360.0f - diff;

    if (!target_reached && diff >= (target_angle - 2.0f))
      target_reached = true;

    if (line_search_enabled && diff >= (target_angle * 0.70f) &&
        Turn_FindCenterLine(hagv)) {
      line_found = true;
      break;
    }

    if (target_reached) {
      if (!line_search_enabled)
        break;
      if (line_search_enabled && Turn_FindCenterLine(hagv)) {
        line_found = true;
        break;
      }
    }

    if (HAL_GetTick() - start_time > timeout_limit)
      break;

    HAL_Delay(5);
  }

  (void)line_found;
  AGV_Stop(hagv);
  agv_state.indicator_state = 0;
  HAL_Delay(120);
}

void AGV_TurnLeft_IMU(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_state.run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  bool enable_search = (agv_state.run_mode != MODE_5_CALIBRATE_MOTORS);

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(700);

  AGV_Stop(hagv);
  HAL_Delay(500);

  Turn_IMU_Based(hagv, 80.0f, -agv_config.turn_speed, agv_config.turn_speed,
                 enable_search);
}

void AGV_TurnRight_IMU(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_state.run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  bool enable_search = (agv_state.run_mode != MODE_5_CALIBRATE_MOTORS);

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(700);

  AGV_Stop(hagv);
  HAL_Delay(500);

  Turn_IMU_Based(hagv, 70.0f, agv_config.turn_speed, -agv_config.turn_speed,
                 enable_search);
}

void AGV_Turn180_IMU(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_state.run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  bool enable_search = (agv_state.run_mode != MODE_5_CALIBRATE_MOTORS);

  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(150);

  AGV_Stop(hagv);
  HAL_Delay(500);

  Turn_IMU_Based(hagv, 170.0f, agv_config.turn_speed, -agv_config.turn_speed,
                 enable_search);
}

/* USER CODE END 1 */
