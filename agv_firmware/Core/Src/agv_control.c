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
#include <stddef.h>
#include <stdbool.h>

extern volatile bool agv_follow_line_enable;
extern volatile bool is_at_intersection;
extern volatile uint32_t intersection_time;
extern volatile uint32_t last_leave_intersection_time;

// Mặc định chạy Full (thực tế có thể đổi thành MODE_1, 2, 3 tùy nhu cầu test)
volatile AGV_RunMode_t agv_run_mode = MODE_6_TEST_TURN_RIGHT;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

float Delta_t = 0.01f; // Tối ưu delta_t như user cấu hình

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
  // Center
  case 0xFC3F:
    return 0.0f; // 1111110000111111

  // Left deviation 4 units
  case 0xF87F:
    return -1.0f; // 1111100001111111
  case 0xF0FF:
    return -1.5f; // 1111000011111111
  case 0xE1FF:
    return -2.0f; // 1110000111111111
  case 0xC3FF:
    return -2.5f; // 1100001111111111
  case 0x87FF:
    return -3.0f; // 1000011111111111
  case 0x0FFF:
    return -3.5f; // 0000111111111111

  // Left deviation 3 units
  case 0xFC7F:
    return -1.0f; // 1111110001111111
  case 0xF8FF:
    return -1.5f; // 1111100011111111
  case 0xF1FF:
    return -2.0f; // 1111000111111111
  case 0xE3FF:
    return -2.5f; // 1110001111111111
  case 0xC7FF:
    return -3.0f; // 1100011111111111
  case 0x8FFF:
    return -3.5f; // 1000111111111111
  case 0x1FFF:
    return -4.0f; // 0001111111111111

  // Left deviation 5 units
  case 0xF07F:
    return -1.0f; // 1111000001111111
  case 0xE0FF:
    return -1.5f; // 1110000011111111
  case 0xC1FF:
    return -2.0f; // 1100000111111111
  case 0x83FF:
    return -2.5f; // 1000001111111111
  case 0x07FF:
    return -3.0f; // 0000011111111111

  // Right deviation 4 units
  case 0xFE1F:
    return 1.0f; // 1111111000011111
  case 0xFF0F:
    return 1.5f; // 1111111100001111
  case 0xFF87:
    return 2.0f; // 1111111110000111
  case 0xFFC3:
    return 2.5f; // 1111111111000011
  case 0xFFE1:
    return 3.0f; // 1111111111100001
  case 0xFFF0:
    return 3.5f; // 1111111111110000

  // Right deviation 3 units
  case 0xFE3F:
    return 1.0f; // 1111111000111111
  case 0xFF1F:
    return 1.5f; // 1111111100011111
  case 0xFF8F:
    return 2.0f; // 1111111110001111
  case 0xFFC7:
    return 2.5f; // 1111111111000111
  case 0xFFE3:
    return 3.0f; // 1111111111100011
  case 0xFFF1:
    return 3.5f; // 1111111111110001
  case 0xFFF8:
    return 4.0f; // 1111111111111000

  // Right deviation 5 units
  case 0xFE0F:
    return 1.0f; // 1111111000001111
  case 0xFF07:
    return 1.5f; // 1111111100000111
  case 0xFF83:
    return 2.0f; // 1111111110000011
  case 0xFFC1:
    return 2.5f; // 1111111111000001
  case 0xFFE0:
    return 3.0f; // 1111111111100000

  default:
    return current_error;
  }
}

void AGV_FollowLine(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL)
    return;

  static uint32_t lost_line_time = 0;

  // Read sensors
  uint16_t line_val = LineSensor_Read(hagv->line_sensor);

  // Stop condition: all 1s (mạch kéo Pull-up đứt dây) hoặc all 0s (Mất vạch / đứt dây Pull-down)
  if (line_val == 0xFFFF || line_val == 0x0000) {
    if (lost_line_time == 0) lost_line_time = HAL_GetTick();
    
    // Nếu mất vạch liên tục quá 1 giây -> Lỗi phần cứng hoặc văng khỏi line -> Phanh gấp!
    if (HAL_GetTick() - lost_line_time > 1000) {
        if (agv_run_mode == MODE_4_FULL_RUN) {
            agv_follow_line_enable = false; // Chỉ khóa vĩnh viễn khi chạy thật
        }
        AGV_Stop(hagv);
    }
    return;
  } else {
    lost_line_time = 0; // Đã tìm lại được vạch, reset bộ đếm
  }

  // CẢM BIẾN NGÃ TƯ: Kiểm tra 2 mắt ngoài cùng (Bit 15 và Bit 0)
  // Trong MODE_1_LINE_ONLY và MODE_3_TEST_SENSORS, lờ đi ngã tư để cảm biến luôn được đọc
  if (agv_run_mode != MODE_1_LINE_ONLY && agv_run_mode != MODE_3_TEST_SENSORS_NO_MOTOR) {
    if (((line_val & 0x8001) != 0x8001) && (HAL_GetTick() - last_leave_intersection_time > 1500)) {
        AGV_Stop(hagv);
        agv_follow_line_enable = false; // Phanh cứng chờ xử lý QR
        is_at_intersection = true;
        intersection_time = HAL_GetTick();
        lost_line_time = 0; // Reset bộ đếm mất vạch để không bị khóa sau khi rẽ xong
        return;
    }
  }

  // Calculate error
  hagv->current_error = AGV_GetLineError(line_val, hagv->current_error);

  // Compute Line Following PID
  // Setpoint is 0 (center of line), current value is the line error.
  hagv->pid_controller->gtht = hagv->current_error;
  float output = AGV_ComputePID(hagv->pid_controller, 0.0f);

  if (hagv->direction == 1) {
    // Cập nhật tốc độ 2 bánh mượt mà theo PID (Differential Drive)
    // Khi vạch ở bên trái (error < 0), output bị âm -> speed_l giảm, speed_r
    // tăng -> rẽ trái. Khi vạch ở bên phải (error > 0), output dương -> speed_l
    // tăng, speed_r giảm -> rẽ phải.

    // Đảo dấu bù trừ: Sửa theo thực tế để xe bẻ lái đúng hướng vạch
    int16_t speed_l = (int16_t)(hagv->base_speed - output);
    int16_t speed_r = (int16_t)(hagv->base_speed + output);

    // Giới hạn max speed (tránh băm xung quá 999 của Timer)
    if (speed_l > 999)
      speed_l = 999;
    if (speed_r > 999)
      speed_r = 999;

    // Có thể giới hạn tốc độ min là 0 (tránh quay ngược bánh) hoặc cho phép âm
    // để quay góc gắt
    if (speed_l < -300)
      speed_l = -300;
    if (speed_r < -300)
      speed_r = -300;

    // Trong MODE_3, không xuất tốc độ ra Motor để test mạch, quét LED an toàn
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

// ---------------------------------------------------------
// LOGIC RẼ CHUẨN CÔNG NGHIỆP (SPIN TURN - XOAY TẠI CHỖ)
// ---------------------------------------------------------

// Hàm Helper: Xoay mù theo thời gian (Sử dụng 100% lực y như MODE_5)
static void Turn_Time_Based(AGV_HandleTypeDef *hagv, int16_t speed_l, int16_t speed_r, uint32_t total_time) {
  // Cấp điện 100% sức mạnh như lúc bạn Calib
  Motor_SetSpeed(hagv->motor_left, speed_l);
  Motor_SetSpeed(hagv->motor_right, speed_r);
  
  // Chờ đúng khoảng thời gian đã được tinh chỉnh
  HAL_Delay(total_time);

  // Phanh chết ngay lập tức
  AGV_Stop(hagv);
  HAL_Delay(200); 
}

void AGV_TurnLeft(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR) return;

  // Gọi biến thời gian và tốc độ từ lúc Calib
  extern volatile uint32_t calib_time_offset;
  extern volatile uint32_t calib_time_turn_90;
  extern volatile int16_t calib_speed;

  // Bù trừ cơ khí: Chạy thẳng để đẩy trục bánh xe vào giữa ngã tư
  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(calib_time_offset);
  
  // Rẽ trái: Bánh trái lùi, bánh phải tiến. Chạy bằng hàm Time_Based
  Turn_Time_Based(hagv, -calib_speed, calib_speed, calib_time_turn_90);
}

void AGV_TurnRight(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR) return;

  extern volatile uint32_t calib_time_offset;
  extern volatile uint32_t calib_time_turn_90;
  extern volatile int16_t calib_speed;

  // Bù trừ cơ khí
  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(calib_time_offset);
  
  // Rẽ phải: Bánh trái tiến, bánh phải lùi
  Turn_Time_Based(hagv, calib_speed, -calib_speed, calib_time_turn_90);
}

void AGV_Turn180(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR) return;

  extern volatile uint32_t calib_time_offset;
  extern volatile uint32_t calib_time_turn_180;
  extern volatile int16_t calib_speed;

  // Bù trừ cơ khí
  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(calib_time_offset);
  
  // Xoay 180 độ: Thuận chiều kim đồng hồ (Xoay phải)
  Turn_Time_Based(hagv, calib_speed, -calib_speed, calib_time_turn_180);
}

/* USER CODE END 1 */
