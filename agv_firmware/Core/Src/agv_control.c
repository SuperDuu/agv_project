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
volatile AGV_RunMode_t agv_run_mode = MODE_4_FULL_RUN;

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
        agv_follow_line_enable = false; 
        AGV_Stop(hagv);
    }
    return;
  } else {
    lost_line_time = 0; // Đã tìm lại được vạch, reset bộ đếm
  }

  // CẢM BIẾN NGÃ TƯ: Kiểm tra 2 mắt ngoài cùng (Bit 15 và Bit 0)
  // Trong MODE_1_LINE_ONLY, ta lờ đi tín hiệu ngã tư để chỉ test bám vạch
  if (agv_run_mode != MODE_1_LINE_ONLY) {
    if (((line_val & 0x8001) != 0x8001) && (HAL_GetTick() - last_leave_intersection_time > 1500)) {
        AGV_Stop(hagv);
        agv_follow_line_enable = false; // Phanh cứng chờ xử lý QR
        is_at_intersection = true;
        intersection_time = HAL_GetTick();
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

    int16_t speed_l = (int16_t)(hagv->base_speed + output);
    int16_t speed_r = (int16_t)(hagv->base_speed - output);

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

void AGV_TurnLeft(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  uint16_t line_val;

  // Bù trừ cơ khí: Chạy thẳng 300ms để đẩy trục bánh xe vào giữa ngã tư
  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(300);

  // Giai đoạn 1: Xoay mù để thoát vạch cũ
  // Rẽ trái -> Bánh trái lùi, Bánh phải tiến
  Motor_SetSpeed(hagv->motor_left, -500);
  Motor_SetSpeed(hagv->motor_right, 500);

  // Delay mù 500ms (Bạn có thể tăng giảm tùy tốc độ bánh xe)
  // Mục đích: Cho xe văng hẳn ra khỏi đường băng dính hiện tại
  HAL_Delay(500);

  // Giai đoạn 2: Tiếp tục xoay và chờ bắt vạch mới
  uint32_t start_time = HAL_GetTick();
  int debounce_cnt = 0;
  
  while (1) {
    line_val = LineSensor_Read(hagv->line_sensor);

    // 0x0180 tương đương nhị phân là 0000 0001 1000 0000
    // -> Khi vạch từ đè lên 2 cảm biến ở TÂM xe (bit 7 và 8), cờ này sẽ khác 0
    if ((line_val & 0x0180) != 0x0180) {
        debounce_cnt++;
        if (debounce_cnt >= 3) { // Chống nhiễu: Phải đọc thấy vạch 3 lần liên tiếp
            break; // Đã bắt được vạch 90 độ! Thoát vòng lặp.
        }
    } else {
        debounce_cnt = 0;
    }
    
    // Watchdog Timer: Nếu xoay quá 3 giây không bắt được vạch -> Lỗi trượt bánh hoặc đứt vạch
    if (HAL_GetTick() - start_time > 3000) {
        agv_follow_line_enable = false; // Báo lỗi toàn hệ thống
        break;
    }
    
    HAL_Delay(5); // Đọc cảm biến mỗi 5ms
  }

  // Giai đoạn 3: Phanh lại để triệt tiêu quán tính
  AGV_Stop(hagv);
  HAL_Delay(200); // Chờ xe đứng yên hẳn trước khi nhường quyền cho bám vạch
}

void AGV_TurnRight(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  uint16_t line_val;

  // Bù trừ cơ khí: Chạy thẳng 300ms để đẩy trục bánh xe vào giữa ngã tư
  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(300);

  // Rẽ phải -> Bánh trái tiến, Bánh phải lùi
  Motor_SetSpeed(hagv->motor_left, 500);
  Motor_SetSpeed(hagv->motor_right, -500);

  // Xoay mù thoát vạch
  HAL_Delay(500);

  // Chờ bắt vạch mới
  uint32_t start_time = HAL_GetTick();
  int debounce_cnt = 0;
  
  while (1) {
    line_val = LineSensor_Read(hagv->line_sensor);
    if ((line_val & 0x0180) != 0x0180) {
        debounce_cnt++;
        if (debounce_cnt >= 3) break;
    } else {
        debounce_cnt = 0;
    }
    
    if (HAL_GetTick() - start_time > 3000) {
        agv_follow_line_enable = false; 
        break;
    }
    HAL_Delay(5);
  }

  AGV_Stop(hagv);
  HAL_Delay(200);
}

void AGV_Turn180(AGV_HandleTypeDef *hagv) {
  if (hagv == NULL || agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR)
    return;

  uint16_t line_val;

  // Bù trừ cơ khí: Chạy thẳng 300ms để đẩy trục bánh xe vào giữa ngã tư
  Motor_SetSpeed(hagv->motor_left, (int16_t)hagv->base_speed);
  Motor_SetSpeed(hagv->motor_right, (int16_t)hagv->base_speed);
  HAL_Delay(300);

  // Xoay 180 độ (Thường chọn xoay phải cho thuận)
  Motor_SetSpeed(hagv->motor_left, 500);
  Motor_SetSpeed(hagv->motor_right, -500);

  // QUAN TRỌNG: Quay 180 độ tốn gấp đôi thời gian so với quay 90 độ!
  // Bạn cần TĂNG thời gian delay mù lên (ví dụ 1000ms hoặc 1200ms).
  // Nếu ngã tư có vạch 90 độ, delay mù đủ lâu sẽ giúp xe "lướt qua" và phớt lờ
  // vạch 90 độ. Nó chỉ dừng lại khi bắt được vạch thứ hai (vạch 180 độ nằm sau
  // lưng).
  HAL_Delay(1200);

  uint32_t start_time = HAL_GetTick();
  int debounce_cnt = 0;
  
  while (1) {
    line_val = LineSensor_Read(hagv->line_sensor);
    if ((line_val & 0x0180) != 0x0180) {
        debounce_cnt++;
        if (debounce_cnt >= 3) break; // Đã bắt được vạch sau lưng
    } else {
        debounce_cnt = 0;
    }
    
    if (HAL_GetTick() - start_time > 4000) { // Quay 180 độ cần nhiều thời gian hơn (4s)
        agv_follow_line_enable = false; 
        break;
    }
    HAL_Delay(5);
  }

  AGV_Stop(hagv);
  HAL_Delay(200);
}

/* USER CODE END 1 */
