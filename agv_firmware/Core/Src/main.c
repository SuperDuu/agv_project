/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpdma.h"
#include "gpio.h"
#include "icache.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "agv_control.h"
#include "agv_eeprom.h"
#include "agv_routing.h"
#include "esp32_hub.h" // Thư viện giao tiếp ESP32
#include "hmi_modbus.h"
#include "qr50_reader.h"

// #include "wiegand.h"
#include "agv_eeprom.h"
#include "ls7366r.h"
#include "motor.h"
#include "sensor.h"
#include <agv_body_step.h>
#include <stdlib.h> // Để sử dụng hàm atoi
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint32_t spi_test_result_e1 = 0;
volatile uint32_t spi_test_result_e2 = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Các biến phục vụ việc xem trực tiếp trên Live Expression
volatile uint32_t debug_rfid_rx_count = 0;
volatile uint16_t debug_rfid_rx_len = 0;
volatile uint8_t debug_rfid_rx_buf[64] = {0};
volatile uint32_t debug_rfid_err_count = 0;
volatile uint8_t debug_rx_byte = 0;      // Byte nhận tạm thời cho ngắt IT
volatile uint32_t debug_qr_rx_count = 0; // Đếm số lần nhảy vào ngắt nhận QR50
volatile uint32_t debug_qr_err_count =
    0; // Đếm số lần bị lỗi khung truyền (Baudrate)
volatile uint32_t debug_hmi_rx_count = 0; // Đếm số lần nhận được ngắt từ HMI

// === BIẾN GLOBAL ĐỂ DEBUG TRÊN LIVE EXPRESSION ===
AGV_HandleTypeDef h_agv;
Motor_HandleTypeDef m_left, m_right;
LineSensor_HandleTypeDef line_ss;
Pid_data pid_ctrl;
QR50_Handler_t qr50;
uint8_t qr50_rx_buffer[QR50_MAX_DATA_LEN];
Wiegand_HandleTypeDef h_wiegand;
volatile uint32_t debug_wiegand_id = 0; // ID thẻ Wiegand (Live Expression)
volatile uint32_t debug_wiegand_bit_count = 0; // Số bit nhận được (phải = 34)
volatile uint64_t debug_wiegand_raw =
    0; // Raw 34 bit trước khi decode (để chẩn đoán)
volatile uint32_t debug_wiegand_high32 = 0;
volatile uint32_t debug_wiegand_low32 = 0;
// Step thân robot
volatile step_command_t cmd = {.angle = 0, .rpm = 30};
static step_command_t cmd_prev = {0};

// --- CẤU HÌNH MÃ THẺ RFID CỦA TỪNG TRẠM ---
#define RFID_NODE_0 N00
#define RFID_NODE_1 N01
#define RFID_NODE_2 N02
#define RFID_NODE_3 N03
#define RFID_NODE_4 N04
#define RFID_NODE_5 N05
#define RFID_NODE_6 N06
#define RFID_NODE_7 N07
#define RFID_NODE_8 N08

// Mảng ánh xạ RFID sang Node ID (Chỉ mục là Node ID, giá trị là Wiegand ID)
uint32_t rfid_node_map[MAX_NODES] = {
    [0] = RFID_NODE_0, [1] = RFID_NODE_1, [2] = RFID_NODE_2,
    [3] = RFID_NODE_3, [4] = RFID_NODE_4, [5] = RFID_NODE_5,
    [6] = RFID_NODE_6, [7] = RFID_NODE_7, [8] = RFID_NODE_8,
};
volatile LS7366R_EncoderData_t encoder_data;
AGV_Map_t factory_map;
AGV_Heading_t current_heading = HEAD_NORTH;
extern AGV_State_t agv_state;
extern AGV_Config_t agv_config;
extern ESP32_SensorData_t esp32_data;
extern HMI_HandleTypeDef h_hmi;

uint16_t current_path[MAX_NODES];
uint16_t path_length = 0;
static uint16_t last_processed_node = 0xFFFF;
char debug_line_binary[17] = "0000000000000000";
uint16_t pending_qr_node = 0xFFFF;
uint32_t blind_zone_end_time = 0;
uint32_t calib_time_offset = 750;
uint32_t calib_time_turn_90 = 3500;
volatile uint32_t mode6_stop_time = 0;

GPIO_TypeDef *sensor_ports[16] = {
    B_In35_GPIO_Port, B_In34_GPIO_Port, B_In33_GPIO_Port, B_In32_GPIO_Port,
    B_In31_GPIO_Port, B_In30_GPIO_Port, B_In27_GPIO_Port, B_In26_GPIO_Port,
    B_In25_GPIO_Port, B_In24_GPIO_Port, B_In23_GPIO_Port, B_In22_GPIO_Port,
    B_In21_GPIO_Port, B_In20_GPIO_Port, B_In17_GPIO_Port, B_In16_GPIO_Port};
uint16_t sensor_pins[16] = {B_In35_Pin, B_In34_Pin, B_In33_Pin, B_In32_Pin,
                            B_In31_Pin, B_In30_Pin, B_In27_Pin, B_In26_Pin,
                            B_In25_Pin, B_In24_Pin, B_In23_Pin, B_In22_Pin,
                            B_In21_Pin, B_In20_Pin, B_In17_Pin, B_In16_Pin};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define AGV_ARM_FORWARD_PERIOD_MS 20u

static bool AGV_IsLegacyArmCommand(const char *arm_command) {
  return arm_command != NULL &&
         (arm_command[0] == 'L' || arm_command[0] == 'R') &&
         arm_command[1] == ':';
}

static void AGV_ForwardArmCommand(const char *arm_command) {
  if (arm_command == NULL) {
    return;
  }

  size_t cmd_len = strnlen(arm_command, ESP32_MAX_ARM_CMD_LEN);
  if (cmd_len < 3) {
    return;
  }

  if (!AGV_IsLegacyArmCommand(arm_command)) {
    return;
  }

  uint8_t tx_buffer[ESP32_MAX_ARM_CMD_LEN + 2];
  memcpy(tx_buffer, arm_command, cmd_len);

  if (tx_buffer[cmd_len - 1] != '\n') {
    tx_buffer[cmd_len++] = '\n';
  }

  HAL_UART_Transmit(&huart3, tx_buffer, (uint16_t)cmd_len, 50);
}

static void AGV_HandleEsp32Safety(AGV_HandleTypeDef *hagv,
                                  const ESP32_SensorData_t *safe_esp32_data) {
  if (safe_esp32_data == NULL || hagv == NULL)
    return;

  if (!safe_esp32_data->IsConnected)
    return;

  if (safe_esp32_data->Yaw == 65535.0f ||
      safe_esp32_data->ObstacleDistance == 0xFFFF) {
    if (agv_state.follow_line_enable) {
      agv_state.follow_line_enable = false;
      AGV_Stop(hagv);
    }
    return;
  }

  // --- LOGIC VẬT CẢN (VL53L5CX) ---
  bool is_obstacle = false;
  if (safe_esp32_data->ObstacleDistance < 500) {
    bool is_exception = false;
    // Ngoại trừ các đoạn sát tường: 1->0, 4->3, 7->6
    if (path_length > 0 && agv_state.path_index < path_length - 1) {
      uint16_t current_n = current_path[agv_state.path_index];
      uint16_t next_n = current_path[agv_state.path_index + 1];
      if ((current_n == 1 && next_n == 0) || (current_n == 4 && next_n == 3) ||
          (current_n == 7 && next_n == 6)) {
        is_exception = true;
      }
    }

    if (!is_exception) {
      is_obstacle = true;
    }
  }

  static bool paused_by_obstacle = false;

  if (is_obstacle) {
    if (agv_state.follow_line_enable && agv_state.run_mode == MODE_4_FULL_RUN) {
      agv_state.follow_line_enable = false;
      AGV_Stop(hagv);
      paused_by_obstacle = true;
    }
  } else {
    if (paused_by_obstacle) {
      if (agv_state.run_mode == MODE_4_FULL_RUN) {
        agv_state.follow_line_enable = true; // Đi tiếp khi vật cản rời đi
        // Quan trọng: Reset lại bộ đếm timeout QR 15s để tránh bị báo lỗi mất
        // line ngay lập tức
        agv_state.last_qr_time = HAL_GetTick();
      }
      paused_by_obstacle = false;
    }
  }
}

static void AGV_ServiceEsp32Request(uint32_t *last_esp32_req_time) {
  if (last_esp32_req_time == NULL)
    return;

  if (HAL_GetTick() - *last_esp32_req_time > 50) {
    *last_esp32_req_time = HAL_GetTick();
    uint8_t is_arrived =
        (agv_state.current_node == agv_state.destination_node) ? 1 : 0;
    ESP32_RequestData(agv_state.current_node, is_arrived);
  }
}

static void AGV_ServiceHeartbeat(uint32_t *last_led_time) {
  if (last_led_time == NULL)
    return;

  if (HAL_GetTick() - *last_led_time <= 500)
    return;

  *last_led_time = HAL_GetTick();
  HAL_GPIO_TogglePin(SYS_LED1_GPIO_Port, SYS_LED1_Pin);
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_FEF | UART_CLEAR_PEF);

  if (HAL_GetTick() - esp32_data.LastUpdateTick > 2000) {
    extern uint8_t esp32_rx_buffer[64];
    HAL_UART_AbortReceive(&huart5);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, esp32_rx_buffer,
                                 sizeof(esp32_rx_buffer));
  }
}

static uint8_t mode5_calib_state = 0;
static uint32_t mode5_calib_start_time = 0;

static bool AGV_HandleMode5Calibration(void) {

  if (!ESP32_GetSafeData().IsConnected) {
    AGV_Stop(&h_agv);
    return true;
  }

  agv_state.follow_line_enable = false;

  if (mode5_calib_start_time == 0)
    mode5_calib_start_time = HAL_GetTick();

  uint32_t elapsed = HAL_GetTick() - mode5_calib_start_time;

  switch (mode5_calib_state) {
  case 0:
    AGV_Stop(&h_agv);
    if (elapsed > 2000) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 1:
    Motor_SetSpeed(&m_left, agv_config.turn_speed);
    Motor_SetSpeed(&m_right, agv_config.turn_speed);
    if (elapsed > agv_config.time_forward) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 2:
    AGV_Stop(&h_agv);
    if (elapsed > 1000) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 3:
    Motor_SetSpeed(&m_left, -agv_config.turn_speed);
    Motor_SetSpeed(&m_right, -agv_config.turn_speed);
    if (elapsed > agv_config.time_forward) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 4:
    AGV_Stop(&h_agv);
    if (elapsed > 1000) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 5:
    AGV_TurnLeft_IMU(&h_agv, 800, 0.70f);
    mode5_calib_state++;
    mode5_calib_start_time = HAL_GetTick();
    break;
  case 6:
    AGV_Stop(&h_agv);
    if (elapsed > 1000) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 7:
    AGV_Turn180_IMU(&h_agv, 0); // 0 = HEAD_NORTH
    mode5_calib_state++;
    mode5_calib_start_time = HAL_GetTick();
    break;
  case 8:
    AGV_Stop(&h_agv);
    if (elapsed > 1000) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 9:
    Motor_SetSpeed(&m_left, -agv_config.turn_speed);
    Motor_SetSpeed(&m_right, agv_config.turn_speed);
    if (elapsed > agv_config.time_turn_180) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 10:
    AGV_Stop(&h_agv);
    if (elapsed > 1000) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  case 11:
    Motor_SetSpeed(&m_left, agv_config.turn_speed);
    Motor_SetSpeed(&m_right, -agv_config.turn_speed);
    if (elapsed > agv_config.time_turn_180) {
      mode5_calib_state++;
      mode5_calib_start_time = HAL_GetTick();
    }
    break;
  default:
    AGV_Stop(&h_agv);
    return false;
  }

  return true;
}

/**
 * @brief Toggle all Output pins for hardware verification
 * @note Excludes Input pins (B_InX, SDO, RX) to avoid electrical conflict
 */
void Toggle_Outputs(void) {
  HAL_GPIO_TogglePin(B_Out0_GPIO_Port, B_Out0_Pin);
  HAL_GPIO_TogglePin(B_Out1_GPIO_Port, B_Out1_Pin);
  HAL_GPIO_TogglePin(B_Out2_GPIO_Port, B_Out2_Pin);
  HAL_GPIO_TogglePin(B_Out3_GPIO_Port, B_Out3_Pin);
  HAL_GPIO_TogglePin(B_Out4_GPIO_Port, B_Out4_Pin);
  HAL_GPIO_TogglePin(B_Out5_GPIO_Port, B_Out5_Pin);
  HAL_GPIO_TogglePin(B_Out6_GPIO_Port, B_Out6_Pin);
  HAL_GPIO_TogglePin(B_Out7_GPIO_Port, B_Out7_Pin);
  HAL_GPIO_TogglePin(B_Out10_GPIO_Port, B_Out10_Pin);
  HAL_GPIO_TogglePin(B_Out11_GPIO_Port, B_Out11_Pin);
  HAL_GPIO_TogglePin(B_Out12_GPIO_Port, B_Out12_Pin);
  HAL_GPIO_TogglePin(B_Out13_GPIO_Port, B_Out13_Pin);
  HAL_GPIO_TogglePin(B_Out14_GPIO_Port, B_Out14_Pin);
  HAL_GPIO_TogglePin(B_Out15_GPIO_Port, B_Out15_Pin);
  HAL_GPIO_TogglePin(B_Out16_GPIO_Port, B_Out16_Pin);
  HAL_GPIO_TogglePin(B_Out17_GPIO_Port, B_Out17_Pin);
  HAL_GPIO_TogglePin(B_Out20_GPIO_Port, B_Out20_Pin);
  HAL_GPIO_TogglePin(B_Out21_GPIO_Port, B_Out21_Pin);
  HAL_GPIO_TogglePin(B_Out22_GPIO_Port, B_Out22_Pin);
  HAL_GPIO_TogglePin(B_Out23_GPIO_Port, B_Out23_Pin);

  HAL_GPIO_TogglePin(SYS_LED1_GPIO_Port, SYS_LED1_Pin);
  HAL_GPIO_TogglePin(SYS_LED2_GPIO_Port, SYS_LED2_Pin);
  HAL_GPIO_TogglePin(SYS_LED3_GPIO_Port, SYS_LED3_Pin);
}

void Load_Factory_Map(void) {
  Map_Init(&factory_map);
  factory_map.total_nodes = 9;

  Map_AddEdge(&factory_map, N00, N01, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N01, N00, 1, HEAD_SOUTH);
  Map_AddEdge(&factory_map, N01, N02, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N02, N01, 1, HEAD_SOUTH);

  Map_AddEdge(&factory_map, N03, N04, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N04, N03, 1, HEAD_SOUTH);
  Map_AddEdge(&factory_map, N04, N05, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N05, N04, 1, HEAD_SOUTH);

  Map_AddEdge(&factory_map, N06, N07, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N07, N06, 1, HEAD_SOUTH);
  Map_AddEdge(&factory_map, N07, N08, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N08, N07, 1, HEAD_SOUTH);

  Map_AddEdge(&factory_map, N02, N05, 1, HEAD_EAST);
  Map_AddEdge(&factory_map, N05, N02, 1, HEAD_WEST);

  Map_AddEdge(&factory_map, N05, N08, 1, HEAD_EAST);
  Map_AddEdge(&factory_map, N08, N05, 1, HEAD_WEST);
}

static void AGV_HandleIntersectionRouting(uint16_t *pending_qr_node,
                                          uint16_t *last_processed_node,
                                          uint16_t *path_length,
                                          AGV_Heading_t *current_heading) {
  if (!agv_state.is_at_intersection)
    return;

  if (agv_state.run_mode == MODE_2_LINE_INTERSECTION)
    return;

  if (agv_state.run_mode == MODE_6_TEST_TURN_RIGHT) {
    AGV_TurnRight_IMU(&h_agv, 800, 0.70f);
    agv_state.last_leave_intersection_time = HAL_GetTick();
    agv_state.is_at_intersection = false;
    agv_state.follow_line_enable = true;
    return;
  }

  if (agv_state.run_mode == MODE_7_DEBUG_NO_QR) {
    if (*path_length > 0 && agv_state.path_index < *path_length - 1) {
      *pending_qr_node = current_path[agv_state.path_index + 1];
    } else {
      *pending_qr_node = agv_state.destination_node;
    }
  }

  static uint8_t nudge_count = 0;

  if (*pending_qr_node == 0xFFFF) {
    if (agv_state.run_mode == MODE_4_FULL_RUN) {
      if (HAL_GetTick() - agv_state.intersection_time > 1500) {
        if (nudge_count < 3) {
          AGV_BlindForward(&h_agv, 50); // Nhích tới 50ms
          AGV_Stop(&h_agv);
          nudge_count++;
          agv_state.intersection_time = HAL_GetTick();
        } else {
          // Báo lỗi nếu đã nhích 3 lần vẫn không thấy mã
          agv_state.indicator_state = 2;
        }
      }
    }
    return;
  }

  // Đã có mã QR, reset lại số lần nhích để dùng cho ngã tư sau
  nudge_count = 0;

  uint16_t read_node_id = *pending_qr_node;
  *pending_qr_node = 0xFFFF;
  *last_processed_node = read_node_id;
  agv_state.is_at_intersection = false;

  if (read_node_id == agv_state.destination_node) {
    agv_state.current_node = read_node_id;
    agv_state.path_index = 0;
    agv_state.follow_line_enable = false;
    AGV_Stop(&h_agv);
    // EEPROM_SaveState(agv_state.current_node, current_heading);
    return;
  }

  if (*path_length > 0 && agv_state.path_index < *path_length - 1 &&
      read_node_id == current_path[agv_state.path_index + 1]) {
    agv_state.path_index++;
    agv_state.current_node = read_node_id;
  } else if (read_node_id != agv_state.current_node) {
    agv_state.current_node = read_node_id;
    agv_state.is_kidnapped = true; // Bật cờ phát hiện bị bắt cóc
    bool found_path =
        Routing_Dijkstra(&factory_map, agv_state.current_node,
                         agv_state.destination_node, current_path, path_length);
    if (found_path) {
      agv_state.path_index = 0;
    } else {
      agv_state.follow_line_enable = false;
      AGV_Stop(&h_agv);
      return;
    }
  }

  if (*path_length > 0 && agv_state.path_index < *path_length - 1) {
    uint16_t next_node = current_path[agv_state.path_index + 1];
    AGV_Heading_t target_heading =
        Routing_GetHeading(&factory_map, agv_state.current_node, next_node);

    // Loại bỏ việc ép hướng khi bị "bắt cóc" (hoặc khi cố tình bỏ qua node).
    // AGV sẽ dùng chính hướng hiện tại (current_heading) để tính góc rẽ chính
    // xác.
    if (agv_state.is_kidnapped) {
      agv_state.is_kidnapped = false; // Xóa cờ, giữ nguyên current_heading
    }

    int diff = (target_heading - *current_heading + 4) % 4;

    uint32_t fwd_delay = 1900;
    float search_ratio = 0.70f;
    if (agv_state.current_node == 8 && next_node == 7) {
      fwd_delay = 1600;
    }

    if (agv_state.current_node == 2 && next_node == 5) {
      fwd_delay = 1800;
    }
    // Căn chỉnh riêng cho Node 5 để tránh bị văng quá đà khi rẽ
    if (agv_state.current_node == 5) {
      fwd_delay = 1700;
    }

    agv_state.follow_line_enable = false;
    switch ((AGV_Action_t)diff) {
    case ACT_TURN_LEFT:
      h_agv.direction = 1;
      AGV_TurnLeft_IMU(&h_agv, fwd_delay, search_ratio);
      break;
    case ACT_TURN_RIGHT:
      h_agv.direction = 1;
      AGV_TurnRight_IMU(&h_agv, fwd_delay, search_ratio);
      break;
    case ACT_STRAIGHT:
      h_agv.direction = 1;
      // Xe đã bị dừng tại ngã tư, cần chạy thẳng qua trước khi bám line
      AGV_BlindForward(&h_agv, 800);
      break;
    case ACT_BACKWARD:
      h_agv.direction = 1;
      // Đặc biệt: Tại các điểm 1, 4, 7 nếu quay 180 độ về 2, 5, 8 thì phải đi
      // thẳng dò line thêm 1.5s Chiều ngược lại (từ 2 về 1) sẽ KHÔNG chạy logic
      // này.
      if ((agv_state.current_node == 1 && next_node == 2) ||
          (agv_state.current_node == 4 && next_node == 5) ||
          (agv_state.current_node == 7 && next_node == 8)) {
        AGV_TrackLine_Sync(&h_agv, 1500);
      }
      AGV_Turn180_IMU(&h_agv, *current_heading);
      break;
    case ACT_STOP:
    default:
      AGV_Stop(&h_agv);
      break;
    }

    *current_heading = target_heading;
    agv_state.last_leave_intersection_time = HAL_GetTick();
    agv_state.follow_line_enable = true;
    *last_processed_node = 0xFFFF;

    // Đã qua ngã tư và cập nhật góc mới, lưu vào EEPROM
    // EEPROM_SaveState(agv_state.current_node, *current_heading);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  step_Stop(); // Đảm bảo nhả phanh/không ghì động cơ bước lúc khởi động
  LineSensor_Init(&line_ss, sensor_ports, sensor_pins);

  // Khởi tạo Motor trái (PWM CH1) và phải (PWM CH2) trên TIM3 (Khớp phần cứng)
  htim3.Instance->PSC = 31;
  htim3.Instance->ARR = 999;

  Motor_Init(&m_left, &htim3, TIM_CHANNEL_1, B_Out23_GPIO_Port, B_Out23_Pin,
             B_Out22_GPIO_Port, B_Out22_Pin);
  Motor_Init(&m_right, &htim3, TIM_CHANNEL_2, B_Out21_GPIO_Port, B_Out21_Pin,
             B_Out20_GPIO_Port, B_Out20_Pin);

  m_right.InvertDirection = 1;

  pid_ctrl.Kp = 65.0f;
  pid_ctrl.Ki = 0.0f;
  pid_ctrl.Kd = 1.5f;
  pid_ctrl.i = 0.0f;

  AGV_Init(&h_agv, &m_left, &m_right, &line_ss, &pid_ctrl, 300.0f);
  LS7366R_InitAll();
  spi_test_result_e1 = LS7366R_TestSPI(1);
  spi_test_result_e2 = LS7366R_TestSPI(2);

  HMI_Init(&huart1, 1); // Đổi HMI sang USART1 (RS232)

  extern TIM_HandleTypeDef htim6;
  HAL_TIM_Base_Start_IT(&htim6);

  QR50_Init(&qr50, &huart2, 99); // Khởi tạo QR50 trên USART2 (RS485_1)
  Wiegand_Init(&h_wiegand);      // Khởi tạo Wiegand reader

  // Khởi động DMA cho QR50 (RS485_1)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, qr50_rx_buffer, sizeof(qr50_rx_buffer));

  // DMA đã được HMI_Init khởi động cho HMI
  extern HMI_HandleTypeDef h_hmi;
  // Cố gắng start lại DMA cho HMI để chắc chắn (nếu HMI_Init có lỗi/trễ)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, h_hmi.rx_buffer,
                               sizeof(h_hmi.rx_buffer));

  // Khởi tạo kênh giao tiếp Master-Slave với ESP32 qua UART5 (RS485_2)
  ESP32_Init(&huart5);

  Load_Factory_Map();
  //  EEPROM_EraseSector();
  // Đọc trạng thái cũ từ Flash EEPROM (nếu có)
  uint16_t saved_node;
  uint8_t saved_heading;
  if (EEPROM_LoadState(&saved_node, &saved_heading)) {
    agv_state.current_node = saved_node;
    current_heading = (AGV_Heading_t)saved_heading;
  }

  // Khởi động với đường đi trống - chờ HMI đặt đích mới tính
  path_length = 0;
  agv_state.path_index = 0;
  HAL_Delay(2000);
  agv_state.last_qr_time = HAL_GetTick();
  uint32_t last_esp32_req_time = 0;
  uint32_t last_led_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    AGV_UpdateGlobalYaw(); // Cập nhật góc Yaw toàn cục từ IMU
    HMI_Process();

    Wiegand_ProcessLoop(&h_wiegand);
    if (h_wiegand.new_data_ready) {
      debug_wiegand_id = h_wiegand.final_card_id;
      h_wiegand.new_data_ready = false;

      // Cập nhật last_qr_time tương tự như khi quét mã QR
      agv_state.last_qr_time = HAL_GetTick();

      // Ánh xạ Wiegand ID sang Node ID
      uint16_t read_node_id = 0xFFFF;
      for (int i = 0; i < MAX_NODES; i++) {
        if (rfid_node_map[i] == debug_wiegand_id && debug_wiegand_id != 0 &&
            debug_wiegand_id != 0xFFFFFFFF) {
          read_node_id = i;
          break;
        }
      }

      if (read_node_id != 0xFFFF && read_node_id != last_processed_node) {
        pending_qr_node = read_node_id;
      }
    }

    ESP32_SensorData_t safe_esp32_data = ESP32_GetSafeData();
    static uint32_t last_arm_send_tick = 0;
    if (safe_esp32_data.HasNewArmCommandLeft ||
        safe_esp32_data.HasNewArmCommandRight ||
        safe_esp32_data.HasNewArmCommand) {
      ESP32_SensorData_t arm_snapshot;

      __disable_irq();
      arm_snapshot = esp32_data;
      esp32_data.HasNewArmCommand = false;
      esp32_data.HasNewArmCommandLeft = false;
      esp32_data.HasNewArmCommandRight = false;
      __enable_irq();

      // Right arm: Forward raw binary frame
      if (arm_snapshot.HasNewArmCommandRight) {
        if (arm_snapshot.RawArmCommandRight[0] == 0xAA &&
            arm_snapshot.RawArmCommandRight[1] == 0x55) {
          HAL_UART_Transmit(&huart3, (uint8_t *)arm_snapshot.RawArmCommandRight,
                            32, 50);
        }
      }
      // Left arm: Forward legacy text string
      if (arm_snapshot.HasNewArmCommandLeft) {
        AGV_ForwardArmCommand(arm_snapshot.ArmCommandLeft);
      }
      if (!arm_snapshot.HasNewArmCommandRight &&
          !arm_snapshot.HasNewArmCommandLeft && arm_snapshot.HasNewArmCommand) {
        if (AGV_IsLegacyArmCommand(arm_snapshot.ArmCommand)) {
          AGV_ForwardArmCommand(arm_snapshot.ArmCommand);
        }
      }

      last_arm_send_tick = HAL_GetTick();
    } else {
      if (HAL_GetTick() - last_arm_send_tick >= AGV_ARM_FORWARD_PERIOD_MS) {
        last_arm_send_tick = HAL_GetTick();

        // Right arm heartbeat (binary)
        if (safe_esp32_data.RawArmCommandRight[0] == 0xAA &&
            safe_esp32_data.RawArmCommandRight[1] == 0x55) {
          HAL_UART_Transmit(
              &huart3, (uint8_t *)safe_esp32_data.RawArmCommandRight, 32, 50);
        }

        // Left arm heartbeat (text)
        if (AGV_IsLegacyArmCommand(safe_esp32_data.ArmCommandLeft)) {
          AGV_ForwardArmCommand(safe_esp32_data.ArmCommandLeft);
        }

        // Fallback
        if (!AGV_IsLegacyArmCommand(safe_esp32_data.ArmCommandLeft) &&
            safe_esp32_data.RawArmCommandRight[0] != 0xAA &&
            AGV_IsLegacyArmCommand(safe_esp32_data.ArmCommand)) {
          AGV_ForwardArmCommand(safe_esp32_data.ArmCommand);
        }
      }
    }

    AGV_HandleEsp32Safety(&h_agv, &safe_esp32_data);
    if (safe_esp32_data.IsConnected &&
        (safe_esp32_data.Yaw == 65535.0f ||
         safe_esp32_data.ObstacleDistance == 0xFFFF)) {
      continue;
    }

    AGV_ServiceEsp32Request(&last_esp32_req_time);
    AGV_ServiceHeartbeat(&last_led_time);

    if (safe_esp32_data.HasNewCommand) {
      esp32_data.HasNewCommand = false;
      esp32_data.TargetNode = safe_esp32_data.TargetNode;
      esp32_data.H_Command = safe_esp32_data.H_Command;
      if (safe_esp32_data.TargetNode != agv_state.destination_node) {
        agv_state.destination_node = safe_esp32_data.TargetNode;
        agv_state.need_recalculate_path = true;

        // Đồng bộ ngược lại cho màn hình HMI biết ESP32 đã đổi đích đến
        extern uint16_t hmi_registers[];
        hmi_registers[0x05] =
            agv_state.destination_node; // 0x05 is REG_DEST_NODE
      }

      if (safe_esp32_data.H_Command != 0 && safe_esp32_data.H_Command != 255) {
        extern uint16_t hmi_registers[];
        hmi_registers[0x06] = safe_esp32_data.H_Command; // 0x06 is REG_COMMAND
      }
    }

    HMI_SyncData();

    if (agv_state.need_recalculate_path) {
      agv_state.need_recalculate_path = false;

      if (agv_state.follow_line_enable && path_length > 0 &&
          agv_state.path_index < path_length - 1) {
        uint16_t next_node = current_path[agv_state.path_index + 1];
        uint16_t new_path[MAX_NODES];
        uint16_t new_len = 0;

        bool found =
            Routing_Dijkstra(&factory_map, next_node,
                             agv_state.destination_node, new_path, &new_len);
        if (found) {
          for (uint16_t i = 0; i < new_len; i++) {
            if (agv_state.path_index + 1 + i < MAX_NODES) {
              current_path[agv_state.path_index + 1 + i] = new_path[i];
            }
          }
          path_length = agv_state.path_index + 1 + new_len;
        }
      } else {
        bool found = Routing_Dijkstra(&factory_map, agv_state.current_node,
                                      agv_state.destination_node, current_path,
                                      &path_length);
        if (found) {
          agv_state.path_index = 0;
          if (path_length > 1) {
            uint16_t next_node = current_path[1];
            AGV_Heading_t target_heading = Routing_GetHeading(
                &factory_map, agv_state.current_node, next_node);

            // Nếu xe đang bị mất phương hướng do bắt cóc, ép hướng về hướng
            // đích
            if (agv_state.is_kidnapped) {
              current_heading = target_heading;
              agv_state.is_kidnapped = false;
            }

            int diff = (target_heading - current_heading + 4) % 4;
            uint32_t fwd_delay = 1900;
            float search_ratio = 0.70f;
            if (agv_state.current_node == 8 && next_node == 7) {
              fwd_delay = 1600;
            }

            AGV_Action_t next_action = (AGV_Action_t)diff;
            switch (next_action) {
            case ACT_TURN_LEFT:
              h_agv.direction = 1;
              AGV_TurnLeft_IMU(&h_agv, fwd_delay, search_ratio);
              break;
            case ACT_TURN_RIGHT:
              h_agv.direction = 1;
              AGV_TurnRight_IMU(&h_agv, fwd_delay, search_ratio);
              break;
            case ACT_BACKWARD:
              h_agv.direction = 1;
              AGV_Turn180_IMU(&h_agv, current_heading);
              break;
            case ACT_STRAIGHT:
            case ACT_STOP:
            default:
              break;
            }
            current_heading = target_heading;

            // Kiểm tra xem sau khi rẽ, góc của xe có khớp với bản đồ không
            if (!AGV_ValidateHeading((uint8_t)current_heading)) {
              AGV_Stop(&h_agv);
              agv_state.follow_line_enable = false;
              agv_state.indicator_state = 2; // Báo lỗi
              continue; // Bỏ qua đoạn lệnh tiếp theo, yêu cầu can thiệp
            }
          }

          agv_state.follow_line_enable = true;
          agv_state.indicator_state = 0;
          agv_state.last_leave_intersection_time = HAL_GetTick();
          agv_state.last_qr_time = HAL_GetTick();
        } else {
          path_length = 0;
        }
      }
    }

    if (agv_state.run_mode == MODE_8_TEST_ENCODER) {
      static uint32_t m8_start = 0;
      if (m8_start == 0)
        m8_start = HAL_GetTick();

      if (HAL_GetTick() - m8_start > 1000) {
        Motor_SetSpeed(&m_left, 100);
        Motor_SetSpeed(&m_right, 100);
      } else {
        AGV_Stop(&h_agv);
      }
      continue;
    }
    if (agv_state.run_mode == MODE_5_CALIBRATE_MOTORS) {
      if (AGV_HandleMode5Calibration()) {
        continue;
      }
    }

    if (qr50.Data.New_Data_Flag) {
      qr50.Data.New_Data_Flag = false;

      // QR50 gửi xuống theo format N00 -> N08.
      // Parse chặt để tránh nhận nhầm các chuỗi rác/không đủ định dạng.
      if (qr50.Data.Data_Length >= 3 && qr50.Data.Data_Buffer[0] == 'N' &&
          qr50.Data.Data_Buffer[1] >= '0' && qr50.Data.Data_Buffer[1] <= '9' &&
          qr50.Data.Data_Buffer[2] >= '0' && qr50.Data.Data_Buffer[2] <= '9') {
        uint16_t read_node_id =
            (uint16_t)((qr50.Data.Data_Buffer[1] - '0') * 10 +
                       (qr50.Data.Data_Buffer[2] - '0'));

        if (read_node_id < MAX_NODES && read_node_id != last_processed_node) {
          agv_state.last_qr_time = HAL_GetTick();
          pending_qr_node = read_node_id;
        }
      }
    }

    if (agv_state.is_at_intersection) {
      AGV_HandleIntersectionRouting(&pending_qr_node, &last_processed_node,
                                    &path_length, &current_heading);
    }

    // chay Step
    if (cmd.angle != cmd_prev.angle || cmd.rpm != cmd_prev.rpm) {
      cmd_prev = cmd;
      step_Run(&cmd);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();

  /** Configure the programming delay
   */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Cờ toàn cục: khi STM32 đang truyền TX, cấm ErrorCallback restart DMA
volatile bool hmi_tx_in_progress = false;
volatile uint32_t debug_hmi_crc_err = 0;
volatile uint32_t debug_hmi_hw_err = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    debug_hmi_hw_err++; // Tăng biến này nếu HMI nhận rác do nhiễu motor

    // Xóa cờ lỗi UART
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_FEF | UART_CLEAR_PEF);

    // Nếu đang trong quá trình truyền TX, KHÔNG restart DMA (để HMI_Process tự
    // xử lý)
    if (hmi_tx_in_progress)
      return;

    // Chỉ restart DMA khi đang ở chế độ nhận bình thường
    HAL_UART_AbortReceive(huart);
    extern HMI_HandleTypeDef h_hmi;
    HAL_UARTEx_ReceiveToIdle_DMA(h_hmi.huart, h_hmi.rx_buffer,
                                 sizeof(h_hmi.rx_buffer));
  } else if (huart->Instance == USART2) {
    debug_qr_err_count++; // Tăng biến đếm lỗi
    // Xóa cờ lỗi UART
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_FEF | UART_CLEAR_PEF);

    HAL_UART_AbortReceive(huart);
    extern uint8_t qr50_rx_buffer[QR50_MAX_DATA_LEN];
    HAL_UARTEx_ReceiveToIdle_DMA(huart, qr50_rx_buffer, sizeof(qr50_rx_buffer));
  } else if (huart->Instance == UART5) {
    // Xóa cờ lỗi UART
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_FEF | UART_CLEAR_PEF);

    HAL_UART_AbortReceive(huart);
    extern uint8_t esp32_rx_buffer[64];
    HAL_UARTEx_ReceiveToIdle_DMA(huart, esp32_rx_buffer,
                                 sizeof(esp32_rx_buffer));
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) { // RS232 (Dành riêng cho HMI)
    extern HMI_HandleTypeDef h_hmi;
    debug_hmi_rx_count++; // Debug đếm số lần nhận từ HMI

    if (Size >= 8 && h_hmi.rx_buffer[0] == h_hmi.slave_address) {
      HMI_RxCallback(huart, Size);
    } else {
      // Phục hồi lại cơ chế abort UART và restart DMA cho HMI nếu bị lỗi
      HAL_UART_AbortReceive(huart);
      __HAL_UART_CLEAR_FLAG(huart,
                            UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
      HAL_UARTEx_ReceiveToIdle_DMA(huart, h_hmi.rx_buffer,
                                   sizeof(h_hmi.rx_buffer));
    }
  } else if (huart->Instance == USART2) { // RS485_1 (Dành riêng cho QR50)
    extern uint8_t qr50_rx_buffer[QR50_MAX_DATA_LEN];
    debug_qr_rx_count++; // Debug đếm số lần nhận
    if (Size > 0) {
      QR50_ParseData(&qr50, qr50_rx_buffer, Size);
    }

    // Khởi động lại DMA sau khi nhận
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, qr50_rx_buffer, sizeof(qr50_rx_buffer));
  } else if (huart->Instance == UART5) { // RS485_2 (ESP32 Sensor Hub)
    ESP32_ParseResponse(Size);

    // Khởi động lại DMA cho ESP32 sau khi nhận xong
    // (HAL_UARTEx_ReceiveToIdle_DMA tự động dừng sau 1 frame)
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    extern uint8_t esp32_rx_buffer[64];
    HAL_UARTEx_ReceiveToIdle_DMA(huart, esp32_rx_buffer,
                                 sizeof(esp32_rx_buffer));
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    step_TIM_Callback(htim);
  }
  if (htim->Instance == TIM6) {
    // Luôn luôn đọc cảm biến line để phục vụ việc xem trên Live Watch (giống
    // code cũ của bạn)
    uint16_t val = LineSensor_Read(&line_ss);
    for (int i = 0; i < 16; i++) {
      debug_line_binary[i] = (val & (1 << (15 - i))) ? '1' : '0';
    }
    debug_line_binary[16] = '\0';

    // Luôn cập nhật trạng thái Encoder mỗi 10ms (dt = 0.01s)
    encoder_data.count_e1 = LS7366R_ReadCounter(ENC_CS1);
    encoder_data.count_e2 = LS7366R_ReadCounter(ENC_CS2);
    encoder_data.count_e3 = LS7366R_ReadCounter(ENC_CS3);
    encoder_data.count_e4 = LS7366R_ReadCounter(ENC_CS4);

    encoder_data.vel_e1 = LS7366R_GetVelocity(ENC_CS1, 0.01f);
    encoder_data.vel_e2 = LS7366R_GetVelocity(ENC_CS2, 0.01f);
    encoder_data.vel_e3 = LS7366R_GetVelocity(ENC_CS3, 0.01f);
    encoder_data.vel_e4 = LS7366R_GetVelocity(ENC_CS4, 0.01f);

    encoder_data.last_update = HAL_GetTick();

    // Ngắt PID mỗi 10ms - Chỉ điều khiển Motor nếu cờ được bật
    if (agv_state.follow_line_enable) {
      AGV_FollowLine(&h_agv);
    }
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == WG0_Pin) {
    Wiegand_ProcessBit(&h_wiegand, 0);
  } else if (GPIO_Pin == WG1_Pin) {
    Wiegand_ProcessBit(&h_wiegand, 1);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
