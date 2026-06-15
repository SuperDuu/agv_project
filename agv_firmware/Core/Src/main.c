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
#include <stdlib.h> // Để sử dụng hàm atoi

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

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_NodeTypeDef Node_GPDMA1_Channel2;
DMA_QListTypeDef List_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel2;
DMA_NodeTypeDef Node_GPDMA1_Channel3;
DMA_QListTypeDef List_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

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
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
        // Quan trọng: Reset lại bộ đếm timeout QR 15s để tránh bị báo lỗi mất line ngay lập tức
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
    extern uint8_t esp32_rx_buffer[15];
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
  EEPROM_EraseSector();
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

/**
 * @brief GPDMA1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPDMA1_Init(void) {

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void) {

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 249;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 249;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE,
                    T_ENC_CS3_Pin | T_ENC_CS4_Pin | B_Out0_Pin | B_Out1_Pin |
                        B_Out2_Pin | B_Out15_Pin | T_ENC_CS1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    B_Out3_Pin | B_Out10_Pin | B_Out11_Pin | B_Out12_Pin |
                        SYS_LAN_CS_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF,
                    B_Out4_Pin | B_Out5_Pin | B_Out6_Pin | B_Out7_Pin |
                        SYS_LED2_Pin | SYS_LED3_Pin | B_Out16_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B_Out13_Pin | T_ADC_CLK_Pin | T_ADC_SDI_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD,
                    B_Out14_Pin | B_Out17_Pin | B_Out21_Pin | B_Out20_Pin |
                        SYS_LED1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG,
                    B_Out23_Pin | B_Out22_Pin | T_DAC_SDI_Pin | T_DAC_CLK_Pin |
                        T_DAC_CS_Pin | T_ENC_CS2_Pin | T_ADC_CS_Pin |
                        T_ADC_RST_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : T_ENC_CS3_Pin T_ENC_CS4_Pin T_ENC_CS1_Pin */
  GPIO_InitStruct.Pin = T_ENC_CS3_Pin | T_ENC_CS4_Pin | T_ENC_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out0_Pin B_Out1_Pin B_Out2_Pin B_Out15_Pin */
  GPIO_InitStruct.Pin = B_Out0_Pin | B_Out1_Pin | B_Out2_Pin | B_Out15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out3_Pin B_Out10_Pin B_Out11_Pin B_Out12_Pin
                           SYS_LAN_CS_Pin */
  GPIO_InitStruct.Pin =
      B_Out3_Pin | B_Out10_Pin | B_Out11_Pin | B_Out12_Pin | SYS_LAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out4_Pin B_Out5_Pin B_Out6_Pin B_Out7_Pin
                           SYS_LED2_Pin SYS_LED3_Pin B_Out16_Pin */
  GPIO_InitStruct.Pin = B_Out4_Pin | B_Out5_Pin | B_Out6_Pin | B_Out7_Pin |
                        SYS_LED2_Pin | SYS_LED3_Pin | B_Out16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In2_Pin B_In3_Pin B_In4_Pin B_In5_Pin */
  GPIO_InitStruct.Pin = B_In2_Pin | B_In3_Pin | B_In4_Pin | B_In5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : WG1_Pin */
  GPIO_InitStruct.Pin = WG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WG0_Pin */
  GPIO_InitStruct.Pin = WG0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WG0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In10_Pin B_In11_Pin B_In12_Pin B_In13_Pin */
  GPIO_InitStruct.Pin = B_In10_Pin | B_In11_Pin | B_In12_Pin | B_In13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B_In14_Pin */
  GPIO_InitStruct.Pin = B_In14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_In14_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out13_Pin T_ADC_CLK_Pin T_ADC_SDI_Pin */
  GPIO_InitStruct.Pin = B_Out13_Pin | T_ADC_CLK_Pin | T_ADC_SDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B_In30_Pin */
  GPIO_InitStruct.Pin = B_In30_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B_In30_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In16_Pin B_In26_Pin B_In27_Pin B_In35_Pin
                           B_In34_Pin */
  GPIO_InitStruct.Pin =
      B_In16_Pin | B_In26_Pin | B_In27_Pin | B_In35_Pin | B_In34_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : B_In15_Pin */
  GPIO_InitStruct.Pin = B_In15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_In15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In17_Pin B_In21_Pin B_In20_Pin B_In23_Pin
                           B_In22_Pin B_In25_Pin B_In24_Pin */
  GPIO_InitStruct.Pin = B_In17_Pin | B_In21_Pin | B_In20_Pin | B_In23_Pin |
                        B_In22_Pin | B_In25_Pin | B_In24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : T_ADC_SDO_Pin */
  GPIO_InitStruct.Pin = T_ADC_SDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_ADC_SDO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out14_Pin B_Out17_Pin B_Out21_Pin B_Out20_Pin
                           SYS_LED1_Pin */
  GPIO_InitStruct.Pin =
      B_Out14_Pin | B_Out17_Pin | B_Out21_Pin | B_Out20_Pin | SYS_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out23_Pin B_Out22_Pin T_DAC_SDI_Pin T_DAC_CLK_Pin
                           T_DAC_CS_Pin T_ADC_CS_Pin T_ADC_RST_Pin */
  GPIO_InitStruct.Pin = B_Out23_Pin | B_Out22_Pin | T_DAC_SDI_Pin |
                        T_DAC_CLK_Pin | T_DAC_CS_Pin | T_ADC_CS_Pin |
                        T_ADC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In32_Pin B_In31_Pin B_In33_Pin */
  GPIO_InitStruct.Pin = B_In32_Pin | B_In31_Pin | B_In33_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : T_ENC_CS2_Pin */
  GPIO_InitStruct.Pin = T_ENC_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(T_ENC_CS2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
    extern uint8_t esp32_rx_buffer[15];
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
    extern uint8_t esp32_rx_buffer[15];
    HAL_UARTEx_ReceiveToIdle_DMA(huart, esp32_rx_buffer,
                                 sizeof(esp32_rx_buffer));
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
