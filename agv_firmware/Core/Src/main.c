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
#include "agv_routing.h"
#include "motor.h"
#include "qr50_reader.h"
#include "sensor.h"
#include <stdlib.h> // Để sử dụng hàm atoi

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
AGV_HandleTypeDef h_agv;
Motor_HandleTypeDef m_left, m_right;
LineSensor_HandleTypeDef line_ss;
Pid_data pid_ctrl;

uint8_t rx_scan[2];  // Mảng nhận dữ liệu từ USART3 qua DMA
QR50_Handler_t qr50; // Đầu đọc QR50 trên USART2
char debug_line_binary[17] = "0000000000000000";
AGV_Map_t factory_map;
uint16_t current_path[MAX_NODES];
uint16_t path_length = 0;
int path_index = 0;
uint16_t current_node = 0;
uint16_t destination_node = 6;
AGV_Heading_t current_heading =
    HEAD_NORTH;
volatile bool agv_follow_line_enable = true;
static uint16_t last_processed_node = 0xFFFF;

volatile bool is_at_intersection = false; // Cờ báo hiệu chạm ngã tư
volatile uint32_t intersection_time = 0;
uint16_t pending_qr_node = 0xFFFF; // Bộ đệm chứa mã QR chờ xử lý
volatile uint32_t last_leave_intersection_time =
    0; // Blind zone: Hẹn giờ mù ngã tư cũ

// Các biến cấu hình cho chế độ MODE_5_CALIBRATE_MOTORS
volatile uint32_t calib_time_offset =
    750; // Thời gian bù trừ tiến lên tâm ngã tư (ms)
volatile uint32_t calib_time_forward = 2000; // Thời gian chạy thẳng/lùi (ms)
volatile uint32_t calib_time_turn_90 =
    3500; // Thời gian xoay 90 độ (tăng theo speed 150)
volatile uint32_t calib_time_turn_180 =
    3000; // Thời gian xoay 180 độ đã được Calib chuẩn
volatile int16_t calib_speed =
    150; // Tốc độ quay (giảm từ 300 để quay chậm, mượt hơn)

volatile uint32_t mode6_stop_time = 0; // Biến hẹn giờ 3s cho MODE 6

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  factory_map.total_nodes = 15;

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

  Map_AddEdge(&factory_map, N09, N10, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N10, N09, 1, HEAD_SOUTH);
  Map_AddEdge(&factory_map, N10, N11, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N11, N10, 1, HEAD_SOUTH);

  Map_AddEdge(&factory_map, N12, N13, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N13, N12, 1, HEAD_SOUTH);
  Map_AddEdge(&factory_map, N13, N14, 1, HEAD_NORTH);
  Map_AddEdge(&factory_map, N14, N13, 1, HEAD_SOUTH);

  Map_AddEdge(&factory_map, N02, N05, 1, HEAD_EAST);
  Map_AddEdge(&factory_map, N05, N02, 1, HEAD_WEST);

  Map_AddEdge(&factory_map, N05, N08, 1, HEAD_EAST);
  Map_AddEdge(&factory_map, N08, N05, 1, HEAD_WEST);

  Map_AddEdge(&factory_map, N08, N11, 1, HEAD_EAST);
  Map_AddEdge(&factory_map, N11, N08, 1, HEAD_WEST);

  Map_AddEdge(&factory_map, N11, N14, 1, HEAD_EAST);
  Map_AddEdge(&factory_map, N14, N11, 1, HEAD_WEST);
}

uint16_t debug_path[20];
uint16_t debug_path_len = 0;
AGV_Heading_t debug_actions[20];

void Debug_Test_N11(void) {
  Routing_Dijkstra(&factory_map, N00, N11, debug_path, &debug_path_len);

  if (debug_path_len > 1) {
    for (int i = 0; i < debug_path_len - 1; i++) {
      debug_actions[i] = Routing_GetHeading(&factory_map, debug_path[i], debug_path[i+1]);
    }
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

  pid_ctrl.Kp = 85.0f;
  pid_ctrl.Ki = 0.0f;
  pid_ctrl.Kd = 0.5f;
  pid_ctrl.i = 0.0f;

  AGV_Init(&h_agv, &m_left, &m_right, &line_ss, &pid_ctrl, 300.0f);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_scan, 2);

  extern TIM_HandleTypeDef htim6;
  HAL_TIM_Base_Start_IT(&htim6);

  QR50_Init(&qr50, &huart2, 255);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, qr50.Data.Data_Buffer,
                               QR50_MAX_DATA_LEN);

  Load_Factory_Map();
  Debug_Test_N11();
  bool initial_path_found = Routing_Dijkstra(
      &factory_map, current_node, destination_node, current_path, &path_length);
  if (!initial_path_found && agv_run_mode == MODE_4_FULL_RUN) {
    agv_follow_line_enable = false;
  }
  path_index = 0;

  uint32_t last_qr_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // STATE MACHINE CHUYÊN DỤNG CHO CALIBRATION (MODE 5)
    if (agv_run_mode == MODE_5_CALIBRATE_MOTORS) {
      agv_follow_line_enable = false;
      static uint8_t calib_state = 0;
      static uint32_t state_start_time = 0;

      if (state_start_time == 0)
        state_start_time = HAL_GetTick();
      uint32_t elapsed = HAL_GetTick() - state_start_time;

      switch (calib_state) {
      case 0:
        AGV_Stop(&h_agv);
        if (elapsed > 2000) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 1:
        Motor_SetSpeed(&m_left, calib_speed);
        Motor_SetSpeed(&m_right, calib_speed);
        if (elapsed > calib_time_forward) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 2:
        AGV_Stop(&h_agv);
        if (elapsed > 1000) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 3:
        Motor_SetSpeed(&m_left, -calib_speed);
        Motor_SetSpeed(&m_right, -calib_speed);
        if (elapsed > calib_time_forward) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 4:
        AGV_Stop(&h_agv);
        if (elapsed > 1000) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 5:
        Motor_SetSpeed(&m_left, -calib_speed);
        Motor_SetSpeed(&m_right, calib_speed);
        if (elapsed > calib_time_turn_90) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 6:
        AGV_Stop(&h_agv);
        if (elapsed > 1000) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 7:
        Motor_SetSpeed(&m_left, calib_speed);
        Motor_SetSpeed(&m_right, -calib_speed);
        if (elapsed > calib_time_turn_90) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 8:
        AGV_Stop(&h_agv);
        if (elapsed > 1000) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 9:
        Motor_SetSpeed(&m_left, -calib_speed);
        Motor_SetSpeed(&m_right, calib_speed);
        if (elapsed > calib_time_turn_180) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 10:
        AGV_Stop(&h_agv);
        if (elapsed > 1000) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 11:
        Motor_SetSpeed(&m_left, calib_speed);
        Motor_SetSpeed(&m_right, -calib_speed);
        if (elapsed > calib_time_turn_180) {
          calib_state++;
          state_start_time = HAL_GetTick();
        }
        break;
      case 12:
        AGV_Stop(&h_agv);
        break;
      }
      continue;
    }

    if (agv_run_mode == MODE_4_FULL_RUN) {
      if (agv_follow_line_enable && (HAL_GetTick() - last_qr_time > 15000)) {
        agv_follow_line_enable = false;
        AGV_Stop(&h_agv);
      }
    }

    if (qr50.Data.New_Data_Flag) {
      qr50.Data.New_Data_Flag = false;

      if (qr50.Data.Data_Buffer[0] == 'N') {
        last_qr_time = HAL_GetTick();

        uint16_t read_node_id = atoi((char *)&qr50.Data.Data_Buffer[1]);

        if (read_node_id < MAX_NODES && read_node_id != last_processed_node) {
          pending_qr_node = read_node_id;
        }
      }
    }

    if (is_at_intersection) {
      if (agv_run_mode == MODE_2_LINE_INTERSECTION) {
        continue;
      }

      if (agv_run_mode == MODE_6_TEST_TURN_RIGHT) {
        AGV_TurnRight(&h_agv);
        last_leave_intersection_time = HAL_GetTick();
        is_at_intersection = false;
        agv_follow_line_enable = true;
        continue;
      }

      if (agv_run_mode == MODE_7_DEBUG_NO_QR) {
        if (path_length > 0 && path_index < path_length - 1) {
          pending_qr_node = current_path[path_index + 1];
        } else {
          pending_qr_node = destination_node;
        }
      }

      if (pending_qr_node != 0xFFFF) {
        uint16_t read_node_id = pending_qr_node;
        pending_qr_node = 0xFFFF;
        last_processed_node = read_node_id;
        is_at_intersection = false;

        if (read_node_id == destination_node) {
          if (agv_run_mode == MODE_7_DEBUG_NO_QR) {
            if (destination_node == 6) {
              AGV_Stop(&h_agv);
              HAL_Delay(3000);
              destination_node = 3;
              current_node = 6;
              Routing_Dijkstra(&factory_map, current_node, destination_node,
                               current_path, &path_length);
              path_index = 0;
            } else if (destination_node == 3) {
              AGV_Stop(&h_agv);
              HAL_Delay(3000);
              destination_node = 1;
              current_node = 3;
              Routing_Dijkstra(&factory_map, current_node, destination_node,
                               current_path, &path_length);
              path_index = 0;
            } else {
              agv_follow_line_enable = false;
              AGV_Stop(&h_agv);
              continue;
            }
          } else {
            agv_follow_line_enable = false;
            AGV_Stop(&h_agv);
            continue;
          }
        }

        if (path_length > 0 && path_index < path_length - 1 &&
            read_node_id == current_path[path_index + 1]) {
          path_index++;
          current_node = read_node_id;
        } else if (read_node_id != current_node) {
          current_node = read_node_id;
          bool found_path =
              Routing_Dijkstra(&factory_map, current_node, destination_node,
                               current_path, &path_length);
          if (found_path) {
            path_index = 0;
          } else {
            agv_follow_line_enable = false;
            AGV_Stop(&h_agv);
            continue;
          }
        }

        if (path_length > 0 && path_index < path_length - 1) {
          uint16_t next_node = current_path[path_index + 1];
          AGV_Heading_t target_heading =
              Routing_GetHeading(&factory_map, current_node, next_node);

          int diff = (target_heading - current_heading + 4) % 4;
          AGV_Action_t next_action = (AGV_Action_t)diff;

          agv_follow_line_enable = false;

          switch (next_action) {
          case ACT_TURN_LEFT:
            h_agv.direction = 1;
            AGV_TurnLeft(&h_agv);
            break;
          case ACT_TURN_RIGHT:
            h_agv.direction = 1;
            AGV_TurnRight(&h_agv);
            break;
          case ACT_STRAIGHT:
            h_agv.direction = 1;
            Motor_SetSpeed(h_agv.motor_left, (int16_t)h_agv.base_speed);
            Motor_SetSpeed(h_agv.motor_right, (int16_t)h_agv.base_speed);
            HAL_Delay(300);
            break;
          case ACT_BACKWARD:
            h_agv.direction = -1;
            Motor_SetSpeed(h_agv.motor_left, (int16_t)-h_agv.base_speed);
            Motor_SetSpeed(h_agv.motor_right, (int16_t)-h_agv.base_speed);
            HAL_Delay(300);
            break;
          case ACT_STOP:
            AGV_Stop(&h_agv);
            break;
          default:
            break;
          }

          if (next_action != ACT_BACKWARD) {
            current_heading = target_heading;
          }
          last_leave_intersection_time = HAL_GetTick();
          agv_follow_line_enable = true;
        }
      } else if (HAL_GetTick() - intersection_time > 2000) {
        AGV_Stop(&h_agv);
        is_at_intersection = false;

        if (agv_run_mode == MODE_3_TEST_SENSORS_NO_MOTOR) {
          // Trong chế độ Test, nếu không cắm QR thì tự động nhả phanh đi tiếp
          // để người dùng có thể test các ngã tư tiếp theo mà không bị treo
          // cứng
          agv_follow_line_enable = true;
          last_leave_intersection_time = HAL_GetTick();
        } else {
          // Chạy thật: Hỏng camera, rách tem QR, hoặc chạm nhầm vạch rác
          agv_follow_line_enable = false; // Tắt xe chờ xử lý sự cố
                                          // Bật còi/đèn nháy báo lỗi tại đây
        }
      }
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
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  HAL_GPIO_WritePin(GPIOB,
                    B_Out13_Pin | T_ADC_CLK_Pin | T_ADC_SDI_Pin |
                        T_ENC_CLK_Pin | T_ENC_SDI_Pin,
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

  /*Configure GPIO pins : T_ENC_CS3_Pin T_ENC_CS4_Pin B_Out0_Pin B_Out1_Pin
                           B_Out2_Pin B_Out15_Pin T_ENC_CS1_Pin */
  GPIO_InitStruct.Pin = T_ENC_CS3_Pin | T_ENC_CS4_Pin | B_Out0_Pin |
                        B_Out1_Pin | B_Out2_Pin | B_Out15_Pin | T_ENC_CS1_Pin;
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

  /*Configure GPIO pins : B_In0_Pin B_In1_Pin B_In2_Pin B_In3_Pin
                           B_In4_Pin B_In5_Pin */
  GPIO_InitStruct.Pin =
      B_In0_Pin | B_In1_Pin | B_In2_Pin | B_In3_Pin | B_In4_Pin | B_In5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out4_Pin B_Out5_Pin B_Out6_Pin B_Out7_Pin
                           SYS_LED2_Pin SYS_LED3_Pin B_Out16_Pin */
  GPIO_InitStruct.Pin = B_Out4_Pin | B_Out5_Pin | B_Out6_Pin | B_Out7_Pin |
                        SYS_LED2_Pin | SYS_LED3_Pin | B_Out16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In6_Pin B_In14_Pin */
  GPIO_InitStruct.Pin = B_In6_Pin | B_In14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In7_Pin B_In10_Pin B_In11_Pin B_In12_Pin
                           B_In13_Pin */
  GPIO_InitStruct.Pin =
      B_In7_Pin | B_In10_Pin | B_In11_Pin | B_In12_Pin | B_In13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out13_Pin T_ADC_CLK_Pin T_ADC_SDI_Pin T_ENC_CLK_Pin
                           T_ENC_SDI_Pin */
  GPIO_InitStruct.Pin = B_Out13_Pin | T_ADC_CLK_Pin | T_ADC_SDI_Pin |
                        T_ENC_CLK_Pin | T_ENC_SDI_Pin;
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

  /*Configure GPIO pins : T_ADC_SDO_Pin T_ENC_SDO_Pin */
  GPIO_InitStruct.Pin = T_ADC_SDO_Pin | T_ENC_SDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out14_Pin B_Out17_Pin B_Out21_Pin B_Out20_Pin
                           SYS_LED1_Pin */
  GPIO_InitStruct.Pin =
      B_Out14_Pin | B_Out17_Pin | B_Out21_Pin | B_Out20_Pin | SYS_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out23_Pin B_Out22_Pin T_DAC_SDI_Pin T_DAC_CLK_Pin
                           T_DAC_CS_Pin T_ENC_CS2_Pin T_ADC_CS_Pin T_ADC_RST_Pin
   */
  GPIO_InitStruct.Pin = B_Out23_Pin | B_Out22_Pin | T_DAC_SDI_Pin |
                        T_DAC_CLK_Pin | T_DAC_CS_Pin | T_ENC_CS2_Pin |
                        T_ADC_CS_Pin | T_ADC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In32_Pin B_In31_Pin B_In33_Pin */
  GPIO_InitStruct.Pin = B_In32_Pin | B_In31_Pin | B_In33_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART3) {
    // Hàm này tự động được gọi khi nhận đủ 2 byte HOẶC khi đường dây RS485 nghỉ
    // Ngay khi đường dây nghỉ, DMA trong chế độ Circular KHÔNG tự reset con trỏ
    // về 0! Bắt buộc phải Abort và Start lại để gói tin tiếp theo bắt đầu ghi
    // từ rx_scan[0]
    HAL_UART_AbortReceive(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_scan, 2);
  } else if (huart->Instance == USART2) {
    // Xử lý gói tin từ đầu đọc QR50
    QR50_ParseData(&qr50, qr50.Data.Data_Buffer, Size);

    // Abort trước khi khởi động lại để đảm bảo con trỏ DMA quay về 0
    // Tránh việc gói tin thứ 2 ghi nối tiếp vào đuôi gói tin thứ 1
    HAL_UART_AbortReceive(&huart2);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, qr50.Data.Data_Buffer,
                                 QR50_MAX_DATA_LEN);
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

    // Ngắt PID mỗi 10ms - Chỉ điều khiển Motor nếu cờ được bật
    if (agv_follow_line_enable) {
      AGV_FollowLine(&h_agv);
    }
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
