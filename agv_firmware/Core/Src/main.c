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
#include "motor.h"
#include "qr50_reader.h"
#include "sensor.h"
#include "agv_routing.h"
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

AGV_Map_t factory_map;
uint16_t current_path[MAX_NODES];
uint16_t path_length = 0;
int path_index = 0;
uint16_t current_node = 0;
uint16_t destination_node = 15; // Ví dụ: Điểm P
AGV_Heading_t current_heading = HEAD_NORTH; // Biến la bàn theo dõi góc nhìn hiện tại
bool agv_follow_line_enable = true; // Cờ khóa/mở ngắt bám vạch
static uint16_t last_processed_node = 0xFFFF; // Cờ chống quét trùng QR

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
    factory_map.total_nodes = 16;
    
    // ĐÂY LÀ ĐIỂM ĂN TIỀN CỦA ADJACENCY LIST!
    // Bạn KHÔNG CẦN khai báo ma trận 100x100 toàn số 99999 nữa.
    // Đường nào có thật trên bản đồ thì bạn mới AddEdge.
    
    // Giả sử: Bắc (0), Đông (1), Nam (2), Tây (3)
    
    // Từ Node N00 (0)
    Map_AddEdge(&factory_map, N00, N11, 3, HEAD_EAST);
    
    // Từ Node N01 (1)
    Map_AddEdge(&factory_map, N01, N12, 3, HEAD_EAST);

    // Từ Node N02 (2)
    Map_AddEdge(&factory_map, N02, N11, 3, HEAD_SOUTH);   

    // Từ Node N03 (3)
    Map_AddEdge(&factory_map, N03, N12, 3, HEAD_NORTH);    

    // Từ Node N04 (4)
    Map_AddEdge(&factory_map, N04, N13, 3, HEAD_EAST);

    // Từ Node N05 (5)
    Map_AddEdge(&factory_map, N05, N14, 3, HEAD_EAST);

    // Từ Node N06 (6) - Ngã ba chữ T
    Map_AddEdge(&factory_map, N06, N07, 15, HEAD_SOUTH); 
    Map_AddEdge(&factory_map, N06, N15, 15, HEAD_NORTH);

    // Từ Node N07 (7) - Ngã tư trung tâm
    Map_AddEdge(&factory_map, N07, N06, 15, HEAD_NORTH);
    Map_AddEdge(&factory_map, N07, N08, 10, HEAD_EAST);
    Map_AddEdge(&factory_map, N07, N11, 7,  HEAD_SOUTH);

    // Từ Node N08 (8)
    Map_AddEdge(&factory_map, N08, N07, 10, HEAD_WEST);
    Map_AddEdge(&factory_map, N08, N13, 7,  HEAD_NORTH);

    // Từ Node N09 (9)
    Map_AddEdge(&factory_map, N09, N10, 10, HEAD_WEST); 
    Map_AddEdge(&factory_map, N09, N14, 7,  HEAD_SOUTH);

    // Từ Node N10 (10)
    Map_AddEdge(&factory_map, N10, N09, 10, HEAD_EAST);
    Map_AddEdge(&factory_map, N10, N12, 7,  HEAD_NORTH);
    Map_AddEdge(&factory_map, N10, N15, 15, HEAD_SOUTH);

    // Từ Node N11 (11) - Ngã tư lớn
    Map_AddEdge(&factory_map, N11, N00, 3, HEAD_WEST);
    Map_AddEdge(&factory_map, N11, N02, 3, HEAD_NORTH);
    Map_AddEdge(&factory_map, N11, N07, 7, HEAD_NORTH); // Sửa tạm: Nếu N07 nằm hướng Bắc
    Map_AddEdge(&factory_map, N11, N12, 5, HEAD_EAST);  

    // Từ Node N12 (12)
    Map_AddEdge(&factory_map, N12, N01, 3, HEAD_WEST);
    Map_AddEdge(&factory_map, N12, N03, 3, HEAD_SOUTH);
    Map_AddEdge(&factory_map, N12, N10, 7, HEAD_SOUTH);
    Map_AddEdge(&factory_map, N12, N11, 5, HEAD_WEST);

    // Từ Node N13 (13)
    Map_AddEdge(&factory_map, N13, N04, 3, HEAD_WEST);
    Map_AddEdge(&factory_map, N13, N08, 7, HEAD_SOUTH);
    Map_AddEdge(&factory_map, N13, N14, 5, HEAD_NORTH);

    // Từ Node N14 (14)
    Map_AddEdge(&factory_map, N14, N05, 3, HEAD_WEST);
    Map_AddEdge(&factory_map, N14, N09, 7, HEAD_NORTH);
    Map_AddEdge(&factory_map, N14, N13, 5, HEAD_SOUTH);

    // Từ Node N15 (15)
    Map_AddEdge(&factory_map, N15, N06, 15, HEAD_SOUTH);
    Map_AddEdge(&factory_map, N15, N10, 15, HEAD_NORTH);
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

  // Khởi tạo Motor trái (PWM CH1) và phải (PWM CH2) trên TIM4
  Motor_Init(&m_left, &htim4, TIM_CHANNEL_1, B_Out23_GPIO_Port, B_Out23_Pin,
             B_Out22_GPIO_Port, B_Out22_Pin);
  Motor_Init(&m_right, &htim4, TIM_CHANNEL_2, B_Out21_GPIO_Port, B_Out21_Pin,
             B_Out20_GPIO_Port, B_Out20_Pin);

  // Cấu hình tham số PID dò vạch (Đã chuẩn hóa Kd theo thời gian Delta_t)
  pid_ctrl.Kp = 55.0f;
  pid_ctrl.Ki = 0.0f;
  pid_ctrl.Kd = 0.1f;
  pid_ctrl.i = 0.0f;

  // Base speed = 300
  AGV_Init(&h_agv, &m_left, &m_right, &line_ss, &pid_ctrl, 300.0f);

  // Khởi động GPDMA nhận dữ liệu UART bằng Ngắt rảnh rỗi (IDLE Line)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_scan, 2);

  // Bật ngắt Timer 6 (nếu bạn dùng TIM6 cho ngắt PID)
  extern TIM_HandleTypeDef htim6;
  HAL_TIM_Base_Start_IT(&htim6);

  // Khởi tạo đầu đọc mã vạch QR50 trên cổng RS485_1 (USART2)
  QR50_Init(&qr50, &huart2, 0);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, qr50.Data.Data_Buffer,
                               QR50_MAX_DATA_LEN);
                               
  // Load bản đồ và tính đường đi mẫu từ A(0) đến P(15)
  Load_Factory_Map();
  Routing_Dijkstra(&factory_map, current_node, destination_node, current_path, &path_length);
  path_index = 0; // Đặt lại index đường đi
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (qr50.Data.New_Data_Flag) {
        qr50.Data.New_Data_Flag = false;
        
        // Lọc rác: Chỉ xử lý nếu chuỗi bắt đầu bằng chữ 'N'
        if (qr50.Data.Data_Buffer[0] != 'N') {
            continue; // Bỏ qua nếu đọc trúng mã vạch rác
        }
        
        // Trích xuất ID từ định dạng "N00", "N05", "N99"...
        // Bỏ qua ký tự 'N' ở đầu, chuyển phần số thành int
        uint16_t read_node_id = atoi((char *)&qr50.Data.Data_Buffer[1]); 
        
        if (read_node_id >= MAX_NODES) continue; // Cầu chì an toàn
        
        // BUG 2: Chống quét trùng mã QR (Debounce)
        if (read_node_id == last_processed_node) continue; 
        last_processed_node = read_node_id;
        
        if (read_node_id == destination_node) {
            // BUG 1: Lỗi "Tông xuyên đích". Phải ngắt FollowLine và phanh cứng!
            agv_follow_line_enable = false;
            AGV_Stop(&h_agv);
            continue;
        }

        // Kiểm tra xem xe có chạy đúng tuyến đường mong muốn không
        // BUG 4: Chống lỗi Tràn số (Underflow) nếu path_length = 0
        if (path_length > 0 && path_index < path_length - 1 && read_node_id == current_path[path_index + 1]) {
            // Đi ĐÚNG đường! -> Cập nhật vị trí hiện tại
            path_index++;
            current_node = read_node_id;
        } 
        else if (read_node_id != current_node) {
            // XE BỊ LẠC / TRƯỢT MÃ / CHẠY SAI ĐƯỜNG!
            // Ví dụ: Đang ở 12, kỳ vọng đến 10, nhưng lại đọc nhầm ra 11
            
            // 1. Dừng xe khẩn cấp để tính toán lại (nếu cần)
            // AGV_Stop(&h_agv); 
            
            // 2. Cập nhật vị trí hiện tại bằng điểm thực tế vừa đọc được
            current_node = read_node_id;
            
            // 3. Chạy lại thuật toán Dijkstra để vẽ đường mới từ điểm bị lạc tới đích
            bool found_path = Routing_Dijkstra(&factory_map, current_node, destination_node, current_path, &path_length);
            
            if (found_path) {
                path_index = 0; // Bắt đầu chạy theo mảng current_path mới
            } else {
                // BUG 6: Lỗi "Mất lái đường cụt". Phải phanh xe!
                agv_follow_line_enable = false;
                AGV_Stop(&h_agv);
                continue; 
            }
        }

        // --- XỬ LÝ CHUYỂN HƯỚNG ---
        if (path_length > 0 && path_index < path_length - 1) {
            uint16_t next_node = current_path[path_index + 1];
            AGV_Heading_t target_heading = Routing_GetHeading(&factory_map, current_node, next_node);
            
            // Tính toán chênh lệch hướng (0: Thẳng, 1: Phải, 2: Quay đầu, 3: Trái)
            int diff = (target_heading - current_heading + 4) % 4;
            AGV_Action_t next_action = (AGV_Action_t)diff;
            
            // BUG 5: Xung đột Ngắt TIM6. Khóa PID bám vạch khi đang rẽ!
            agv_follow_line_enable = false;
            
            switch (next_action) {
                case ACT_TURN_LEFT: 
                    AGV_TurnLeft(&h_agv);
                    break;
                case ACT_TURN_RIGHT: 
                    AGV_TurnRight(&h_agv);
                    break;
                case ACT_STRAIGHT: 
                    /* Tiếp tục bám vạch đi thẳng */ 
                    break;
                case ACT_TURN_180: 
                    AGV_Turn180(&h_agv);
                    break;
                case ACT_STOP: 
                    AGV_Stop(&h_agv);
                    break;
                default: 
                    break;
            }
            
            // Cập nhật lại góc nhìn la bàn của xe sau khi đã rẽ
            current_heading = target_heading;
            
            // Rẽ xong, cho phép ngắt TIM6 tiếp tục bám vạch
            agv_follow_line_enable = true;
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
  htim3.Init.Period = 65535;
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
    // (IDLE). Ngay khi đường dây nghỉ, DMA tự động reset con trỏ về rx_scan[0]
    // cho gói tin tiếp theo!

    // (Tùy chọn) Có thể xử lý dữ liệu rx_scan ở đây nếu cần phản hồi tức thời
  } else if (huart->Instance == USART2) {
    // Xử lý gói tin từ đầu đọc QR50
    QR50_ParseData(&qr50, qr50.Data.Data_Buffer, Size);

    // Khởi động lại ngắt nhận cho lần quét tiếp theo
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, qr50.Data.Data_Buffer,
                                 QR50_MAX_DATA_LEN);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
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
