/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>   /* この行を追加 */
#include <string.h>  /* この行を追加 */
#include <stdint.h>  /* この行を追加 */
#include <stdbool.h> /* この行を追加 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define limiter 90
#define CLAMP(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))
#define NUM_MY_IDS 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint8_t rx_ready = 0;
static volatile uint8_t last_rx[8];
static volatile uint8_t last_len = 0;
static volatile int16_t power[4]; // volatileは付けておく
static uint32_t last_can_rx_tick = 0;
static volatile uint8_t processing_flag = 0;
//static uint32_t debug_counter = 0; /* デバッグ用カウンター */
//static uint32_t rx_process_counter = 0; /* 受信処理カウンター */
static uint32_t last_can_any_rx_tick = 0;   // 任意のCANメッセージ受信時刻
int my_can_ids[NUM_MY_IDS] = {1};  // 標準ID 0x001
int decimal;
void select_ID(){
	GPIO_PinState state_A = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	GPIO_PinState state_B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	GPIO_PinState state_C = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	bool A = !state_A;
	bool B = !state_B;
	bool C = !state_C;

	decimal = (A << 2) | (B << 1) | (C << 0);
	my_can_ids[0] = decimal + 320;
}
bool is_my_can_id(uint32_t id) {
    for (int i = 0; i < NUM_MY_IDS; i++) {
        if (my_can_ids[i] == id) {
            return true;
        }
    }
    return false;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    /* FIFOのすべてのメッセージを処理（最新を優先） */
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        last_can_any_rx_tick = HAL_GetTick();  // どんなメッセージでも更新

        if (rxHeader.IDE == CAN_ID_STD && is_my_can_id(rxHeader.StdId)) {
            /* LED点灯（受信成功表示） */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

            last_can_rx_tick = HAL_GetTick();  // 自分宛なら更新

            /* 常に最新データに更新（rx_readyの状態に関係なく） */
            last_len = rxHeader.DLC;
            for (uint8_t i = 0; i < last_len; ++i)
            {
                last_rx[i] = rxData[i];
            }

            /* メモリバリアを追加 */
            __DSB();

            /* 新しいデータが来たことを通知 */
            rx_ready = 1;
        }

        /* 継続して次のメッセージもチェック */
    }
}


/* FIFO0オーバーフロー時のコールバック（デバッグ用） */
//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
//{
//	char buf[] = "FIFO FULL!\r\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
//  /* FIFOがフルになったことを示すLED表示 */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
//
//  /* エラー対応: 古いメッセージを破棄して新しいメッセージを受信できるようにする */
//  uint8_t dummy_data[8];
//  CAN_RxHeaderTypeDef dummy_header;
//  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
//  {
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &dummy_header, dummy_data);
//  }
//}
//
///* CANエラー時のコールバック（デバッグ用） */
//void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
//{
//
//  /* エラー状態をLEDで表示 */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//
//  /* UART経由でエラー情報を出力 */
//  char error_buf[50];
//  uint32_t error_code = HAL_CAN_GetError(hcan);
//  sprintf(error_buf, "CAN Error: 0x%lX\r\n", error_code);
//  HAL_UART_Transmit(&huart2, (uint8_t *)error_buf, strlen(error_buf), 100);
//}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t pwm_limit(TIM_HandleTypeDef *htim, int32_t val)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  uint32_t mag = (uint32_t)(val >= 0 ? val : -val);
  return (mag > arr) ? arr : mag;
}

static inline void set_bidirectional_pwm(
    TIM_HandleTypeDef *htim_pos, uint32_t ch_pos,
    TIM_HandleTypeDef *htim_neg, uint32_t ch_neg,
    int16_t val)
{
  if (val > 0)
  {
    uint32_t duty = pwm_limit(htim_pos, val);
    __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, duty);
    __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
  }
  else if (val < 0)
  {
    uint32_t duty = pwm_limit(htim_neg, val);
    __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, duty);
    __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, 0);
    __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
  select_ID();
  // フィルタ（全受信に変更）
  CAN_FilterTypeDef sFilterConfig = {0};
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  // 全ての標準IDを受信するための設定
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;

  // マスクで全許可（フィルタを無効化）
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;


  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan,
                                   CAN_IT_RX_FIFO0_MSG_PENDING |
                                   CAN_IT_RX_FIFO0_OVERRUN |
                                   CAN_IT_ERROR_WARNING |
                                   CAN_IT_ERROR_PASSIVE |
                                   CAN_IT_BUSOFF) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

        if (rx_ready) {
            __disable_irq();
            uint8_t len = last_len;
            uint8_t buf[8];
            // ガード（万一の異常長対策）
            if (len > 8) len = 8;
            for (uint8_t i = 0; i < len; ++i) buf[i] = last_rx[i];
            rx_ready = 0;
            __enable_irq();

            // 必須: 8バイト揃っているか確認（4チャネル×16bit前提）
            if (len >= 8) {
                for (int i = 0; i < 4; ++i) {
                    int16_t v = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]); // little-endian
                    // 対称に ±32767 へクランプ（absで32768にならないよう配慮するならこれでOK）
                    v = CLAMP(v, -32767, 32767);
                    power[i] = v;
                }

                // M0
                set_bidirectional_pwm(&htim2, TIM_CHANNEL_2, &htim2, TIM_CHANNEL_4, power[0]);
                // M1
                set_bidirectional_pwm(&htim3, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_1, power[1]);
                // M2（別タイマの組み合わせ）
                set_bidirectional_pwm(&htim1, TIM_CHANNEL_1, &htim17, TIM_CHANNEL_1, power[2]);
                // M3（同一タイマ内の別チャネル）
                set_bidirectional_pwm(&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_2, power[3]);
            } else {
                // 足りないフレームは無視（必要ならフェイルセーフ）
                // 例: 全停止
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            }
            // ここで buf[0..len-1] を使って追加処理があれば
        }

        // CAN通信状態に応じたLED表示とフェイルセーフ
        uint32_t now = HAL_GetTick();

        if (now - last_can_any_rx_tick > 50) {
            // 50ms以上CAN無信号ならLED両方消灯全停止
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

            // 全停止
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        } else if (now - last_can_rx_tick > 50) {
            // 自分宛CAN通信が50ms以上来なかったらPIN6消灯PIN7点灯で全停止
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

            // 全停止
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }

    }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;  /* 自動再送を有効化 */
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32767;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32767;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 32767;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 31;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 32767;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
