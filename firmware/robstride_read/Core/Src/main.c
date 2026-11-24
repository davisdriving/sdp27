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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "robstride.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ID-change mode. Leave at 0 for the bus test. To renumber a motor that
 * is ALONE on the bus, set SET_ID_NEW to 3 (or 4), build, flash, run once,
 * then set it back to 0. SET_ID_OLD is the ID the motor has now.         */
#define SET_ID_NEW   0
#define SET_ID_OLD   127

/* Bus capacity test: 4 motors, one request + one reply each, 500 Hz. */
#define N_MOTORS     4
#define CYCLE_US     2000u                          /* 2 ms = 500 Hz      */
#define CYCLE_TICKS  (CYCLE_US * 170u)              /* SYSCLK = 170 MHz   */
#define ALL_REPLIED  ((1u << N_MOTORS) - 1u)

/* Scope pin: PB3 = net Status_to_MPU = H3 pin 1 (H3 pin 3 = GND).
 * HIGH at the start of each cycle, LOW when the last reply arrives.  */
#define TEST_PIN_Port GPIOB
#define TEST_PIN      GPIO_PIN_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
static const uint8_t motor_ids[N_MOTORS] = {1, 2, 3, 4};

/* Test results, reset every second when they are printed */
static uint32_t t_next;            /* DWT tick at which the next cycle starts */
static uint32_t n_cycles, n_full, n_tx, n_rx, n_tx_fail, n_missed;
static uint32_t rtt_max, rtt_sum;  /* round trip of a full cycle, in ticks  */
static uint32_t missed_by[N_MOTORS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if SET_ID_NEW != 0   /* these two are only used in ID-change mode */
/* Ask motor 'id' for its angle and wait up to timeout_ms for the answer. */
static bool read_mech_pos(uint8_t id, float *rad, uint32_t timeout_ms)
{
  uint8_t rid;
  uint16_t idx;
  float v;

  if (RS_RequestParam(&hfdcan1, id, RS_PARAM_MECHPOS) != HAL_OK) {
    return false;
  }
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 <= timeout_ms) {
    if (RS_GetParamReply(&hfdcan1, &rid, &idx, &v) &&
        rid == id && idx == RS_PARAM_MECHPOS) {
      *rad = v;
      return true;
    }
  }
  return false;
}

/* Type 7 "set motor CAN_ID" (manual 4.1.7, printed p.58-59):
 * ID bits 28-24 = 7, bits 23-16 = new ID, bits 15-8 = host ID,
 * bits 7-0 = the motor's current ID. Data bytes are unused (sent as 0). */
static HAL_StatusTypeDef set_can_id(uint8_t old_id, uint8_t new_id)
{
  FDCAN_TxHeaderTypeDef h = {0};
  uint8_t d[8] = {0};
  h.Identifier          = (7u << 24) | ((uint32_t)new_id << 16) |
                          ((uint32_t)RS_HOST_ID << 8) | old_id;
  h.IdType              = FDCAN_EXTENDED_ID;
  h.TxFrameType         = FDCAN_DATA_FRAME;
  h.DataLength          = FDCAN_DLC_BYTES_8;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch       = FDCAN_BRS_OFF;
  h.FDFormat            = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, d);
}
#endif

/* Empty the RX FIFO, setting bit i of *got when motor_ids[i] has replied. */
static void poll_replies(uint8_t *got)
{
  uint8_t rid;
  uint16_t idx;
  float v;
  while (RS_GetParamReply(&hfdcan1, &rid, &idx, &v)) {
    for (int i = 0; i < N_MOTORS; i++) {
      if (rid == motor_ids[i]) {
        if ((*got & (1u << i)) == 0) n_rx++;
        *got |= (uint8_t)(1u << i);
      }
    }
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
  MX_FDCAN1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  /* PB5 LOW enables buffer U6 so the transceiver's RXD reaches PB8.
   * PA15 LOW puts the TCAN1042 in Normal mode (HIGH = Standby).    */
  HAL_GPIO_WritePin(BOOT_SELECTION_GPIO_Port, BOOT_SELECTION_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

  /* Put every extended-ID frame in RX FIFO 0; drop standard-ID frames. */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  /* Give you 3 s to open the serial terminal. */
  HAL_Delay(3000);

#if SET_ID_NEW != 0
  /* ---- ID-change mode: runs once, then reports forever ---- */
  {
    static char m[128];
    float rad;
    HAL_StatusTypeDef st = set_can_id(SET_ID_OLD, SET_ID_NEW);
    HAL_Delay(100);
    while (1) {
      bool at_new = read_mech_pos(SET_ID_NEW, &rad, 10);
      bool at_old = read_mech_pos(SET_ID_OLD, &rad, 10);
      int n = snprintf(m, sizeof m,
                       "Set ID %d -> %d: send %s. ID %d answers: %s. ID %d answers: %s\r\n",
                       SET_ID_OLD, SET_ID_NEW, (st == HAL_OK) ? "ok" : "FAILED",
                       SET_ID_NEW, at_new ? "YES" : "no",
                       SET_ID_OLD, at_old ? "YES" : "no");
      CDC_Transmit_FS((uint8_t *)m, (uint16_t)n);
      HAL_Delay(1000);
    }
  }
#endif

  /* ---- Bus capacity test setup ---- */
  /* PB3 as push-pull output for the scope. Done here, not in CubeMX,
   * so no .ioc change is needed. GPIOB's clock is already on.       */
  GPIO_InitTypeDef g = {0};
  g.Pin   = TEST_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_PIN_Port, &g);
  HAL_GPIO_WritePin(TEST_PIN_Port, TEST_PIN, GPIO_PIN_RESET);

  /* DWT cycle counter: counts CPU clocks (170 per microsecond), so it
   * times the 2 ms cycle far more finely than the 1 ms HAL tick.     */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  t_next = DWT->CYCCNT + CYCLE_TICKS;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Wait for the cycle to start. (int32_t) cast handles counter wrap. */
    while ((int32_t)(DWT->CYCCNT - t_next) < 0) {}
    uint32_t t0 = t_next;
    t_next += CYCLE_TICKS;
    HAL_GPIO_WritePin(TEST_PIN_Port, TEST_PIN, GPIO_PIN_SET);

    /* Send one "read MechPos" request to every motor. The TX FIFO holds
     * only 3 frames, so wait for a free slot (reading replies meanwhile,
     * because the RX FIFO holds only 3 too).                            */
    uint8_t got = 0;
    for (int i = 0; i < N_MOTORS; i++) {
      while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0 &&
             (int32_t)(DWT->CYCCNT - t_next) < 0) {
        poll_replies(&got);
      }
      if (RS_RequestParam(&hfdcan1, motor_ids[i], RS_PARAM_MECHPOS) == HAL_OK) {
        n_tx++;
      } else {
        n_tx_fail++;
      }
    }

    /* Wait for all replies, but never past the start of the next cycle. */
    while (got != ALL_REPLIED && (int32_t)(DWT->CYCCNT - t_next) < 0) {
      poll_replies(&got);
    }
    uint32_t rtt = DWT->CYCCNT - t0;
    HAL_GPIO_WritePin(TEST_PIN_Port, TEST_PIN, GPIO_PIN_RESET);

    n_cycles++;
    if (got == ALL_REPLIED) {
      n_full++;
      rtt_sum += rtt;
      if (rtt > rtt_max) rtt_max = rtt;
    }
    for (int i = 0; i < N_MOTORS; i++) {
      if ((got & (1u << i)) == 0) {
        n_missed++;
        missed_by[i]++;
      }
    }

    /* Once per second (500 cycles), print the results and reset them. */
    if (n_cycles >= 1000000u / CYCLE_US) {
      static char m[256];
      FDCAN_ErrorCountersTypeDef ec;
      FDCAN_ProtocolStatusTypeDef ps;
      HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
      HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
      int n = snprintf(m, sizeof m,
                       "cyc %lu tx %lu rx %lu txfail %lu missed %lu (by ID 1-4: %lu %lu %lu %lu) "
                       "rtt avg %lu us max %lu us | TEC %lu REC %lu LEC %lu busoff %lu\r\n",
                       (unsigned long)n_cycles, (unsigned long)n_tx, (unsigned long)n_rx,
                       (unsigned long)n_tx_fail, (unsigned long)n_missed,
                       (unsigned long)missed_by[0], (unsigned long)missed_by[1],
                       (unsigned long)missed_by[2], (unsigned long)missed_by[3],
                       (unsigned long)(n_full ? rtt_sum / n_full / 170u : 0),
                       (unsigned long)(rtt_max / 170u),
                       (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
                       (unsigned long)ps.LastErrorCode, (unsigned long)ps.BusOff);
      CDC_Transmit_FS((uint8_t *)m, (uint16_t)n);
      n_cycles = n_full = n_tx = n_rx = n_tx_fail = n_missed = 0;
      rtt_max = rtt_sum = 0;
      for (int i = 0; i < N_MOTORS; i++) missed_by[i] = 0;
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
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 34;
  hfdcan1.Init.NominalTimeSeg1 = 135;
  hfdcan1.Init.NominalTimeSeg2 = 34;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOOT_SELECTION_GPIO_Port, BOOT_SELECTION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_STB_Pin */
  GPIO_InitStruct.Pin = CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_SELECTION_Pin */
  GPIO_InitStruct.Pin = BOOT_SELECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOT_SELECTION_GPIO_Port, &GPIO_InitStruct);

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
