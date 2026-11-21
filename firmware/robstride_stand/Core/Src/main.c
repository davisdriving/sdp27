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
#include <string.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "robstride.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_MOTORS            2
#define TARGET_JOINT_DEG    35.0f   /* where to go, normalized joint degrees */
#define RAMP_DEG_PER_S      10.0f   /* how fast the target moves             */
#define KP                   12.0f   /* Nm/rad                                */
#define KD                   0.5f   /* Nm*s/rad                              */
#define CTRL_PERIOD_MS       5u     /* 200 Hz command rate per motor         */
#define FEEDBACK_TIMEOUT_MS 50u     /* stop if a motor goes quiet this long  */
/* Torque step-up: while a joint is short of its target, add feed-forward
 * torque at TFF_RATE; if it overshoots, take torque away at the same rate.
 * Inside REACHED_TOL_DEG of the final target the torque is held as is.  */
#define TFF_RATE_NM_PER_S    1.0f   /* how fast the extra torque grows       */
#define TFF_MAX_NM           8.0f   /* safety cap (RS01 peak is 17 Nm)       */
#define REACHED_TOL_DEG      1.0f   /* "close enough" to the target          */
#define DEG2RAD  0.01745329f
#define RAD2DEG 57.29578f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
volatile uint8_t stop_request = 0;   /* set by usbd_cdc_if.c on any keypress */

static const uint8_t motor_ids[N_MOTORS]      = {1, 2};            /* right, left */
static const float   motor_sign[N_MOTORS]     = {-1.0f, 1.0f};
static const float   motor_rest_deg[N_MOTORS] = {-10.80f, -46.37f}; /* measured 11/20/26 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Normalized joint angle <-> raw motor angle, per motor index i */
static float joint_to_raw_deg(int i, float joint) { return motor_rest_deg[i] + motor_sign[i] * joint; }
static float raw_to_joint_deg(int i, float raw)   { return motor_sign[i] * (raw - motor_rest_deg[i]); }

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

static void say(const char *s)
{
  CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s));
}

/* Integer x100 formatting so no float printf is needed */
static int fmt2(char *buf, size_t n, float x)
{
  long c = (long)(x * 100.0f + (x < 0.0f ? -0.5f : 0.5f));
  return snprintf(buf, n, "%s%ld.%02ld", (c < 0) ? "-" : "", labs(c) / 100, labs(c) % 100);
}

/* Move 'from' toward 'to' by at most 'step' */
static float ramp(float from, float to, float step)
{
  if (to > from) return (from + step < to) ? from + step : to;
  return (from - step > to) ? from - step : to;
}

static void stop_all(void)
{
  for (int i = 0; i < N_MOTORS; i++) {
    RS_Stop(&hfdcan1, motor_ids[i]);
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

  /* Give user 5 s to open the terminal before anything moves. */
  HAL_Delay(5000);

  /* 1) Read where BOTH motors are now. Enable nothing unless both answer. */
  float start_rad[N_MOTORS] = {0};
  float start_joint[N_MOTORS] = {0};
  /*    Auto-retransmission is off, so one corrupted frame is simply lost:
   *    try each motor up to 10 times before giving up.                    */
  bool running = true;
  static char msg0[128];
  for (int i = 0; i < N_MOTORS; i++) {
    bool ok = false;
    for (int attempt = 0; attempt < 10 && !ok; attempt++) {
      ok = read_mech_pos(motor_ids[i], &start_rad[i], 20);
    }
    if (!ok) {
      FDCAN_ProtocolStatusTypeDef ps;
      FDCAN_ErrorCountersTypeDef ec;
      HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
      HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
      snprintf(msg0, sizeof msg0,
               "Motor %u did not answer. LEC %lu TEC %lu REC %lu bus-off %lu. Not enabling.\r\n",
               motor_ids[i], (unsigned long)ps.LastErrorCode, (unsigned long)ec.TxErrorCnt,
               (unsigned long)ec.RxErrorCnt, (unsigned long)ps.BusOff);
      running = false;
    }
    start_joint[i] = raw_to_joint_deg(i, start_rad[i] * RAD2DEG);
  }

  if (!running) {
    /* repeat the reason every second so it is not missed */
    while (1) {
      say(msg0);
      HAL_Delay(1000);
    }
  } else {
    /* 2) Reset mode, operation-control mode (run_mode 0), enable, and
     *    first command = hold the CURRENT angle so nothing jumps.        */
    for (int i = 0; i < N_MOTORS; i++) {
      RS_Stop(&hfdcan1, motor_ids[i]);
      HAL_Delay(2);
      RS_SetRunMode(&hfdcan1, motor_ids[i], 0);
      HAL_Delay(2);
      RS_Enable(&hfdcan1, motor_ids[i]);
      HAL_Delay(2);
      RS_Control(&hfdcan1, motor_ids[i], start_rad[i], 0.0f, KP, KD, 0.0f);
      HAL_Delay(2);
    }
    say("Motors 1 and 2 enabled. Press any key to stop.\r\n");
  }

  uint32_t t_start = HAL_GetTick();
  uint32_t t_last_fb[N_MOTORS] = { HAL_GetTick(), HAL_GetTick() };
  uint32_t t_last_print = 0;
  RS_Feedback fb[N_MOTORS] = {0};
  float joint_target[N_MOTORS] = {0};
  float tff[N_MOTORS] = {0};          /* feed-forward torque, motor frame, Nm */
  bool  reached[N_MOTORS] = {false};  /* true = torque frozen at target       */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (!running) {
      HAL_Delay(100);
      continue;
    }

    /* Each target ramps from that motor's start angle to TARGET_JOINT_DEG,
     * in joint degrees, then is converted to that motor's raw radians.   */
    float step = RAMP_DEG_PER_S * (HAL_GetTick() - t_start) / 1000.0f;
    for (int i = 0; i < N_MOTORS; i++) {
      joint_target[i] = ramp(start_joint[i], TARGET_JOINT_DEG, step);

      /* Adjust the extra torque in BOTH directions: more push while the joint
       * is below target, less push once it has overshot. Inside the
       * +/-REACHED_TOL_DEG band the torque is left alone (HOLD).           */
      float err = joint_target[i] - raw_to_joint_deg(i, fb[i].pos_rad * RAD2DEG);
      reached[i] = (joint_target[i] == TARGET_JOINT_DEG &&
                    fabsf(err) < REACHED_TOL_DEG);
      if (fabsf(err) >= REACHED_TOL_DEG) {
        /* motor sign converts joint direction to motor direction (ID 1 is
         * mirrored)                                                       */
        float d = TFF_RATE_NM_PER_S * CTRL_PERIOD_MS / 1000.0f;
        tff[i] += (err > 0.0f ? d : -d) * motor_sign[i];
        if (tff[i] >  TFF_MAX_NM) tff[i] =  TFF_MAX_NM;
        if (tff[i] < -TFF_MAX_NM) tff[i] = -TFF_MAX_NM;
      }

      RS_Control(&hfdcan1, motor_ids[i],
                 joint_to_raw_deg(i, joint_target[i]) * DEG2RAD,
                 0.0f, KP, KD, tff[i]);
    }
    HAL_Delay(CTRL_PERIOD_MS);

    /* Each command is answered by a type-2 feedback frame; sort by motor. */
    RS_Feedback f;
    while (RS_GetFeedback(&hfdcan1, &f)) {
      for (int i = 0; i < N_MOTORS; i++) {
        if (f.id == motor_ids[i]) {
          fb[i] = f;
          t_last_fb[i] = HAL_GetTick();
        }
      }
    }

    /* Stop conditions: keypress, fault on either motor, or either goes quiet. */
    const char *why = NULL;
    int who = 0;
    if (stop_request) {
      why = "key pressed";
    }
    for (int i = 0; i < N_MOTORS && why == NULL; i++) {
      if (fb[i].faults != 0) {
        why = "motor fault"; who = motor_ids[i];
      } else if (HAL_GetTick() - t_last_fb[i] > FEEDBACK_TIMEOUT_MS) {
        why = "no feedback"; who = motor_ids[i];
      }
    }
    if (why != NULL) {
      stop_all();
      running = false;
      static char m[96];
      snprintf(m, sizeof m, "STOPPED (%s, motor %d), faults 0x%02X 0x%02X. Both limp.\r\n",
               why, who, fb[0].faults, fb[1].faults);
      say(m);
      continue;
    }

    /* Print 10x per second: per motor target, actual (joint deg), torque */
    if (HAL_GetTick() - t_last_print >= 100) {
      t_last_print = HAL_GetTick();
      static char m[224];
      int n = 0;
      for (int i = 0; i < N_MOTORS; i++) {
        char a[16], b[16], c[16], d[16];
        fmt2(a, sizeof a, joint_target[i]);
        fmt2(b, sizeof b, raw_to_joint_deg(i, fb[i].pos_rad * RAD2DEG));
        fmt2(c, sizeof c, fb[i].torque_nm);
        fmt2(d, sizeof d, tff[i]);
        n += snprintf(m + n, sizeof m - n, "ID %u: tgt %s act %s deg %s Nm ff %s%s   ",
                      motor_ids[i], a, b, c, d,
                      reached[i] ? " HOLD" : (fabsf(tff[i]) >= TFF_MAX_NM ? " MAX" : ""));
      }
      snprintf(m + n, sizeof m - n, "\r\n");
      say(m);
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
  hfdcan1.Init.AutoRetransmission = DISABLE;
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
