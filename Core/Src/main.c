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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
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

/* USER CODE BEGIN PV */
GPIO_PinState relay_state = GPIO_PIN_RESET;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t rx_data[64];
uint8_t tx_data[64];
uint8_t counter_tx;
volatile uint16_t interval_led = 1000;
uint16_t interval_tx = 10;
uint32_t timer_led, timer_tx;

int main_state = 0;

// Time for waiting camera to bootup
const unsigned long pc_on_delay_s = 5;

// Duration waiting ecoflow to turn on
const unsigned long init_wait_s = 5;

// Time for waiting PC to fully off
const unsigned long pc_poff_wait_s = 60;

// Time for waiting PC to bootup
const unsigned long pc_boot_timeout_s = 10;

// Timeout for ecoflow to turn on
const unsigned long ecoflow_timeout_s = 60;

// delay hardcode ecoflow to turn on
const unsigned long delay_ecoflow_on_s = 5;

// Duration for DC to remain active
const unsigned long dc_gone_timeout_s = 30;

// Wait timeout for PC to send shutdown command
const unsigned long pc_poff_timeout_s = 300;

// Time of long button press
const unsigned long button_long_press_ms = 2000;

// Time of short button press
const unsigned long button_short_press_ms = 100;

// Minimum number of AC detections to consider AC is on
const int ac_det_min_cnt = 10;

volatile uint8_t send_code = 0;
uint32_t last_time = 0;
uint32_t timer_sec = 0;
int fail_ctr = 0;
bool force_on = false;

uint16_t adc_val[2];
uint16_t ac_det = 0, dc_det = 0;
float ac_voltage = 0, dc_voltage = 0;

volatile uint32_t fdcan_tx_ok = 0;
volatile uint32_t fdcan_tx_err = 0;
volatile uint32_t fdcan_bus_off_cnt = 0;
volatile bool fdcan_need_recover = false;
volatile bool fdcan_need_start = false;
static uint8_t fdcan_tx_err_streak = 0; // local helper
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void bripip();
void bupup();
void beep();
bool is_button_long_pressed();
bool is_button_short_pressed();
bool is_dc_on();
bool is_ac_on();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void reset_timer(uint32_t *last_time_ptr, uint32_t *timer_sec_ptr)
{
  *last_time_ptr = HAL_GetTick();
  *timer_sec_ptr = 0;
}

void FDCAN_Init();
uint8_t calculate_crc(const uint8_t *data, size_t length, uint8_t poly, uint8_t xor_output)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ poly;
      }
      else
      {
        crc <<= 1;
      }
      crc &= 0xFF;
    }
  }
  return (crc ^ xor_output);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {

    memset(rx_data, 0, sizeof(rx_data));
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rx_data) != HAL_OK)
    {
      Error_Handler();
    }

    if (RxHeader.Identifier == 0x069)
    {
      uint8_t crc_ = calculate_crc(rx_data, 4, 0x1D, 0xA);

      if (crc_ == rx_data[4])
      {
        if (relay_state != rx_data[0])
        {
          relay_state = rx_data[0];
          HAL_GPIO_WritePin(RELAY_CAN_GPIO_Port, RELAY_CAN_Pin, relay_state);
        }
        interval_led = (rx_data[1] << 8) | rx_data[2];
      }
    }
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  (void)hfdcan;
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
  {
    fdcan_bus_off_cnt++;
    fdcan_need_recover = true;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) // timer 1khz
  {
    static uint32_t ctr1 = 0, ctr2 = 0;
    if (++ctr1 >= interval_led)
    {
      HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
      ctr1 = 0;
    }
    if (++ctr2 >= interval_tx)
    {
      TxHeader.Identifier = 0x065;
      tx_data[0] = relay_state;
      tx_data[1] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
      tx_data[2] = send_code;
      counter_tx++;
      tx_data[3] = counter_tx;
      uint8_t crc_ = calculate_crc(tx_data, 4, 0x1D, 0xA);
      tx_data[4] = crc_;

      // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_data);
      uint32_t free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
      if (free > 0)
      {
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_data) == HAL_OK)
        {
          fdcan_tx_ok++;
          fdcan_tx_err_streak = 0;
        }
        else
        {
          fdcan_tx_err++;
          if (++fdcan_tx_err_streak >= 3) // threshold, adjust jika perlu
            fdcan_need_recover = true;
        }
      }
      ctr2 = 0;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // Process the data
  ac_det = adc_val[0];
  dc_det = adc_val[1];
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  FDCAN_Init();

  for (int i = 0; i < 10; i++)
  {
    HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
    HAL_Delay(100);
  }
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 2);
  HAL_Delay(100);

  HAL_TIM_Base_Start_IT(&htim6);
  last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t timer_ms = HAL_GetTick() - last_time;
    if (timer_ms > 1000)
    {
      last_time = HAL_GetTick();
      timer_sec += 1;
    }

    // auto reset_timer = []()
    // {
    //   last_time = HAL_GetTick();
    //   timer_sec = 0;
    // };

    if (is_button_long_pressed())
    {
      if (main_state < 7)
      {
        main_state = 7;
        HAL_GPIO_WritePin(SW_AC_A_GPIO_Port, SW_AC_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SW_AC_B_GPIO_Port, SW_AC_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);
        beep();
        force_on = true;
      }
      else
      {
        main_state = 10;
        fail_ctr = 0;
        HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_AC_A_GPIO_Port, SW_AC_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_AC_B_GPIO_Port, SW_AC_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);
        bupup();

        force_on = false;
      }
      reset_timer(&last_time, &timer_sec);
    }

    switch (main_state)
    {
    case -1: // Error
        ;    // Do nothing
      if (is_button_short_pressed())
      {
        HAL_GPIO_WritePin(SW_AC_A_GPIO_Port, SW_AC_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_AC_B_GPIO_Port, SW_AC_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);
        send_code = 0;
        fail_ctr = 0;
        reset_timer(&last_time, &timer_sec);
        main_state = 0;
      }
      break;

    case 0:
      HAL_GPIO_WritePin(SW_AC_A_GPIO_Port, SW_AC_A_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SW_AC_B_GPIO_Port, SW_AC_B_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);

      send_code = 1;

      if (!is_dc_on())
      {
        break;
      }

      if (fail_ctr < 5)
      {
        beep();
        main_state = 1;
        reset_timer(&last_time, &timer_sec);
      }
      else
      {
        for (int i = 0; i < 3; i++)
        {
          bupup();
          HAL_Delay(1000);
        }
        main_state = -1;
      }
      break;

    case 1: // Wait for  init delay
      send_code = 2;
      if (timer_ms > 200)
      {
        main_state = 2;
        HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SW_AC_A_GPIO_Port, SW_AC_A_Pin, GPIO_PIN_SET);
        reset_timer(&last_time, &timer_sec);
      };
      break;
    case 2: // Wait few seconds for ecoflow to turn on
      send_code = 3;
      if (timer_sec > init_wait_s)
      {
        main_state = 3;
        reset_timer(&last_time, &timer_sec);
      }
      break;
    case 3: // Check if AC is on
    {
      send_code = 4;
//      static int ac_det_ctr = 0;
      // if (is_ac_on())
      // {
      //   ac_det_ctr += 1;
      // }

      // if (ac_det_ctr > ac_det_min_cnt)
      // {
      //   main_state = 4;
      //   ac_det_ctr = 0;
      //   reset_timer(&last_time, &timer_sec);
      // }
      if (timer_sec > delay_ecoflow_on_s)
      {
        main_state = 4;
//        ac_det_ctr = 0;
        reset_timer(&last_time, &timer_sec);
      }

      // if (timer_sec > ecoflow_timeout_s)
      // {
      //   HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);
      //   main_state = 10;
      //   fail_ctr += 5;
      // }
    }

    break;
    case 4: // Wait few seconds for camera bootup
      send_code = 5;
      if (timer_sec > pc_on_delay_s)
      {
        main_state = 5;
      }
      break;
    case 5: // switch on AC for PC
      send_code = 6;
      HAL_GPIO_WritePin(SW_AC_B_GPIO_Port, SW_AC_B_Pin, GPIO_PIN_SET);
      reset_timer(&last_time, &timer_sec);
      main_state = 6;
      break;
    case 6: // Wait for PC to bootup
      send_code = 7;
      if (timer_sec > pc_boot_timeout_s)
      {
        main_state = 7;
      }
      break;
    case 7: // Idle
      send_code = 8;
      if (is_dc_on() || force_on)
      {
        reset_timer(&last_time, &timer_sec);
      }

      if (timer_sec > dc_gone_timeout_s)
      {
        main_state = 8;
        reset_timer(&last_time, &timer_sec);
      }

      break;
    case 8: // wait PC to process the shutdown request
      send_code = 9;
      if (timer_sec > pc_poff_timeout_s)
      {
        main_state = 9;
        fail_ctr += 1;
      }
      break;
    case 9:
      send_code = 10;
      if (timer_sec > pc_poff_wait_s)
      {
        main_state = 10;
        HAL_GPIO_WritePin(SW_AC_A_GPIO_Port, SW_AC_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_AC_B_GPIO_Port, SW_AC_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_DC_GPIO_Port, SW_DC_Pin, GPIO_PIN_SET);

        // should not execute until here
        reset_timer(&last_time, &timer_sec);
        force_on = false;
      }
      break;
    case 10: // Wait for capacitor to discharge
      send_code = 11;
      if (timer_sec > dc_gone_timeout_s)
      {
        main_state = 0;
      }
      break;
    default:
      main_state = 0;
      fail_ctr += 1;
      break;
    } /***** END OF CASE *****/

    if (fdcan_need_recover)
    {
      fdcan_need_recover = false;
      fdcan_tx_err_streak = 0;

      HAL_FDCAN_Stop(&hfdcan1);
      fdcan_need_start = true;
    }
    HAL_Delay(10);
    if (fdcan_need_start)
    {
      fdcan_need_start = false;
      if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
      {
        // jika gagal start ulang, bisa retry atau set error state
        fdcan_need_recover = true;
      }
      // re-enable notifications (include BUS_OFF)
      HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
      bupup();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void FDCAN_Init()
{
  FDCAN_FilterTypeDef sFilterConfig = {};

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x069;
  sFilterConfig.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_FDCAN_ConfigGlobalFilter(
      &hfdcan1,
      FDCAN_REJECT, // Non-matching STD
      FDCAN_REJECT, // Non-matching EXT
      FDCAN_REJECT, // Remote STD
      FDCAN_REJECT  // Remote EXT
  );

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK)
  {
    Error_Handler();
  }
  // if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_5;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  //	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  //	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  memset(tx_data, 0, sizeof(tx_data));
}

void bripip()
{
  for (int i = 0; i < 6; i++)
  {
    HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
  }
}

void bupup()
{
  for (int i = 0; i < 4; i++)
  {
    HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
  }
}

void beep()
{
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
}

bool is_button_long_pressed(void)
{
  static uint32_t press_start_ms = 0;
  static bool fired = false;

  const bool pressed = (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET); // active-low

  if (pressed)
  {
    if (press_start_ms == 0)
    {
      press_start_ms = HAL_GetTick(); // start timing on first detect press
      fired = false;
    }

    if (!fired && (HAL_GetTick() - press_start_ms >= button_long_press_ms))
    {
      fired = true; // only fire once per press
      // beep();
      return true;
    }
  }
  else
  {
    // released: reset state
    press_start_ms = 0;
    fired = false;
  }

  return false;
}

bool is_button_short_pressed(void)
{
  static uint32_t press_start_ms = 0;
  static bool was_pressed = false;

  const bool pressed = (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET); // active-low

  if (pressed)
  {
    if (!was_pressed)
    {
      was_pressed = true;
      press_start_ms = HAL_GetTick(); // timestamp at press-down
    }
  }
  else
  {
    if (was_pressed)
    {
      was_pressed = false;

      uint32_t duration = HAL_GetTick() - press_start_ms;
      press_start_ms = 0;

      if (duration >= button_short_press_ms && duration < button_long_press_ms)
        return true;
    }
  }

  return false;
}

bool is_ac_on(void)
{
  if (ac_det > 300)
    return true;
  return false;
}

bool is_dc_on(void)
{
  if (dc_det > 300)
    return true;
  return false;
}

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
