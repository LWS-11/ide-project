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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "acquisition.h"
#include "channel_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIC_EDGE_RAW_THRESHOLD 1600U
#define MIC_FILTER_TAPS 5U
#define MIC_CENTER_MV 1500
#define MIC_MIN_AUTO_CENTER_SPAN_MV 120U
#define MIC_MAX_DISPLAY_SPAN_MV 2400U
#define MIC_RAIL_LOW_MV 250U
#define MIC_RAIL_HIGH_MV 2900U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t ch1_samples[PROTOCOL_SAMPLES_PER_CHANNEL];
static uint16_t ch2_samples[PROTOCOL_SAMPLES_PER_CHANNEL];
static uint16_t ch1_mv[PROTOCOL_SAMPLES_PER_CHANNEL];
static uint16_t ch2_mv[PROTOCOL_SAMPLES_PER_CHANNEL];
static uint8_t digital_samples[PROTOCOL_SAMPLES_PER_CHANNEL];
static uint8_t tx_frame[PROTOCOL_FULL_FRAME_SIZE];
static uint16_t tx_sequence;
static uint16_t pot_filtered_mv;
static uint32_t pot_filter_last_tick;
static uint8_t pot_filter_initialized;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint16_t AdcRawToMillivolts(uint16_t raw)
{
  return (uint16_t)(((uint32_t)raw * 3300U + 2047U) / 4095U);
}

/*
 * Smooth the microphone signal with a short five-sample Gaussian kernel:
 *
 *     (1*previous2 + 2*previous1 + 3*current + 2*next1 + 1*next2) / 9
 *
 * A normal 500 Hz to 2 kHz sine wave only changes by a few hundred ADC
 * counts between adjacent samples at the 20 us setting, so it is smoothed
 * without losing its shape. A very large jump larger than
 * MIC_EDGE_RAW_THRESHOLD is treated as a square-wave edge and kept unchanged.
 */
static uint16_t MicrophoneFilterSample(const uint16_t *samples, uint16_t index)
{
  uint16_t previous2 = samples[(index >= 2U) ? (index - 2U) : 0U];
  uint16_t previous1 = samples[(index >= 1U) ? (index - 1U) : 0U];
  uint16_t current = samples[index];
  uint16_t next1 =
      samples[(index + 1U < PROTOCOL_SAMPLES_PER_CHANNEL) ? (index + 1U)
                                                         : index];
  uint16_t next2 =
      samples[(index + 2U < PROTOCOL_SAMPLES_PER_CHANNEL) ? (index + 2U)
                                                         : index];
  uint16_t delta_previous =
      (current > previous1) ? (current - previous1) : (previous1 - current);
  uint16_t delta_next =
      (current > next1) ? (current - next1) : (next1 - current);

  if ((delta_previous >= MIC_EDGE_RAW_THRESHOLD) ||
      (delta_next >= MIC_EDGE_RAW_THRESHOLD))
  {
    return current;
  }

  return (uint16_t)((previous2 +
                     (2U * previous1) +
                     (3U * current) +
                     (2U * next1) +
                     next2 +
                     (MIC_FILTER_TAPS / 2U)) /
                    9U);
}

/*
 * Software AC centering for microphone signals.
 *
 * The external microphone module can output a signal whose DC level is too
 * close to 0 V. The negative half wave then reaches the ADC's lower limit,
 * making the bottom of the sine look flat. Move the measured waveform's
 * vertical midpoint to 1.5 V so both halves stay visible. If the peak-to-peak
 * value is too large, compress it symmetrically instead of clipping it.
 *
 * A frame recognized as a real square wave bypasses this function. This keeps
 * the rectangular-wave test signal in its original 0/3.3 V form.
 */
static void CenterMicrophoneWaveform(uint16_t *samples, uint16_t count,
                                     uint8_t is_square_wave,
                                     uint16_t *output_min,
                                     uint16_t *output_max)
{
  uint16_t minimum = 0xFFFFU;
  uint16_t maximum = 0U;
  uint16_t span;
  int32_t midpoint;
  uint32_t scale_q12 = 4096U;
  uint16_t new_minimum = 0xFFFFU;
  uint16_t new_maximum = 0U;

  if ((samples == NULL) || (count == 0U) || (is_square_wave != 0U))
  {
    return;
  }

  for (uint16_t i = 0U; i < count; ++i)
  {
    if (samples[i] < minimum) minimum = samples[i];
    if (samples[i] > maximum) maximum = samples[i];
  }
  span = (uint16_t)(maximum - minimum);
  if (span < MIC_MIN_AUTO_CENTER_SPAN_MV)
  {
    return;
  }

  midpoint = ((int32_t)minimum + (int32_t)maximum) / 2;
  if (span > MIC_MAX_DISPLAY_SPAN_MV)
  {
    scale_q12 =
        ((uint32_t)MIC_MAX_DISPLAY_SPAN_MV * 4096U + span / 2U) / span;
  }

  for (uint16_t i = 0U; i < count; ++i)
  {
    int32_t relative = ((int32_t)samples[i] - midpoint);
    int32_t value =
        MIC_CENTER_MV + (int32_t)((relative * (int32_t)scale_q12) / 4096);

    if (value < 0) value = 0;
    if (value > 3000) value = 3000;
    samples[i] = (uint16_t)value;
    if (samples[i] < new_minimum) new_minimum = samples[i];
    if (samples[i] > new_maximum) new_maximum = samples[i];
  }

  if (output_min != NULL) *output_min = new_minimum;
  if (output_max != NULL) *output_max = new_maximum;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  if (!Acquisition_Init())
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t next_frame_tick = HAL_GetTick();

  while (1)
  {
    Acquisition_Process();

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_frame_tick) >= 0)
    {
      ProtocolScanMode scan_mode;
      uint32_t capture_tick;

      if (Acquisition_GetLatestFrame(ch1_samples,
                                     ch2_samples,
                                     digital_samples,
                                     PROTOCOL_SAMPLES_PER_CHANNEL,
                                     &scan_mode,
                                     &capture_tick))
      {
        uint16_t ch1_min = 0xFFFFU;
        uint16_t ch1_max = 0U;
        uint16_t ch2_min = 0xFFFFU;
        uint16_t ch2_max = 0U;
        uint16_t ch1_raw_min = 0xFFFFU;
        uint16_t ch1_raw_max = 0U;
        uint16_t ch2_raw_min = 0xFFFFU;
        uint16_t ch2_raw_max = 0U;
        uint32_t ch1_raw_sum = 0U;
        uint16_t d1_high_count = 0U;
        uint16_t d1_low_run = 0U;
        uint16_t d1_high_run = 0U;
        uint16_t d1_max_low_run = 0U;
        uint16_t d1_max_high_run = 0U;
        uint16_t d2_high_count = 0U;
        uint16_t d3_high_count = 0U;
        uint16_t ch2_low_rail_count = 0U;
        uint16_t ch2_high_rail_count = 0U;

        for (uint16_t i = 0U; i < PROTOCOL_SAMPLES_PER_CHANNEL; ++i)
        {
          uint16_t mic_filtered = MicrophoneFilterSample(ch2_samples, i);

          /*
           * The five-point filter suppresses ADC and audio-module noise.
           * Large edges are handled inside MicrophoneFilterSample(), so the
           * same acquisition path still displays the generated square wave.
           */
          ch2_mv[i] = AdcRawToMillivolts(mic_filtered);
          if (ch2_mv[i] < ch2_min) ch2_min = ch2_mv[i];
          if (ch2_mv[i] > ch2_max) ch2_max = ch2_mv[i];
          if (ch2_mv[i] <= MIC_RAIL_LOW_MV) ch2_low_rail_count++;
          if (ch2_mv[i] >= MIC_RAIL_HIGH_MV) ch2_high_rail_count++;

          ch1_raw_sum += ch1_samples[i];
          if (ch1_samples[i] < ch1_raw_min) ch1_raw_min = ch1_samples[i];
          if (ch1_samples[i] > ch1_raw_max) ch1_raw_max = ch1_samples[i];
          if (ch2_samples[i] < ch2_raw_min) ch2_raw_min = ch2_samples[i];
          if (ch2_samples[i] > ch2_raw_max) ch2_raw_max = ch2_samples[i];

          if ((digital_samples[i] & 0x01U) != 0U)
          {
            d1_high_count++;
            d1_high_run++;
            d1_low_run = 0U;
            if (d1_high_run > d1_max_high_run) d1_max_high_run = d1_high_run;
          }
          else
          {
            d1_low_run++;
            d1_high_run = 0U;
            if (d1_low_run > d1_max_low_run) d1_max_low_run = d1_low_run;
          }
          if ((digital_samples[i] & 0x02U) != 0U) d2_high_count++;
          if ((digital_samples[i] & 0x04U) != 0U) d3_high_count++;
        }

        /*
         * A real 0/3.3 V square wave occupies both rails for much of the
         * frame. A microphone transient may produce one large jump, but does
         * not fill both rails. Requiring both rails avoids incorrectly
         * disabling microphone centering during loud audio.
         */
        {
          uint8_t is_square_wave =
              ((ch2_low_rail_count * 4U) > PROTOCOL_SAMPLES_PER_CHANNEL) &&
              ((ch2_high_rail_count * 4U) > PROTOCOL_SAMPLES_PER_CHANNEL);

          CenterMicrophoneWaveform(ch2_mv,
                                   PROTOCOL_SAMPLES_PER_CHANNEL,
                                   is_square_wave,
                                   &ch2_min,
                                   &ch2_max);
        }

        /*
         * CH1 is a potentiometer: its meaningful information is one DC level.
         * A frame average removes ADC noise and supply ripple while preserving
         * the voltage selected by the knob. CH2 remains unfiltered audio.
         */
        {
          uint16_t ch1_raw_average = (uint16_t)(
              (ch1_raw_sum + (PROTOCOL_SAMPLES_PER_CHANNEL / 2U)) /
              PROTOCOL_SAMPLES_PER_CHANNEL);
          uint16_t ch1_mv_average = AdcRawToMillivolts(ch1_raw_average);
          uint32_t now = HAL_GetTick();

          if (!pot_filter_initialized)
          {
            pot_filtered_mv = ch1_mv_average;
            pot_filter_initialized = 1U;
          }
          else
          {
            uint32_t elapsed_ms = now - pot_filter_last_tick;
            int32_t difference =
                (int32_t)ch1_mv_average - (int32_t)pot_filtered_mv;

            if (elapsed_ms == 0U) elapsed_ms = 1U;
            if (elapsed_ms > 1000U) elapsed_ms = 1000U;
            pot_filtered_mv = (uint16_t)(
                (int32_t)pot_filtered_mv +
                (difference * (int32_t)elapsed_ms) /
                    (int32_t)(elapsed_ms + 250U));
          }
          pot_filter_last_tick = now;

          for (uint16_t i = 0U; i < PROTOCOL_SAMPLES_PER_CHANNEL; ++i)
          {
            ch1_mv[i] = pot_filtered_mv;
          }
          /* CH1 in the log is the value actually transmitted to the display. */
          ch1_min = pot_filtered_mv;
          ch1_max = pot_filtered_mv;
        }

        uint16_t frame_length =
            Protocol_BuildFullFrame(ch1_mv,
                                    ch2_mv,
                                    digital_samples,
                                    PROTOCOL_SAMPLES_PER_CHANNEL,
                                    Protocol_ScanIntervalUs(scan_mode),
                                    tx_sequence,
                                    tx_frame);

        if (frame_length == PROTOCOL_FULL_FRAME_SIZE)
        {
          if (Acquisition_TransmitFrame(tx_frame, frame_length))
          {
            Acquisition_ReportFrameStatus(
                tx_sequence,
                PROTOCOL_SAMPLES_PER_CHANNEL,
                Protocol_ScanIntervalUs(scan_mode),
                frame_length,
                ch1_min,
                ch1_max,
                ch2_min,
                ch2_max,
                ch1_raw_min,
                ch1_raw_max,
                ch2_raw_min,
                ch2_raw_max,
                d1_high_count,
                d1_max_low_run,
                d1_max_high_run,
                d2_high_count,
                d3_high_count);
          }
          tx_sequence++;
        }
      }

      next_frame_tick += 20U;
      now = HAL_GetTick();
      if ((int32_t)(now - next_frame_tick) >= 0)
      {
        next_frame_tick = now + 20U;
      }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
