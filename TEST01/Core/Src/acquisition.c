/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    acquisition.c
  * @brief   ADC3/TIM8/DMA acquisition and serial command handling.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "acquisition.h"

#include "adc.h"
#include "tim.h"
#include "usart.h"

#define ADC_FRAME_PAIRS                 PROTOCOL_SAMPLES_PER_CHANNEL
#define ADC_FRAME_HALF_WORDS            (ADC_FRAME_PAIRS * 2U)
#define ADC_DMA_HALF_WORDS              (ADC_FRAME_HALF_WORDS * 2U)

#define TIMER_CLOCK_HZ                  16000000UL
#define TIMER_ARR_SCAN_10MS             ((TIMER_CLOCK_HZ / 5000UL) - 1UL)
#define TIMER_ARR_SCAN_5MS              ((TIMER_CLOCK_HZ / 10000UL) - 1UL)
#define TIMER_ARR_SCAN_1MS              ((TIMER_CLOCK_HZ / 50000UL) - 1UL)

#define UART_RX_RING_SIZE               256U
#define UART_RX_RING_MASK               (UART_RX_RING_SIZE - 1U)
#define UART7_DMA_RX_SIZE               32U

typedef struct
{
  volatile uint8_t buffer[UART_RX_RING_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
  uint8_t byte;
  char command[8];
  uint8_t command_length;
  ProtocolControlParser parser;
} UartRxRing;

static uint16_t adc_dma_buffer[ADC_DMA_HALF_WORDS];
static uint16_t ready_frames[2][ADC_FRAME_HALF_WORDS];
static uint8_t digital_dma_buffer[ADC_FRAME_PAIRS * 2U];
static uint8_t ready_digital[2][ADC_FRAME_PAIRS];

static volatile uint8_t ready_write_index;
static volatile uint8_t latest_ready_index;
static volatile uint8_t ready_frame_available;
static volatile uint32_t capture_tick;
static volatile uint8_t adc_error_latched;
static volatile uint8_t acquisition_running;
static volatile uint32_t uart1_rx_bytes;
static volatile uint32_t uart7_rx_bytes;
static volatile uint16_t digital_sample_index;

static UartRxRing uart1_rx;
static UartRxRing uart7_rx;
static uint8_t uart7_dma_rx[UART7_DMA_RX_SIZE];
static ProtocolScanMode current_scan_mode = PROTOCOL_SCAN_10MS;
static AcquisitionStats acquisition_stats;

static uint8_t Acquisition_PushAsciiControl(UartRxRing *ring,
                                            uint8_t byte,
                                            ProtocolScanMode *mode)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    if ((ring->command_length == 4U) &&
        (ring->command[0] == 'T') &&
        (ring->command[1] == 'B') &&
        (ring->command[2] == '=') &&
        (ring->command[3] >= '0') &&
        (ring->command[3] <= '2'))
    {
      *mode = (ProtocolScanMode)(ring->command[3] - '0');
      ring->command_length = 0U;
      return 1U;
    }
    ring->command_length = 0U;
    return 0U;
  }

  if ((byte >= 0x20U) && (byte <= 0x7EU) &&
      (ring->command_length < sizeof(ring->command)))
  {
    ring->command[ring->command_length++] = (char)byte;
  }
  else
  {
    ring->command_length = 0U;
  }
  return 0U;
}

static void Acquisition_UartWriteRaw(UART_HandleTypeDef *uart,
                                     const uint8_t *data,
                                     uint16_t length)
{
  uint16_t index;

  if ((uart == NULL) || (data == NULL))
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    while ((uart->Instance->ISR & USART_ISR_TXE) == 0U)
    {
    }
    uart->Instance->TDR = data[index];
  }
  while ((uart->Instance->ISR & USART_ISR_TC) == 0U)
  {
  }
}

static void Acquisition_FormatUint(char *output, uint32_t value)
{
  char reverse[11];
  uint8_t count = 0U;
  uint8_t index = 0U;

  if (value == 0U)
  {
    output[0] = '0';
    output[1] = '\0';
    return;
  }

  while ((value > 0U) && (count < sizeof(reverse)))
  {
    reverse[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while (count > 0U)
  {
    output[index++] = reverse[--count];
  }
  output[index] = '\0';
}

static void Acquisition_AppendText(char *output,
                                   uint8_t *position,
                                   uint8_t capacity,
                                   const char *text)
{
  while ((*text != '\0') && (*position < (uint8_t)(capacity - 1U)))
  {
    output[(*position)++] = *text++;
  }
}

static void Acquisition_AppendUint(char *output,
                                   uint8_t *position,
                                   uint8_t capacity,
                                   uint32_t value)
{
  char number[11];
  uint8_t index = 0U;

  Acquisition_FormatUint(number, value);
  while ((number[index] != '\0') &&
         (*position < (uint8_t)(capacity - 1U)))
  {
    output[(*position)++] = number[index++];
  }
}

static void Acquisition_DebugControl(ProtocolScanMode mode,
                                     uint16_t interval_us)
{
  char line[48];
  uint8_t position = 0U;

  Acquisition_AppendText(line, &position, sizeof(line), "RX CTRL,TB=");
  Acquisition_AppendUint(line, &position, sizeof(line), (uint32_t)mode);
  Acquisition_AppendText(line, &position, sizeof(line), ",INTERVAL=");
  Acquisition_AppendUint(line, &position, sizeof(line), interval_us);
  Acquisition_AppendText(line, &position, sizeof(line), "\r\n");
  Acquisition_UartWriteRaw(&huart1, (const uint8_t *)line, position);
}

static uint32_t Acquisition_TimerPeriod(ProtocolScanMode scan_mode)
{
  switch (scan_mode)
  {
    case PROTOCOL_SCAN_5MS:
      return TIMER_ARR_SCAN_5MS;

    case PROTOCOL_SCAN_1MS:
      return TIMER_ARR_SCAN_1MS;

    case PROTOCOL_SCAN_10MS:
    default:
      return TIMER_ARR_SCAN_10MS;
  }
}

static void Acquisition_ClearCapture(void)
{
  latest_ready_index = 0U;
  ready_write_index = 1U;
  ready_frame_available = 0U;
  capture_tick = 0U;
  digital_sample_index = 0U;
}

static void Acquisition_Stop(void)
{
  (void)HAL_TIM_Base_Stop_IT(&htim8);
  (void)HAL_ADC_Stop_DMA(&hadc3);
  acquisition_running = 0U;
}

static uint8_t Acquisition_Start(void)
{
  Acquisition_ClearCapture();
  __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_OVR);

  if (HAL_ADC_Start_DMA(&hadc3,
                        (uint32_t *)adc_dma_buffer,
                        ADC_DMA_HALF_WORDS) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_TIM_Base_Start_IT(&htim8) != HAL_OK)
  {
    (void)HAL_ADC_Stop_DMA(&hadc3);
    return 0U;
  }

  acquisition_running = 1U;
  return 1U;
}

/*
 * Test-wave frequencies are scaled with the scan-time setting so that every
 * setting shows approximately 10, 5 and 25 periods across the full screen.
 *
 * TIM3/TIM5/TIM12 run from a 1 MHz counter after /16 prescaling.
 */
static void Acquisition_ConfigureTestOutputs(ProtocolScanMode scan_mode)
{
  uint32_t d1_hz;
  uint32_t d2_hz;
  uint32_t d3_hz;
  uint32_t d1_period;
  uint32_t d2_period;
  uint32_t d3_period;

  switch (scan_mode)
  {
    case PROTOCOL_SCAN_5MS:
      d1_hz = 200U;
      d2_hz = 100U;
      d3_hz = 500U;
      break;

    case PROTOCOL_SCAN_1MS:
      d1_hz = 1000U;
      d2_hz = 500U;
      d3_hz = 2500U;
      break;

    case PROTOCOL_SCAN_10MS:
    default:
      d1_hz = 100U;
      d2_hz = 50U;
      d3_hz = 250U;
      break;
  }

  /* Timer counter clock = 16 MHz / (15 + 1) = 1 MHz. */
  __HAL_TIM_SET_PRESCALER(&htim3, 15U);
  __HAL_TIM_SET_PRESCALER(&htim5, 15U);
  __HAL_TIM_SET_PRESCALER(&htim12, 15U);

  d1_period = (1000000UL / d1_hz) - 1U;
  d2_period = (1000000UL / d2_hz) - 1U;
  d3_period = (1000000UL / d3_hz) - 1U;

  __HAL_TIM_SET_AUTORELOAD(&htim3, d1_period);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (d1_period + 1U) / 2U);
  __HAL_TIM_SET_AUTORELOAD(&htim5, d2_period);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (d2_period + 1U) / 2U);
  __HAL_TIM_SET_AUTORELOAD(&htim12, d3_period);
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, (d3_period + 1U) / 2U);
}

static uint8_t Acquisition_ApplyMode(ProtocolScanMode scan_mode)
{
  uint32_t period = Acquisition_TimerPeriod(scan_mode);

  __HAL_TIM_SET_PRESCALER(&htim8, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim8, period);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  htim8.Init.Period = period;
  current_scan_mode = scan_mode;
  Acquisition_ConfigureTestOutputs(scan_mode);

  return Acquisition_Start();
}

static void Acquisition_HandleCommand(ProtocolScanMode requested_mode)
{
  if ((requested_mode == current_scan_mode) && acquisition_running)
  {
    return;
  }

  ProtocolScanMode previous_mode = current_scan_mode;
  Acquisition_Stop();

  if (!Acquisition_ApplyMode(requested_mode))
  {
    Acquisition_Stop();
    current_scan_mode = previous_mode;
    (void)Acquisition_ApplyMode(previous_mode);
    acquisition_stats.invalid_commands++;
  }
  else
  {
    Acquisition_DebugControl(requested_mode,
                             Protocol_ScanIntervalUs(requested_mode));
  }
}

static void Acquisition_RearmUartReceive(UART_HandleTypeDef *huart,
                                         UartRxRing *ring)
{
  if (HAL_UART_Receive_IT(huart, &ring->byte, 1U) != HAL_OK)
  {
    acquisition_stats.uart_errors++;
  }
}

static void Acquisition_RearmUart7Dma(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_rx,
                                   UART7_DMA_RX_SIZE) != HAL_OK)
  {
    acquisition_stats.uart_errors++;
  }
}

uint8_t Acquisition_Init(void)
{
  acquisition_stats.adc_errors = 0U;
  acquisition_stats.uart_errors = 0U;
  acquisition_stats.uart_dropped_bytes = 0U;
  acquisition_stats.invalid_commands = 0U;
  uart1_rx_bytes = 0U;
  uart7_rx_bytes = 0U;
  adc_error_latched = 0U;
  uart1_rx.head = 0U;
  uart1_rx.tail = 0U;
  uart7_rx.head = 0U;
  uart7_rx.tail = 0U;
  uart1_rx.command_length = 0U;
  uart7_rx.command_length = 0U;
  Protocol_InitControlParser(&uart1_rx.parser);
  Protocol_InitControlParser(&uart7_rx.parser);

  if (!Acquisition_ApplyMode(PROTOCOL_SCAN_10MS))
  {
    return 0U;
  }

  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  Acquisition_RearmUartReceive(&huart1, &uart1_rx);
  Acquisition_RearmUart7Dma();
  Acquisition_UartWriteRaw(
      &huart1,
      (const uint8_t *)"A1 READY,ROLE=ACQ,UART7=1000000,CH1=PF9/CH2=PA0\r\n",
      (uint16_t)(sizeof("A1 READY,ROLE=ACQ,UART7=1000000,CH1=PF9/CH2=PA0\r\n") -
                 1U));
  Acquisition_UartWriteRaw(
      &huart1,
      (const uint8_t *)"TEST PWM:D1=TIM3_CH1/D3,D2=TIM5_CH4/D5,D3=TIM12_CH1/D6\r\n",
      (uint16_t)(sizeof("TEST PWM:D1=TIM3_CH1/D3,D2=TIM5_CH4/D5,D3=TIM12_CH1/D6\r\n") -
                 1U));
  Acquisition_UartWriteRaw(
      &huart7,
      (const uint8_t *)"LINK READY,ROLE=TX\r\n",
      (uint16_t)(sizeof("LINK READY,ROLE=TX\r\n") - 1U));
  return 1U;
}

uint8_t Acquisition_TransmitFrame(const uint8_t *frame, uint16_t length)
{
  if ((frame == NULL) || (length == 0U))
  {
    return 0U;
  }

  Acquisition_UartWriteRaw(&huart7, frame, length);
  return 1U;
}

void Acquisition_ReportFrameStatus(uint16_t sequence,
                                   uint16_t sample_count,
                                   uint16_t interval_us,
                                   uint16_t frame_length,
                                   uint16_t ch1_min_mv,
                                   uint16_t ch1_max_mv,
                                   uint16_t ch2_min_mv,
                                   uint16_t ch2_max_mv,
                                   uint16_t ch1_raw_min,
                                   uint16_t ch1_raw_max,
                                   uint16_t ch2_raw_min,
                                   uint16_t ch2_raw_max)
{
  char line[224];
  uint8_t position = 0U;

  Acquisition_AppendText(line, &position, sizeof(line), "TX,SEQ=");
  Acquisition_AppendUint(line, &position, sizeof(line), sequence);
  Acquisition_AppendText(line, &position, sizeof(line), ",COUNT=");
  Acquisition_AppendUint(line, &position, sizeof(line), sample_count);
  Acquisition_AppendText(line, &position, sizeof(line), ",INTERVAL=");
  Acquisition_AppendUint(line, &position, sizeof(line), interval_us);
  Acquisition_AppendText(line, &position, sizeof(line), ",LEN=");
  Acquisition_AppendUint(line, &position, sizeof(line), frame_length);
  Acquisition_AppendText(line, &position, sizeof(line), ",CH1=");
  Acquisition_AppendUint(line, &position, sizeof(line), ch1_min_mv);
  Acquisition_AppendText(line, &position, sizeof(line), "/");
  Acquisition_AppendUint(line, &position, sizeof(line), ch1_max_mv);
  Acquisition_AppendText(line, &position, sizeof(line), ",CH2=");
  Acquisition_AppendUint(line, &position, sizeof(line), ch2_min_mv);
  Acquisition_AppendText(line, &position, sizeof(line), "/");
  Acquisition_AppendUint(line, &position, sizeof(line), ch2_max_mv);
  Acquisition_AppendText(line, &position, sizeof(line), ",CH1RAW=");
  Acquisition_AppendUint(line, &position, sizeof(line), ch1_raw_min);
  Acquisition_AppendText(line, &position, sizeof(line), "/");
  Acquisition_AppendUint(line, &position, sizeof(line), ch1_raw_max);
  Acquisition_AppendText(line, &position, sizeof(line), ",CH2RAW=");
  Acquisition_AppendUint(line, &position, sizeof(line), ch2_raw_min);
  Acquisition_AppendText(line, &position, sizeof(line), "/");
  Acquisition_AppendUint(line, &position, sizeof(line), ch2_raw_max);
  Acquisition_AppendText(line, &position, sizeof(line), ",ADCERR=");
  Acquisition_AppendUint(line, &position, sizeof(line),
                         acquisition_stats.adc_errors);
  Acquisition_AppendText(line, &position, sizeof(line), ",UARTERR=");
  Acquisition_AppendUint(line, &position, sizeof(line),
                         acquisition_stats.uart_errors);
  Acquisition_AppendText(line, &position, sizeof(line), ",DROP=");
  Acquisition_AppendUint(line, &position, sizeof(line),
                         acquisition_stats.uart_dropped_bytes);
  Acquisition_AppendText(line, &position, sizeof(line), ",RX1=");
  Acquisition_AppendUint(line, &position, sizeof(line), uart1_rx_bytes);
  Acquisition_AppendText(line, &position, sizeof(line), ",RX7=");
  Acquisition_AppendUint(line, &position, sizeof(line), uart7_rx_bytes);
  Acquisition_AppendText(line, &position, sizeof(line), "\r\n");
  Acquisition_UartWriteRaw(&huart1, (const uint8_t *)line, position);
}

void Acquisition_Process(void)
{
  while (uart1_rx.tail != uart1_rx.head)
  {
    ProtocolScanMode mode;
    uint8_t byte = uart1_rx.buffer[uart1_rx.tail];
    uart1_rx.tail = (uint8_t)((uart1_rx.tail + 1U) & UART_RX_RING_MASK);

    uint8_t ascii_command =
        Acquisition_PushAsciiControl(&uart1_rx, byte, &mode);
    uint8_t binary_command =
        Protocol_PushControlByte(&uart1_rx.parser, byte, &mode);

    if (binary_command)
    {
      uart1_rx.command_length = 0U;
    }
    if (ascii_command || binary_command)
    {
      Acquisition_HandleCommand(mode);
    }
  }

  while (uart7_rx.tail != uart7_rx.head)
  {
    ProtocolScanMode mode;
    uint8_t byte = uart7_rx.buffer[uart7_rx.tail];
    uart7_rx.tail = (uint8_t)((uart7_rx.tail + 1U) & UART_RX_RING_MASK);

    uint8_t ascii_command =
        Acquisition_PushAsciiControl(&uart7_rx, byte, &mode);
    uint8_t binary_command =
        Protocol_PushControlByte(&uart7_rx.parser, byte, &mode);

    if (binary_command)
    {
      uart7_rx.command_length = 0U;
    }
    if (ascii_command || binary_command)
    {
      Acquisition_HandleCommand(mode);
    }
  }

  if (adc_error_latched && !acquisition_running)
  {
    static uint32_t next_retry_tick;
    uint32_t now = HAL_GetTick();

    if (next_retry_tick == 0U)
    {
      next_retry_tick = now + 100U;
    }
    else if ((int32_t)(now - next_retry_tick) >= 0)
    {
      next_retry_tick = 0U;
      if (Acquisition_ApplyMode(current_scan_mode))
      {
        adc_error_latched = 0U;
      }
    }
  }
}

uint8_t Acquisition_GetLatestFrame(uint16_t *ch1,
                                   uint16_t *ch2,
                                   uint8_t *digital_samples,
                                   uint16_t sample_count,
                                   ProtocolScanMode *scan_mode,
                                   uint32_t *capture_tick_out)
{
  uint8_t updated = 0U;
  uint8_t index;

  if ((ch1 == NULL) || (ch2 == NULL) || (digital_samples == NULL) ||
      (scan_mode == NULL) ||
      (capture_tick_out == NULL) || (sample_count != ADC_FRAME_PAIRS))
  {
    return 0U;
  }

  if (ready_frame_available)
  {
    index = latest_ready_index;
    const uint16_t *source = ready_frames[index];

    for (uint16_t i = 0U; i < sample_count; ++i)
    {
      /* Rank 1 = PF9/pot is CH1, rank 2 = PA0/microphone is CH2. */
      ch1[i] = source[i * 2U];
      ch2[i] = source[(i * 2U) + 1U];
      digital_samples[i] = ready_digital[index][i];
    }

    *scan_mode = current_scan_mode;
    *capture_tick_out = capture_tick;
    if (latest_ready_index == index)
    {
      ready_frame_available = 0U;
    }
    updated = 1U;
  }

  return updated;
}

ProtocolScanMode Acquisition_GetScanMode(void)
{
  return current_scan_mode;
}

uint8_t Acquisition_SetMode(ProtocolScanMode mode)
{
  if ((mode != PROTOCOL_SCAN_10MS) &&
      (mode != PROTOCOL_SCAN_5MS) &&
      (mode != PROTOCOL_SCAN_1MS))
  {
    return 0U;
  }

  Acquisition_HandleCommand(mode);
  return (uint8_t)(current_scan_mode == mode);
}

void Acquisition_GetStats(AcquisitionStats *stats)
{
  if (stats != NULL)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *stats = acquisition_stats;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
}

static void Acquisition_StoreDmaHalf(const uint16_t *source,
                                     const uint8_t *digital_source)
{
  uint8_t next_index = (uint8_t)(ready_write_index ^ 1U);

  for (uint16_t i = 0U; i < ADC_FRAME_HALF_WORDS; ++i)
  {
    ready_frames[next_index][i] = source[i];
    ready_digital[next_index][i / 2U] = digital_source[i / 2U];
  }

  latest_ready_index = next_index;
  ready_write_index = next_index;
  capture_tick = HAL_GetTick();
  ready_frame_available = 1U;
}

void Acquisition_TimerTick(void)
{
  uint8_t value = 0U;

  if (!acquisition_running)
  {
    return;
  }

  /* D1=Arduino D7/PI3, D2=Arduino D8/PI2, D3=Arduino D9/PA15. */
  if ((DIG_CH1_GPIO_Port->IDR & DIG_CH1_Pin) != 0U) value |= 0x01U;
  if ((DIG_CH2_GPIO_Port->IDR & DIG_CH2_Pin) != 0U) value |= 0x02U;
  if ((DIG_CH3_GPIO_Port->IDR & DIG_CH3_Pin) != 0U) value |= 0x04U;

  digital_dma_buffer[digital_sample_index++] = value;
  if (digital_sample_index >= (ADC_FRAME_PAIRS * 2U))
  {
    digital_sample_index = 0U;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    Acquisition_StoreDmaHalf(&adc_dma_buffer[0], &digital_dma_buffer[0]);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    Acquisition_StoreDmaHalf(&adc_dma_buffer[ADC_FRAME_HALF_WORDS],
                             &digital_dma_buffer[ADC_FRAME_PAIRS]);
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    acquisition_stats.adc_errors++;
    acquisition_running = 0U;
    adc_error_latched = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_rx_bytes++;
    {
      uint8_t next_head =
          (uint8_t)((uart1_rx.head + 1U) & UART_RX_RING_MASK);

      if (next_head != uart1_rx.tail)
      {
        uart1_rx.buffer[uart1_rx.head] = uart1_rx.byte;
        uart1_rx.head = next_head;
      }
      else
      {
        acquisition_stats.uart_dropped_bytes++;
      }

      Acquisition_RearmUartReceive(huart, &uart1_rx);
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART7)
  {
    uart7_rx_bytes += Size;
    for (uint16_t i = 0U; i < Size; ++i)
    {
      uint8_t next_head =
          (uint8_t)((uart7_rx.head + 1U) & UART_RX_RING_MASK);
      if (next_head != uart7_rx.tail)
      {
        uart7_rx.buffer[uart7_rx.head] = uart7_dma_rx[i];
        uart7_rx.head = next_head;
      }
      else
      {
        acquisition_stats.uart_dropped_bytes++;
      }
    }
    Acquisition_RearmUart7Dma();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    acquisition_stats.uart_errors++;
    Acquisition_RearmUartReceive(&huart1, &uart1_rx);
  }
  else if (huart->Instance == UART7)
  {
    acquisition_stats.uart_errors++;
    (void)HAL_UART_AbortReceive(&huart7);
    Acquisition_RearmUart7Dma();
  }
}

