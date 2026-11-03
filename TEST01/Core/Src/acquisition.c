/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    acquisition.c
  * @brief   ADC3/TIM8/DMA acquisition and serial command handling.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "acquisition.h"

#include <string.h>

#include "adc.h"
#include "tim.h"
#include "usart.h"

#define ADC_FRAME_PAIRS                 PROTOCOL_SAMPLES_PER_CHANNEL
#define ADC_FRAME_HALF_WORDS            (ADC_FRAME_PAIRS * 2U)
#define ADC_DMA_HALF_WORDS              (ADC_FRAME_HALF_WORDS * 2U)

#define TIM8_PRESCALER                  215U
#define TIMER_CLOCK_HZ                  1000000UL
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
static uint8_t digital_capture[2][ADC_FRAME_PAIRS];
static uint8_t ready_digital[2][ADC_FRAME_PAIRS];

static volatile uint8_t ready_write_index;
static volatile uint8_t latest_ready_index;
static volatile uint8_t ready_frame_available;
static volatile uint32_t capture_tick;
static volatile uint8_t adc_error_latched;
static volatile uint8_t acquisition_running;
static volatile uint32_t uart1_rx_bytes;
static volatile uint32_t uart7_rx_bytes;
static volatile uint16_t digital_write_index;
static volatile uint8_t digital_write_half;
static volatile uint8_t digital_completed_half;
static volatile uint8_t digital_previous_value;
static volatile uint32_t digital_edge_count[3];
static uint16_t generator_frequency_hz = 1000U;

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
                                   uint16_t *position,
                                   uint16_t capacity,
                                   const char *text)
{
  while ((*text != '\0') && (*position < (uint16_t)(capacity - 1U)))
  {
    output[(*position)++] = *text++;
  }
}

static void Acquisition_AppendUint(char *output,
                                   uint16_t *position,
                                   uint16_t capacity,
                                   uint32_t value)
{
  char number[11];
  uint8_t index = 0U;

  Acquisition_FormatUint(number, value);
  while ((number[index] != '\0') &&
         (*position < (uint16_t)(capacity - 1U)))
  {
    output[(*position)++] = number[index++];
  }
}

static void Acquisition_DebugControl(ProtocolScanMode mode,
                                     uint16_t interval_us)
{
  char line[48];
  uint16_t position = 0U;

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
  memset(digital_capture, 0, sizeof(digital_capture));
  memset(ready_digital, 0, sizeof(ready_digital));
  digital_write_index = 0U;
  digital_write_half = 0U;
  digital_completed_half = 0U;
  digital_previous_value = 0U;
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

static uint8_t Acquisition_ApplyMode(ProtocolScanMode scan_mode)
{
  uint32_t period = Acquisition_TimerPeriod(scan_mode);

  __HAL_TIM_SET_PRESCALER(&htim8, TIM8_PRESCALER);
  __HAL_TIM_SET_AUTORELOAD(&htim8, period);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  htim8.Init.Period = period;
  current_scan_mode = scan_mode;

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
  digital_previous_value = 0U;
  digital_edge_count[0] = 0U;
  digital_edge_count[1] = 0U;
  digital_edge_count[2] = 0U;
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

  if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
      !Acquisition_SetGeneratorFrequency(1000U))
  {
    return 0U;
  }

  {
    uint8_t d1_started =
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK) ? 1U : 0U;
    uint8_t d2_started =
        (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) == HAL_OK) ? 1U : 0U;
    uint8_t d3_started =
        (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) == HAL_OK) ? 1U : 0U;
    char test_status[40];
    uint16_t position = 0U;

    Acquisition_AppendText(test_status, &position, sizeof(test_status),
                           "TEST PWM START,D1=");
    Acquisition_AppendUint(test_status, &position, sizeof(test_status),
                           d1_started);
    Acquisition_AppendText(test_status, &position, sizeof(test_status), ",D2=");
    Acquisition_AppendUint(test_status, &position, sizeof(test_status),
                           d2_started);
    Acquisition_AppendText(test_status, &position, sizeof(test_status), ",D3=");
    Acquisition_AppendUint(test_status, &position, sizeof(test_status),
                           d3_started);
    Acquisition_AppendText(test_status, &position, sizeof(test_status), "\r\n");
    Acquisition_UartWriteRaw(&huart1, (const uint8_t *)test_status, position);
  }

  /*
   * USART1 is consumed directly by its interrupt handler.  Draining every
   * byte already present in RXNE avoids losing the remaining characters when
   * a one-byte HAL_UART_Receive_IT transaction cannot be restarted quickly
   * enough between two back-to-back characters.
   */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
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
      &huart1,
      (const uint8_t *)"GEN PWM:D10/PA8,TIM1_CH1,100-10000Hz\r\n",
      (uint16_t)(sizeof("GEN PWM:D10/PA8,TIM1_CH1,100-10000Hz\r\n") - 1U));
  Acquisition_UartWriteRaw(
      &huart7,
      (const uint8_t *)"LINK READY,ROLE=TX\r\n",
      (uint16_t)(sizeof("LINK READY,ROLE=TX\r\n") - 1U));
  return 1U;
}

uint8_t Acquisition_SetGeneratorFrequency(uint16_t frequency_hz)
{
  uint32_t period;
  uint32_t pulse;

  if ((frequency_hz < 100U) || (frequency_hz > 10000U))
  {
    return 0U;
  }

  period = 1000000UL / frequency_hz;
  if (period < 2U)
  {
    return 0U;
  }
  pulse = period / 2U;
  __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
  htim1.Init.Period = period - 1U;
  generator_frequency_hz = frequency_hz;

  {
    char line[32];
    uint16_t position = 0U;
    Acquisition_AppendText(line, &position, sizeof(line), "GEN FREQ=");
    Acquisition_AppendUint(line, &position, sizeof(line), frequency_hz);
    Acquisition_AppendText(line, &position, sizeof(line), "\r\n");
    Acquisition_UartWriteRaw(&huart1, (const uint8_t *)line, position);
  }
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
                                   uint16_t ch2_raw_max,
                                   uint16_t d1_high_count,
                                   uint16_t d1_max_low_run,
                                   uint16_t d1_max_high_run,
                                   uint16_t d2_high_count,
                                   uint16_t d3_high_count)
{
  char line[288];
  uint16_t position = 0U;

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
  Acquisition_AppendText(line, &position, sizeof(line), ",EDGE1=");
  Acquisition_AppendUint(line, &position, sizeof(line), digital_edge_count[0]);
  Acquisition_AppendText(line, &position, sizeof(line), ",EDGE2=");
  Acquisition_AppendUint(line, &position, sizeof(line), digital_edge_count[1]);
  Acquisition_AppendText(line, &position, sizeof(line), ",EDGE3=");
  Acquisition_AppendUint(line, &position, sizeof(line), digital_edge_count[2]);
  Acquisition_AppendText(line, &position, sizeof(line), ",D1H=");
  Acquisition_AppendUint(line, &position, sizeof(line), d1_high_count);
  Acquisition_AppendText(line, &position, sizeof(line), ",D1LMAX=");
  Acquisition_AppendUint(line, &position, sizeof(line), d1_max_low_run);
  Acquisition_AppendText(line, &position, sizeof(line), ",D1HMAX=");
  Acquisition_AppendUint(line, &position, sizeof(line), d1_max_high_run);
  Acquisition_AppendText(line, &position, sizeof(line), ",GEN=");
  Acquisition_AppendUint(line, &position, sizeof(line), generator_frequency_hz);
  Acquisition_AppendText(line, &position, sizeof(line), ",D2H=");
  Acquisition_AppendUint(line, &position, sizeof(line), d2_high_count);
  Acquisition_AppendText(line, &position, sizeof(line), ",D3H=");
  Acquisition_AppendUint(line, &position, sizeof(line), d3_high_count);
  Acquisition_AppendText(line, &position, sizeof(line), "\r\n");
  Acquisition_UartWriteRaw(&huart1, (const uint8_t *)line, position);
}

void Acquisition_Process(void)
{
  while (uart1_rx.tail != uart1_rx.head)
  {
    ProtocolScanMode mode;
    uint16_t generator_frequency = 0U;
    uint8_t byte = uart1_rx.buffer[uart1_rx.tail];
    uart1_rx.tail = (uint8_t)((uart1_rx.tail + 1U) & UART_RX_RING_MASK);

    uint8_t ascii_command =
        Acquisition_PushAsciiControl(&uart1_rx, byte, &mode);
    uint8_t binary_command =
        Protocol_PushControlByte(&uart1_rx.parser, byte, &mode,
                                 &generator_frequency);

    if (binary_command)
    {
      uart1_rx.command_length = 0U;
    }
    if (ascii_command || (binary_command == 1U))
    {
      Acquisition_HandleCommand(mode);
    }
    else if (binary_command == 2U)
    {
      (void)Acquisition_SetGeneratorFrequency(generator_frequency);
    }
  }

  while (uart7_rx.tail != uart7_rx.head)
  {
    ProtocolScanMode mode;
    uint16_t generator_frequency = 0U;
    uint8_t byte = uart7_rx.buffer[uart7_rx.tail];
    uart7_rx.tail = (uint8_t)((uart7_rx.tail + 1U) & UART_RX_RING_MASK);

    uint8_t ascii_command =
        Acquisition_PushAsciiControl(&uart7_rx, byte, &mode);
    uint8_t binary_command =
        Protocol_PushControlByte(&uart7_rx.parser, byte, &mode,
                                 &generator_frequency);

    if (binary_command)
    {
      uart7_rx.command_length = 0U;
    }
    if (ascii_command || (binary_command == 1U))
    {
      Acquisition_HandleCommand(mode);
    }
    else if (binary_command == 2U)
    {
      (void)Acquisition_SetGeneratorFrequency(generator_frequency);
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

  /* D1=Arduino D11/PB15, D2=Arduino D12/PB14, D3=Arduino D13/PI1. */
  if ((DIG_CH1_GPIO_Port->IDR & DIG_CH1_Pin) != 0U) value |= 0x01U;
  if ((DIG_CH2_GPIO_Port->IDR & DIG_CH2_Pin) != 0U) value |= 0x02U;
  if ((DIG_CH3_GPIO_Port->IDR & DIG_CH3_Pin) != 0U) value |= 0x04U;

  {
    uint8_t changed = (uint8_t)(value ^ digital_previous_value);

    if ((changed & 0x01U) != 0U) digital_edge_count[0]++;
    if ((changed & 0x02U) != 0U) digital_edge_count[1]++;
    if ((changed & 0x04U) != 0U) digital_edge_count[2]++;
    digital_previous_value = value;
  }

  digital_capture[digital_write_half][digital_write_index++] = value;
  if (digital_write_index >= ADC_FRAME_PAIRS)
  {
    /*
     * Seal the completed half before switching writers.  The ADC DMA
     * callback only reads this sealed half, so a timer interrupt can never
     * overwrite bytes while the callback is copying them.
     */
    digital_completed_half = digital_write_half;
    digital_write_half ^= 1U;
    digital_write_index = 0U;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    Acquisition_StoreDmaHalf(
        &adc_dma_buffer[0], digital_capture[digital_completed_half]);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    Acquisition_StoreDmaHalf(&adc_dma_buffer[ADC_FRAME_HALF_WORDS],
                             digital_capture[digital_completed_half]);
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

void Acquisition_Uart1IsrByte(uint8_t byte)
{
  uint8_t next_head =
      (uint8_t)((uart1_rx.head + 1U) & UART_RX_RING_MASK);

  if (next_head != uart1_rx.tail)
  {
    uart1_rx.buffer[uart1_rx.head] = byte;
    uart1_rx.head = next_head;
    uart1_rx_bytes++;
  }
  else
  {
    acquisition_stats.uart_dropped_bytes++;
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
    __HAL_UART_CLEAR_OREFLAG(&huart1);
  }
  else if (huart->Instance == UART7)
  {
    acquisition_stats.uart_errors++;
    (void)HAL_UART_AbortReceive(&huart7);
    Acquisition_RearmUart7Dma();
  }
}

