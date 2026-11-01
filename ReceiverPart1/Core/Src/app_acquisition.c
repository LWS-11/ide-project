#include "app_acquisition.h"

#include <stdbool.h>
#include <string.h>

#include "board_link.h"
#include "main.h"

#define ACQ_SAMPLE_COUNT 500U
#define ACQ_SAMPLE_BYTES 4U
#define ACQ_FRAME_SIZE \
  (APP_BOARD_LINK_HEADER_SIZE + (ACQ_SAMPLE_BYTES * ACQ_SAMPLE_COUNT) + 2U)
#define ACQ_FRAME_PERIOD_MS 50U

/* Header layout shared with the receiver. */
#define APP_BOARD_LINK_HEADER_SIZE 12U
#define ACQ_FRAME_SYNC_0 0xA5U
#define ACQ_FRAME_SYNC_1 0x5AU
#define ACQ_FRAME_VERSION 0x01U
#define ACQ_FRAME_TYPE_BOTH 0x30U
#define ACQ_FRAME_FLAGS_BOTH 0x03U

#define ACQ_CONTROL_SYNC_0 0xC5U
#define ACQ_CONTROL_SYNC_1 0x5CU
#define ACQ_CONTROL_VERSION 0x01U
#define ACQ_CONTROL_TIMEBASE 0x01U

static uint8_t frame_buffer[ACQ_FRAME_SIZE];
static uint8_t control_buffer[9];
static uint8_t control_length;
static uint16_t frame_sequence;
static uint16_t sample_interval_us = 200U;
static uint32_t next_frame_ms;
static uint32_t control_packet_count;
static uint32_t transmitted_frame_count;

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;
  uint32_t index;
  uint8_t bit;

  for (index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8;
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void format_uint(char *output, uint32_t value)
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

static void append_text(char *output, uint8_t *position, uint8_t capacity,
                        const char *text)
{
  while ((*text != '\0') && (*position < (uint8_t)(capacity - 1U)))
  {
    output[(*position)++] = *text++;
  }
}

static void append_number(char *output, uint8_t *position, uint8_t capacity,
                          uint32_t value)
{
  char number[11];
  uint8_t index = 0U;

  format_uint(number, value);
  while ((number[index] != '\0') &&
         (*position < (uint8_t)(capacity - 1U)))
  {
    output[(*position)++] = number[index++];
  }
}

static void send_debug_message(const char *prefix, uint32_t sequence,
                               uint32_t interval)
{
  char line[64];
  uint8_t position = 0U;

  append_text(line, &position, sizeof(line), prefix);
  append_number(line, &position, sizeof(line), sequence);
  append_text(line, &position, sizeof(line), ",INTERVAL=");
  append_number(line, &position, sizeof(line), interval);
  append_text(line, &position, sizeof(line), ",FRAMES=");
  append_number(line, &position, sizeof(line), transmitted_frame_count);
  append_text(line, &position, sizeof(line), ",CTRL=");
  append_number(line, &position, sizeof(line), control_packet_count);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
}

static uint16_t triangle_sample(uint8_t phase, uint16_t low_mv,
                                uint16_t high_mv)
{
  uint16_t triangle = (phase < 128U) ? phase : (255U - phase);
  return low_mv + (uint16_t)((triangle * (high_mv - low_mv)) / 127U);
}

static void build_frame(void)
{
  uint16_t index;
  uint16_t crc;

  frame_buffer[0] = ACQ_FRAME_SYNC_0;
  frame_buffer[1] = ACQ_FRAME_SYNC_1;
  frame_buffer[2] = ACQ_FRAME_VERSION;
  frame_buffer[3] = ACQ_FRAME_TYPE_BOTH;
  frame_buffer[4] = (uint8_t)(frame_sequence & 0xFFU);
  frame_buffer[5] = (uint8_t)(frame_sequence >> 8);
  frame_buffer[6] = (uint8_t)(ACQ_SAMPLE_COUNT & 0xFFU);
  frame_buffer[7] = (uint8_t)(ACQ_SAMPLE_COUNT >> 8);
  frame_buffer[8] = (uint8_t)(sample_interval_us & 0xFFU);
  frame_buffer[9] = (uint8_t)(sample_interval_us >> 8);
  frame_buffer[10] = ACQ_FRAME_FLAGS_BOTH;
  frame_buffer[11] = 0U;

  for (index = 0U; index < ACQ_SAMPLE_COUNT; ++index)
  {
    uint8_t phase1 = (uint8_t)(
        ((index * 4U * 256U) / ACQ_SAMPLE_COUNT) + (frame_sequence * 3U));
    uint8_t phase2 = (uint8_t)(
        ((index * 9U * 256U) / ACQ_SAMPLE_COUNT) + (frame_sequence * 5U));
    uint16_t ch1 = triangle_sample(phase1, 200U, 2900U);
    uint16_t ch2 = triangle_sample(phase2, 300U, 2700U);
    uint16_t offset = (uint16_t)(APP_BOARD_LINK_HEADER_SIZE +
                                 (index * ACQ_SAMPLE_BYTES));

    frame_buffer[offset] = (uint8_t)(ch1 & 0xFFU);
    frame_buffer[offset + 1U] = (uint8_t)(ch1 >> 8);
    frame_buffer[offset + 2U] = (uint8_t)(ch2 & 0xFFU);
    frame_buffer[offset + 3U] = (uint8_t)(ch2 >> 8);
  }

  crc = crc16_ccitt(frame_buffer,
                    APP_BOARD_LINK_HEADER_SIZE +
                    (ACQ_SAMPLE_BYTES * ACQ_SAMPLE_COUNT));
  frame_buffer[ACQ_FRAME_SIZE - 2U] = (uint8_t)(crc & 0xFFU);
  frame_buffer[ACQ_FRAME_SIZE - 1U] = (uint8_t)(crc >> 8);
}

static void handle_control_packet(void)
{
  uint16_t received_crc;
  uint16_t calculated_crc;
  uint8_t timebase_index;
  uint16_t interval;

  if ((control_buffer[0] != ACQ_CONTROL_SYNC_0) ||
      (control_buffer[1] != ACQ_CONTROL_SYNC_1) ||
      (control_buffer[2] != ACQ_CONTROL_VERSION) ||
      (control_buffer[3] != ACQ_CONTROL_TIMEBASE))
  {
    return;
  }

  received_crc = (uint16_t)control_buffer[7] |
                 ((uint16_t)control_buffer[8] << 8);
  calculated_crc = crc16_ccitt(control_buffer, 7U);
  if (received_crc != calculated_crc)
  {
    return;
  }

  timebase_index = control_buffer[4];
  interval = (uint16_t)control_buffer[5] |
             ((uint16_t)control_buffer[6] << 8);
  if ((timebase_index <= 2U) &&
      ((interval == 200U) || (interval == 100U) || (interval == 20U)))
  {
    sample_interval_us = interval;
    ++control_packet_count;
    send_debug_message("RX CTRL,TB=", timebase_index, sample_interval_us);
  }
}

static void poll_link_input(void)
{
  uint8_t value;

  while (BoardLink_ReadBoardByte(&value))
  {
    if ((control_length == 0U) && (value != ACQ_CONTROL_SYNC_0))
    {
      continue;
    }

    if ((control_length == 1U) && (value != ACQ_CONTROL_SYNC_1))
    {
      control_length = (value == ACQ_CONTROL_SYNC_0) ? 1U : 0U;
      continue;
    }

    control_buffer[control_length++] = value;
    if (control_length == sizeof(control_buffer))
    {
      handle_control_packet();
      control_length = 0U;
    }
  }
}

void App_AcquisitionInit(void)
{
  BoardLink_Init();
  frame_sequence = 0U;
  sample_interval_us = 200U;
  control_length = 0U;
  control_packet_count = 0U;
  transmitted_frame_count = 0U;
  next_frame_ms = HAL_GetTick() + 200U;

  BoardLink_WritePc(
      (const uint8_t *)"A1 READY,ROLE=ACQ,UART7=1000000\r\n",
      (uint16_t)(sizeof("A1 READY,ROLE=ACQ,UART7=1000000\r\n") - 1U));
  BoardLink_WriteBoard(
      (const uint8_t *)"LINK READY,ROLE=TX\r\n", 20U);
}

void App_AcquisitionLoop(void)
{
  uint32_t now;

  poll_link_input();
  BoardLink_Service();

  now = HAL_GetTick();
  if ((int32_t)(now - next_frame_ms) >= 0)
  {
    build_frame();
    BoardLink_WriteBoard(frame_buffer, sizeof(frame_buffer));
    send_debug_message("TX,SEQ=", frame_sequence, sample_interval_us);
    ++frame_sequence;
    ++transmitted_frame_count;

    next_frame_ms += ACQ_FRAME_PERIOD_MS;
    if ((int32_t)(HAL_GetTick() - next_frame_ms) >= 0)
    {
      next_frame_ms = HAL_GetTick() + ACQ_FRAME_PERIOD_MS;
    }
  }
}
