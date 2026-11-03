/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    channel_protocol.c
  * @brief   Acquisition-to-receiver protocol shared with ReceiverPart1.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "channel_protocol.h"

static void Protocol_ResetControlParser(ProtocolControlParser *parser)
{
  parser->state = 0U;
}

uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < length; ++i)
  {
    crc ^= (uint16_t)data[i] << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                            : (uint16_t)(crc << 1U);
    }
  }

  return crc;
}

uint16_t Protocol_ScanIntervalUs(ProtocolScanMode mode)
{
  switch (mode)
  {
    case PROTOCOL_SCAN_5MS:
      return 100U;

    case PROTOCOL_SCAN_1MS:
      return 20U;

    case PROTOCOL_SCAN_10MS:
    default:
      return 200U;
  }
}

uint8_t Protocol_IntervalToMode(uint16_t interval_us, ProtocolScanMode *mode)
{
  if (mode == NULL)
  {
    return 0U;
  }

  switch (interval_us)
  {
    case 200U:
      *mode = PROTOCOL_SCAN_10MS;
      return 1U;

    case 100U:
      *mode = PROTOCOL_SCAN_5MS;
      return 1U;

    case 20U:
      *mode = PROTOCOL_SCAN_1MS;
      return 1U;

    default:
      return 0U;
  }
}

uint16_t Protocol_BuildFullFrame(const uint16_t *ch1_mv,
                                 const uint16_t *ch2_mv,
                                 const uint8_t *digital_samples,
                                 uint16_t sample_count,
                                 uint16_t interval_us,
                                 uint16_t sequence,
                                 uint8_t *frame)
{
  uint16_t payload_index;

  if ((ch1_mv == NULL) || (ch2_mv == NULL) ||
      (digital_samples == NULL) || (frame == NULL) ||
      (sample_count == 0U) || (sample_count > 500U))
  {
    return 0U;
  }

  frame[0] = PROTOCOL_DATA_SYNC_0;
  frame[1] = PROTOCOL_DATA_SYNC_1;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = PROTOCOL_FRAME_TYPE_DUAL;
  frame[4] = (uint8_t)(sequence & 0x00FFU);
  frame[5] = (uint8_t)(sequence >> 8U);
  frame[6] = (uint8_t)(sample_count & 0x00FFU);
  frame[7] = (uint8_t)(sample_count >> 8U);
  frame[8] = (uint8_t)(interval_us & 0x00FFU);
  frame[9] = (uint8_t)(interval_us >> 8U);
  frame[10] = PROTOCOL_CHANNEL_FLAG_ALL;
  frame[11] = 0U;

  payload_index = PROTOCOL_DATA_HEADER_SIZE;
  for (uint16_t i = 0U; i < sample_count; ++i)
  {
    frame[payload_index++] = (uint8_t)(ch1_mv[i] & 0x00FFU);
    frame[payload_index++] = (uint8_t)(ch1_mv[i] >> 8U);
    frame[payload_index++] = (uint8_t)(ch2_mv[i] & 0x00FFU);
    frame[payload_index++] = (uint8_t)(ch2_mv[i] >> 8U);
    frame[payload_index++] = (uint8_t)(digital_samples[i] & 0x07U);
  }

  uint16_t crc = Protocol_Crc16(frame, payload_index);
  frame[payload_index++] = (uint8_t)(crc & 0x00FFU);
  frame[payload_index++] = (uint8_t)(crc >> 8U);

  return payload_index;
}

void Protocol_InitControlParser(ProtocolControlParser *parser)
{
  if (parser != NULL)
  {
    Protocol_ResetControlParser(parser);
  }
}

uint8_t Protocol_PushControlByte(ProtocolControlParser *parser,
                                 uint8_t byte,
                                 ProtocolScanMode *mode,
                                 uint16_t *generator_frequency_hz)
{
  if ((parser == NULL) || (mode == NULL) ||
      (generator_frequency_hz == NULL))
  {
    return 0U;
  }

  switch (parser->state)
  {
    case 0U:
      if (byte == PROTOCOL_CONTROL_SYNC_0)
      {
        parser->state = 1U;
      }
      break;

    case 1U:
      if (byte == PROTOCOL_CONTROL_SYNC_1)
      {
        parser->state = 2U;
      }
      else
      {
        parser->state = (byte == PROTOCOL_CONTROL_SYNC_0) ? 1U : 0U;
      }
      break;

    case 2U:
      parser->state = (byte == PROTOCOL_VERSION) ? 3U : 0U;
      break;

    case 3U:
      if ((byte == PROTOCOL_CONTROL_COMMAND_SET_TIMEBASE) ||
          (byte == PROTOCOL_CONTROL_COMMAND_SET_FREQUENCY))
      {
        parser->command = byte;
        parser->state = 4U;
      }
      else
      {
        parser->state = 0U;
      }
      break;

    case 4U:
      parser->gear = byte;
      parser->state = 5U;
      break;

    case 5U:
      parser->interval_low = byte;
      parser->state = 6U;
      break;

    case 6U:
      parser->interval_high = byte;
      parser->state = 7U;
      break;

    case 7U:
      parser->crc_low = byte;
      parser->state = 8U;
      break;

    case 8U:
    {
      uint8_t crc_input[PROTOCOL_CONTROL_FRAME_SIZE - 2U] = {
        PROTOCOL_CONTROL_SYNC_0,
        PROTOCOL_CONTROL_SYNC_1,
        PROTOCOL_VERSION,
        parser->command,
        parser->gear,
        parser->interval_low,
        parser->interval_high
      };
      uint16_t received_crc =
          (uint16_t)parser->crc_low | ((uint16_t)byte << 8U);
      uint16_t interval =
          (uint16_t)parser->interval_low |
          ((uint16_t)parser->interval_high << 8U);
      uint16_t frequency =
          (uint16_t)parser->gear |
          ((uint16_t)parser->interval_low << 8U);
      ProtocolScanMode requested_mode;
      uint8_t command = parser->command;

      Protocol_ResetControlParser(parser);
      if (Protocol_Crc16(crc_input, sizeof(crc_input)) != received_crc)
      {
        break;
      }
      if (command == PROTOCOL_CONTROL_COMMAND_SET_TIMEBASE)
      {
        if ((parser->gear <= 2U) &&
            Protocol_IntervalToMode(interval, &requested_mode) &&
            ((uint8_t)requested_mode == parser->gear))
        {
          *mode = requested_mode;
          return 1U;
        }
      }
      else if (command == PROTOCOL_CONTROL_COMMAND_SET_FREQUENCY)
      {
        if ((frequency >= 100U) && (frequency <= 10000U) &&
            (parser->interval_high == 0U))
        {
          *generator_frequency_hz = frequency;
          return 2U;
        }
      }
      break;
    }

    default:
      Protocol_ResetControlParser(parser);
      break;
  }

  return 0U;
}
