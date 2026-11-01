/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    channel_protocol.h
  * @brief   Acquisition-to-receiver protocol shared with ReceiverPart1.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CHANNEL_PROTOCOL_H
#define __CHANNEL_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PROTOCOL_DATA_SYNC_0            0xA5U
#define PROTOCOL_DATA_SYNC_1            0x5AU
#define PROTOCOL_CONTROL_SYNC_0         0xC5U
#define PROTOCOL_CONTROL_SYNC_1         0x5CU
#define PROTOCOL_VERSION                0x01U

#define PROTOCOL_FRAME_TYPE_CH1         0x10U
#define PROTOCOL_FRAME_TYPE_CH2         0x20U
#define PROTOCOL_FRAME_TYPE_DUAL        0x30U
#define PROTOCOL_CHANNEL_FLAG_CH1       0x01U
#define PROTOCOL_CHANNEL_FLAG_CH2       0x02U
#define PROTOCOL_CHANNEL_FLAG_DUAL      0x03U

/*
 * Signal meaning is part of the application contract:
 *   CH1 = potentiometer, ADC3_IN7 / PF9 / Arduino A2;
 *   CH2 = microphone,    ADC3_IN0 / PA0 / Arduino A0.
 */
#define PROTOCOL_SAMPLES_PER_CHANNEL    500U
#define PROTOCOL_DATA_HEADER_SIZE       12U
#define PROTOCOL_DUAL_FRAME_SIZE        (PROTOCOL_DATA_HEADER_SIZE + \
                                         (PROTOCOL_SAMPLES_PER_CHANNEL * 4U) + 2U)

#define PROTOCOL_CONTROL_COMMAND_SET_TIMEBASE 0x01U
#define PROTOCOL_CONTROL_FRAME_SIZE     9U

typedef enum
{
  PROTOCOL_SCAN_10MS = 0,
  PROTOCOL_SCAN_5MS = 1,
  PROTOCOL_SCAN_1MS = 2
} ProtocolScanMode;

typedef struct
{
  uint8_t state;
  uint8_t gear;
  uint8_t interval_low;
  uint8_t interval_high;
  uint8_t crc_low;
} ProtocolControlParser;

uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length);

uint16_t Protocol_BuildDualChannelFrame(const uint16_t *ch1_mv,
                                        const uint16_t *ch2_mv,
                                        uint16_t sample_count,
                                        uint16_t interval_us,
                                        uint16_t sequence,
                                        uint8_t *frame);

uint16_t Protocol_ScanIntervalUs(ProtocolScanMode mode);
uint8_t Protocol_IntervalToMode(uint16_t interval_us, ProtocolScanMode *mode);

void Protocol_InitControlParser(ProtocolControlParser *parser);
uint8_t Protocol_PushControlByte(ProtocolControlParser *parser,
                                 uint8_t byte,
                                 ProtocolScanMode *mode);

#ifdef __cplusplus
}
#endif

#endif /* __CHANNEL_PROTOCOL_H */
