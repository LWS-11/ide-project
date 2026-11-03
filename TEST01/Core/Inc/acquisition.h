/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    acquisition.h
  * @brief   ADC3/TIM8/DMA acquisition and serial command handling.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ACQUISITION_H
#define __ACQUISITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "channel_protocol.h"

typedef struct
{
  uint32_t adc_errors;
  uint32_t uart_errors;
  uint32_t uart_dropped_bytes;
  uint32_t invalid_commands;
} AcquisitionStats;

uint8_t Acquisition_Init(void);
uint8_t Acquisition_SetGeneratorFrequency(uint16_t frequency_hz);
void Acquisition_Process(void);
uint8_t Acquisition_TransmitFrame(const uint8_t *frame, uint16_t length);
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
                                   uint16_t d3_high_count);

uint8_t Acquisition_GetLatestFrame(uint16_t *ch1,
                                   uint16_t *ch2,
                                   uint8_t *digital_samples,
                                   uint16_t sample_count,
                                   ProtocolScanMode *scan_mode,
                                   uint32_t *capture_tick);
void Acquisition_TimerTick(void);

ProtocolScanMode Acquisition_GetScanMode(void);
uint8_t Acquisition_SetMode(ProtocolScanMode mode);
void Acquisition_GetStats(AcquisitionStats *stats);

/* Called by the USART1 interrupt to queue a received PC byte without HAL
 * restarting a one-byte receive transaction for every character. */
void Acquisition_Uart1IsrByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* __ACQUISITION_H */
