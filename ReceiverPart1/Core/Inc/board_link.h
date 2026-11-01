#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * UART7 is the board-to-board link:
 *   PF7 = UART7_TX, PF6 = UART7_RX, 1 Mbit/s, 8N1.
 *
 * USART1 remains the ST-LINK Virtual COM Port used by the PC.
 */
#define BOARD_LINK_UART_BAUD 1000000U

/*
 * Set to 1 for a manual-direction MAX3485-style module whose DE/RE pin is
 * connected to PE4. Keep 0 for direct TTL jumper testing or an automatic
 * direction RS-485 module.
 */
#ifndef BOARD_LINK_RS485_DIRECTION_CONTROL
#define BOARD_LINK_RS485_DIRECTION_CONTROL 0
#endif

#define BOARD_LINK_RS485_DE_Pin GPIO_PIN_4
#define BOARD_LINK_RS485_DE_GPIO_Port GPIOE

typedef enum
{
  BOARD_LINK_SOURCE_PC = 0,
  BOARD_LINK_SOURCE_BOARD
} BoardLinkSource;

void BoardLink_Init(void);
void BoardLink_Service(void);

bool BoardLink_ReadBoardByte(uint8_t *value);
bool BoardLink_ReadPcByte(uint8_t *value);

void BoardLink_WriteBoard(const uint8_t *data, uint16_t length);
void BoardLink_WritePc(const uint8_t *data, uint16_t length);

uint32_t BoardLink_GetBoardDropCount(void);
uint32_t BoardLink_GetPcDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LINK_H */
