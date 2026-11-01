#include "board_link.h"

#include <stddef.h>

#include "main.h"
#include "usart.h"

#define BOARD_RX_RING_SIZE 16384U
#define BOARD_RX_RING_MASK (BOARD_RX_RING_SIZE - 1U)
#define PC_RX_RING_SIZE 8192U
#define PC_RX_RING_MASK (PC_RX_RING_SIZE - 1U)

static volatile uint8_t board_rx_byte;
static volatile uint8_t pc_rx_byte;
static volatile uint8_t board_rx_ring[BOARD_RX_RING_SIZE];
static volatile uint8_t pc_rx_ring[PC_RX_RING_SIZE];
static volatile uint32_t board_rx_head;
static volatile uint32_t board_rx_tail;
static volatile uint32_t pc_rx_head;
static volatile uint32_t pc_rx_tail;
static volatile uint32_t board_drop_count;
static volatile uint32_t pc_drop_count;

static void board_link_driver_enable(bool enable)
{
#if BOARD_LINK_RS485_DIRECTION_CONTROL
  HAL_GPIO_WritePin(
      BOARD_LINK_RS485_DE_GPIO_Port,
      BOARD_LINK_RS485_DE_Pin,
      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (enable)
  {
    volatile uint32_t delay_loop;
    for (delay_loop = 0U; delay_loop < 40U; ++delay_loop)
    {
      __NOP();
    }
  }
#else
  (void)enable;
#endif
}

static void uart_write_raw(UART_HandleTypeDef *uart, const uint8_t *data,
                           uint16_t length)
{
  uint16_t index;

  if ((uart == NULL) || (data == NULL))
  {
    return;
  }

  if ((uart->Instance == UART7) && (length > 0U))
  {
    board_link_driver_enable(true);
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

  if (uart->Instance == UART7)
  {
    board_link_driver_enable(false);
  }
}

void BoardLink_Init(void)
{
#if BOARD_LINK_RS485_DIRECTION_CONTROL
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(
      BOARD_LINK_RS485_DE_GPIO_Port,
      BOARD_LINK_RS485_DE_Pin,
      GPIO_PIN_RESET);
  gpio.Pin = BOARD_LINK_RS485_DE_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BOARD_LINK_RS485_DE_GPIO_Port, &gpio);
#endif
  BoardLink_Service();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == UART7)
  {
    uint32_t next = (board_rx_tail + 1U) & BOARD_RX_RING_MASK;
    if (next != board_rx_head)
    {
      board_rx_ring[board_rx_tail] = board_rx_byte;
      board_rx_tail = next;
    }
    else
    {
      ++board_drop_count;
    }
    (void)HAL_UART_Receive_IT(&huart7, (uint8_t *)&board_rx_byte, 1U);
  }
  else if (huart->Instance == USART1)
  {
    uint32_t next = (pc_rx_tail + 1U) & PC_RX_RING_MASK;
    if (next != pc_rx_head)
    {
      pc_rx_ring[pc_rx_tail] = pc_rx_byte;
      pc_rx_tail = next;
    }
    else
    {
      ++pc_drop_count;
    }
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&pc_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) &&
      ((huart->Instance == UART7) || (huart->Instance == USART1)))
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
}

static void service_one_uart(UART_HandleTypeDef *uart,
                             volatile uint8_t *rx_byte)
{
  if ((uart->Instance->ISR &
       (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U)
  {
    volatile uint32_t discard_isr = uart->Instance->ISR;
    volatile uint32_t discard_data = uart->Instance->RDR;
    (void)discard_isr;
    (void)discard_data;

    uart->RxState = HAL_UART_STATE_READY;
    uart->ErrorCode = HAL_UART_ERROR_NONE;
  }

  if (uart->RxState == HAL_UART_STATE_READY)
  {
    (void)HAL_UART_Receive_IT(uart, (uint8_t *)rx_byte, 1U);
  }
}

void BoardLink_Service(void)
{
  service_one_uart(&huart7, &board_rx_byte);
  service_one_uart(&huart1, &pc_rx_byte);
}

bool BoardLink_ReadBoardByte(uint8_t *value)
{
  if ((value == NULL) || (board_rx_head == board_rx_tail))
  {
    return false;
  }

  *value = board_rx_ring[board_rx_head];
  board_rx_head = (board_rx_head + 1U) & BOARD_RX_RING_MASK;
  return true;
}

bool BoardLink_ReadPcByte(uint8_t *value)
{
  if ((value == NULL) || (pc_rx_head == pc_rx_tail))
  {
    return false;
  }

  *value = pc_rx_ring[pc_rx_head];
  pc_rx_head = (pc_rx_head + 1U) & PC_RX_RING_MASK;
  return true;
}

void BoardLink_WriteBoard(const uint8_t *data, uint16_t length)
{
  uart_write_raw(&huart7, data, length);
}

void BoardLink_WritePc(const uint8_t *data, uint16_t length)
{
  uart_write_raw(&huart1, data, length);
}

uint32_t BoardLink_GetBoardDropCount(void)
{
  return board_drop_count;
}

uint32_t BoardLink_GetPcDropCount(void)
{
  return pc_drop_count;
}
