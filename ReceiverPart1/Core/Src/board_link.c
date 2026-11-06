#include "board_link.h"

#include <stddef.h>

#include "main.h"
#include "usart.h"

#define BOARD_RX_RING_SIZE 16384U
#define BOARD_RX_RING_MASK (BOARD_RX_RING_SIZE - 1U)
#define PC_RX_RING_SIZE 8192U
#define PC_RX_RING_MASK (PC_RX_RING_SIZE - 1U)
#define BOARD_UART_DMA_RX_SIZE 8192U

static volatile uint8_t pc_rx_byte;
static uint8_t board_uart_dma_rx[BOARD_UART_DMA_RX_SIZE];
static volatile uint8_t board_rx_ring[BOARD_RX_RING_SIZE];
static volatile uint8_t pc_rx_ring[PC_RX_RING_SIZE];
static volatile uint32_t board_rx_head;
static volatile uint32_t board_rx_tail;
static volatile uint32_t pc_rx_head;
static volatile uint32_t pc_rx_tail;
static volatile uint32_t board_drop_count;
static volatile uint32_t pc_drop_count;

/** 把 DMA 收到的一段数据复制到板间环形缓冲区。 */
static void board_link_store_dma(const uint8_t *data, uint16_t length) // DMA 数据/长度
{
  for (uint16_t index = 0U; index < length; ++index)
  {
    uint32_t next = (board_rx_tail + 1U) & BOARD_RX_RING_MASK; // 环形下一位置
    if (next != board_rx_head)
    {
      board_rx_ring[board_rx_tail] = data[index]; // 写入新字节
      board_rx_tail = next;                       // 推进写指针
    }
    else
    {
      ++board_drop_count; // 缓冲区满，记录丢字节
    }
  }
}

/** 控制可选 RS-485 收发器的发送方向。 */
static void board_link_driver_enable(bool enable)
{
#if BOARD_LINK_RS485_DIRECTION_CONTROL
  HAL_GPIO_WritePin(
      BOARD_LINK_RS485_DE_GPIO_Port,
      BOARD_LINK_RS485_DE_Pin,
      enable ? GPIO_PIN_SET : GPIO_PIN_RESET); // 控制 RS-485 收发方向
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

/** 阻塞发送一段原始字节，并等待发送完成。 */
static void uart_write_raw(UART_HandleTypeDef *uart, const uint8_t *data,
                           uint16_t length) // UART/数据/长度
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
    while ((uart->Instance->ISR & USART_ISR_TXE) == 0U) // 等发送寄存器空
    {
    }
    uart->Instance->TDR = data[index]; // 写入一个发送字节
  }
  while ((uart->Instance->ISR & USART_ISR_TC) == 0U) // 等最后一位发送完成
  {
  }

  if (uart->Instance == UART7)
  {
    board_link_driver_enable(false);
  }
}

/** 启动 UART7 DMA 并初始化可选的 RS-485 方向引脚。 */
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
  if (HAL_UART_Receive_DMA( // UART7 循环 DMA 接收
          &huart7, board_uart_dma_rx, BOARD_UART_DMA_RX_SIZE) != HAL_OK)
  {
    ++board_drop_count;
  }
  BoardLink_Service();
}

/** UART7 DMA 收满前半缓冲区时转存数据。 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == UART7))
  {
    board_link_store_dma(board_uart_dma_rx, BOARD_UART_DMA_RX_SIZE / 2U); // 前半区
  }
}

/** UART7 转存后半区数据；USART1 接收单个电脑字节。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == UART7)
  {
    board_link_store_dma( // UART7 后半区
        &board_uart_dma_rx[BOARD_UART_DMA_RX_SIZE / 2U],
        BOARD_UART_DMA_RX_SIZE / 2U);
  }
  else if (huart->Instance == USART1)
  {
    uint32_t next = (pc_rx_tail + 1U) & PC_RX_RING_MASK; // PC 环形下一位置
    if (next != pc_rx_head)
    {
      pc_rx_ring[pc_rx_tail] = pc_rx_byte; // 保存电脑字节
      pc_rx_tail = next;                   // 推进写指针
    }
    else
    {
      ++pc_drop_count; // PC 缓冲区满
    }
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&pc_rx_byte, 1U); // 继续收下一字节
  }
}

/** 清除串口错误并恢复 UART7 DMA 接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) &&
      ((huart->Instance == UART7) || (huart->Instance == USART1)))
  {
    __HAL_UART_CLEAR_OREFLAG(huart); // 清除溢出标志
    if (huart->Instance == UART7)
    {
      (void)HAL_UART_AbortReceive(&huart7); // 中止异常 DMA
      (void)HAL_UART_Receive_DMA( // 重新启动 UART7 DMA
          &huart7, board_uart_dma_rx, BOARD_UART_DMA_RX_SIZE);
    }
  }
}

/** 检查串口错误并重新开启接收中断。 */
static void service_one_uart(UART_HandleTypeDef *uart,
                             volatile uint8_t *rx_byte) // UART/单字节接收地址
{
  if ((uart->Instance->ISR &
       (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U)
  {
    volatile uint32_t discard_isr = uart->Instance->ISR;  // 读 ISR 清错误标志
    volatile uint32_t discard_data = uart->Instance->RDR; // 读 RDR 清接收数据
    (void)discard_isr;
    (void)discard_data;

    uart->RxState = HAL_UART_STATE_READY;       // 恢复 HAL 接收状态
    uart->ErrorCode = HAL_UART_ERROR_NONE;      // 清除错误码
  }

  if (uart->RxState == HAL_UART_STATE_READY)
  {
    (void)HAL_UART_Receive_IT(uart, (uint8_t *)rx_byte, 1U); // 重新开启接收
  }
}

/** 维护电脑调试串口的接收状态。 */
void BoardLink_Service(void)
{
  service_one_uart(&huart1, &pc_rx_byte);
}

/** 从板间缓冲区读取一个字节。 */
bool BoardLink_ReadBoardByte(uint8_t *value)
{
  if ((value == NULL) || (board_rx_head == board_rx_tail))
  {
    return false;
  }

  *value = board_rx_ring[board_rx_head];                         // 取板间字节
  board_rx_head = (board_rx_head + 1U) & BOARD_RX_RING_MASK;    // 推进读指针
  return true;
}

/** 从电脑串口缓冲区读取一个字节。 */
bool BoardLink_ReadPcByte(uint8_t *value)
{
  if ((value == NULL) || (pc_rx_head == pc_rx_tail))
  {
    return false;
  }

  *value = pc_rx_ring[pc_rx_head];                           // 取电脑字节
  pc_rx_head = (pc_rx_head + 1U) & PC_RX_RING_MASK;          // 推进读指针
  return true;
}

/** 向另一块开发板发送数据。 */
void BoardLink_WriteBoard(const uint8_t *data, uint16_t length)
{
  uart_write_raw(&huart7, data, length);
}

/** 向电脑调试串口发送数据。 */
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
