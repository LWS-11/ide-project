#include "touchscreen.h"

#include <stddef.h>

#include "i2c.h"

#define TOUCH_I2C_ADDRESS 0x70U
#define TOUCH_CHIP_ID_REG 0xA8U
#define TOUCH_G_MODE_REG 0xA4U
#define TOUCH_STATUS_REG 0x02U
#define TOUCH_EXPECTED_CHIP_ID 0x51U
#define TOUCH_I2C_TIMEOUT_MS 20U

static bool touch_ready;
static uint8_t touch_chip_id;

/** 通过 I2C3 读取触摸控制器寄存器。 */
static bool touch_read_registers(uint8_t reg, uint8_t *data, uint16_t length) // 寄存器/输出/长度
{
  return HAL_I2C_Mem_Read(&hi2c3, TOUCH_I2C_ADDRESS,
                          reg, I2C_MEMADD_SIZE_8BIT, data, length,
                          TOUCH_I2C_TIMEOUT_MS) == HAL_OK;
}

/** 检测 FT5336 并设置为轮询模式。 */
void Touchscreen_Init(void)
{
  uint8_t attempt;
  HAL_StatusTypeDef status = HAL_ERROR;

  touch_ready = false;   // 先标记触摸不可用
  touch_chip_id = 0U;    // 清空芯片 ID

  HAL_Delay(200U); // FT5336 上电后至少等待 200ms

  /*
   * The official STM32Cube F7 BSP retries device-ID detection because the
   * shared audio/touch I2C bus can still be settling immediately after reset.
   */
  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    status = HAL_I2C_IsDeviceReady( // 检测触摸控制器地址
        &hi2c3, TOUCH_I2C_ADDRESS, 2U, 20U);
    if (status == HAL_OK)
    {
      break;
    }
    HAL_Delay(20U); // 失败后短暂等待再试
  }

  if ((status == HAL_OK) &&
      touch_read_registers(TOUCH_CHIP_ID_REG, &touch_chip_id, 1U)) // 读取芯片 ID
  {
    touch_ready = (touch_chip_id == TOUCH_EXPECTED_CHIP_ID); // 检查 ID 是否为 FT5336
  }

  if (touch_ready)
  {
    uint8_t polling_mode = 0U; // 0 = 轮询模式

    (void)HAL_I2C_Mem_Write(&hi2c3, TOUCH_I2C_ADDRESS, // 关闭触摸中断输出
                            TOUCH_G_MODE_REG, I2C_MEMADD_SIZE_8BIT,
                            &polling_mode, 1U, TOUCH_I2C_TIMEOUT_MS);
  }
}

/** 返回触摸控制器是否初始化成功。 */
bool Touchscreen_IsReady(void)
{
  return touch_ready;
}

/** 返回触摸控制器芯片 ID。 */
uint8_t Touchscreen_GetChipId(void)
{
  return touch_chip_id;
}

/** 读取触摸坐标，并换算为横屏 LCD 坐标。 */
bool Touchscreen_ReadPoint(uint16_t *x, uint16_t *y) // 输出横纵坐标
{
  uint8_t data[5];
  uint8_t touch_points;
  uint16_t raw_x;
  uint16_t raw_y;

  if (!touch_ready || (x == NULL) || (y == NULL))
  {
    return false;
  }

  if (!touch_read_registers(TOUCH_STATUS_REG, data, sizeof(data))) // 读触摸状态
  {
    return false;
  }

  touch_points = data[0] & 0x0FU; // 当前触摸点数量
  if ((touch_points == 0U) || (touch_points > 5U))
  {
    return false;
  }

  raw_x = (uint16_t)(((uint16_t)(data[1] & 0x0FU) << 8) | data[2]); // 12 位 X
  raw_y = (uint16_t)(((uint16_t)(data[3] & 0x0FU) << 8) | data[4]); // 12 位 Y

  /*
   * STM32F746G-DISCO convention: the panel controller reports X/Y swapped
   * relative to the landscape LTDC coordinate system.
   */
  *x = raw_y; // 横屏坐标交换
  *y = raw_x; // 横屏坐标交换

  if ((*x >= 480U) || (*y >= 272U))
  {
    return false;
  }
  return true;
}
