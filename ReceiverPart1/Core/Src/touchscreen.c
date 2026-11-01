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

static bool touch_read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
  return HAL_I2C_Mem_Read(&hi2c3, TOUCH_I2C_ADDRESS,
                          reg, I2C_MEMADD_SIZE_8BIT, data, length,
                          TOUCH_I2C_TIMEOUT_MS) == HAL_OK;
}

void Touchscreen_Init(void)
{
  uint8_t attempt;
  HAL_StatusTypeDef status = HAL_ERROR;

  touch_ready = false;
  touch_chip_id = 0U;

  /* FT5336 requires at least 200 ms after power-up before first access. */
  HAL_Delay(200U);

  /*
   * The official STM32Cube F7 BSP retries device-ID detection because the
   * shared audio/touch I2C bus can still be settling immediately after reset.
   */
  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    status = HAL_I2C_IsDeviceReady(
        &hi2c3, TOUCH_I2C_ADDRESS, 2U, 20U);
    if (status == HAL_OK)
    {
      break;
    }
    HAL_Delay(20U);
  }

  if ((status == HAL_OK) &&
      touch_read_registers(TOUCH_CHIP_ID_REG, &touch_chip_id, 1U))
  {
    touch_ready = (touch_chip_id == TOUCH_EXPECTED_CHIP_ID);
  }

  if (touch_ready)
  {
    uint8_t polling_mode = 0U;

    (void)HAL_I2C_Mem_Write(&hi2c3, TOUCH_I2C_ADDRESS,
                            TOUCH_G_MODE_REG, I2C_MEMADD_SIZE_8BIT,
                            &polling_mode, 1U, TOUCH_I2C_TIMEOUT_MS);
  }
}

bool Touchscreen_IsReady(void)
{
  return touch_ready;
}

uint8_t Touchscreen_GetChipId(void)
{
  return touch_chip_id;
}

bool Touchscreen_ReadPoint(uint16_t *x, uint16_t *y)
{
  uint8_t data[5];
  uint8_t touch_points;
  uint16_t raw_x;
  uint16_t raw_y;

  if (!touch_ready || (x == NULL) || (y == NULL))
  {
    return false;
  }

  if (!touch_read_registers(TOUCH_STATUS_REG, data, sizeof(data)))
  {
    return false;
  }

  touch_points = data[0] & 0x0FU;
  if ((touch_points == 0U) || (touch_points > 5U))
  {
    return false;
  }

  raw_x = (uint16_t)(((uint16_t)(data[1] & 0x0FU) << 8) | data[2]);
  raw_y = (uint16_t)(((uint16_t)(data[3] & 0x0FU) << 8) | data[4]);

  /*
   * STM32F746G-DISCO convention: the panel controller reports X/Y swapped
   * relative to the landscape LTDC coordinate system.
   */
  *x = raw_y;
  *y = raw_x;

  if ((*x >= 480U) || (*y >= 272U))
  {
    return false;
  }
  return true;
}
