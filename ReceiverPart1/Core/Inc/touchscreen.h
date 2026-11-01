#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal polling driver for the FT5336 touch controller on the
 * STM32F746G-DISCO board.
 *
 * The display and touch controller use the same 480 x 272 coordinate system.
 */
void Touchscreen_Init(void);
bool Touchscreen_IsReady(void);
uint8_t Touchscreen_GetChipId(void);

/*
 * Returns true while at least one finger is present. x/y are valid only when
 * the function returns true.
 */
bool Touchscreen_ReadPoint(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif /* TOUCHSCREEN_H */
