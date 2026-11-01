#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c3;

void MX_I2C3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */
