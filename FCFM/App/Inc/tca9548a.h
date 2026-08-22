#ifndef TCA9548A_H
#define TCA9548A_H

#include "main.h"

#include <stdint.h>

typedef struct
{
  I2C_HandleTypeDef *i2c;
  uint16_t address_8bit;
  uint32_t timeout_ms;
} Tca9548a;

void Tca9548a_Construct(Tca9548a *mux,
                        I2C_HandleTypeDef *i2c,
                        uint16_t address_8bit);
HAL_StatusTypeDef Tca9548a_SelectOne(Tca9548a *mux,
                                     uint8_t channel,
                                     uint8_t *control);
HAL_StatusTypeDef Tca9548a_DisableAll(Tca9548a *mux);

#endif
