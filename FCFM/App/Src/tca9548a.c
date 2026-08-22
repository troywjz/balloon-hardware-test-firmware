#include "tca9548a.h"

#include <stddef.h>

void Tca9548a_Construct(Tca9548a *mux,
                        I2C_HandleTypeDef *i2c,
                        uint16_t address_8bit)
{
  if (mux == NULL)
  {
    return;
  }

  mux->i2c = i2c;
  mux->address_8bit = address_8bit;
  mux->timeout_ms = 10U;
}

HAL_StatusTypeDef Tca9548a_SelectOne(Tca9548a *mux,
                                     uint8_t channel,
                                     uint8_t *control)
{
  uint8_t selection;
  uint8_t readback = 0U;
  HAL_StatusTypeDef result;

  if ((mux == NULL) || (mux->i2c == NULL) || (channel > 7U))
  {
    return HAL_ERROR;
  }

  selection = (uint8_t)(1U << channel);
  result = HAL_I2C_Master_Transmit(mux->i2c,
                                   mux->address_8bit,
                                   &selection,
                                   1U,
                                   mux->timeout_ms);
  if (result == HAL_OK)
  {
    result = HAL_I2C_Master_Receive(mux->i2c,
                                    mux->address_8bit,
                                    &readback,
                                    1U,
                                    mux->timeout_ms);
  }
  if ((result == HAL_OK) && (readback != selection))
  {
    result = HAL_ERROR;
  }
  if (control != NULL)
  {
    *control = readback;
  }
  return result;
}

HAL_StatusTypeDef Tca9548a_DisableAll(Tca9548a *mux)
{
  uint8_t disable = 0U;

  if ((mux == NULL) || (mux->i2c == NULL))
  {
    return HAL_ERROR;
  }
  return HAL_I2C_Master_Transmit(mux->i2c,
                                 mux->address_8bit,
                                 &disable,
                                 1U,
                                 mux->timeout_ms);
}
