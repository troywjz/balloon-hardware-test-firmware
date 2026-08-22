#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "tca9548a.h"

#include <stdbool.h>
#include <stdint.h>

#define MMC5983MA_PRODUCT_ID_EXPECTED 0x30U

typedef struct
{
  int32_t raw[3];
  int16_t milli_gauss[3];
  uint8_t product_id;
  bool valid;
} Mmc5983maSample;

typedef struct
{
  I2C_HandleTypeDef *i2c;
  Tca9548a *mux;
  uint8_t mux_channel;
  uint16_t measurements_since_set;
  uint32_t timeout_ms;
  bool initialized;
} Mmc5983ma;

void Mmc5983ma_Construct(Mmc5983ma *device,
                         I2C_HandleTypeDef *i2c,
                         Tca9548a *mux,
                         uint8_t mux_channel);
HAL_StatusTypeDef Mmc5983ma_ReadProductId(Mmc5983ma *device,
                                          uint8_t *product_id);
HAL_StatusTypeDef Mmc5983ma_Initialize(Mmc5983ma *device);
HAL_StatusTypeDef Mmc5983ma_ReadSample(Mmc5983ma *device,
                                       Mmc5983maSample *sample);

#endif
