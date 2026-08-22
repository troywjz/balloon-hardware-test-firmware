#include "mmc5983ma.h"

#include <string.h>

#define MMC5983MA_ADDRESS_8BIT          (0x30U << 1U)
#define MMC5983MA_DATA_REGISTER         0x00U
#define MMC5983MA_STATUS_REGISTER       0x08U
#define MMC5983MA_CONTROL0_REGISTER     0x09U
#define MMC5983MA_PRODUCT_ID_REGISTER   0x2FU
#define MMC5983MA_STATUS_MEAS_DONE      0x01U
#define MMC5983MA_CONTROL_MEASURE       0x01U
#define MMC5983MA_CONTROL_SET           0x08U
#define MMC5983MA_DATA_LENGTH           7U
#define MMC5983MA_MEASUREMENT_TIMEOUT_MS 20U
#define MMC5983MA_SET_INTERVAL_MEASUREMENTS 1000U
#define MMC5983MA_ZERO_FIELD_COUNTS     131072L
#define MMC5983MA_COUNTS_PER_GAUSS      16384L

static HAL_StatusTypeDef Mmc5983ma_Select(Mmc5983ma *device)
{
  if ((device == NULL) || (device->i2c == NULL) || (device->mux == NULL))
  {
    return HAL_ERROR;
  }
  return Tca9548a_SelectOne(device->mux, device->mux_channel, NULL);
}

static int16_t Mmc5983ma_CountsToMilliGauss(int32_t counts)
{
  int32_t scaled = counts * 1000L;
  int32_t value = scaled >= 0
                      ? (scaled + (MMC5983MA_COUNTS_PER_GAUSS / 2L)) /
                            MMC5983MA_COUNTS_PER_GAUSS
                      : (scaled - (MMC5983MA_COUNTS_PER_GAUSS / 2L)) /
                            MMC5983MA_COUNTS_PER_GAUSS;

  if (value > INT16_MAX)
  {
    value = INT16_MAX;
  }
  else if (value < INT16_MIN)
  {
    value = INT16_MIN;
  }
  return (int16_t)value;
}

static HAL_StatusTypeDef Mmc5983ma_ReadProductIdSelected(Mmc5983ma *device,
                                                         uint8_t *product_id)
{
  return HAL_I2C_Mem_Read(device->i2c,
                          MMC5983MA_ADDRESS_8BIT,
                          MMC5983MA_PRODUCT_ID_REGISTER,
                          I2C_MEMADD_SIZE_8BIT,
                          product_id,
                          1U,
                          device->timeout_ms);
}

void Mmc5983ma_Construct(Mmc5983ma *device,
                         I2C_HandleTypeDef *i2c,
                         Tca9548a *mux,
                         uint8_t mux_channel)
{
  if (device == NULL)
  {
    return;
  }

  memset(device, 0, sizeof(*device));
  device->i2c = i2c;
  device->mux = mux;
  device->mux_channel = mux_channel;
  device->timeout_ms = 10U;
}

HAL_StatusTypeDef Mmc5983ma_ReadProductId(Mmc5983ma *device,
                                          uint8_t *product_id)
{
  HAL_StatusTypeDef result;

  if (product_id == NULL)
  {
    return HAL_ERROR;
  }
  result = Mmc5983ma_Select(device);
  if (result == HAL_OK)
  {
    result = Mmc5983ma_ReadProductIdSelected(device, product_id);
  }
  (void)Tca9548a_DisableAll(device != NULL ? device->mux : NULL);
  return result;
}

HAL_StatusTypeDef Mmc5983ma_Initialize(Mmc5983ma *device)
{
  uint8_t product_id = 0U;
  uint8_t command = MMC5983MA_CONTROL_SET;
  HAL_StatusTypeDef result;

  if (device == NULL)
  {
    return HAL_ERROR;
  }

  device->initialized = false;
  result = Mmc5983ma_Select(device);
  if (result == HAL_OK)
  {
    result = Mmc5983ma_ReadProductIdSelected(device, &product_id);
  }
  if ((result == HAL_OK) && (product_id != MMC5983MA_PRODUCT_ID_EXPECTED))
  {
    result = HAL_ERROR;
  }
  if (result == HAL_OK)
  {
    result = HAL_I2C_Mem_Write(device->i2c,
                               MMC5983MA_ADDRESS_8BIT,
                               MMC5983MA_CONTROL0_REGISTER,
                               I2C_MEMADD_SIZE_8BIT,
                               &command,
                               1U,
                               device->timeout_ms);
  }
  (void)Tca9548a_DisableAll(device->mux);
  if (result == HAL_OK)
  {
    HAL_Delay(1U);
    device->measurements_since_set = 0U;
    device->initialized = true;
  }
  return result;
}

HAL_StatusTypeDef Mmc5983ma_ReadSample(Mmc5983ma *device,
                                       Mmc5983maSample *sample)
{
  uint8_t command = MMC5983MA_CONTROL_MEASURE;
  uint8_t status = 0U;
  uint8_t data[MMC5983MA_DATA_LENGTH] = {0U};
  uint32_t start_tick;
  HAL_StatusTypeDef result;

  if ((device == NULL) || (sample == NULL) || !device->initialized)
  {
    return HAL_ERROR;
  }

  memset(sample, 0, sizeof(*sample));
  sample->product_id = MMC5983MA_PRODUCT_ID_EXPECTED;
  result = Mmc5983ma_Select(device);
  if ((result == HAL_OK) &&
      (device->measurements_since_set >=
       MMC5983MA_SET_INTERVAL_MEASUREMENTS))
  {
    command = MMC5983MA_CONTROL_SET;
    result = HAL_I2C_Mem_Write(device->i2c,
                               MMC5983MA_ADDRESS_8BIT,
                               MMC5983MA_CONTROL0_REGISTER,
                               I2C_MEMADD_SIZE_8BIT,
                               &command,
                               1U,
                               device->timeout_ms);
    if (result == HAL_OK)
    {
      HAL_Delay(1U);
      device->measurements_since_set = 0U;
      command = MMC5983MA_CONTROL_MEASURE;
    }
  }
  if (result == HAL_OK)
  {
    result = HAL_I2C_Mem_Write(device->i2c,
                               MMC5983MA_ADDRESS_8BIT,
                               MMC5983MA_CONTROL0_REGISTER,
                               I2C_MEMADD_SIZE_8BIT,
                               &command,
                               1U,
                               device->timeout_ms);
  }

  start_tick = HAL_GetTick();
  while (result == HAL_OK)
  {
    result = HAL_I2C_Mem_Read(device->i2c,
                              MMC5983MA_ADDRESS_8BIT,
                              MMC5983MA_STATUS_REGISTER,
                              I2C_MEMADD_SIZE_8BIT,
                              &status,
                              1U,
                              device->timeout_ms);
    if ((result != HAL_OK) || ((status & MMC5983MA_STATUS_MEAS_DONE) != 0U))
    {
      break;
    }
    if ((uint32_t)(HAL_GetTick() - start_tick) >=
        MMC5983MA_MEASUREMENT_TIMEOUT_MS)
    {
      result = HAL_TIMEOUT;
      break;
    }
    HAL_Delay(1U);
  }

  if (result == HAL_OK)
  {
    result = HAL_I2C_Mem_Read(device->i2c,
                              MMC5983MA_ADDRESS_8BIT,
                              MMC5983MA_DATA_REGISTER,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              sizeof(data),
                              device->timeout_ms);
  }
  (void)Tca9548a_DisableAll(device->mux);

  if (result == HAL_OK)
  {
    uint32_t raw_x = ((uint32_t)data[0] << 10U) |
                     ((uint32_t)data[1] << 2U) |
                     ((uint32_t)data[6] >> 6U);
    uint32_t raw_y = ((uint32_t)data[2] << 10U) |
                     ((uint32_t)data[3] << 2U) |
                     (((uint32_t)data[6] >> 4U) & 0x03U);
    uint32_t raw_z = ((uint32_t)data[4] << 10U) |
                     ((uint32_t)data[5] << 2U) |
                     (((uint32_t)data[6] >> 2U) & 0x03U);

    sample->raw[0] = (int32_t)raw_x - MMC5983MA_ZERO_FIELD_COUNTS;
    sample->raw[1] = (int32_t)raw_y - MMC5983MA_ZERO_FIELD_COUNTS;
    sample->raw[2] = (int32_t)raw_z - MMC5983MA_ZERO_FIELD_COUNTS;
    sample->milli_gauss[0] = Mmc5983ma_CountsToMilliGauss(sample->raw[0]);
    sample->milli_gauss[1] = Mmc5983ma_CountsToMilliGauss(sample->raw[1]);
    sample->milli_gauss[2] = Mmc5983ma_CountsToMilliGauss(sample->raw[2]);
    sample->valid = true;
    ++device->measurements_since_set;
  }
  return result;
}
