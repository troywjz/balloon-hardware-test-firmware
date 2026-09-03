#include "icm45686.h"

#include <string.h>

#define ICM45686_SPI_READ_BIT             0x80U
#define ICM45686_ACCEL_DATA_X1_REGISTER   0x00U
#define ICM45686_PWR_MGMT0_REGISTER       0x10U
#define ICM45686_ACCEL_CONFIG0_REGISTER   0x1BU
#define ICM45686_GYRO_CONFIG0_REGISTER    0x1CU
#define ICM45686_WHO_AM_I_REGISTER        0x72U
#define ICM45686_REG_MISC2_REGISTER       0x7FU
#define ICM45686_SOFT_RESET               0x02U
#define ICM45686_ACCEL_16G_100HZ          0x19U
#define ICM45686_GYRO_2000DPS_100HZ       0x19U
#define ICM45686_SIX_AXIS_LOW_NOISE       0x0FU
#define ICM45686_MAX_BURST_READ_LENGTH    32U

HAL_StatusTypeDef Icm45686_WriteRegister(Icm45686 *device,
                                         uint8_t register_address,
                                         uint8_t value)
{
  uint8_t data[2] = {
      (uint8_t)(register_address & (uint8_t)~ICM45686_SPI_READ_BIT), value};
  HAL_StatusTypeDef result;

  if ((device == NULL) || (device->spi == NULL) ||
      (device->chip_select_port == NULL))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(device->chip_select_port,
                    device->chip_select_pin,
                    GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(device->spi, data, sizeof(data), device->timeout_ms);
  HAL_GPIO_WritePin(device->chip_select_port,
                    device->chip_select_pin,
                    GPIO_PIN_SET);
  return result;
}

static int16_t Icm45686_DecodeInt16(const uint8_t *data)
{
  /* ICM-45686 defaults to little-endian sensor-register data. */
  return (int16_t)((uint16_t)data[0] |
                   ((uint16_t)data[1] << 8U));
}

void Icm45686_Construct(Icm45686 *device,
                        SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *chip_select_port,
                        uint16_t chip_select_pin)
{
  if (device == NULL)
  {
    return;
  }

  memset(device, 0, sizeof(*device));
  device->spi = spi;
  device->chip_select_port = chip_select_port;
  device->chip_select_pin = chip_select_pin;
  device->timeout_ms = 100U;
  HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef Icm45686_ReadRegistersContinuous(Icm45686 *device,
                                                   uint8_t register_address,
                                                   uint8_t *data,
                                                   uint16_t length)
{
  uint8_t tx_data[ICM45686_MAX_BURST_READ_LENGTH + 1U];
  uint8_t rx_data[ICM45686_MAX_BURST_READ_LENGTH + 1U];
  HAL_StatusTypeDef result;

  if ((device == NULL) || (device->spi == NULL) ||
      (device->chip_select_port == NULL) || (data == NULL) || (length == 0U) ||
      (length > ICM45686_MAX_BURST_READ_LENGTH))
  {
    return HAL_ERROR;
  }

  memset(tx_data, 0xFF, (size_t)length + 1U);
  memset(rx_data, 0, (size_t)length + 1U);
  tx_data[0] = register_address | ICM45686_SPI_READ_BIT;

  HAL_GPIO_WritePin(device->chip_select_port,
                    device->chip_select_pin,
                    GPIO_PIN_RESET);
  result = HAL_SPI_TransmitReceive(device->spi,
                                   tx_data,
                                   rx_data,
                                   (uint16_t)(length + 1U),
                                   device->timeout_ms);
  HAL_GPIO_WritePin(device->chip_select_port,
                    device->chip_select_pin,
                    GPIO_PIN_SET);
  if (result == HAL_OK)
  {
    memcpy(data, &rx_data[1], length);
  }
  return result;
}

HAL_StatusTypeDef Icm45686_ReadRegister(Icm45686 *device,
                                        uint8_t register_address,
                                        uint8_t *value)
{
  return Icm45686_ReadRegisters(device, register_address, value, 1U);
}

HAL_StatusTypeDef Icm45686_ReadRegistersSplit(Icm45686 *device,
                                             uint8_t register_address,
                                             uint8_t *data,
                                             uint16_t length)
{
  uint8_t command;
  HAL_StatusTypeDef result;

  if ((device == NULL) || (device->spi == NULL) ||
      (device->chip_select_port == NULL) || (data == NULL) ||
      (length == 0U) || (length > ICM45686_MAX_BURST_READ_LENGTH))
  {
    return HAL_ERROR;
  }

  command = register_address | ICM45686_SPI_READ_BIT;
  HAL_GPIO_WritePin(device->chip_select_port,
                    device->chip_select_pin,
                    GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(device->spi, &command, 1U, device->timeout_ms);
  if (result == HAL_OK)
  {
    /* Keep CS asserted while the master clocks the response byte. */
    result = HAL_SPI_Receive(device->spi, data, length, device->timeout_ms);
  }
  HAL_GPIO_WritePin(device->chip_select_port,
                    device->chip_select_pin,
                    GPIO_PIN_SET);
  return result;
}

HAL_StatusTypeDef Icm45686_ReadRegisters(Icm45686 *device,
                                         uint8_t register_address,
                                         uint8_t *data,
                                         uint16_t length)
{
  /* Use the RoEx-style split transaction for normal ICM-45686 reads. */
  return Icm45686_ReadRegistersSplit(device, register_address, data, length);
}

HAL_StatusTypeDef Icm45686_ReadRegisterSplit(Icm45686 *device,
                                             uint8_t register_address,
                                             uint8_t *value)
{
  return Icm45686_ReadRegistersSplit(device, register_address, value, 1U);
}

HAL_StatusTypeDef Icm45686_ReadWhoAmI(Icm45686 *device, uint8_t *who_am_i)
{
  return Icm45686_ReadRegister(device, ICM45686_WHO_AM_I_REGISTER, who_am_i);
}

HAL_StatusTypeDef Icm45686_SoftReset(Icm45686 *device)
{
  HAL_StatusTypeDef result = Icm45686_WriteRegister(
      device, ICM45686_REG_MISC2_REGISTER, ICM45686_SOFT_RESET);

  if (device != NULL)
  {
    device->configured = false;
  }
  if (result == HAL_OK)
  {
    HAL_Delay(10U);
  }
  return result;
}

HAL_StatusTypeDef Icm45686_Initialize(Icm45686 *device)
{
  uint8_t who_am_i = 0U;
  uint8_t accel_config = 0U;
  uint8_t gyro_config = 0U;
  uint8_t power_config = 0U;
  HAL_StatusTypeDef result;

  if (device == NULL)
  {
    return HAL_ERROR;
  }

  device->configured = false;
  result = Icm45686_ReadWhoAmI(device, &who_am_i);
  if ((result != HAL_OK) || (who_am_i != ICM45686_WHO_AM_I_EXPECTED))
  {
    return result == HAL_OK ? HAL_ERROR : result;
  }

  result = Icm45686_ConfigureForDiagnostic(device,
                                           &accel_config,
                                           &gyro_config,
                                           &power_config);
  if (result == HAL_OK)
  {
    device->configured = true;
  }
  return result;
}

HAL_StatusTypeDef Icm45686_ConfigureForDiagnostic(
    Icm45686 *device,
    uint8_t *accel_config_readback,
    uint8_t *gyro_config_readback,
    uint8_t *power_config_readback)
{
  HAL_StatusTypeDef result;

  if ((device == NULL) || (accel_config_readback == NULL) ||
      (gyro_config_readback == NULL) || (power_config_readback == NULL))
  {
    return HAL_ERROR;
  }

  *accel_config_readback = 0U;
  *gyro_config_readback = 0U;
  *power_config_readback = 0U;
  result = Icm45686_WriteRegister(device,
                                  ICM45686_ACCEL_CONFIG0_REGISTER,
                                  ICM45686_ACCEL_16G_100HZ);
  if (result == HAL_OK)
  {
    result = Icm45686_WriteRegister(device,
                                    ICM45686_GYRO_CONFIG0_REGISTER,
                                    ICM45686_GYRO_2000DPS_100HZ);
  }
  if (result == HAL_OK)
  {
    result = Icm45686_WriteRegister(device,
                                    ICM45686_PWR_MGMT0_REGISTER,
                                    ICM45686_SIX_AXIS_LOW_NOISE);
  }
  if (result == HAL_OK)
  {
    /* Use a conservative margin over the documented gyro startup time. */
    HAL_Delay(70U);
    result = Icm45686_ReadRegister(device,
                                   ICM45686_ACCEL_CONFIG0_REGISTER,
                                   accel_config_readback);
  }
  if (result == HAL_OK)
  {
    result = Icm45686_ReadRegister(device,
                                   ICM45686_GYRO_CONFIG0_REGISTER,
                                   gyro_config_readback);
  }
  if (result == HAL_OK)
  {
    result = Icm45686_ReadRegister(device,
                                   ICM45686_PWR_MGMT0_REGISTER,
                                   power_config_readback);
  }
  if ((result == HAL_OK) &&
      ((*accel_config_readback != ICM45686_ACCEL_16G_100HZ) ||
       (*gyro_config_readback != ICM45686_GYRO_2000DPS_100HZ) ||
       (*power_config_readback != ICM45686_SIX_AXIS_LOW_NOISE)))
  {
    result = HAL_ERROR;
  }
  return result;
}

HAL_StatusTypeDef Icm45686_ReadDiagnosticSample(
    Icm45686 *device,
    Icm45686Sample *sample,
    uint8_t raw_data[ICM45686_RAW_DATA_LENGTH])
{
  HAL_StatusTypeDef result;

  if ((device == NULL) || (sample == NULL) || (raw_data == NULL))
  {
    return HAL_ERROR;
  }

  memset(sample, 0, sizeof(*sample));
  memset(raw_data, 0, ICM45686_RAW_DATA_LENGTH);
  result = Icm45686_ReadRegisters(device,
                                  ICM45686_ACCEL_DATA_X1_REGISTER,
                                  raw_data,
                                  ICM45686_RAW_DATA_LENGTH);
  if (result == HAL_OK)
  {
    sample->accel[0] = Icm45686_DecodeInt16(&raw_data[0]);
    sample->accel[1] = Icm45686_DecodeInt16(&raw_data[2]);
    sample->accel[2] = Icm45686_DecodeInt16(&raw_data[4]);
    sample->gyro[0] = Icm45686_DecodeInt16(&raw_data[6]);
    sample->gyro[1] = Icm45686_DecodeInt16(&raw_data[8]);
    sample->gyro[2] = Icm45686_DecodeInt16(&raw_data[10]);
    sample->valid = true;
  }
  return result;
}

HAL_StatusTypeDef Icm45686_ReadSample(Icm45686 *device,
                                      Icm45686Sample *sample)
{
  uint8_t raw_data[ICM45686_RAW_DATA_LENGTH];

  if ((device == NULL) || !device->configured)
  {
    return HAL_ERROR;
  }
  return Icm45686_ReadDiagnosticSample(device, sample, raw_data);
}
