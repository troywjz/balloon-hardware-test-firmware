#ifndef ICM45686_H
#define ICM45686_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define ICM45686_WHO_AM_I_EXPECTED 0xE9U
#define ICM45686_RAW_DATA_LENGTH    12U

typedef struct
{
  int16_t accel[3];
  int16_t gyro[3];
  bool valid;
} Icm45686Sample;

typedef struct
{
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *chip_select_port;
  uint16_t chip_select_pin;
  uint32_t timeout_ms;
  bool configured;
} Icm45686;

void Icm45686_Construct(Icm45686 *device,
                        SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *chip_select_port,
                        uint16_t chip_select_pin);
HAL_StatusTypeDef Icm45686_ReadRegisters(Icm45686 *device,
                                         uint8_t register_address,
                                         uint8_t *data,
                                         uint16_t length);
HAL_StatusTypeDef Icm45686_ReadRegister(Icm45686 *device,
                                        uint8_t register_address,
                                        uint8_t *value);
HAL_StatusTypeDef Icm45686_WriteRegister(Icm45686 *device,
                                         uint8_t register_address,
                                         uint8_t value);
HAL_StatusTypeDef Icm45686_ReadWhoAmI(Icm45686 *device, uint8_t *who_am_i);
HAL_StatusTypeDef Icm45686_SoftReset(Icm45686 *device);
HAL_StatusTypeDef Icm45686_Initialize(Icm45686 *device);
HAL_StatusTypeDef Icm45686_ConfigureForDiagnostic(
    Icm45686 *device,
    uint8_t *accel_config_readback,
    uint8_t *gyro_config_readback,
    uint8_t *power_config_readback);
HAL_StatusTypeDef Icm45686_ReadDiagnosticSample(
    Icm45686 *device,
    Icm45686Sample *sample,
    uint8_t raw_data[ICM45686_RAW_DATA_LENGTH]);
HAL_StatusTypeDef Icm45686_ReadSample(Icm45686 *device,
                                      Icm45686Sample *sample);

#endif
