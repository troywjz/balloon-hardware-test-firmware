#include "e28_sx1281.h"

#include <string.h>

#define SX1281_CMD_SET_STANDBY            0x80U
#define SX1281_CMD_SET_RX                 0x82U
#define SX1281_CMD_SET_TX                 0x83U
#define SX1281_CMD_WRITE_REGISTER         0x18U
#define SX1281_CMD_READ_REGISTER          0x19U
#define SX1281_CMD_WRITE_BUFFER           0x1AU
#define SX1281_CMD_READ_BUFFER            0x1BU
#define SX1281_CMD_SET_DIO_IRQ_PARAMS      0x8DU
#define SX1281_CMD_GET_IRQ_STATUS          0x15U
#define SX1281_CMD_CLEAR_IRQ_STATUS        0x97U
#define SX1281_CMD_SET_RF_FREQUENCY        0x86U
#define SX1281_CMD_SET_PACKET_TYPE         0x8AU
#define SX1281_CMD_GET_PACKET_TYPE         0x03U
#define SX1281_CMD_SET_TX_PARAMS           0x8EU
#define SX1281_CMD_SET_MODULATION_PARAMS   0x8BU
#define SX1281_CMD_SET_PACKET_PARAMS       0x8CU
#define SX1281_CMD_SET_BUFFER_BASE_ADDRESS 0x8FU
#define SX1281_CMD_GET_STATUS              0xC0U
#define SX1281_CMD_GET_RX_BUFFER_STATUS    0x17U
#define SX1281_CMD_GET_PACKET_STATUS       0x1DU

#define SX1281_PACKET_TYPE_LORA 0x01U
#define SX1281_LORA_SF7          0x70U
#define SX1281_LORA_BW_812_KHZ   0x18U
#define SX1281_LORA_CR_4_5       0x01U
#define SX1281_LORA_PREAMBLE_12  0x0CU
#define SX1281_LORA_EXPLICIT     0x00U
#define SX1281_LORA_CRC_ON       0x20U
#define SX1281_LORA_STANDARD_IQ  0x40U
#define SX1281_LORA_SYNC_WORD    0x12U

#define SX1281_REG_LORA_SF_CFG0        0x0925U
#define SX1281_REG_LORA_FREQ_ERR_CORR  0x093CU
#define SX1281_REG_LORA_IQ_CONFIG      0x093BU
#define SX1281_REG_LORA_SYNC_WORD      0x0944U

#define SX1281_IRQ_TX_DONE       0x0001U
#define SX1281_IRQ_RX_DONE       0x0002U
#define SX1281_IRQ_HEADER_ERROR  0x0020U
#define SX1281_IRQ_CRC_ERROR     0x0040U
#define SX1281_IRQ_TIMEOUT       0x4000U
#define SX1281_IRQ_ALL           0xFFFFU
#define SX1281_IRQ_DIO1_MASK     (SX1281_IRQ_TX_DONE | SX1281_IRQ_RX_DONE | \
                                  SX1281_IRQ_HEADER_ERROR | \
                                  SX1281_IRQ_CRC_ERROR | SX1281_IRQ_TIMEOUT)

#define SX1281_BUSY_TIMEOUT_MS  50U
#define SX1281_SPI_TIMEOUT_MS   100U
#define SX1281_RX_TIMEOUT_MS    1000U
#define SX1281_TX_TIMEOUT_MS    3000U
#define SX1281_TICK_SIZE_1_MS   0x02U
#define SX1281_RAMP_20_US       0xE0U
#define SX1281_MAX_READ_LENGTH  E28_SX1281_MAX_PACKET_SIZE

static const uint8_t sx1281_nop_buffer[SX1281_MAX_READ_LENGTH] = {0U};

static void E28Sx1281_EnterPowerOffState(E28Sx1281 *radio);

static bool E28Sx1281_DeadlineExpired(uint32_t deadline)
{
  return (int32_t)(HAL_GetTick() - deadline) >= 0;
}

static void E28Sx1281_SetError(E28Sx1281 *radio, E28Sx1281Error error)
{
  if (radio != NULL)
  {
    radio->last_error = error;
    if (error != E28_SX1281_ERROR_NONE)
    {
      ++radio->error_count;
    }
  }
}

static bool E28Sx1281_UpdatePowerState(E28Sx1281 *radio)
{
  bool present;

  if ((radio == NULL) || (radio->pins.power_present == NULL))
  {
    return false;
  }

  present = radio->pins.power_present(radio->pins.callback_context);
  if (!radio->power_state_known || (present != radio->power_present))
  {
    radio->power_state_known = true;
    radio->power_present = present;
    if (radio->pins.set_bus_enabled != NULL)
    {
      radio->pins.set_bus_enabled(radio->pins.callback_context, present);
    }
  }
  return present;
}

static HAL_StatusTypeDef E28Sx1281_RequirePower(E28Sx1281 *radio)
{
  if (radio == NULL)
  {
    return HAL_ERROR;
  }
  if (!E28Sx1281_UpdatePowerState(radio))
  {
    E28Sx1281_EnterPowerOffState(radio);
    if ((radio != NULL) &&
        (radio->last_error != E28_SX1281_ERROR_POWER_OFF))
    {
      E28Sx1281_SetError(radio, E28_SX1281_ERROR_POWER_OFF);
    }
    return HAL_ERROR;
  }
  return HAL_OK;
}

static void E28Sx1281_SetRfPathOff(E28Sx1281 *radio)
{
  if (radio->pins.rxen_port == radio->pins.txen_port)
  {
    HAL_GPIO_WritePin(radio->pins.rxen_port,
                      radio->pins.rxen_pin | radio->pins.txen_pin,
                      GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(radio->pins.rxen_port,
                      radio->pins.rxen_pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(radio->pins.txen_port,
                      radio->pins.txen_pin,
                      GPIO_PIN_RESET);
  }
}

static void E28Sx1281_EnterPowerOffState(E28Sx1281 *radio)
{
  E28Sx1281_SetRfPathOff(radio);
  HAL_GPIO_WritePin(radio->pins.reset_port, radio->pins.reset_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_RESET);
  radio->initialized = false;
  radio->tx_permitted = false;
  radio->resume_rx_after_tx = false;
  radio->tx_deadline = 0U;
  radio->rx_deadline = 0U;
  radio->mode = E28_SX1281_MODE_RESET;
}

static void E28Sx1281_SetRfPath(E28Sx1281 *radio, E28Sx1281Mode mode)
{
  E28Sx1281_SetRfPathOff(radio);
  if (mode == E28_SX1281_MODE_RX)
  {
    HAL_GPIO_WritePin(radio->pins.rxen_port,
                      radio->pins.rxen_pin,
                      GPIO_PIN_SET);
  }
  else if (mode == E28_SX1281_MODE_TX)
  {
    HAL_GPIO_WritePin(radio->pins.txen_port,
                      radio->pins.txen_pin,
                      GPIO_PIN_SET);
  }
}

static HAL_StatusTypeDef E28Sx1281_WaitWhileBusy(E28Sx1281 *radio)
{
  uint32_t deadline = HAL_GetTick() + SX1281_BUSY_TIMEOUT_MS;

  while (HAL_GPIO_ReadPin(radio->pins.busy_port,
                          radio->pins.busy_pin) == GPIO_PIN_SET)
  {
    if (E28Sx1281_DeadlineExpired(deadline))
    {
      E28Sx1281_SetError(radio, E28_SX1281_ERROR_BUSY_TIMEOUT);
      return HAL_TIMEOUT;
    }
    HAL_Delay(1U);
  }
  return HAL_OK;
}

static HAL_StatusTypeDef E28Sx1281_CommandWrite(E28Sx1281 *radio,
                                                const uint8_t *command,
                                                uint16_t command_length,
                                                const uint8_t *data,
                                                uint16_t data_length)
{
  HAL_StatusTypeDef result;

  result = E28Sx1281_RequirePower(radio);
  if (result == HAL_OK)
  {
    result = E28Sx1281_WaitWhileBusy(radio);
  }
  if (result != HAL_OK)
  {
    return result;
  }

  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(radio->pins.spi,
                            (uint8_t *)command,
                            command_length,
                            SX1281_SPI_TIMEOUT_MS);
  if ((result == HAL_OK) && (data_length > 0U))
  {
    result = HAL_SPI_Transmit(radio->pins.spi,
                              (uint8_t *)data,
                              data_length,
                              SX1281_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_SPI);
    return result;
  }
  return E28Sx1281_WaitWhileBusy(radio);
}

static HAL_StatusTypeDef E28Sx1281_CommandRead(E28Sx1281 *radio,
                                               const uint8_t *command,
                                               uint16_t command_length,
                                               uint8_t *data,
                                               uint16_t data_length)
{
  HAL_StatusTypeDef result;

  if (data_length > SX1281_MAX_READ_LENGTH)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_ARGUMENT);
    return HAL_ERROR;
  }

  result = E28Sx1281_RequirePower(radio);
  if (result == HAL_OK)
  {
    result = E28Sx1281_WaitWhileBusy(radio);
  }
  if (result != HAL_OK)
  {
    return result;
  }

  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(radio->pins.spi,
                            (uint8_t *)command,
                            command_length,
                            SX1281_SPI_TIMEOUT_MS);
  if ((result == HAL_OK) && (data_length > 0U))
  {
    result = HAL_SPI_TransmitReceive(radio->pins.spi,
                                     (uint8_t *)sx1281_nop_buffer,
                                     data,
                                     data_length,
                                     SX1281_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_SPI);
    return result;
  }
  return E28Sx1281_WaitWhileBusy(radio);
}

static HAL_StatusTypeDef E28Sx1281_HardwareReset(E28Sx1281 *radio)
{
  if (E28Sx1281_RequirePower(radio) != HAL_OK)
  {
    return HAL_ERROR;
  }

  E28Sx1281_SetRfPathOff(radio);
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(radio->pins.reset_port, radio->pins.reset_pin, GPIO_PIN_RESET);
  HAL_Delay(5U);
  HAL_GPIO_WritePin(radio->pins.reset_port, radio->pins.reset_pin, GPIO_PIN_SET);
  HAL_Delay(5U);
  radio->initialized = false;
  radio->mode = E28_SX1281_MODE_STANDBY;
  return E28Sx1281_WaitWhileBusy(radio);
}

static HAL_StatusTypeDef E28Sx1281_SetStandby(E28Sx1281 *radio)
{
  const uint8_t command[] = {SX1281_CMD_SET_STANDBY, 0x00U};
  HAL_StatusTypeDef result = E28Sx1281_CommandWrite(
      radio, command, sizeof(command), NULL, 0U);

  if (result == HAL_OK)
  {
    E28Sx1281_SetRfPathOff(radio);
    radio->mode = E28_SX1281_MODE_STANDBY;
    radio->rx_deadline = 0U;
  }
  return result;
}

static HAL_StatusTypeDef E28Sx1281_GetStatus(E28Sx1281 *radio,
                                             uint8_t *status)
{
  uint8_t command = SX1281_CMD_GET_STATUS;
  HAL_StatusTypeDef result;

  if ((radio == NULL) || (status == NULL))
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_ARGUMENT);
    return HAL_ERROR;
  }

  result = E28Sx1281_RequirePower(radio);
  if (result == HAL_OK)
  {
    result = E28Sx1281_WaitWhileBusy(radio);
  }
  if (result != HAL_OK)
  {
    return result;
  }

  /* GetStatus is a one-byte full-duplex SPI transaction: the status is
     returned while the host clocks out opcode 0xC0.  Using CommandRead here
     would discard that byte and clock an extra byte after the command. */
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_RESET);
  result = HAL_SPI_TransmitReceive(radio->pins.spi,
                                   &command,
                                   status,
                                   1U,
                                   SX1281_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_SET);

  if (result != HAL_OK)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_SPI);
    return result;
  }
  result = E28Sx1281_WaitWhileBusy(radio);

  if (result == HAL_OK)
  {
    radio->last_chip_status = *status;
  }
  return result;
}

static bool E28Sx1281_StatusIsSuccessful(uint8_t status)
{
  uint8_t chip_mode = (uint8_t)((status >> 5U) & 0x07U);
  uint8_t command_status = (uint8_t)((status >> 2U) & 0x07U);

  return (status != 0x00U) && (status != 0xFFU) &&
         (chip_mode >= 2U) && (chip_mode <= 6U) &&
         ((command_status == 1U) || (command_status == 2U));
}

static HAL_StatusTypeDef E28Sx1281_SetPacketType(E28Sx1281 *radio)
{
  const uint8_t command[] = {
      SX1281_CMD_SET_PACKET_TYPE,
      SX1281_PACKET_TYPE_LORA,
  };
  return E28Sx1281_CommandWrite(radio, command, sizeof(command), NULL, 0U);
}

static HAL_StatusTypeDef E28Sx1281_GetPacketType(E28Sx1281 *radio,
                                                 uint8_t *packet_type)
{
  const uint8_t command[] = {SX1281_CMD_GET_PACKET_TYPE, 0x00U};
  HAL_StatusTypeDef result = E28Sx1281_CommandRead(
      radio, command, sizeof(command), packet_type, 1U);

  if (result == HAL_OK)
  {
    radio->last_packet_type = *packet_type;
  }
  return result;
}

static HAL_StatusTypeDef E28Sx1281_WriteRegister(E28Sx1281 *radio,
                                                 uint16_t address,
                                                 const uint8_t *data,
                                                 uint16_t length)
{
  const uint8_t command[] = {
      SX1281_CMD_WRITE_REGISTER,
      (uint8_t)(address >> 8U),
      (uint8_t)address,
  };
  return E28Sx1281_CommandWrite(
      radio, command, sizeof(command), data, length);
}

static HAL_StatusTypeDef E28Sx1281_ReadRegister(E28Sx1281 *radio,
                                                uint16_t address,
                                                uint8_t *data,
                                                uint16_t length)
{
  const uint8_t command[] = {
      SX1281_CMD_READ_REGISTER,
      (uint8_t)(address >> 8U),
      (uint8_t)address,
      0x00U,
  };
  return E28Sx1281_CommandRead(
      radio, command, sizeof(command), data, length);
}

static HAL_StatusTypeDef E28Sx1281_SetFrequency(E28Sx1281 *radio)
{
  uint32_t frequency_word = (uint32_t)((((uint64_t)radio->frequency_hz << 18U) +
                                        26000000ULL) /
                                       52000000ULL);
  const uint8_t command[] = {
      SX1281_CMD_SET_RF_FREQUENCY,
      (uint8_t)(frequency_word >> 16U),
      (uint8_t)(frequency_word >> 8U),
      (uint8_t)frequency_word,
  };
  return E28Sx1281_CommandWrite(radio, command, sizeof(command), NULL, 0U);
}

static HAL_StatusTypeDef E28Sx1281_SetModulation(E28Sx1281 *radio)
{
  const uint8_t command[] = {
      SX1281_CMD_SET_MODULATION_PARAMS,
      SX1281_LORA_SF7,
      SX1281_LORA_BW_812_KHZ,
      SX1281_LORA_CR_4_5,
  };
  uint8_t sf_config = 0x37U;
  uint8_t correction = 0U;
  HAL_StatusTypeDef result;

  result = E28Sx1281_CommandWrite(radio, command, sizeof(command), NULL, 0U);
  if (result == HAL_OK)
  {
    result = E28Sx1281_WriteRegister(
        radio, SX1281_REG_LORA_SF_CFG0, &sf_config, 1U);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_ReadRegister(
        radio, SX1281_REG_LORA_FREQ_ERR_CORR, &correction, 1U);
  }
  if (result == HAL_OK)
  {
    correction = (uint8_t)((correction & 0xF8U) | 0x01U);
    result = E28Sx1281_WriteRegister(
        radio, SX1281_REG_LORA_FREQ_ERR_CORR, &correction, 1U);
  }
  return result;
}

static HAL_StatusTypeDef E28Sx1281_SetLoraSyncWord(E28Sx1281 *radio)
{
  uint8_t registers[2] = {0U, 0U};
  HAL_StatusTypeDef result = E28Sx1281_ReadRegister(
      radio, SX1281_REG_LORA_SYNC_WORD, registers, sizeof(registers));

  if (result == HAL_OK)
  {
    registers[0] = (uint8_t)((registers[0] & 0x0FU) |
                             (SX1281_LORA_SYNC_WORD & 0xF0U));
    registers[1] = (uint8_t)((registers[1] & 0x0FU) |
                             ((SX1281_LORA_SYNC_WORD & 0x0FU) << 4U));
    result = E28Sx1281_WriteRegister(
        radio, SX1281_REG_LORA_SYNC_WORD, registers, sizeof(registers));
  }
  return result;
}

static HAL_StatusTypeDef E28Sx1281_SetPacketParameters(E28Sx1281 *radio,
                                                       uint8_t payload_length)
{
  const uint8_t command[] = {
      SX1281_CMD_SET_PACKET_PARAMS,
      SX1281_LORA_PREAMBLE_12,
      SX1281_LORA_EXPLICIT,
      payload_length,
      SX1281_LORA_CRC_ON,
      SX1281_LORA_STANDARD_IQ,
      0x00U,
      0x00U,
  };
  uint8_t iq_config = 0x09U;
  HAL_StatusTypeDef result = E28Sx1281_CommandWrite(
      radio, command, sizeof(command), NULL, 0U);

  if (result == HAL_OK)
  {
    result = E28Sx1281_WriteRegister(
        radio, SX1281_REG_LORA_IQ_CONFIG, &iq_config, 1U);
  }
  return result;
}

static HAL_StatusTypeDef E28Sx1281_SetTxParameters(E28Sx1281 *radio)
{
  /* SX128x encodes -18..+13 dBm as 0..31 (power code = dBm + 18). */
  const uint8_t command[] = {
      SX1281_CMD_SET_TX_PARAMS,
      (uint8_t)(E28_SX1281_MIN_CHIP_POWER_DBM + 18),
      SX1281_RAMP_20_US,
  };
  return E28Sx1281_CommandWrite(radio, command, sizeof(command), NULL, 0U);
}

static HAL_StatusTypeDef E28Sx1281_SetIrqParameters(E28Sx1281 *radio)
{
  const uint8_t command[] = {
      SX1281_CMD_SET_DIO_IRQ_PARAMS,
      (uint8_t)(SX1281_IRQ_DIO1_MASK >> 8U),
      (uint8_t)SX1281_IRQ_DIO1_MASK,
      (uint8_t)(SX1281_IRQ_DIO1_MASK >> 8U),
      (uint8_t)SX1281_IRQ_DIO1_MASK,
      0x00U,
      0x00U,
      0x00U,
      0x00U,
  };
  return E28Sx1281_CommandWrite(radio, command, sizeof(command), NULL, 0U);
}

static HAL_StatusTypeDef E28Sx1281_ClearIrq(E28Sx1281 *radio,
                                            uint16_t irq)
{
  const uint8_t command[] = {
      SX1281_CMD_CLEAR_IRQ_STATUS,
      (uint8_t)(irq >> 8U),
      (uint8_t)irq,
  };
  return E28Sx1281_CommandWrite(radio, command, sizeof(command), NULL, 0U);
}

static HAL_StatusTypeDef E28Sx1281_GetIrq(E28Sx1281 *radio,
                                          uint16_t *irq)
{
  const uint8_t command[] = {SX1281_CMD_GET_IRQ_STATUS, 0x00U};
  uint8_t raw[2] = {0U, 0U};
  HAL_StatusTypeDef result = E28Sx1281_CommandRead(
      radio, command, sizeof(command), raw, sizeof(raw));

  if (result == HAL_OK)
  {
    *irq = ((uint16_t)raw[0] << 8U) | raw[1];
    radio->last_irq = *irq;
  }
  return result;
}

static HAL_StatusTypeDef E28Sx1281_WriteBuffer(E28Sx1281 *radio,
                                               const uint8_t *data,
                                               uint8_t length)
{
  const uint8_t command[] = {SX1281_CMD_WRITE_BUFFER, 0x00U};
  return E28Sx1281_CommandWrite(
      radio, command, sizeof(command), data, length);
}

static HAL_StatusTypeDef E28Sx1281_ReadPacket(E28Sx1281 *radio,
                                              E28Sx1281Packet *packet)
{
  const uint8_t buffer_status_command[] = {
      SX1281_CMD_GET_RX_BUFFER_STATUS,
      0x00U,
  };
  const uint8_t packet_status_command[] = {
      SX1281_CMD_GET_PACKET_STATUS,
      0x00U,
  };
  uint8_t buffer_status[2] = {0U, 0U};
  uint8_t packet_status[5] = {0U, 0U, 0U, 0U, 0U};
  uint8_t read_command[3];
  HAL_StatusTypeDef result;

  result = E28Sx1281_CommandRead(radio,
                                 buffer_status_command,
                                 sizeof(buffer_status_command),
                                 buffer_status,
                                 sizeof(buffer_status));
  if (result != HAL_OK)
  {
    return result;
  }
  if ((buffer_status[0] == 0U) ||
      (buffer_status[0] > E28_SX1281_MAX_PACKET_SIZE))
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_COMMAND);
    return HAL_ERROR;
  }

  packet->length = buffer_status[0];
  read_command[0] = SX1281_CMD_READ_BUFFER;
  read_command[1] = buffer_status[1];
  read_command[2] = 0x00U;
  result = E28Sx1281_CommandRead(radio,
                                 read_command,
                                 sizeof(read_command),
                                 packet->data,
                                 packet->length);
  if (result == HAL_OK)
  {
    result = E28Sx1281_CommandRead(radio,
                                   packet_status_command,
                                   sizeof(packet_status_command),
                                   packet_status,
                                   sizeof(packet_status));
  }
  if (result == HAL_OK)
  {
    packet->rssi_dbm = (int8_t)(-((int16_t)packet_status[0] / 2));
    packet->snr_db = (int8_t)((int8_t)packet_status[1] / 4);
  }
  return result;
}

void E28Sx1281_Construct(E28Sx1281 *radio,
                        const E28Sx1281Pins *pins,
                        uint32_t frequency_hz)
{
  if ((radio == NULL) || (pins == NULL))
  {
    return;
  }

  memset(radio, 0, sizeof(*radio));
  radio->pins = *pins;
  radio->frequency_hz = frequency_hz;
  radio->mode = E28_SX1281_MODE_RESET;
  radio->last_error = E28_SX1281_ERROR_NONE;
  E28Sx1281_ForceSafeReset(radio);
}

void E28Sx1281_ForceSafeReset(E28Sx1281 *radio)
{
  bool power_present;

  if (radio == NULL)
  {
    return;
  }

  /* Emergency RF shutdown must not wait for ADC or board callbacks. */
  E28Sx1281_SetRfPathOff(radio);
  HAL_GPIO_WritePin(radio->pins.reset_port, radio->pins.reset_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_RESET);
  radio->initialized = false;
  radio->tx_permitted = false;
  radio->resume_rx_after_tx = false;
  radio->tx_deadline = 0U;
  radio->rx_deadline = 0U;
  radio->mode = E28_SX1281_MODE_RESET;

  power_present = E28Sx1281_UpdatePowerState(radio);
  if (power_present)
  {
    HAL_GPIO_WritePin(radio->pins.nss_port, radio->pins.nss_pin, GPIO_PIN_SET);
  }
}

bool E28Sx1281_IsPowerPresent(E28Sx1281 *radio)
{
  return E28Sx1281_RequirePower(radio) == HAL_OK;
}

void E28Sx1281_SetTransmitPermission(E28Sx1281 *radio, bool permitted)
{
  if (radio == NULL)
  {
    return;
  }
  radio->tx_permitted = permitted && radio->initialized &&
                        E28Sx1281_UpdatePowerState(radio);
}

HAL_StatusTypeDef E28Sx1281_Probe(E28Sx1281 *radio,
                                  uint8_t *chip_status,
                                  uint8_t *packet_type)
{
  HAL_StatusTypeDef result;
  uint8_t status = 0U;
  uint8_t type = 0xFFU;

  if ((radio == NULL) || (chip_status == NULL) || (packet_type == NULL))
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_ARGUMENT);
    return HAL_ERROR;
  }

  E28Sx1281_SetError(radio, E28_SX1281_ERROR_NONE);
  radio->tx_permitted = false;
  result = E28Sx1281_HardwareReset(radio);
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetStandby(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetPacketType(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_GetPacketType(radio, &type);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_GetStatus(radio, &status);
  }

  *chip_status = status;
  *packet_type = type;
  if (result != HAL_OK)
  {
    return result;
  }

  if (!E28Sx1281_StatusIsSuccessful(status))
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_BAD_STATUS);
    return HAL_ERROR;
  }
  if (type != SX1281_PACKET_TYPE_LORA)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_BAD_PACKET_TYPE);
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef E28Sx1281_Initialize(E28Sx1281 *radio)
{
  const uint8_t buffer_base_command[] = {
      SX1281_CMD_SET_BUFFER_BASE_ADDRESS,
      0x00U,
      0x00U,
  };
  HAL_StatusTypeDef result;
  uint8_t chip_status = 0U;
  uint8_t packet_type = 0U;

  if (radio == NULL)
  {
    return HAL_ERROR;
  }

  result = E28Sx1281_Probe(radio, &chip_status, &packet_type);
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetFrequency(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_CommandWrite(radio,
                                    buffer_base_command,
                                    sizeof(buffer_base_command),
                                    NULL,
                                    0U);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetModulation(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetLoraSyncWord(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetPacketParameters(
        radio, E28_SX1281_MAX_PACKET_SIZE);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetTxParameters(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetIrqParameters(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_ClearIrq(radio, SX1281_IRQ_ALL);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetStandby(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_GetStatus(radio, &chip_status);
  }
  if ((result == HAL_OK) && !E28Sx1281_StatusIsSuccessful(chip_status))
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_BAD_STATUS);
    result = HAL_ERROR;
  }

  if (result == HAL_OK)
  {
    radio->initialized = true;
    radio->tx_permitted = false;
    radio->last_error = E28_SX1281_ERROR_NONE;
  }
  else
  {
    E28Sx1281_ForceSafeReset(radio);
  }
  return result;
}

HAL_StatusTypeDef E28Sx1281_StartReceive(E28Sx1281 *radio)
{
  const uint8_t command[] = {
      SX1281_CMD_SET_RX,
      SX1281_TICK_SIZE_1_MS,
      (uint8_t)(SX1281_RX_TIMEOUT_MS >> 8U),
      (uint8_t)SX1281_RX_TIMEOUT_MS,
  };
  HAL_StatusTypeDef result;

  if ((radio == NULL) || !radio->initialized)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_NOT_INITIALIZED);
    return HAL_ERROR;
  }
  if (E28Sx1281_RequirePower(radio) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (radio->mode == E28_SX1281_MODE_TX)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_TX_BUSY);
    return HAL_BUSY;
  }

  result = E28Sx1281_SetStandby(radio);
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetPacketParameters(
        radio, E28_SX1281_MAX_PACKET_SIZE);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_ClearIrq(radio, SX1281_IRQ_ALL);
  }
  if (result == HAL_OK)
  {
    E28Sx1281_SetRfPath(radio, E28_SX1281_MODE_RX);
    result = E28Sx1281_CommandWrite(
        radio, command, sizeof(command), NULL, 0U);
  }
  if (result == HAL_OK)
  {
    radio->mode = E28_SX1281_MODE_RX;
    radio->rx_deadline = HAL_GetTick() + SX1281_RX_TIMEOUT_MS + 50U;
    radio->last_error = E28_SX1281_ERROR_NONE;
  }
  else
  {
    E28Sx1281_SetRfPathOff(radio);
    radio->mode = E28_SX1281_MODE_STANDBY;
    radio->rx_deadline = 0U;
  }
  return result;
}

HAL_StatusTypeDef E28Sx1281_Send(E28Sx1281 *radio,
                                const uint8_t *data,
                                uint8_t length,
                                bool resume_receive)
{
  const uint8_t command[] = {
      SX1281_CMD_SET_TX,
      SX1281_TICK_SIZE_1_MS,
      (uint8_t)(SX1281_TX_TIMEOUT_MS >> 8U),
      (uint8_t)SX1281_TX_TIMEOUT_MS,
  };
  HAL_StatusTypeDef result;

  if ((radio == NULL) || (data == NULL) || (length == 0U) ||
      (length > E28_SX1281_MAX_PACKET_SIZE))
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_ARGUMENT);
    return HAL_ERROR;
  }
  if (!radio->initialized)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_NOT_INITIALIZED);
    return HAL_ERROR;
  }
  if (E28Sx1281_RequirePower(radio) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (!radio->tx_permitted)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_TX_LOCKED);
    return HAL_ERROR;
  }
  if (radio->mode == E28_SX1281_MODE_TX)
  {
    E28Sx1281_SetError(radio, E28_SX1281_ERROR_TX_BUSY);
    return HAL_BUSY;
  }

  result = E28Sx1281_SetStandby(radio);
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetPacketParameters(radio, length);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_SetTxParameters(radio);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_WriteBuffer(radio, data, length);
  }
  if (result == HAL_OK)
  {
    result = E28Sx1281_ClearIrq(radio, SX1281_IRQ_ALL);
  }
  if (result == HAL_OK)
  {
    E28Sx1281_SetRfPath(radio, E28_SX1281_MODE_TX);
    HAL_Delay(1U);
    result = E28Sx1281_CommandWrite(
        radio, command, sizeof(command), NULL, 0U);
  }
  if (result == HAL_OK)
  {
    radio->mode = E28_SX1281_MODE_TX;
    radio->rx_deadline = 0U;
    radio->resume_rx_after_tx = resume_receive;
    radio->tx_deadline = HAL_GetTick() + SX1281_TX_TIMEOUT_MS + 50U;
    radio->last_error = E28_SX1281_ERROR_NONE;
  }
  else
  {
    E28Sx1281_SetRfPathOff(radio);
    radio->mode = E28_SX1281_MODE_STANDBY;
  }
  return result;
}

E28Sx1281Event E28Sx1281_Service(E28Sx1281 *radio,
                                 E28Sx1281Packet *packet)
{
  HAL_StatusTypeDef result;
  uint16_t irq = 0U;
  bool resume_rx;

  if ((radio == NULL) || !radio->initialized)
  {
    return E28_SX1281_EVENT_NONE;
  }
  if (E28Sx1281_RequirePower(radio) != HAL_OK)
  {
    return E28_SX1281_EVENT_ERROR;
  }

  if ((radio->mode == E28_SX1281_MODE_TX) &&
      E28Sx1281_DeadlineExpired(radio->tx_deadline))
  {
    resume_rx = radio->resume_rx_after_tx;
    E28Sx1281_SetRfPathOff(radio);
    result = E28Sx1281_SetStandby(radio);
    if (result == HAL_OK)
    {
      result = E28Sx1281_ClearIrq(radio, SX1281_IRQ_ALL);
    }
    radio->resume_rx_after_tx = false;
    radio->tx_permitted = false;
    if ((result == HAL_OK) && resume_rx)
    {
      result = E28Sx1281_StartReceive(radio);
    }
    if (result != HAL_OK)
    {
      E28Sx1281_ForceSafeReset(radio);
      return E28_SX1281_EVENT_ERROR;
    }
    return E28_SX1281_EVENT_TX_TIMEOUT;
  }

  if (HAL_GPIO_ReadPin(radio->pins.dio1_port,
                       radio->pins.dio1_pin) != GPIO_PIN_SET)
  {
    if ((radio->mode == E28_SX1281_MODE_RX) &&
        (radio->rx_deadline != 0U) &&
        E28Sx1281_DeadlineExpired(radio->rx_deadline))
    {
      E28Sx1281_SetRfPathOff(radio);
      result = E28Sx1281_SetStandby(radio);
      if (result == HAL_OK)
      {
        result = E28Sx1281_ClearIrq(radio, SX1281_IRQ_ALL);
      }
      if (result == HAL_OK)
      {
        result = E28Sx1281_StartReceive(radio);
      }
      if (result != HAL_OK)
      {
        E28Sx1281_ForceSafeReset(radio);
        return E28_SX1281_EVENT_ERROR;
      }
    }
    return E28_SX1281_EVENT_NONE;
  }

  result = E28Sx1281_GetIrq(radio, &irq);
  if (result != HAL_OK)
  {
    E28Sx1281_ForceSafeReset(radio);
    return E28_SX1281_EVENT_ERROR;
  }

  if ((irq & SX1281_IRQ_TX_DONE) != 0U)
  {
    resume_rx = radio->resume_rx_after_tx;
    E28Sx1281_SetRfPathOff(radio);
    result = E28Sx1281_ClearIrq(radio, irq);
    if (result == HAL_OK)
    {
      result = E28Sx1281_SetStandby(radio);
    }
    radio->resume_rx_after_tx = false;
    if ((result == HAL_OK) && resume_rx)
    {
      result = E28Sx1281_StartReceive(radio);
    }
    if (result != HAL_OK)
    {
      E28Sx1281_ForceSafeReset(radio);
      return E28_SX1281_EVENT_ERROR;
    }
    ++radio->tx_count;
    return E28_SX1281_EVENT_TX_DONE;
  }

  if ((irq & (SX1281_IRQ_HEADER_ERROR | SX1281_IRQ_CRC_ERROR)) != 0U)
  {
    E28Sx1281Event event = (irq & SX1281_IRQ_CRC_ERROR) != 0U
                                ? E28_SX1281_EVENT_RX_CRC_ERROR
                                : E28_SX1281_EVENT_RX_HEADER_ERROR;
    E28Sx1281_SetRfPathOff(radio);
    result = E28Sx1281_ClearIrq(radio, irq);
    radio->mode = E28_SX1281_MODE_STANDBY;
    if (result == HAL_OK)
    {
      result = E28Sx1281_StartReceive(radio);
    }
    if (result != HAL_OK)
    {
      E28Sx1281_ForceSafeReset(radio);
      return E28_SX1281_EVENT_ERROR;
    }
    return event;
  }

  if ((irq & SX1281_IRQ_RX_DONE) != 0U)
  {
    E28Sx1281_SetRfPathOff(radio);
    result = packet != NULL ? E28Sx1281_ReadPacket(radio, packet) : HAL_ERROR;
    if (result == HAL_OK)
    {
      result = E28Sx1281_ClearIrq(radio, irq);
    }
    radio->mode = E28_SX1281_MODE_STANDBY;
    if (result == HAL_OK)
    {
      ++radio->rx_count;
      result = E28Sx1281_StartReceive(radio);
    }
    if (result != HAL_OK)
    {
      E28Sx1281_ForceSafeReset(radio);
      return E28_SX1281_EVENT_ERROR;
    }
    return E28_SX1281_EVENT_RX_DONE;
  }

  if ((irq & SX1281_IRQ_TIMEOUT) != 0U)
  {
    bool was_tx = radio->mode == E28_SX1281_MODE_TX;
    resume_rx = radio->resume_rx_after_tx;
    E28Sx1281_SetRfPathOff(radio);
    result = E28Sx1281_ClearIrq(radio, irq);
    if (result == HAL_OK)
    {
      result = E28Sx1281_SetStandby(radio);
    }
    radio->resume_rx_after_tx = false;
    if (was_tx)
    {
      radio->tx_permitted = false;
    }
    if ((result == HAL_OK) && (!was_tx || resume_rx))
    {
      result = E28Sx1281_StartReceive(radio);
    }
    if (result != HAL_OK)
    {
      E28Sx1281_ForceSafeReset(radio);
      return E28_SX1281_EVENT_ERROR;
    }
    return was_tx ? E28_SX1281_EVENT_TX_TIMEOUT
                  : E28_SX1281_EVENT_NONE;
  }

  result = E28Sx1281_ClearIrq(radio, irq);
  if (result != HAL_OK)
  {
    E28Sx1281_ForceSafeReset(radio);
    return E28_SX1281_EVENT_ERROR;
  }
  return E28_SX1281_EVENT_NONE;
}

const char *E28Sx1281_ModeName(E28Sx1281Mode mode)
{
  switch (mode)
  {
    case E28_SX1281_MODE_STANDBY:
      return "standby";
    case E28_SX1281_MODE_RX:
      return "rx";
    case E28_SX1281_MODE_TX:
      return "tx";
    default:
      return "reset";
  }
}

const char *E28Sx1281_ErrorName(E28Sx1281Error error)
{
  switch (error)
  {
    case E28_SX1281_ERROR_NONE:
      return "none";
    case E28_SX1281_ERROR_ARGUMENT:
      return "argument";
    case E28_SX1281_ERROR_POWER_OFF:
      return "power_off";
    case E28_SX1281_ERROR_NOT_INITIALIZED:
      return "not_initialized";
    case E28_SX1281_ERROR_BUSY_TIMEOUT:
      return "busy_timeout";
    case E28_SX1281_ERROR_SPI:
      return "spi";
    case E28_SX1281_ERROR_BAD_STATUS:
      return "bad_status";
    case E28_SX1281_ERROR_BAD_PACKET_TYPE:
      return "bad_packet_type";
    case E28_SX1281_ERROR_TX_LOCKED:
      return "tx_locked";
    case E28_SX1281_ERROR_TX_BUSY:
      return "tx_busy";
    default:
      return "command";
  }
}
