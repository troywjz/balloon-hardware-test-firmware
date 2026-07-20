#ifndef E28_SX1281_H
#define E28_SX1281_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define E28_SX1281_MAX_PACKET_SIZE       128U
#define E28_SX1281_DEFAULT_FREQUENCY_HZ  2405000000UL
#define E28_SX1281_MIN_CHIP_POWER_DBM    (-18)

typedef enum
{
  E28_SX1281_MODE_RESET = 0,
  E28_SX1281_MODE_STANDBY,
  E28_SX1281_MODE_RX,
  E28_SX1281_MODE_TX
} E28Sx1281Mode;

typedef enum
{
  E28_SX1281_ERROR_NONE = 0,
  E28_SX1281_ERROR_ARGUMENT,
  E28_SX1281_ERROR_POWER_OFF,
  E28_SX1281_ERROR_NOT_INITIALIZED,
  E28_SX1281_ERROR_BUSY_TIMEOUT,
  E28_SX1281_ERROR_SPI,
  E28_SX1281_ERROR_BAD_STATUS,
  E28_SX1281_ERROR_BAD_PACKET_TYPE,
  E28_SX1281_ERROR_TX_LOCKED,
  E28_SX1281_ERROR_TX_BUSY,
  E28_SX1281_ERROR_COMMAND
} E28Sx1281Error;

typedef enum
{
  E28_SX1281_EVENT_NONE = 0,
  E28_SX1281_EVENT_TX_DONE,
  E28_SX1281_EVENT_TX_TIMEOUT,
  E28_SX1281_EVENT_RX_DONE,
  E28_SX1281_EVENT_RX_CRC_ERROR,
  E28_SX1281_EVENT_RX_HEADER_ERROR,
  E28_SX1281_EVENT_ERROR
} E28Sx1281Event;

typedef struct
{
  uint8_t data[E28_SX1281_MAX_PACKET_SIZE];
  uint8_t length;
  int8_t rssi_dbm;
  int8_t snr_db;
} E28Sx1281Packet;

typedef bool (*E28Sx1281PowerPresentCallback)(void *context);
/*
 * Required when the radio can be unpowered while the MCU remains powered.
 * enabled=false must leave every MCU-to-radio signal low or high-impedance.
 */
typedef void (*E28Sx1281BusEnableCallback)(void *context, bool enabled);

typedef struct
{
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *nss_port;
  uint16_t nss_pin;
  GPIO_TypeDef *reset_port;
  uint16_t reset_pin;
  GPIO_TypeDef *busy_port;
  uint16_t busy_pin;
  GPIO_TypeDef *dio1_port;
  uint16_t dio1_pin;
  GPIO_TypeDef *rxen_port;
  uint16_t rxen_pin;
  GPIO_TypeDef *txen_port;
  uint16_t txen_pin;
  E28Sx1281PowerPresentCallback power_present;
  E28Sx1281BusEnableCallback set_bus_enabled;
  void *callback_context;
} E28Sx1281Pins;

typedef struct
{
  E28Sx1281Pins pins;
  E28Sx1281Mode mode;
  E28Sx1281Error last_error;
  uint32_t frequency_hz;
  uint32_t tx_deadline;
  uint32_t rx_deadline;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t error_count;
  uint16_t last_irq;
  uint8_t last_chip_status;
  uint8_t last_packet_type;
  bool initialized;
  bool power_state_known;
  bool power_present;
  bool tx_permitted;
  bool resume_rx_after_tx;
} E28Sx1281;

void E28Sx1281_Construct(E28Sx1281 *radio,
                        const E28Sx1281Pins *pins,
                        uint32_t frequency_hz);
void E28Sx1281_ForceSafeReset(E28Sx1281 *radio);
bool E28Sx1281_IsPowerPresent(E28Sx1281 *radio);
/* false blocks future packets; use ForceSafeReset to abort an active TX. */
void E28Sx1281_SetTransmitPermission(E28Sx1281 *radio, bool permitted);

HAL_StatusTypeDef E28Sx1281_Probe(E28Sx1281 *radio,
                                  uint8_t *chip_status,
                                  uint8_t *packet_type);
HAL_StatusTypeDef E28Sx1281_Initialize(E28Sx1281 *radio);
HAL_StatusTypeDef E28Sx1281_StartReceive(E28Sx1281 *radio);
HAL_StatusTypeDef E28Sx1281_Send(E28Sx1281 *radio,
                                const uint8_t *data,
                                uint8_t length,
                                bool resume_receive);
E28Sx1281Event E28Sx1281_Service(E28Sx1281 *radio,
                                E28Sx1281Packet *packet);

const char *E28Sx1281_ModeName(E28Sx1281Mode mode);
const char *E28Sx1281_ErrorName(E28Sx1281Error error);

#endif
