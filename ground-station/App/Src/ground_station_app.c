#include "ground_station_app.h"

#include "FreeRTOS.h"
#include "adc.h"
#include "balloon_csv_logger.h"
#include "balloon_radio_protocol.h"
#include "cmsis_os.h"
#include "e28_sx1281.h"
#include "fatfs.h"
#include "fatfs_platform.h"
#include "main.h"
#include "queue.h"
#include "sdio.h"
#include "spi.h"
#include "task.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define W25Q_READ_JEDEC_ID_COMMAND  0x9FU
#define ADC_FULL_SCALE              4095UL
#define ADC_REFERENCE_MV            3300UL
#define SUPPLY_DIVIDER_MULTIPLIER   2UL
#define COMMAND_BUFFER_SIZE         128U
#define USB_RX_QUEUE_SIZE           256U
#define RADIO_TX_ARM_TIMEOUT_MS     60000U
#define GROUND_HARDWARE_VERSION     "V1.1.0"
#define GROUND_FIRMWARE_VERSION     "V1.1.0.7"
#define MISSION_ID                  1U
#define MISSION_COMMAND_TTL_MS      2000U
#define MISSION_HEARTBEAT_PERIOD_MS 2000U
#define MISSION_LOG_SYNC_PERIOD_MS  1000U
#define MISSION_LOG_QUEUE_LENGTH     32U
#define MISSION_LOG_RESERVED_SLOTS   8U
#define MISSION_LOG_TASK_STACK_WORDS 768U
#define SD_MOUNT_RETRY_PERIOD_MS     2000U
#define SD_WARNING_PERIOD_MS         30000U
#define SD_TEST_FILE_PATH            "GS_SD.TMP"
#define SD_TEST_BUFFER_SIZE          128U
#define SD_RAW_SECTOR_SIZE           512U

typedef struct
{
  FRESULT sd_result;
  bool sd_mounted;
  bool sd_last_present;
  bool sd_last_raw_present;
  bool sd_logger_was_owner;
  bool sd_auto_retry_enabled;
  bool sd_auto_fallback_active;
  bool sd_auto_success_reported;
  bool sd_fault_active;
  uint32_t sd_next_retry_tick;
  uint32_t sd_next_warning_tick;
} GroundStationStatus;

typedef struct
{
  bool valid;
  BalloonRadioType type;
  uint16_t sequence;
  uint8_t payload_length;
  uint8_t frame_length;
} GroundStationPendingRadioTx;

typedef enum
{
  GROUND_LOG_CONTROL_START = 1,
  GROUND_LOG_DATA,
  GROUND_LOG_EVENT,
  GROUND_LOG_CONTROL_STOP
} GroundStationLogRecordKind;

typedef enum
{
  GROUND_LOG_EVENT_MISSION_START = 1,
  GROUND_LOG_EVENT_MISSION_STOP,
  GROUND_LOG_EVENT_COMMAND_QUEUED,
  GROUND_LOG_EVENT_TX_DONE,
  GROUND_LOG_EVENT_TX_TIMEOUT,
  GROUND_LOG_EVENT_ACK_RX,
  GROUND_LOG_EVENT_PROTOCOL_ERROR,
  GROUND_LOG_EVENT_RADIO_ERROR
} GroundStationLogEventKind;

typedef struct
{
  uint8_t event;
  uint8_t command;
  uint8_t stage;
  uint8_t reason;
  uint8_t flags;
  uint8_t channel;
  uint16_t sequence;
  uint16_t value;
  uint16_t duration_ms;
  uint8_t action;
  uint8_t action_channel;
  int8_t rssi_dbm;
  int8_t snr_db;
} GroundStationLogEvent;

typedef struct
{
  BalloonTelemetryPayload telemetry;
  uint16_t frame_sequence;
  int8_t rssi_dbm;
  int8_t snr_db;
} GroundStationLogData;

typedef struct
{
  uint8_t kind;
  uint32_t timestamp_ms;
  union
  {
    GroundStationLogData data;
    GroundStationLogEvent event;
  } payload;
} GroundStationLogRecord;

_Static_assert(sizeof(GroundStationLogRecord) <= 64U,
               "GroundStationLogRecord must remain queue friendly");

static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static uint8_t usb_rx_queue[USB_RX_QUEUE_SIZE];
static char output_buffer[384];
static E28Sx1281 radio;
static bool radio_attached;
static bool radio_tx_armed;
static uint32_t radio_tx_arm_deadline;
static uint16_t radio_sequence;
static GroundStationPendingRadioTx pending_radio_tx;
static bool mission_active;
static uint32_t mission_next_heartbeat_tick;
static BalloonCsvLogger mission_data_log;
static BalloonCsvLogger mission_event_log;
static volatile BalloonLogState mission_log_state = BALLOON_LOG_STATE_OFF;
static bool mission_log_mounted;
static uint32_t mission_data_log_sequence;
static uint32_t mission_event_log_sequence;
static char mission_csv_line[512];
static QueueHandle_t mission_log_queue;
static TaskHandle_t mission_log_task_handle;
static volatile bool mission_log_owns_sd;
static volatile bool mission_log_stop_requested;
static volatile uint32_t mission_log_drop_count;
static volatile FRESULT mission_log_last_result = FR_NOT_READY;
static char sd_test_expected[SD_TEST_BUFFER_SIZE];
static char sd_test_actual[SD_TEST_BUFFER_SIZE];
static uint32_t sd_raw_sector_words[SD_RAW_SECTOR_SIZE / sizeof(uint32_t)];
static uint32_t sd_diagnostic_bus_width = SDIO_BUS_WIDE_4B;

static void GroundStation_StartMissionLogging(void);
static void GroundStation_StopMissionLogging(void);
static void GroundStation_LogEvent(GroundStationLogEventKind event,
                                   BalloonCommandCode command,
                                   BalloonAckStage stage,
                                   BalloonRejectReason reason,
                                   uint16_t sequence,
                                   uint8_t flags,
                                   uint8_t channel,
                                   uint16_t value,
                                   uint16_t duration_ms,
                                   int8_t rssi_dbm,
                                   int8_t snr_db);
static void GroundStation_LogTelemetry(const BalloonTelemetryPayload *telemetry,
                                       uint16_t frame_sequence,
                                       int8_t rssi_dbm,
                                       int8_t snr_db);
static const char *GroundStation_ActionDirection(uint8_t action, bool reverse);
static const char *GroundStation_TelemetryLogState(
    const BalloonTelemetryPayload *telemetry);

static bool GroundStation_RadioPowerPresent(void *context)
{
  (void)context;
  return true;
}

static void GroundStation_ConfigureOutput(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pins;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &gpio);
}

void GroundStationApp_EarlySafetyInit(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, RADIO_RXEN_Pin | RADIO_TXEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_CS_GPIO_Port, RADIO_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  GroundStation_ConfigureOutput(GPIOC, RADIO_RXEN_Pin | RADIO_TXEN_Pin);
  GroundStation_ConfigureOutput(GPIOB, RADIO_RST_Pin | RADIO_CS_Pin);
  GroundStation_ConfigureOutput(GPIOA, FLASH_CS_Pin);
}

/* Strong application override of the weak CubeMX BSP function. */
uint8_t BSP_SD_Init(void)
{
  if (BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR;
  }
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    return MSD_ERROR;
  }
  return HAL_SD_ConfigWideBusOperation(&hsd, sd_diagnostic_bus_width) == HAL_OK
             ? MSD_OK
             : MSD_ERROR;
}

static void GroundStation_EnforceRadioLockdown(void)
{
  if (radio_attached)
  {
    E28Sx1281_ForceSafeReset(&radio);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, RADIO_RXEN_Pin | RADIO_TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RADIO_CS_GPIO_Port, RADIO_CS_Pin, GPIO_PIN_SET);
  }
  radio_tx_armed = false;
  radio_tx_arm_deadline = 0U;
  pending_radio_tx.valid = false;
  mission_active = false;
}

void GroundStationApp_OnUsbData(const uint8_t *data, uint32_t length)
{
  uint32_t index;

  if (data == NULL)
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    uint16_t next_head = (uint16_t)((usb_rx_head + 1U) % USB_RX_QUEUE_SIZE);

    if (next_head == usb_rx_tail)
    {
      break;
    }

    usb_rx_queue[usb_rx_head] = data[index];
    usb_rx_head = next_head;
  }
}

static bool GroundStation_TryReadUsbByte(uint8_t *value)
{
  if ((value == NULL) || (usb_rx_tail == usb_rx_head))
  {
    return false;
  }

  *value = usb_rx_queue[usb_rx_tail];
  usb_rx_tail = (uint16_t)((usb_rx_tail + 1U) % USB_RX_QUEUE_SIZE);
  return true;
}

static void GroundStation_Send(const char *format, ...)
{
  va_list arguments;
  int length;
  uint32_t attempts;

  va_start(arguments, format);
  length = vsnprintf(output_buffer, sizeof(output_buffer), format, arguments);
  va_end(arguments);

  if (length <= 0)
  {
    return;
  }
  if ((size_t)length >= sizeof(output_buffer))
  {
    length = (int)sizeof(output_buffer) - 1;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)output_buffer, (uint16_t)length, 100U);
  for (attempts = 0U; attempts < 100U; ++attempts)
  {
    if (CDC_Transmit_FS((uint8_t *)output_buffer, (uint16_t)length) == USBD_OK)
    {
      osDelay(5U);
      return;
    }
    osDelay(1U);
  }
}

static bool GroundStation_DeadlineExpired(uint32_t deadline)
{
  return (int32_t)(HAL_GetTick() - deadline) >= 0;
}

static uint32_t GroundStation_RemainingMs(uint32_t deadline)
{
  uint32_t now = HAL_GetTick();

  if ((deadline == 0U) || ((int32_t)(now - deadline) >= 0))
  {
    return 0U;
  }
  return deadline - now;
}

static void GroundStation_ServiceRadioArm(void)
{
  if (!mission_active && radio_tx_armed &&
      GroundStation_DeadlineExpired(radio_tx_arm_deadline))
  {
    radio_tx_armed = false;
    radio_tx_arm_deadline = 0U;
    E28Sx1281_SetTransmitPermission(&radio, false);
    GroundStation_Send("GS radio tx_arm expired; transmit locked\r\n");
  }
}

static bool GroundStation_IsRadioTxArmed(void)
{
  GroundStation_ServiceRadioArm();
  return radio_tx_armed;
}

static void GroundStation_SendRadioStatus(void)
{
  bool power_present = E28Sx1281_IsPowerPresent(&radio);

  GroundStation_Send(
      "GS radio power_present=%u mode=%s initialized=%u tx_arm=%u arm_remaining_ms=%lu "
      "freq_hz=%lu chip_power_dbm=%d status=0x%02X packet_type=0x%02X "
      "irq=0x%04X tx=%lu rx=%lu errors=%lu last_error=%s "
      "pins(rst/busy/dio1/cs/txen/rxen)=%u/%u/%u/%u/%u/%u\r\n",
      power_present ? 1U : 0U,
      E28Sx1281_ModeName(radio.mode),
      radio.initialized ? 1U : 0U,
      GroundStation_IsRadioTxArmed() ? 1U : 0U,
      (unsigned long)GroundStation_RemainingMs(radio_tx_arm_deadline),
      (unsigned long)radio.frequency_hz,
      E28_SX1281_MIN_CHIP_POWER_DBM,
      radio.last_chip_status,
      radio.last_packet_type,
      (unsigned int)radio.last_irq,
      (unsigned long)radio.tx_count,
      (unsigned long)radio.rx_count,
      (unsigned long)radio.error_count,
      E28Sx1281_ErrorName(radio.last_error),
      HAL_GPIO_ReadPin(RADIO_RST_GPIO_Port, RADIO_RST_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_CS_GPIO_Port, RADIO_CS_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_TXEN_GPIO_Port, RADIO_TXEN_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_RXEN_GPIO_Port, RADIO_RXEN_Pin) == GPIO_PIN_SET ? 1U : 0U);
}

static void GroundStation_ProbeRadio(void)
{
  uint8_t chip_status = 0U;
  uint8_t packet_type = 0xFFU;
  HAL_StatusTypeDef result;

  radio_tx_armed = false;
  radio_tx_arm_deadline = 0U;
  pending_radio_tx.valid = false;
  result = E28Sx1281_Probe(&radio, &chip_status, &packet_type);
  GroundStation_Send(
      "GS radio probe power_present=%u hal=%u status=0x%02X packet_type=0x%02X "
      "expected_type=0x01 error=%s result=%s\r\n",
      radio.power_present ? 1U : 0U,
      (unsigned int)result,
      chip_status,
      packet_type,
      E28Sx1281_ErrorName(radio.last_error),
      result == HAL_OK ? "PASS" : "FAIL");
  if (result != HAL_OK)
  {
    E28Sx1281_ForceSafeReset(&radio);
  }
}

static HAL_StatusTypeDef GroundStation_InitializeRadio(void)
{
  HAL_StatusTypeDef result;

  radio_tx_armed = false;
  radio_tx_arm_deadline = 0U;
  pending_radio_tx.valid = false;
  result = E28Sx1281_Initialize(&radio);
  GroundStation_Send(
      "GS radio init power_present=%u hal=%u mode=%s freq_hz=%lu chip_power_dbm=%d "
      "tx_locked=1 error=%s result=%s\r\n",
      radio.power_present ? 1U : 0U,
      (unsigned int)result,
      E28Sx1281_ModeName(radio.mode),
      (unsigned long)radio.frequency_hz,
      E28_SX1281_MIN_CHIP_POWER_DBM,
      E28Sx1281_ErrorName(radio.last_error),
      result == HAL_OK ? "PASS" : "FAIL");
  return result;
}

static bool GroundStation_QueueRadioFrame(BalloonRadioType type,
                                          uint16_t sequence,
                                          const uint8_t *payload,
                                          uint8_t payload_length)
{
  uint8_t encoded[BALLOON_RADIO_MAX_FRAME_SIZE];
  uint8_t encoded_length = 0U;
  HAL_StatusTypeDef result;

  if (pending_radio_tx.valid || (radio.mode == E28_SX1281_MODE_TX))
  {
    return false;
  }
  if (!GroundStation_IsRadioTxArmed())
  {
    GroundStation_Send(
        "GS radio tx rejected reason=locked command=radio_arm_antenna\r\n");
    return false;
  }
  if (!BalloonRadio_Encode(type,
                           BALLOON_RADIO_SOURCE_GROUND,
                           sequence,
                           payload,
                           payload_length,
                           encoded,
                           sizeof(encoded),
                           &encoded_length))
  {
    GroundStation_Send("GS radio tx rejected reason=frame_encode\r\n");
    return false;
  }

  result = E28Sx1281_Send(&radio, encoded, encoded_length, true);
  if (result == HAL_OK)
  {
    pending_radio_tx.valid = true;
    pending_radio_tx.type = type;
    pending_radio_tx.sequence = sequence;
    pending_radio_tx.payload_length = payload_length;
    pending_radio_tx.frame_length = encoded_length;
    return true;
  }

  GroundStation_Send("GS radio tx failed type=%s seq=%u hal=%u error=%s\r\n",
                     BalloonRadio_TypeName(type),
                     (unsigned int)sequence,
                     (unsigned int)result,
                     E28Sx1281_ErrorName(radio.last_error));
  return false;
}

static void GroundStation_HandleRadioPacket(const E28Sx1281Packet *packet)
{
  BalloonRadioFrame frame;
  char text[BALLOON_RADIO_MAX_PAYLOAD + 1U];
  uint8_t index;

  if (!BalloonRadio_Decode(packet->data, packet->length, &frame))
  {
    GroundStation_LogEvent(GROUND_LOG_EVENT_PROTOCOL_ERROR,
                           (BalloonCommandCode)0,
                           (BalloonAckStage)0,
                           BALLOON_REJECT_BAD_PAYLOAD,
                           0U,
                           0U,
                           0U,
                           0U,
                           0U,
                           packet->rssi_dbm,
                           packet->snr_db);
    GroundStation_Send(
        "GS radio rx frame_len=%u rssi_dbm=%d snr_db=%d protocol=invalid\r\n",
        (unsigned int)packet->length,
        packet->rssi_dbm,
        packet->snr_db);
    return;
  }
  if (frame.source != BALLOON_RADIO_SOURCE_FLIGHT)
  {
    GroundStation_LogEvent(GROUND_LOG_EVENT_PROTOCOL_ERROR,
                           (BalloonCommandCode)0,
                           (BalloonAckStage)0,
                           BALLOON_REJECT_BAD_PAYLOAD,
                           frame.sequence,
                           0U,
                           0U,
                           0U,
                           0U,
                           packet->rssi_dbm,
                           packet->snr_db);
    GroundStation_Send(
        "GS radio rx source=%s protocol=source_rejected expected=flight\r\n",
        BalloonRadio_SourceName(frame.source));
    return;
  }

  if (frame.type == BALLOON_RADIO_TYPE_TELEMETRY)
  {
    BalloonTelemetryPayload telemetry;

    if (!BalloonRadio_DecodeTelemetryPayload(
            frame.payload, frame.payload_length, &telemetry))
    {
      GroundStation_LogEvent(GROUND_LOG_EVENT_PROTOCOL_ERROR,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_BAD_PAYLOAD,
                             frame.sequence,
                             0U,
                             0U,
                             0U,
                             0U,
                             packet->rssi_dbm,
                             packet->snr_db);
      GroundStation_Send(
          "GS telemetry seq=%u result=invalid_payload\r\n",
          (unsigned int)frame.sequence);
      return;
    }
    GroundStation_Send(
        "GS telemetry seq=%u telemetry_v=%u mission=%u mode=%s tick_ms=%lu fault=0x%04X "
        "vbat_mv=%u adc=%u imu=%02X/%u sd=%u link=%u "
        "action=%u ch=%u direction=%s value=%u remain_ms=%u radio_rx/tx/err=%u/%u/%u "
        "fc_log=%s rssi_dbm=%d snr_db=%d\r\n",
        (unsigned int)frame.sequence,
        (unsigned int)telemetry.payload_version,
        (unsigned int)telemetry.mission_id,
        BalloonRadio_SystemModeName(telemetry.system_mode),
        (unsigned long)telemetry.timestamp_ms,
        (unsigned int)telemetry.fault_bits,
        (unsigned int)telemetry.battery_mv,
        (unsigned int)telemetry.adc_raw,
        telemetry.imu_who_am_i,
        telemetry.imu_valid ? 1U : 0U,
        telemetry.sd_present ? 1U : 0U,
        telemetry.link_valid ? 1U : 0U,
        (unsigned int)telemetry.action,
        (unsigned int)telemetry.action_channel,
        telemetry.payload_version >= BALLOON_TELEMETRY_PAYLOAD_VERSION
            ? GroundStation_ActionDirection(telemetry.action,
                                            telemetry.action_reverse)
            : "unknown",
        (unsigned int)telemetry.action_value,
        (unsigned int)telemetry.action_remaining_ms,
        (unsigned int)telemetry.radio_rx_count,
        (unsigned int)telemetry.radio_tx_count,
        (unsigned int)telemetry.radio_error_count,
        GroundStation_TelemetryLogState(&telemetry),
        packet->rssi_dbm,
        packet->snr_db);
    GroundStation_LogTelemetry(&telemetry,
                               frame.sequence,
                               packet->rssi_dbm,
                               packet->snr_db);
    return;
  }

  if (frame.type == BALLOON_RADIO_TYPE_ACK)
  {
    BalloonAckPayload ack;

    if (!BalloonRadio_DecodeAckPayload(
            frame.payload, frame.payload_length, &ack))
    {
      GroundStation_LogEvent(GROUND_LOG_EVENT_PROTOCOL_ERROR,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_BAD_PAYLOAD,
                             frame.sequence,
                             0U,
                             0U,
                             0U,
                             0U,
                             packet->rssi_dbm,
                             packet->snr_db);
      GroundStation_Send("GS ack seq=%u result=invalid_payload\r\n",
                         (unsigned int)frame.sequence);
      return;
    }
    GroundStation_Send(
        "GS ack seq=%u command=%s stage=%s reason=%s mission=%u mode=%s "
        "action=%u ch=%u protocol=ok\r\n",
        (unsigned int)ack.command_sequence,
        BalloonRadio_CommandName(ack.command),
        BalloonRadio_AckStageName(ack.stage),
        BalloonRadio_RejectReasonName(ack.reason),
        (unsigned int)ack.mission_id,
        BalloonRadio_SystemModeName(ack.system_mode),
        (unsigned int)ack.action,
        (unsigned int)ack.channel);
    GroundStation_LogEvent(GROUND_LOG_EVENT_ACK_RX,
                           ack.command,
                           ack.stage,
                           ack.reason,
                           ack.command_sequence,
                           0U,
                           ack.channel,
                           0U,
                           0U,
                           packet->rssi_dbm,
                           packet->snr_db);
    return;
  }

  for (index = 0U; index < frame.payload_length; ++index)
  {
    uint8_t value = frame.payload[index];
    text[index] = (value >= 0x20U) && (value <= 0x7EU) ? (char)value : '.';
  }
  text[frame.payload_length] = '\0';
  GroundStation_Send(
      "GS radio rx type=%s source=%s seq=%u payload_len=%u "
      "rssi_dbm=%d snr_db=%d protocol=ok data=%s\r\n",
      BalloonRadio_TypeName(frame.type),
      BalloonRadio_SourceName(frame.source),
      (unsigned int)frame.sequence,
      (unsigned int)frame.payload_length,
      packet->rssi_dbm,
      packet->snr_db,
      frame.payload_length > 0U ? text : "<empty>");

  if (frame.type == BALLOON_RADIO_TYPE_PING)
  {
    if (GroundStation_IsRadioTxArmed())
    {
      (void)GroundStation_QueueRadioFrame(BALLOON_RADIO_TYPE_PONG,
                                          frame.sequence,
                                          frame.payload,
                                          frame.payload_length);
    }
    else
    {
      GroundStation_Send(
          "GS radio pong blocked reason=tx_locked; run=radio arm antenna\r\n");
    }
  }
}

static void GroundStation_ServiceRadio(void)
{
  E28Sx1281Packet packet;
  E28Sx1281Event event = E28Sx1281_Service(&radio, &packet);

  switch (event)
  {
    case E28_SX1281_EVENT_TX_DONE:
      if (pending_radio_tx.valid)
      {
        GroundStation_LogEvent(GROUND_LOG_EVENT_TX_DONE,
                               (BalloonCommandCode)0,
                               (BalloonAckStage)0,
                               BALLOON_REJECT_NONE,
                               pending_radio_tx.sequence,
                               0U,
                               0U,
                               0U,
                               0U,
                               0,
                               0);
        GroundStation_Send(
            "GS radio tx_done type=%s seq=%u payload_len=%u frame_len=%u "
            "mode=%s count=%lu\r\n",
            BalloonRadio_TypeName(pending_radio_tx.type),
            (unsigned int)pending_radio_tx.sequence,
            (unsigned int)pending_radio_tx.payload_length,
            (unsigned int)pending_radio_tx.frame_length,
            E28Sx1281_ModeName(radio.mode),
            (unsigned long)radio.tx_count);
      }
      else
      {
        GroundStation_Send("GS radio tx_done mode=%s count=%lu\r\n",
                           E28Sx1281_ModeName(radio.mode),
                           (unsigned long)radio.tx_count);
      }
      pending_radio_tx.valid = false;
      break;
    case E28_SX1281_EVENT_TX_TIMEOUT:
      GroundStation_LogEvent(GROUND_LOG_EVENT_TX_TIMEOUT,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_HARDWARE,
                             pending_radio_tx.sequence,
                             0U,
                             0U,
                             0U,
                             0U,
                             0,
                             0);
      pending_radio_tx.valid = false;
      radio_tx_armed = false;
      radio_tx_arm_deadline = 0U;
      mission_active = false;
      GroundStation_StopMissionLogging();
      GroundStation_Send("GS radio tx_timeout mode=%s result=FAIL\r\n",
                         E28Sx1281_ModeName(radio.mode));
      break;
    case E28_SX1281_EVENT_RX_DONE:
      GroundStation_HandleRadioPacket(&packet);
      break;
    case E28_SX1281_EVENT_RX_CRC_ERROR:
      GroundStation_LogEvent(GROUND_LOG_EVENT_PROTOCOL_ERROR,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_BAD_PAYLOAD,
                             0U,
                             0U,
                             0U,
                             0U,
                             0U,
                             0,
                             0);
      GroundStation_Send("GS radio rx_error type=crc restarted=1 mode=%s\r\n",
                         E28Sx1281_ModeName(radio.mode));
      break;
    case E28_SX1281_EVENT_RX_HEADER_ERROR:
      GroundStation_LogEvent(GROUND_LOG_EVENT_PROTOCOL_ERROR,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_BAD_PAYLOAD,
                             0U,
                             0U,
                             0U,
                             0U,
                             0U,
                             0,
                             0);
      GroundStation_Send(
          "GS radio rx_error type=header restarted=1 mode=%s\r\n",
          E28Sx1281_ModeName(radio.mode));
      break;
    case E28_SX1281_EVENT_ERROR:
      GroundStation_LogEvent(GROUND_LOG_EVENT_RADIO_ERROR,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_HARDWARE,
                             pending_radio_tx.sequence,
                             0U,
                             0U,
                             0U,
                             0U,
                             0,
                             0);
      pending_radio_tx.valid = false;
      radio_tx_armed = false;
      radio_tx_arm_deadline = 0U;
      mission_active = false;
      GroundStation_StopMissionLogging();
      GroundStation_Send("GS radio service error=%s mode=%s safe_reset=1\r\n",
                         E28Sx1281_ErrorName(radio.last_error),
                         E28Sx1281_ModeName(radio.mode));
      break;
    default:
      break;
  }
}

static HAL_StatusTypeDef GroundStation_ReadSupply(uint32_t *raw, uint32_t *supply_mv)
{
  HAL_StatusTypeDef result;

  if ((raw == NULL) || (supply_mv == NULL))
  {
    return HAL_ERROR;
  }

  result = HAL_ADC_Start(&hadc1);
  if (result == HAL_OK)
  {
    result = HAL_ADC_PollForConversion(&hadc1, 10U);
  }
  if (result == HAL_OK)
  {
    *raw = HAL_ADC_GetValue(&hadc1);
    *supply_mv = (*raw * ADC_REFERENCE_MV * SUPPLY_DIVIDER_MULTIPLIER) /
                 ADC_FULL_SCALE;
  }
  (void)HAL_ADC_Stop(&hadc1);
  return result;
}

static HAL_StatusTypeDef GroundStation_ReadFlashJedecId(uint8_t jedec_id[3])
{
  uint8_t command = W25Q_READ_JEDEC_ID_COMMAND;
  HAL_StatusTypeDef result;

  if (jedec_id == NULL)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(&hspi1, &command, 1U, 100U);
  if (result == HAL_OK)
  {
    result = HAL_SPI_Receive(&hspi1, jedec_id, 3U, 100U);
  }
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  return result;
}

static bool GroundStation_IsSdCardPresent(void)
{
  return BSP_PlatformIsDetected() == SD_PRESENT;
}

static const char *GroundStation_ActionDirection(uint8_t action, bool reverse)
{
  if ((action != 2U) && (action != 3U))
  {
    return "na";
  }
  return reverse ? "reverse" : "forward";
}

static const char *GroundStation_TelemetryLogState(
    const BalloonTelemetryPayload *telemetry)
{
  if ((telemetry == NULL) ||
      (telemetry->payload_version < BALLOON_TELEMETRY_PAYLOAD_VERSION))
  {
    return "unknown";
  }
  switch ((BalloonLogState)telemetry->log_state)
  {
    case BALLOON_LOG_STATE_OFF:
      return "off";
    case BALLOON_LOG_STATE_ACTIVE:
      return "active";
    case BALLOON_LOG_STATE_ERROR:
      return "error";
    default:
      return "invalid";
  }
}

static const char *GroundStation_LogEventName(uint8_t event)
{
  switch ((GroundStationLogEventKind)event)
  {
    case GROUND_LOG_EVENT_MISSION_START:
      return "MISSION_START";
    case GROUND_LOG_EVENT_MISSION_STOP:
      return "MISSION_STOP";
    case GROUND_LOG_EVENT_COMMAND_QUEUED:
      return "COMMAND_QUEUED";
    case GROUND_LOG_EVENT_TX_DONE:
      return "TX_DONE";
    case GROUND_LOG_EVENT_TX_TIMEOUT:
      return "TX_TIMEOUT";
    case GROUND_LOG_EVENT_ACK_RX:
      return "ACK_RX";
    case GROUND_LOG_EVENT_PROTOCOL_ERROR:
      return "PROTOCOL_ERROR";
    case GROUND_LOG_EVENT_RADIO_ERROR:
      return "RADIO_ERROR";
    default:
      return "UNKNOWN";
  }
}

static bool GroundStation_QueueLogRecord(const GroundStationLogRecord *record,
                                         bool periodic)
{
  if ((record == NULL) || (mission_log_queue == NULL) ||
      !mission_log_owns_sd)
  {
    return false;
  }
  if (periodic &&
      (uxQueueSpacesAvailable(mission_log_queue) <=
       MISSION_LOG_RESERVED_SLOTS))
  {
    ++mission_log_drop_count;
    mission_log_state = BALLOON_LOG_STATE_ERROR;
    mission_log_last_result = FR_INT_ERR;
    return false;
  }
  if (xQueueSend(mission_log_queue, record, 0U) != pdPASS)
  {
    ++mission_log_drop_count;
    mission_log_state = BALLOON_LOG_STATE_ERROR;
    mission_log_last_result = FR_INT_ERR;
    return false;
  }
  return true;
}

static void GroundStation_LogEvent(GroundStationLogEventKind event,
                                   BalloonCommandCode command,
                                   BalloonAckStage stage,
                                   BalloonRejectReason reason,
                                   uint16_t sequence,
                                   uint8_t flags,
                                   uint8_t channel,
                                   uint16_t value,
                                   uint16_t duration_ms,
                                   int8_t rssi_dbm,
                                   int8_t snr_db)
{
  GroundStationLogRecord record = {0};

  if (!mission_log_owns_sd)
  {
    return;
  }
  record.kind = GROUND_LOG_EVENT;
  record.timestamp_ms = HAL_GetTick();
  record.payload.event.event = (uint8_t)event;
  record.payload.event.command = (uint8_t)command;
  record.payload.event.stage = (uint8_t)stage;
  record.payload.event.reason = (uint8_t)reason;
  record.payload.event.flags = flags;
  record.payload.event.channel = channel;
  record.payload.event.sequence = sequence;
  record.payload.event.value = value;
  record.payload.event.duration_ms = duration_ms;
  record.payload.event.rssi_dbm = rssi_dbm;
  record.payload.event.snr_db = snr_db;
  (void)GroundStation_QueueLogRecord(&record, false);
}

static void GroundStation_LogTelemetry(const BalloonTelemetryPayload *telemetry,
                                       uint16_t frame_sequence,
                                       int8_t rssi_dbm,
                                       int8_t snr_db)
{
  GroundStationLogRecord record = {0};

  if ((telemetry == NULL) || !mission_log_owns_sd)
  {
    return;
  }
  record.kind = GROUND_LOG_DATA;
  record.timestamp_ms = HAL_GetTick();
  record.payload.data.telemetry = *telemetry;
  record.payload.data.frame_sequence = frame_sequence;
  record.payload.data.rssi_dbm = rssi_dbm;
  record.payload.data.snr_db = snr_db;
  (void)GroundStation_QueueLogRecord(&record, true);
}

static bool GroundStation_FindLogPairIndex(unsigned int *sequence)
{
  FILINFO info;
  char data_path[BALLOON_CSV_LOGGER_PATH_SIZE];
  char event_path[BALLOON_CSV_LOGGER_PATH_SIZE];
  unsigned int index;

  if (sequence == NULL)
  {
    mission_log_last_result = FR_INVALID_PARAMETER;
    return false;
  }
  for (index = 1U; index <= 9999U; ++index)
  {
    FRESULT data_result;
    FRESULT event_result;
    int data_length = snprintf(data_path,
                               sizeof(data_path),
                               "%sGSD%04u.CSV",
                               SDPath,
                               index);
    int event_length = snprintf(event_path,
                                sizeof(event_path),
                                "%sGSE%04u.CSV",
                                SDPath,
                                index);

    if ((data_length <= 0) || ((size_t)data_length >= sizeof(data_path)) ||
        (event_length <= 0) || ((size_t)event_length >= sizeof(event_path)))
    {
      mission_log_last_result = FR_INVALID_NAME;
      return false;
    }
    data_result = f_stat(data_path, &info);
    event_result = f_stat(event_path, &info);
    if ((data_result == FR_NO_FILE) && (event_result == FR_NO_FILE))
    {
      *sequence = index;
      return true;
    }
    if (((data_result != FR_OK) && (data_result != FR_NO_FILE)) ||
        ((event_result != FR_OK) && (event_result != FR_NO_FILE)))
    {
      mission_log_last_result = data_result != FR_OK ? data_result
                                                      : event_result;
      return false;
    }
  }
  mission_log_last_result = FR_EXIST;
  return false;
}

static void GroundStation_CloseLogFiles(bool normal_stop)
{
  FRESULT first_result = FR_OK;
  FRESULT session_result = mission_log_last_result;
  bool session_failed = (mission_log_state == BALLOON_LOG_STATE_ERROR) ||
                        (mission_log_drop_count != 0U) ||
                        (session_result != FR_OK);

  if (mission_data_log.opened && !BalloonCsvLogger_Close(&mission_data_log))
  {
    first_result = mission_data_log.last_result;
  }
  if (mission_event_log.opened && !BalloonCsvLogger_Close(&mission_event_log) &&
      (first_result == FR_OK))
  {
    first_result = mission_event_log.last_result;
  }
  if (mission_log_mounted)
  {
    FRESULT unmount_result = f_mount(NULL, (TCHAR const *)SDPath, 0U);

    if ((first_result == FR_OK) && (unmount_result != FR_OK))
    {
      first_result = unmount_result;
    }
  }
  mission_log_mounted = false;
  if ((first_result == FR_OK) && (session_result != FR_OK))
  {
    first_result = session_result;
  }
  mission_log_last_result = first_result;
  if (normal_stop)
  {
    mission_log_state = (!session_failed && (first_result == FR_OK))
                            ? BALLOON_LOG_STATE_OFF
                            : BALLOON_LOG_STATE_ERROR;
    mission_log_owns_sd = false;
    mission_log_stop_requested = false;
  }
  else
  {
    mission_log_state = BALLOON_LOG_STATE_ERROR;
  }
}

static void GroundStation_LoggerTask(void *argument)
{
  GroundStationLogRecord record;

  (void)argument;
  BalloonCsvLogger_Reset(&mission_data_log);
  BalloonCsvLogger_Reset(&mission_event_log);
  for (;;)
  {
    bool received = xQueueReceive(mission_log_queue,
                                  &record,
                                  pdMS_TO_TICKS(100U)) == pdPASS;

    if (received && (record.kind == GROUND_LOG_CONTROL_START))
    {
      unsigned int file_sequence = 0U;

      mission_data_log_sequence = 0U;
      mission_event_log_sequence = 0U;
      mission_log_last_result = FR_NOT_READY;
      if (!GroundStation_IsSdCardPresent() || (retSD != 0U))
      {
        mission_log_state = BALLOON_LOG_STATE_ERROR;
      }
      else
      {
        mission_log_last_result =
            f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
        mission_log_mounted = mission_log_last_result == FR_OK;
        if (mission_log_mounted &&
            GroundStation_FindLogPairIndex(&file_sequence) &&
            BalloonCsvLogger_OpenAt(
                &mission_data_log,
                SDPath,
                "GSD",
                file_sequence,
                "schema,record_seq,gs_tick_ms,frame_seq,telemetry_version,fc_tick_ms,mission_id,mode,fault_bits,battery_mv,adc_raw,imu_who_am_i,imu_valid,sd_present,link_valid,action,action_channel,direction_requested,action_value,action_remaining_ms,fc_radio_rx_count,fc_radio_tx_count,fc_radio_error_count,fc_log_state,rssi_dbm,snr_db\r\n",
                record.timestamp_ms) &&
            BalloonCsvLogger_OpenAt(
                &mission_event_log,
                SDPath,
                "GSE",
                file_sequence,
                "schema,record_seq,gs_tick_ms,event,command,stage,reason,sequence,flags,channel,direction_requested,value,duration_ms,rssi_dbm,snr_db\r\n",
                record.timestamp_ms))
        {
          mission_log_last_result = FR_OK;
          mission_log_state = mission_log_drop_count == 0U
                                  ? BALLOON_LOG_STATE_ACTIVE
                                  : BALLOON_LOG_STATE_ERROR;
        }
        else
        {
          if (mission_log_last_result == FR_OK)
          {
            mission_log_last_result = mission_event_log.last_result != FR_OK
                                          ? mission_event_log.last_result
                                          : mission_data_log.last_result;
          }
          GroundStation_CloseLogFiles(false);
        }
      }
    }
    else if (received && (record.kind == GROUND_LOG_DATA) &&
             mission_data_log.opened)
    {
      const GroundStationLogData *data = &record.payload.data;
      const BalloonTelemetryPayload *telemetry = &data->telemetry;
      int length;

      ++mission_data_log_sequence;
      length = snprintf(
          mission_csv_line,
          sizeof(mission_csv_line),
          "1,%lu,%lu,%u,%u,%lu,%u,%s,0x%04X,%u,%u,%u,%u,%u,%u,%u,%u,%s,%u,%u,%u,%u,%u,%s,%d,%d\r\n",
          (unsigned long)mission_data_log_sequence,
          (unsigned long)record.timestamp_ms,
          (unsigned int)data->frame_sequence,
          (unsigned int)telemetry->payload_version,
          (unsigned long)telemetry->timestamp_ms,
          (unsigned int)telemetry->mission_id,
          BalloonRadio_SystemModeName(telemetry->system_mode),
          (unsigned int)telemetry->fault_bits,
          (unsigned int)telemetry->battery_mv,
          (unsigned int)telemetry->adc_raw,
          (unsigned int)telemetry->imu_who_am_i,
          telemetry->imu_valid ? 1U : 0U,
          telemetry->sd_present ? 1U : 0U,
          telemetry->link_valid ? 1U : 0U,
          (unsigned int)telemetry->action,
          (unsigned int)telemetry->action_channel,
          telemetry->payload_version >= BALLOON_TELEMETRY_PAYLOAD_VERSION
              ? GroundStation_ActionDirection(telemetry->action,
                                              telemetry->action_reverse)
              : "unknown",
          (unsigned int)telemetry->action_value,
          (unsigned int)telemetry->action_remaining_ms,
          (unsigned int)telemetry->radio_rx_count,
          (unsigned int)telemetry->radio_tx_count,
          (unsigned int)telemetry->radio_error_count,
          GroundStation_TelemetryLogState(telemetry),
          data->rssi_dbm,
          data->snr_db);
      if ((length <= 0) || ((size_t)length >= sizeof(mission_csv_line)) ||
          !BalloonCsvLogger_Write(&mission_data_log,
                                  mission_csv_line,
                                  (size_t)length))
      {
        mission_log_last_result = length <= 0 ? FR_INT_ERR
                                               : mission_data_log.last_result;
        GroundStation_CloseLogFiles(false);
      }
    }
    else if (received && (record.kind == GROUND_LOG_EVENT) &&
             mission_event_log.opened)
    {
      const GroundStationLogEvent *event = &record.payload.event;
      int length;

      ++mission_event_log_sequence;
      length = snprintf(
          mission_csv_line,
          sizeof(mission_csv_line),
          "1,%lu,%lu,%s,%s,%s,%s,%u,0x%02X,%u,%s,%u,%u,%d,%d\r\n",
          (unsigned long)mission_event_log_sequence,
          (unsigned long)record.timestamp_ms,
          GroundStation_LogEventName(event->event),
          event->command == 0U
              ? "none"
              : BalloonRadio_CommandName((BalloonCommandCode)event->command),
          event->event == (uint8_t)GROUND_LOG_EVENT_ACK_RX
              ? BalloonRadio_AckStageName((BalloonAckStage)event->stage)
              : "none",
          BalloonRadio_RejectReasonName((BalloonRejectReason)event->reason),
          (unsigned int)event->sequence,
          (unsigned int)event->flags,
          (unsigned int)event->channel,
          event->event == (uint8_t)GROUND_LOG_EVENT_COMMAND_QUEUED
              ? GroundStation_ActionDirection(
                    (event->command == (uint8_t)BALLOON_COMMAND_PUMP) ? 2U :
                    (event->command == (uint8_t)BALLOON_COMMAND_MOTOR) ? 3U : 0U,
                    (event->flags & 0x01U) != 0U)
              : "na",
          (unsigned int)event->value,
          (unsigned int)event->duration_ms,
          event->rssi_dbm,
          event->snr_db);
      if ((length <= 0) || ((size_t)length >= sizeof(mission_csv_line)) ||
          !BalloonCsvLogger_Write(&mission_event_log,
                                  mission_csv_line,
                                  (size_t)length))
      {
        mission_log_last_result = length <= 0 ? FR_INT_ERR
                                               : mission_event_log.last_result;
        GroundStation_CloseLogFiles(false);
      }
    }

    if (mission_data_log.opened &&
        (!BalloonCsvLogger_Service(&mission_data_log,
                                   HAL_GetTick(),
                                   MISSION_LOG_SYNC_PERIOD_MS) ||
         !BalloonCsvLogger_Service(&mission_event_log,
                                   HAL_GetTick(),
                                   MISSION_LOG_SYNC_PERIOD_MS)))
    {
      mission_log_last_result = mission_data_log.last_result != FR_OK
                                    ? mission_data_log.last_result
                                    : mission_event_log.last_result;
      GroundStation_CloseLogFiles(false);
    }

    if ((received && (record.kind == GROUND_LOG_CONTROL_STOP)) ||
        (mission_log_stop_requested &&
         (uxQueueMessagesWaiting(mission_log_queue) == 0U)))
    {
      GroundStation_CloseLogFiles(true);
    }
  }
}

static void GroundStation_StartMissionLogging(void)
{
  GroundStationLogRecord record = {0};

  if ((mission_log_queue == NULL) || (mission_log_task_handle == NULL))
  {
    mission_log_state = BALLOON_LOG_STATE_ERROR;
    mission_log_last_result = FR_NOT_ENABLED;
    return;
  }
  mission_log_state = BALLOON_LOG_STATE_OFF;
  mission_log_drop_count = 0U;
  mission_log_stop_requested = false;
  mission_log_owns_sd = true;
  record.kind = GROUND_LOG_CONTROL_START;
  record.timestamp_ms = HAL_GetTick();
  if (xQueueSend(mission_log_queue, &record, 0U) != pdPASS)
  {
    mission_log_state = BALLOON_LOG_STATE_ERROR;
    mission_log_last_result = FR_NOT_ENOUGH_CORE;
    mission_log_owns_sd = false;
  }
}

static void GroundStation_StopMissionLogging(void)
{
  GroundStationLogRecord record = {0};

  if (!mission_log_owns_sd)
  {
    return;
  }
  record.kind = GROUND_LOG_CONTROL_STOP;
  record.timestamp_ms = HAL_GetTick();
  mission_log_stop_requested = true;
  (void)xQueueSend(mission_log_queue, &record, 0U);
}

static void GroundStation_UpdateSdState(GroundStationStatus *status)
{
  uint32_t now = HAL_GetTick();
  bool raw_present = BSP_PlatformRawDetectLevel() == 0U;
  bool card_present;

  if (mission_log_owns_sd)
  {
    status->sd_result = mission_log_last_result;
    status->sd_mounted = mission_log_mounted;
    status->sd_last_present = GroundStation_IsSdCardPresent();
    status->sd_last_raw_present = raw_present;
    status->sd_logger_was_owner = true;
    return;
  }

  if (status->sd_logger_was_owner)
  {
    status->sd_logger_was_owner = false;
    status->sd_mounted = false;
    status->sd_result = FR_NOT_READY;
    status->sd_next_retry_tick = now;
  }

  if (status->sd_last_raw_present && !raw_present)
  {
    if (status->sd_mounted)
    {
      (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
      status->sd_mounted = false;
    }
    status->sd_result = FR_NOT_READY;
    status->sd_next_retry_tick = now;
    status->sd_auto_success_reported = false;
  }
  status->sd_last_raw_present = raw_present;

  if (raw_present && status->sd_auto_fallback_active)
  {
    BSP_PlatformSetForcePresent(0U);
    status->sd_auto_fallback_active = false;
    status->sd_auto_success_reported = false;
  }
  else if (!raw_present && status->sd_auto_retry_enabled)
  {
    if (BSP_PlatformIsForcePresentEnabled() == 0U)
    {
      sd_diagnostic_bus_width = SDIO_BUS_WIDE_4B;
      BSP_PlatformSetForcePresent(1U);
      status->sd_next_retry_tick = now;
    }
    status->sd_auto_fallback_active = true;
  }

  card_present = GroundStation_IsSdCardPresent();
  if (status->sd_last_present != card_present)
  {
    status->sd_last_present = card_present;
    status->sd_next_retry_tick = now;
  }

  if (card_present && !status->sd_mounted &&
      ((int32_t)(now - status->sd_next_retry_tick) >= 0))
  {
    bool was_fault_active = status->sd_fault_active;

    if (status->sd_auto_fallback_active)
    {
      sd_diagnostic_bus_width = SDIO_BUS_WIDE_4B;
    }
    status->sd_result = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    status->sd_mounted = status->sd_result == FR_OK;
    status->sd_next_retry_tick = now + SD_MOUNT_RETRY_PERIOD_MS;
    if (status->sd_mounted)
    {
      if (status->sd_auto_fallback_active &&
          (!status->sd_auto_success_reported || was_fault_active))
      {
        GroundStation_Send(
            "GS sd auto_fallback raw_cd=%u force=%u mounted=1 fresult=0 result=%s\r\n",
            (unsigned int)BSP_PlatformRawDetectLevel(),
            (unsigned int)BSP_PlatformIsForcePresentEnabled(),
            was_fault_active ? "RECOVERED" : "PASS");
      }
      status->sd_auto_success_reported = status->sd_auto_fallback_active;
      status->sd_fault_active = false;
      status->sd_next_warning_tick = now;
    }
    else if (status->sd_auto_fallback_active)
    {
      if (!status->sd_fault_active ||
          ((int32_t)(now - status->sd_next_warning_tick) >= 0))
      {
        GroundStation_Send(
            "GS warning sd auto_fallback raw_cd=%u force=%u mounted=0 fresult=%u retry_ms=%lu\r\n",
            (unsigned int)BSP_PlatformRawDetectLevel(),
            (unsigned int)BSP_PlatformIsForcePresentEnabled(),
            (unsigned int)status->sd_result,
            (unsigned long)SD_MOUNT_RETRY_PERIOD_MS);
        status->sd_next_warning_tick = now + SD_WARNING_PERIOD_MS;
      }
      status->sd_fault_active = true;
    }
  }
  else if (!card_present && status->sd_mounted)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    status->sd_result = FR_NOT_READY;
    status->sd_mounted = false;
  }
  else if (!card_present)
  {
    status->sd_result = FR_NOT_READY;
    status->sd_fault_active = false;
  }
}

static void GroundStation_SetSdForcePresent(GroundStationStatus *status,
                                            bool enabled)
{
  if (mission_log_owns_sd)
  {
    GroundStation_Send(
        "GS sd force result=rejected reason=logger_owns_sd command=mission_stop_first\r\n");
    return;
  }

  if (status->sd_mounted)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
  }

  sd_diagnostic_bus_width = SDIO_BUS_WIDE_4B;
  BSP_PlatformSetForcePresent(enabled ? 1U : 0U);
  status->sd_result = FR_NOT_READY;
  status->sd_mounted = false;
  status->sd_auto_retry_enabled = enabled;
  status->sd_auto_fallback_active =
      enabled && (BSP_PlatformRawDetectLevel() != 0U);
  status->sd_auto_success_reported = false;
  status->sd_fault_active = false;
  status->sd_last_present = GroundStation_IsSdCardPresent();
  status->sd_last_raw_present = BSP_PlatformRawDetectLevel() == 0U;
  status->sd_logger_was_owner = false;
  status->sd_next_retry_tick = HAL_GetTick();
  status->sd_next_warning_tick = HAL_GetTick();
  GroundStation_UpdateSdState(status);

  GroundStation_Send(
      "GS sd force=%u auto_retry=%u raw_cd=%u effective_present=%u mounted=%u fresult=%u\r\n",
      (unsigned int)BSP_PlatformIsForcePresentEnabled(),
      status->sd_auto_retry_enabled ? 1U : 0U,
      (unsigned int)BSP_PlatformRawDetectLevel(),
      GroundStation_IsSdCardPresent() ? 1U : 0U,
      status->sd_mounted ? 1U : 0U,
      (unsigned int)status->sd_result);
}

static uint16_t GroundStation_ReadLe16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t GroundStation_ReadLe32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static bool GroundStation_PrepareSdDiagnostic(GroundStationStatus *status,
                                              uint8_t bus_width,
                                              DSTATUS *initialize_status,
                                              HAL_StatusTypeDef *bus_result)
{
  if (mission_log_owns_sd)
  {
    GroundStation_Send(
        "GS sd diagnostic result=rejected reason=logger_owns_sd command=mission_stop_first\r\n");
    return false;
  }
  if (mission_active)
  {
    GroundStation_Send(
        "GS sd diagnostic result=rejected reason=mission_active command=mission_stop_first\r\n");
    return false;
  }
  if (BSP_PlatformIsForcePresentEnabled() == 0U)
  {
    GroundStation_Send(
        "GS sd diagnostic result=rejected reason=detect_disconnected command=sd_force_on_first\r\n");
    return false;
  }
  if ((bus_width != 1U) && (bus_width != 4U))
  {
    GroundStation_Send("GS sd diagnostic result=rejected reason=bus_width\r\n");
    return false;
  }

  if (status->sd_mounted)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
  }
  status->sd_mounted = false;
  status->sd_result = FR_NOT_READY;
  status->sd_next_retry_tick = HAL_GetTick() + SD_MOUNT_RETRY_PERIOD_MS;

  sd_diagnostic_bus_width = bus_width == 1U
                                ? SDIO_BUS_WIDE_1B
                                : SDIO_BUS_WIDE_4B;
  *initialize_status = SD_Driver.disk_initialize(0U);
  if ((*initialize_status & STA_NOINIT) != 0U)
  {
    *bus_result = HAL_ERROR;
    return false;
  }

  *bus_result = HAL_OK;
  return true;
}

static void GroundStation_PrintSdBootSector(const char *label,
                                            uint32_t lba,
                                            const uint8_t *sector)
{
  char oem[9];
  uint32_t index;

  for (index = 0U; index < 8U; ++index)
  {
    uint8_t value = sector[3U + index];
    oem[index] = ((value >= 0x20U) && (value <= 0x7EU)) ? (char)value : '.';
  }
  oem[8] = '\0';

  GroundStation_Send(
      "GS sd raw %s_lba=%lu jump=%02X%02X%02X oem='%s' bps=%u spc=%u "
      "reserved=%u fats=%u root_entries=%u total16=%u total32=%lu "
      "fat16=%u fat32=%lu sig=%02X%02X\r\n",
      label,
      (unsigned long)lba,
      sector[0],
      sector[1],
      sector[2],
      oem,
      (unsigned int)GroundStation_ReadLe16(&sector[11]),
      (unsigned int)sector[13],
      (unsigned int)GroundStation_ReadLe16(&sector[14]),
      (unsigned int)sector[16],
      (unsigned int)GroundStation_ReadLe16(&sector[17]),
      (unsigned int)GroundStation_ReadLe16(&sector[19]),
      (unsigned long)GroundStation_ReadLe32(&sector[32]),
      (unsigned int)GroundStation_ReadLe16(&sector[22]),
      (unsigned long)GroundStation_ReadLe32(&sector[36]),
      sector[510],
      sector[511]);
}

static void GroundStation_RunSdRawDiagnostic(GroundStationStatus *status,
                                             uint8_t bus_width)
{
  HAL_SD_CardInfoTypeDef card_info = {0};
  DSTATUS initialize_status = STA_NOINIT;
  DSTATUS disk_status = STA_NOINIT;
  DRESULT read_result = RES_NOTRDY;
  HAL_StatusTypeDef bus_result = HAL_ERROR;
  uint8_t *sector = (uint8_t *)sd_raw_sector_words;
  uint32_t first_partition_lba = 0U;
  uint32_t first_partition_sectors = 0U;
  uint8_t first_partition_type = 0U;
  uint32_t nonzero_bytes = 0U;
  uint32_t index;

  if (!GroundStation_PrepareSdDiagnostic(status,
                                         bus_width,
                                         &initialize_status,
                                         &bus_result))
  {
    GroundStation_Send(
        "GS sd raw bus=%u init=0x%02X bus_hal=%u hal_state=%u hal_error=0x%08lX result=prepare_failed\r\n",
        (unsigned int)bus_width,
        (unsigned int)initialize_status,
        (unsigned int)bus_result,
        (unsigned int)HAL_SD_GetState(&hsd),
        (unsigned long)HAL_SD_GetError(&hsd));
    return;
  }

  disk_status = SD_Driver.disk_status(0U);
  BSP_SD_GetCardInfo(&card_info);
  memset(sector, 0, SD_RAW_SECTOR_SIZE);
  read_result = SD_Driver.disk_read(0U, sector, 0U, 1U);
  for (index = 0U; index < SD_RAW_SECTOR_SIZE; ++index)
  {
    if (sector[index] != 0U)
    {
      ++nonzero_bytes;
    }
  }

  GroundStation_Send(
      "GS sd raw bus=%u init=0x%02X status=0x%02X bus_hal=%u read0=%u "
      "hal_state=%u card_state=%u hal_error=0x%08lX type=%lu version=%lu "
      "blocks=%lu block_size=%lu capacity_mib=%lu\r\n",
      (unsigned int)bus_width,
      (unsigned int)initialize_status,
      (unsigned int)disk_status,
      (unsigned int)bus_result,
      (unsigned int)read_result,
      (unsigned int)HAL_SD_GetState(&hsd),
      (unsigned int)HAL_SD_GetCardState(&hsd),
      (unsigned long)HAL_SD_GetError(&hsd),
      (unsigned long)card_info.CardType,
      (unsigned long)card_info.CardVersion,
      (unsigned long)card_info.LogBlockNbr,
      (unsigned long)card_info.LogBlockSize,
      (unsigned long)(card_info.LogBlockNbr / 2048U));

  if (read_result != RES_OK)
  {
    sd_diagnostic_bus_width = SDIO_BUS_WIDE_4B;
    return;
  }

  GroundStation_Send(
      "GS sd raw sector0 nonzero=%lu sig=%02X%02X first16="
      "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
      (unsigned long)nonzero_bytes,
      sector[510],
      sector[511],
      sector[0], sector[1], sector[2], sector[3],
      sector[4], sector[5], sector[6], sector[7],
      sector[8], sector[9], sector[10], sector[11],
      sector[12], sector[13], sector[14], sector[15]);

  for (index = 0U; index < 4U; ++index)
  {
    const uint8_t *partition = &sector[446U + (index * 16U)];
    uint32_t start_lba = GroundStation_ReadLe32(&partition[8]);
    uint32_t sector_count = GroundStation_ReadLe32(&partition[12]);

    if ((partition[4] != 0U) || (start_lba != 0U) || (sector_count != 0U))
    {
      GroundStation_Send(
          "GS sd raw partition=%lu boot=0x%02X type=0x%02X start_lba=%lu sectors=%lu\r\n",
          (unsigned long)(index + 1U),
          partition[0],
          partition[4],
          (unsigned long)start_lba,
          (unsigned long)sector_count);
      if ((first_partition_lba == 0U) && (partition[4] != 0U) &&
          (start_lba != 0U) && (sector_count != 0U) &&
          (start_lba < card_info.LogBlockNbr) &&
          (sector_count <= (card_info.LogBlockNbr - start_lba)))
      {
        first_partition_lba = start_lba;
        first_partition_sectors = sector_count;
        first_partition_type = partition[4];
      }
    }
  }

  if (first_partition_type == 0xEEU)
  {
    GroundStation_Send(
        "GS sd raw layout=gpt_protective_mbr result=unsupported_by_current_fatfs\r\n");
  }
  else if (first_partition_lba != 0U)
  {
    memset(sector, 0, SD_RAW_SECTOR_SIZE);
    read_result = SD_Driver.disk_read(0U, sector, first_partition_lba, 1U);
    GroundStation_Send(
        "GS sd raw partition_read lba=%lu sectors=%lu result=%u\r\n",
        (unsigned long)first_partition_lba,
        (unsigned long)first_partition_sectors,
        (unsigned int)read_result);
    if (read_result == RES_OK)
    {
      GroundStation_PrintSdBootSector("volume", first_partition_lba, sector);
    }
  }
  else if ((sector[0] == 0xEBU) || (sector[0] == 0xE9U))
  {
    GroundStation_PrintSdBootSector("volume", 0U, sector);
  }
  else
  {
    GroundStation_Send("GS sd raw layout=unknown_no_valid_partition\r\n");
  }
  sd_diagnostic_bus_width = SDIO_BUS_WIDE_4B;
}

static void GroundStation_MountSdWithBus(GroundStationStatus *status,
                                         uint8_t bus_width)
{
  DSTATUS initialize_status = STA_NOINIT;
  HAL_StatusTypeDef bus_result = HAL_ERROR;

  if (!GroundStation_PrepareSdDiagnostic(status,
                                         bus_width,
                                         &initialize_status,
                                         &bus_result))
  {
    GroundStation_Send(
        "GS sd mount bus=%u init=0x%02X bus_hal=%u mounted=0 fresult=%u result=prepare_failed\r\n",
        (unsigned int)bus_width,
        (unsigned int)initialize_status,
        (unsigned int)bus_result,
        (unsigned int)status->sd_result);
    return;
  }

  status->sd_result = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  status->sd_mounted = status->sd_result == FR_OK;
  GroundStation_Send(
      "GS sd mount bus=%u init=0x%02X bus_hal=%u mounted=%u fresult=%u\r\n",
      (unsigned int)bus_width,
      (unsigned int)initialize_status,
      (unsigned int)bus_result,
      status->sd_mounted ? 1U : 0U,
      (unsigned int)status->sd_result);
}

static void GroundStation_RunSdWriteReadTest(GroundStationStatus *status)
{
  char file_path[BALLOON_CSV_LOGGER_PATH_SIZE];
  UINT bytes_written = 0U;
  UINT bytes_read = 0U;
  int expected_length;
  int path_length;
  FRESULT result;
  FRESULT close_result;
  FRESULT unlink_result = FR_OK;

  if (mission_log_owns_sd)
  {
    GroundStation_Send(
        "GS sdtest result=rejected reason=logger_owns_sd command=mission_stop_first\r\n");
    return;
  }
  GroundStation_UpdateSdState(status);
  if (!GroundStation_IsSdCardPresent())
  {
    GroundStation_Send("GS sdtest present=0 result=no_card\r\n");
    return;
  }
  if (!status->sd_mounted)
  {
    GroundStation_Send("GS sdtest mount=%u result=mount_failed\r\n",
                       (unsigned int)status->sd_result);
    return;
  }

  path_length = snprintf(file_path,
                         sizeof(file_path),
                         "%s%s",
                         SDPath,
                         SD_TEST_FILE_PATH);
  expected_length = snprintf(sd_test_expected,
                             sizeof(sd_test_expected),
                             "Balloon ground station SD test tick=%lu safe=1\r\n",
                             (unsigned long)HAL_GetTick());
  if ((path_length <= 0) || ((size_t)path_length >= sizeof(file_path)) ||
      (expected_length <= 0) ||
      ((size_t)expected_length >= sizeof(sd_test_expected)))
  {
    GroundStation_Send("GS sdtest result=format_failed\r\n");
    return;
  }

  result = f_open(&SDFile, file_path, FA_CREATE_ALWAYS | FA_WRITE);
  if (result == FR_OK)
  {
    result = f_write(&SDFile,
                     sd_test_expected,
                     (UINT)expected_length,
                     &bytes_written);
    if (result == FR_OK)
    {
      result = f_sync(&SDFile);
    }
    close_result = f_close(&SDFile);
    if ((result == FR_OK) && (close_result != FR_OK))
    {
      result = close_result;
    }
  }
  if ((result != FR_OK) || (bytes_written != (UINT)expected_length))
  {
    GroundStation_Send("GS sdtest write=%u bytes=%u/%u result=write_failed\r\n",
                       (unsigned int)result,
                       (unsigned int)bytes_written,
                       (unsigned int)expected_length);
    (void)f_unlink(file_path);
    return;
  }

  memset(sd_test_actual, 0, sizeof(sd_test_actual));
  result = f_open(&SDFile, file_path, FA_READ);
  if (result == FR_OK)
  {
    result = f_read(&SDFile,
                    sd_test_actual,
                    sizeof(sd_test_actual) - 1U,
                    &bytes_read);
    close_result = f_close(&SDFile);
    if ((result == FR_OK) && (close_result != FR_OK))
    {
      result = close_result;
    }
  }

  unlink_result = f_unlink(file_path);
  if ((result == FR_OK) && (bytes_read == (UINT)expected_length) &&
      (memcmp(sd_test_actual,
              sd_test_expected,
              (size_t)expected_length) == 0) &&
      (unlink_result == FR_OK))
  {
    GroundStation_Send(
        "GS sdtest temp_file=%s bytes=%u verify=match cleanup=deleted result=PASS\r\n",
        SD_TEST_FILE_PATH,
        (unsigned int)bytes_read);
  }
  else
  {
    GroundStation_Send(
        "GS sdtest read=%u bytes=%u/%u unlink=%u verify_or_cleanup=failed result=FAIL\r\n",
        (unsigned int)result,
        (unsigned int)bytes_read,
        (unsigned int)expected_length,
        (unsigned int)unlink_result);
  }
}

static void GroundStation_SendStatus(const GroundStationStatus *status)
{
  uint32_t adc_raw = 0U;
  uint32_t supply_mv = 0U;
  uint8_t flash_id[3] = {0U, 0U, 0U};
  HAL_StatusTypeDef adc_result = GroundStation_ReadSupply(&adc_raw, &supply_mv);
  HAL_StatusTypeDef flash_result = GroundStation_ReadFlashJedecId(flash_id);

  GroundStation_Send(
      "GS status radio=%s initialized=%u tx_arm=%u txen=%u rxen=%u usb_det=%u "
      "adc=%lu supply_mv=%lu(%u) flash=%02X%02X%02X(%u) "
      "sd=%s/%u log=%u log_owner=%u log_files=%s/%s log_result=%u log_drop=%lu "
      "heap=%lu/%lu stack_words=%lu tick=%lu\r\n",
      E28Sx1281_ModeName(radio.mode),
      radio.initialized ? 1U : 0U,
      GroundStation_IsRadioTxArmed() ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_TXEN_GPIO_Port, RADIO_TXEN_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_RXEN_GPIO_Port, RADIO_RXEN_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(USB_DET_GPIO_Port, USB_DET_Pin) == GPIO_PIN_SET ? 1U : 0U,
      (unsigned long)adc_raw,
      (unsigned long)supply_mv,
      (unsigned int)adc_result,
      flash_id[0],
      flash_id[1],
      flash_id[2],
      (unsigned int)flash_result,
      status->sd_mounted ? "mounted" : "offline",
      (unsigned int)status->sd_result,
      (unsigned int)mission_log_state,
      mission_log_owns_sd ? 1U : 0U,
      mission_data_log.opened ? mission_data_log.path : "-",
      mission_event_log.opened ? mission_event_log.path : "-",
      (unsigned int)mission_log_last_result,
      (unsigned long)mission_log_drop_count,
      (unsigned long)xPortGetFreeHeapSize(),
      (unsigned long)xPortGetMinimumEverFreeHeapSize(),
      (unsigned long)uxTaskGetStackHighWaterMark(NULL),
      (unsigned long)HAL_GetTick());
}

static void GroundStation_RunAllStatusTests(GroundStationStatus *status)
{
  if (mission_log_owns_sd)
  {
    GroundStation_LogEvent(GROUND_LOG_EVENT_MISSION_STOP,
                           (BalloonCommandCode)0,
                           (BalloonAckStage)0,
                           BALLOON_REJECT_OPERATOR_STOP,
                           0U,
                           0U,
                           0U,
                           0U,
                           0U,
                           0,
                           0);
    GroundStation_StopMissionLogging();
  }
  GroundStation_EnforceRadioLockdown();
  GroundStation_UpdateSdState(status);
  GroundStation_Send(
      "GS test begin hardware=%s firmware=%s radio_tx=disabled\r\n",
      GROUND_HARDWARE_VERSION,
      GROUND_FIRMWARE_VERSION);
  GroundStation_Send("GS test step=board_status\r\n");
  GroundStation_SendStatus(status);
  GroundStation_Send("GS test step=sd_write_read\r\n");
  GroundStation_RunSdWriteReadTest(status);
  GroundStation_Send("GS test step=radio_probe_no_tx\r\n");
  GroundStation_ProbeRadio();
  GroundStation_EnforceRadioLockdown();
  GroundStation_Send("GS test end radio=reset\r\n");
}

static bool GroundStation_SendMissionCommand(BalloonCommandCode code,
                                             uint8_t channel,
                                             uint8_t flags,
                                             uint16_t value,
                                             uint16_t duration_ms)
{
  BalloonCommandPayload command = {0};
  uint8_t payload[BALLOON_COMMAND_PAYLOAD_SIZE];
  uint8_t payload_length = 0U;

  if (!mission_active)
  {
    GroundStation_Send(
        "GS command rejected reason=mission_not_started command=mission_start_antenna\r\n");
    return false;
  }

  command.code = code;
  command.mission_id = MISSION_ID;
  command.issued_ms = HAL_GetTick();
  command.ttl_ms = MISSION_COMMAND_TTL_MS;
  command.channel = channel;
  command.flags = flags;
  command.value = value;
  command.duration_ms = duration_ms;
  if (!BalloonRadio_EncodeCommandPayload(
          &command, payload, &payload_length))
  {
    GroundStation_Send("GS command rejected reason=encode\r\n");
    return false;
  }

  ++radio_sequence;
  if (!GroundStation_QueueRadioFrame(BALLOON_RADIO_TYPE_COMMAND,
                                      radio_sequence,
                                      payload,
                                      payload_length))
  {
    GroundStation_Send("GS command rejected reason=radio_busy_or_locked\r\n");
    return false;
  }
  GroundStation_LogEvent(GROUND_LOG_EVENT_COMMAND_QUEUED,
                         code,
                         (BalloonAckStage)0,
                         BALLOON_REJECT_NONE,
                         radio_sequence,
                         flags,
                         channel,
                         value,
                         duration_ms,
                         0,
                         0);
  return true;
}

static void GroundStation_StartMission(GroundStationStatus *status)
{
  HAL_StatusTypeDef result;

  if (mission_active)
  {
    GroundStation_Send("GS mission start rejected reason=already_active\r\n");
    return;
  }
  if (mission_log_owns_sd)
  {
    GroundStation_Send(
        "GS mission start rejected reason=log_flushing wait=status_log_owner_0\r\n");
    return;
  }
  result = GroundStation_InitializeRadio();

  if (result == HAL_OK)
  {
    result = E28Sx1281_StartReceive(&radio);
  }
  if (result != HAL_OK)
  {
    GroundStation_EnforceRadioLockdown();
    GroundStation_Send("GS mission start result=FAIL error=%s\r\n",
                       E28Sx1281_ErrorName(radio.last_error));
    return;
  }

  mission_active = true;
  radio_tx_armed = true;
  radio_tx_arm_deadline = 0U;
  E28Sx1281_SetTransmitPermission(&radio, true);
  mission_next_heartbeat_tick = HAL_GetTick() + 1000U;
  if ((status != NULL) && status->sd_mounted)
  {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    status->sd_mounted = false;
    status->sd_result = FR_NOT_READY;
  }
  GroundStation_StartMissionLogging();
  GroundStation_LogEvent(GROUND_LOG_EVENT_MISSION_START,
                         (BalloonCommandCode)0,
                         (BalloonAckStage)0,
                         BALLOON_REJECT_NONE,
                         0U,
                         0U,
                         0U,
                         0U,
                         0U,
                         0,
                         0);
  GroundStation_Send(
      "GS mission start result=PASS mission_id=%u heartbeat_ms=%u radio_tx=session log=requested\r\n",
      (unsigned int)MISSION_ID,
      (unsigned int)MISSION_HEARTBEAT_PERIOD_MS);
}

static void GroundStation_StopMission(void)
{
  GroundStation_EnforceRadioLockdown();
  GroundStation_LogEvent(GROUND_LOG_EVENT_MISSION_STOP,
                         (BalloonCommandCode)0,
                         (BalloonAckStage)0,
                         BALLOON_REJECT_OPERATOR_STOP,
                         0U,
                         0U,
                         0U,
                         0U,
                         0U,
                         0,
                         0);
  GroundStation_StopMissionLogging();
  GroundStation_Send("GS mission stop result=safe radio=reset log=flushing\r\n");
}

static void GroundStation_ServiceMission(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t heartbeat[4];

  if (!mission_active || pending_radio_tx.valid ||
      (radio.mode == E28_SX1281_MODE_TX) ||
      ((int32_t)(now - mission_next_heartbeat_tick) < 0))
  {
    return;
  }

  heartbeat[0] = (uint8_t)(now & 0xFFU);
  heartbeat[1] = (uint8_t)((now >> 8U) & 0xFFU);
  heartbeat[2] = (uint8_t)((now >> 16U) & 0xFFU);
  heartbeat[3] = (uint8_t)(now >> 24U);
  ++radio_sequence;
  if (GroundStation_QueueRadioFrame(BALLOON_RADIO_TYPE_PING,
                                    radio_sequence,
                                    heartbeat,
                                    sizeof(heartbeat)))
  {
    mission_next_heartbeat_tick = now + MISSION_HEARTBEAT_PERIOD_MS;
  }
}

static bool GroundStation_CommandStartsWithNoCase(const char *command,
                                                   const char *prefix)
{
  size_t index;

  for (index = 0U; prefix[index] != '\0'; ++index)
  {
    if ((command[index] == '\0') ||
        (tolower((unsigned char)command[index]) !=
         tolower((unsigned char)prefix[index])))
    {
      return false;
    }
  }
  return true;
}

static bool GroundStation_IsRadioEmergencyCommand(const char *command)
{
  return (GroundStation_CommandStartsWithNoCase(command, "radio disarm") &&
          (command[12] == '\0')) ||
         (GroundStation_CommandStartsWithNoCase(command, "radio reset") &&
          (command[11] == '\0')) ||
         (GroundStation_CommandStartsWithNoCase(command, "mission stop") &&
          (command[12] == '\0'));
}

static void GroundStation_HandleCommand(char *command, GroundStationStatus *status)
{
  size_t index;
  unsigned int channel = 0U;
  unsigned int value = 0U;
  unsigned long duration_ms = 0UL;
  char direction[8] = {0};
  char extra = '\0';
  int fields;
  bool reverse;

  if (GroundStation_CommandStartsWithNoCase(command, "radio send "))
  {
    const char *payload = &command[11];
    size_t payload_length = strlen(payload);

    if ((payload_length == 0U) ||
        (payload_length > BALLOON_RADIO_MAX_PAYLOAD))
    {
      GroundStation_Send(
          "GS radio tx rejected reason=text_length valid=1..%u\r\n",
          (unsigned int)BALLOON_RADIO_MAX_PAYLOAD);
    }
    else
    {
      ++radio_sequence;
      (void)GroundStation_QueueRadioFrame(BALLOON_RADIO_TYPE_TEXT,
                                          radio_sequence,
                                          (const uint8_t *)payload,
                                          (uint8_t)payload_length);
    }
    return;
  }

  for (index = 0U; command[index] != '\0'; ++index)
  {
    command[index] = (char)tolower((unsigned char)command[index]);
  }

  if ((strcmp(command, "help") == 0) || (strcmp(command, "?") == 0))
  {
    GroundStation_Send(
        "GS commands: version | status | test | mission start antenna | mission stop\r\n");
    GroundStation_Send(
        "GS local: adc | flash | sd | sdtest | sd force <on|off>\r\n");
    GroundStation_Send(
        "GS sd diagnostic: sd raw <1|4> | sd mount <1|4>\r\n");
    GroundStation_Send(
        "GS flight: fc status | fc stop | fc valve <1|2> <ms>\r\n");
    GroundStation_Send(
        "GS flight: fc pump <1|2> <fwd|rev> <ms> | fc motor <1|2> <fwd|rev> <duty1..30> <ms>\r\n");
    GroundStation_Send(
        "GS flight: fc servo <1|2> <pulse1000..2000us> <ms>\r\n");
  }
  else if (strcmp(command, "version") == 0)
  {
    GroundStation_Send(
        "GS hardware=%s firmware=%s mission=%u command_payload_v=%u telemetry_accept=v1/v2\r\n",
                       GROUND_HARDWARE_VERSION,
                       GROUND_FIRMWARE_VERSION,
                       mission_active ? 1U : 0U,
                       (unsigned int)BALLOON_MISSION_PAYLOAD_VERSION);
  }
  else if (strcmp(command, "status") == 0)
  {
    GroundStation_SendStatus(status);
  }
  else if (strcmp(command, "test") == 0)
  {
    GroundStation_RunAllStatusTests(status);
  }
  else if (strcmp(command, "mission start antenna") == 0)
  {
    GroundStation_StartMission(status);
  }
  else if (strcmp(command, "mission stop") == 0)
  {
    GroundStation_StopMission();
  }
  else if (strcmp(command, "fc status") == 0)
  {
    (void)GroundStation_SendMissionCommand(
        BALLOON_COMMAND_STATUS, 0U, 0U, 0U, 0U);
  }
  else if (strcmp(command, "fc stop") == 0)
  {
    (void)GroundStation_SendMissionCommand(
        BALLOON_COMMAND_STOP, 0U, 0U, 0U, 0U);
  }
  else if ((fields = sscanf(command,
                            "fc valve %u %lu %c",
                            &channel,
                            &duration_ms,
                            &extra)) == 2)
  {
    if ((channel < 1U) || (channel > 2U) ||
        (duration_ms < 50UL) || (duration_ms > 3000UL))
    {
      GroundStation_Send(
          "GS command rejected reason=range channel=1..2 duration_ms=50..3000\r\n");
    }
    else
    {
      (void)GroundStation_SendMissionCommand(
          BALLOON_COMMAND_VALVE,
          (uint8_t)channel,
          0U,
          0U,
          (uint16_t)duration_ms);
    }
  }
  else if ((fields = sscanf(command,
                            "fc pump %u %7s %lu %c",
                            &channel,
                            direction,
                            &duration_ms,
                            &extra)) == 3)
  {
    reverse = (strcmp(direction, "rev") == 0) ||
              (strcmp(direction, "reverse") == 0);
    if ((channel < 1U) || (channel > 2U) ||
        (duration_ms < 50UL) || (duration_ms > 3000UL))
    {
      GroundStation_Send(
          "GS command rejected reason=range channel=1..2 duration_ms=50..3000\r\n");
    }
    else if (!reverse && (strcmp(direction, "fwd") != 0) &&
        (strcmp(direction, "forward") != 0))
    {
      GroundStation_Send("GS command rejected reason=direction use=fwd|rev\r\n");
    }
    else
    {
      (void)GroundStation_SendMissionCommand(
          BALLOON_COMMAND_PUMP,
          (uint8_t)channel,
          reverse ? 1U : 0U,
          0U,
          (uint16_t)duration_ms);
    }
  }
  else if ((fields = sscanf(command,
                            "fc motor %u %7s %u %lu %c",
                            &channel,
                            direction,
                            &value,
                            &duration_ms,
                            &extra)) == 4)
  {
    reverse = (strcmp(direction, "rev") == 0) ||
              (strcmp(direction, "reverse") == 0);
    if ((channel < 1U) || (channel > 2U) ||
        (value < 1U) || (value > 30U) ||
        (duration_ms < 50UL) || (duration_ms > 3000UL))
    {
      GroundStation_Send(
          "GS command rejected reason=range channel=1..2 duty=1..30 duration_ms=50..3000\r\n");
    }
    else if (!reverse && (strcmp(direction, "fwd") != 0) &&
        (strcmp(direction, "forward") != 0))
    {
      GroundStation_Send("GS command rejected reason=direction use=fwd|rev\r\n");
    }
    else
    {
      (void)GroundStation_SendMissionCommand(
          BALLOON_COMMAND_MOTOR,
          (uint8_t)channel,
          reverse ? 1U : 0U,
          (uint16_t)value,
          (uint16_t)duration_ms);
    }
  }
  else if ((fields = sscanf(command,
                            "fc servo %u %u %lu %c",
                            &channel,
                            &value,
                            &duration_ms,
                            &extra)) == 3)
  {
    if ((channel < 1U) || (channel > 2U) ||
        (value < 1000U) || (value > 2000U) ||
        (duration_ms < 50UL) || (duration_ms > 3000UL))
    {
      GroundStation_Send(
          "GS command rejected reason=range channel=1..2 pulse_us=1000..2000 duration_ms=50..3000\r\n");
    }
    else
    {
      (void)GroundStation_SendMissionCommand(
          BALLOON_COMMAND_SERVO,
          (uint8_t)channel,
          0U,
          (uint16_t)value,
          (uint16_t)duration_ms);
    }
  }
  else if (strcmp(command, "adc") == 0)
  {
    uint32_t raw = 0U;
    uint32_t supply_mv = 0U;
    HAL_StatusTypeDef result = GroundStation_ReadSupply(&raw, &supply_mv);
    GroundStation_Send("GS adc raw=%lu supply_mv=%lu hal=%u\r\n",
                       (unsigned long)raw,
                       (unsigned long)supply_mv,
                       (unsigned int)result);
  }
  else if (strcmp(command, "flash") == 0)
  {
    uint8_t flash_id[3] = {0U, 0U, 0U};
    HAL_StatusTypeDef result = GroundStation_ReadFlashJedecId(flash_id);
    GroundStation_Send("GS flash jedec=%02X%02X%02X expected=EF4018 hal=%u\r\n",
                       flash_id[0],
                       flash_id[1],
                       flash_id[2],
                       (unsigned int)result);
  }
  else if (strcmp(command, "sd force on") == 0)
  {
    GroundStation_SetSdForcePresent(status, true);
  }
  else if (strcmp(command, "sd force off") == 0)
  {
    GroundStation_SetSdForcePresent(status, false);
  }
  else if (strcmp(command, "sd raw 1") == 0)
  {
    GroundStation_RunSdRawDiagnostic(status, 1U);
  }
  else if (strcmp(command, "sd raw 4") == 0)
  {
    GroundStation_RunSdRawDiagnostic(status, 4U);
  }
  else if (strcmp(command, "sd mount 1") == 0)
  {
    GroundStation_MountSdWithBus(status, 1U);
  }
  else if (strcmp(command, "sd mount 4") == 0)
  {
    GroundStation_MountSdWithBus(status, 4U);
  }
  else if (strcmp(command, "sd") == 0)
  {
    GroundStation_UpdateSdState(status);
    GroundStation_Send(
        "GS sd raw_cd=%u force=%u auto_retry=%u auto_fallback=%u present=%u mounted=%u fresult=%u log=%u log_owner=%u data=%s event=%s dropped=%lu\r\n",
                       (unsigned int)BSP_PlatformRawDetectLevel(),
                       (unsigned int)BSP_PlatformIsForcePresentEnabled(),
                       status->sd_auto_retry_enabled ? 1U : 0U,
                       status->sd_auto_fallback_active ? 1U : 0U,
                       GroundStation_IsSdCardPresent() ? 1U : 0U,
                       status->sd_mounted ? 1U : 0U,
                       (unsigned int)status->sd_result,
                       (unsigned int)mission_log_state,
                       mission_log_owns_sd ? 1U : 0U,
                       mission_data_log.opened ? mission_data_log.path : "-",
                       mission_event_log.opened ? mission_event_log.path : "-",
                       (unsigned long)mission_log_drop_count);
  }
  else if (strcmp(command, "sdtest") == 0)
  {
    GroundStation_RunSdWriteReadTest(status);
  }
  else if ((strcmp(command, "radio") == 0) ||
           (strcmp(command, "radio status") == 0))
  {
    GroundStation_SendRadioStatus();
  }
  else if (strcmp(command, "radio probe") == 0)
  {
    GroundStation_ProbeRadio();
  }
  else if (strcmp(command, "radio init") == 0)
  {
    GroundStation_InitializeRadio();
  }
  else if (strcmp(command, "radio rx") == 0)
  {
    HAL_StatusTypeDef result = E28Sx1281_StartReceive(&radio);
    GroundStation_Send(
        "GS radio rx hal=%u mode=%s error=%s result=%s\r\n",
        (unsigned int)result,
        E28Sx1281_ModeName(radio.mode),
        E28Sx1281_ErrorName(radio.last_error),
        result == HAL_OK ? "PASS" : "FAIL");
  }
  else if (strcmp(command, "radio arm antenna") == 0)
  {
    if (!radio.initialized)
    {
      GroundStation_Send(
          "GS radio arm rejected reason=not_initialized command=radio_init_first\r\n");
    }
    else
    {
      radio_tx_armed = true;
      radio_tx_arm_deadline = HAL_GetTick() + RADIO_TX_ARM_TIMEOUT_MS;
      E28Sx1281_SetTransmitPermission(&radio, true);
      GroundStation_Send(
          "GS radio tx_arm=1 expires_ms=%u confirmation=antenna_installed "
          "chip_power_dbm=%d\r\n",
          (unsigned int)RADIO_TX_ARM_TIMEOUT_MS,
          E28_SX1281_MIN_CHIP_POWER_DBM);
    }
  }
  else if ((strcmp(command, "radio disarm") == 0) ||
           (strcmp(command, "radio reset") == 0))
  {
    GroundStation_EnforceRadioLockdown();
    GroundStation_Send(
        "GS radio mode=reset tx_arm=0 txen=0 rxen=0 result=safe "
        "note=run_radio_init_before_reuse\r\n");
  }
  else if (strcmp(command, "radio ping") == 0)
  {
    static const uint8_t ping_payload[] = "ground";

    ++radio_sequence;
    (void)GroundStation_QueueRadioFrame(BALLOON_RADIO_TYPE_PING,
                                        radio_sequence,
                                        ping_payload,
                                        sizeof(ping_payload) - 1U);
  }
  else if (command[0] != '\0')
  {
    GroundStation_Send("GS error unknown_command=%s\r\n", command);
  }
}

static void GroundStation_ProcessUsbCommands(GroundStationStatus *status)
{
  static char command[COMMAND_BUFFER_SIZE];
  static size_t command_length;
  static bool command_ready;
  uint8_t value;

  if (command_ready)
  {
    if ((radio.mode == E28_SX1281_MODE_TX) &&
        !GroundStation_IsRadioEmergencyCommand(command))
    {
      command_length = 0U;
      command_ready = false;
    }
    else
    {
      GroundStation_HandleCommand(command, status);
      command_length = 0U;
      command_ready = false;
      if (radio.mode == E28_SX1281_MODE_TX)
      {
        return;
      }
    }
  }

  while (GroundStation_TryReadUsbByte(&value))
  {
    if ((value == '\r') || (value == '\n'))
    {
      if (command_length > 0U)
      {
        command[command_length] = '\0';
        command_ready = true;
        if ((radio.mode == E28_SX1281_MODE_TX) &&
            !GroundStation_IsRadioEmergencyCommand(command))
        {
          command_length = 0U;
          command_ready = false;
          continue;
        }
        GroundStation_HandleCommand(command, status);
        command_length = 0U;
        command_ready = false;
        if (radio.mode == E28_SX1281_MODE_TX)
        {
          return;
        }
      }
    }
    else if (command_length < (sizeof(command) - 1U))
    {
      command[command_length++] = (char)value;
    }
    else
    {
      command_length = 0U;
      command_ready = false;
      if (radio.mode != E28_SX1281_MODE_TX)
      {
        GroundStation_Send("GS error command_too_long\r\n");
      }
      else
      {
        continue;
      }
    }
  }
}

void GroundStationApp_Run(void)
{
  const E28Sx1281Pins radio_pins = {
      .spi = &hspi2,
      .nss_port = RADIO_CS_GPIO_Port,
      .nss_pin = RADIO_CS_Pin,
      .reset_port = RADIO_RST_GPIO_Port,
      .reset_pin = RADIO_RST_Pin,
      .busy_port = RADIO_BUSY_GPIO_Port,
      .busy_pin = RADIO_BUSY_Pin,
      .dio1_port = RADIO_DIO1_GPIO_Port,
      .dio1_pin = RADIO_DIO1_Pin,
      .rxen_port = RADIO_RXEN_GPIO_Port,
      .rxen_pin = RADIO_RXEN_Pin,
      .txen_port = RADIO_TXEN_GPIO_Port,
      .txen_pin = RADIO_TXEN_Pin,
      .power_present = GroundStation_RadioPowerPresent,
      .set_bus_enabled = NULL,
      .callback_context = NULL,
  };
  GroundStationStatus status = {
      .sd_result = FR_NOT_READY,
      .sd_mounted = false,
      .sd_auto_retry_enabled = true,
  };

  E28Sx1281_Construct(
      &radio, &radio_pins, E28_SX1281_DEFAULT_FREQUENCY_HZ);
  radio_attached = true;
  mission_log_queue = xQueueCreate(MISSION_LOG_QUEUE_LENGTH,
                                   sizeof(GroundStationLogRecord));
  if ((mission_log_queue == NULL) ||
      (xTaskCreate(GroundStation_LoggerTask,
                   "GSLogger",
                   MISSION_LOG_TASK_STACK_WORDS,
                   NULL,
                   tskIDLE_PRIORITY + 1U,
                   &mission_log_task_handle) != pdPASS))
  {
    if (mission_log_queue != NULL)
    {
      vQueueDelete(mission_log_queue);
      mission_log_queue = NULL;
    }
    mission_log_task_handle = NULL;
    mission_log_state = BALLOON_LOG_STATE_ERROR;
    mission_log_last_result = FR_NOT_ENOUGH_CORE;
  }
  GroundStation_EnforceRadioLockdown();
  osDelay(1000U);
  GroundStation_Send(
      "GS hardware=%s firmware=%s ready; E28 reset; RF TX locked; type help\r\n",
      GROUND_HARDWARE_VERSION,
      GROUND_FIRMWARE_VERSION);

  for (;;)
  {
    GroundStation_ServiceRadio();
    if (radio.mode != E28_SX1281_MODE_TX)
    {
      GroundStation_ServiceRadioArm();
      GroundStation_UpdateSdState(&status);
      GroundStation_ServiceMission();
    }
    GroundStation_ProcessUsbCommands(&status);
    osDelay(5U);
  }
}
