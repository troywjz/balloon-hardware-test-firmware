#ifndef BALLOON_RADIO_PROTOCOL_H
#define BALLOON_RADIO_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BALLOON_RADIO_PROTOCOL_VERSION 1U
#define BALLOON_RADIO_MAX_PAYLOAD      64U
#define BALLOON_RADIO_FRAME_OVERHEAD   10U
#define BALLOON_MISSION_PAYLOAD_VERSION 1U
#define BALLOON_TELEMETRY_PAYLOAD_VERSION_V2 2U
#define BALLOON_TELEMETRY_PAYLOAD_VERSION 3U
#define BALLOON_TELEMETRY_PAYLOAD_SIZE_V1 30U
#define BALLOON_TELEMETRY_PAYLOAD_SIZE_V2 32U
#define BALLOON_COMMAND_PAYLOAD_SIZE    16U
#define BALLOON_ACK_PAYLOAD_SIZE        12U
#define BALLOON_TELEMETRY_PAYLOAD_SIZE  57U
#define BALLOON_RADIO_MAX_FRAME_SIZE   \
  (BALLOON_RADIO_MAX_PAYLOAD + BALLOON_RADIO_FRAME_OVERHEAD)

typedef enum
{
  BALLOON_RADIO_SOURCE_FLIGHT = 1,
  BALLOON_RADIO_SOURCE_GROUND = 2
} BalloonRadioSource;

typedef enum
{
  BALLOON_RADIO_TYPE_PING = 1,
  BALLOON_RADIO_TYPE_PONG = 2,
  BALLOON_RADIO_TYPE_TEXT = 3,
  BALLOON_RADIO_TYPE_TELEMETRY = 4,
  BALLOON_RADIO_TYPE_COMMAND = 5,
  BALLOON_RADIO_TYPE_ACK = 6
} BalloonRadioType;

typedef struct
{
  BalloonRadioType type;
  BalloonRadioSource source;
  uint16_t sequence;
  uint8_t payload_length;
  uint8_t payload[BALLOON_RADIO_MAX_PAYLOAD];
} BalloonRadioFrame;

typedef enum
{
  BALLOON_SYSTEM_MODE_STANDBY = 0,
  BALLOON_SYSTEM_MODE_MISSION = 1,
  BALLOON_SYSTEM_MODE_FAILSAFE = 2,
  BALLOON_SYSTEM_MODE_MAINTENANCE = 3
} BalloonSystemMode;

typedef enum
{
  BALLOON_LOG_STATE_OFF = 0,
  BALLOON_LOG_STATE_ACTIVE = 1,
  BALLOON_LOG_STATE_ERROR = 2
} BalloonLogState;

typedef enum
{
  BALLOON_COMMAND_STATUS = 1,
  BALLOON_COMMAND_STOP = 2,
  BALLOON_COMMAND_VALVE = 10,
  BALLOON_COMMAND_PUMP = 11,
  BALLOON_COMMAND_MOTOR = 12,
  BALLOON_COMMAND_SERVO = 13
} BalloonCommandCode;

typedef enum
{
  BALLOON_ACK_REJECTED = 0,
  BALLOON_ACK_ACCEPTED = 1,
  BALLOON_ACK_STARTED = 2,
  BALLOON_ACK_COMPLETED = 3,
  BALLOON_ACK_STOPPED = 4
} BalloonAckStage;

typedef enum
{
  BALLOON_REJECT_NONE = 0,
  BALLOON_REJECT_BAD_PAYLOAD = 1,
  BALLOON_REJECT_WRONG_MISSION = 2,
  BALLOON_REJECT_DUPLICATE = 3,
  BALLOON_REJECT_NOT_MISSION = 4,
  BALLOON_REJECT_OUTPUTS_DISARMED = 5,
  BALLOON_REJECT_BUSY = 6,
  BALLOON_REJECT_RANGE = 7,
  BALLOON_REJECT_HARDWARE = 8,
  BALLOON_REJECT_LINK_LOST = 9,
  BALLOON_REJECT_OPERATOR_STOP = 10,
  BALLOON_REJECT_EXPIRED = 11
} BalloonRejectReason;

typedef struct
{
  BalloonCommandCode code;
  uint16_t mission_id;
  uint32_t issued_ms;
  uint16_t ttl_ms;
  uint8_t channel;
  uint8_t flags;
  uint16_t value;
  uint16_t duration_ms;
} BalloonCommandPayload;

typedef struct
{
  BalloonCommandCode command;
  BalloonAckStage stage;
  BalloonRejectReason reason;
  uint16_t mission_id;
  uint16_t command_sequence;
  BalloonSystemMode system_mode;
  uint8_t action;
  uint8_t channel;
} BalloonAckPayload;

typedef struct
{
  BalloonSystemMode system_mode;
  uint16_t mission_id;
  uint16_t fault_bits;
  uint32_t timestamp_ms;
  uint16_t battery_mv;
  uint16_t adc_raw;
  uint8_t imu_who_am_i;
  bool imu_valid;
  bool sd_present;
  bool link_valid;
  uint8_t action;
  uint8_t action_channel;
  uint16_t action_value;
  uint16_t action_remaining_ms;
  uint16_t radio_rx_count;
  uint16_t radio_tx_count;
  uint16_t radio_error_count;
  bool action_reverse;
  uint8_t log_state;
  int16_t imu_accel[3];
  int16_t imu_gyro[3];
  int16_t mag_onboard_mg[3];
  int16_t mag_external_mg[3];
  uint8_t sensor_valid_flags;
  uint8_t payload_version;
} BalloonTelemetryPayload;

bool BalloonRadio_Encode(BalloonRadioType type,
                         BalloonRadioSource source,
                         uint16_t sequence,
                         const uint8_t *payload,
                         uint8_t payload_length,
                         uint8_t *output,
                         size_t output_capacity,
                         uint8_t *output_length);
bool BalloonRadio_Decode(const uint8_t *data,
                         uint8_t length,
                         BalloonRadioFrame *frame);
bool BalloonRadio_EncodeCommandPayload(const BalloonCommandPayload *command,
                                       uint8_t *output,
                                       uint8_t *output_length);
bool BalloonRadio_DecodeCommandPayload(const uint8_t *data,
                                       uint8_t length,
                                       BalloonCommandPayload *command);
bool BalloonRadio_EncodeAckPayload(const BalloonAckPayload *ack,
                                   uint8_t *output,
                                   uint8_t *output_length);
bool BalloonRadio_DecodeAckPayload(const uint8_t *data,
                                   uint8_t length,
                                   BalloonAckPayload *ack);
bool BalloonRadio_EncodeTelemetryPayload(
    const BalloonTelemetryPayload *telemetry,
    uint8_t *output,
    uint8_t *output_length);
bool BalloonRadio_DecodeTelemetryPayload(const uint8_t *data,
                                         uint8_t length,
                                         BalloonTelemetryPayload *telemetry);
const char *BalloonRadio_TypeName(BalloonRadioType type);
const char *BalloonRadio_SourceName(BalloonRadioSource source);
const char *BalloonRadio_SystemModeName(BalloonSystemMode mode);
const char *BalloonRadio_CommandName(BalloonCommandCode command);
const char *BalloonRadio_AckStageName(BalloonAckStage stage);
const char *BalloonRadio_RejectReasonName(BalloonRejectReason reason);

#endif
