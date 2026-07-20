#include "balloon_radio_protocol.h"

#include <string.h>

#define BALLOON_RADIO_MAGIC_0 0x42U
#define BALLOON_RADIO_MAGIC_1 0x4CU
#define BALLOON_RADIO_HEADER_SIZE 8U
#define BALLOON_RADIO_CRC_SIZE    2U

static void BalloonRadio_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8U);
}

static void BalloonRadio_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)(value >> 24U);
}

static uint16_t BalloonRadio_ReadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t BalloonRadio_ReadU32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static uint16_t BalloonRadio_Crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  size_t index;

  for (index = 0U; index < length; ++index)
  {
    uint8_t bit;

    crc ^= (uint16_t)data[index] << 8U;
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

bool BalloonRadio_Encode(BalloonRadioType type,
                         BalloonRadioSource source,
                         uint16_t sequence,
                         const uint8_t *payload,
                         uint8_t payload_length,
                         uint8_t *output,
                         size_t output_capacity,
                         uint8_t *output_length)
{
  size_t frame_length = BALLOON_RADIO_HEADER_SIZE + payload_length +
                        BALLOON_RADIO_CRC_SIZE;
  uint16_t crc;

  if ((output == NULL) || (output_length == NULL) ||
      ((type < BALLOON_RADIO_TYPE_PING) ||
       (type > BALLOON_RADIO_TYPE_ACK)) ||
      ((source != BALLOON_RADIO_SOURCE_FLIGHT) &&
       (source != BALLOON_RADIO_SOURCE_GROUND)) ||
      (payload_length > BALLOON_RADIO_MAX_PAYLOAD) ||
      ((payload_length > 0U) && (payload == NULL)) ||
      (output_capacity < frame_length))
  {
    return false;
  }

  output[0] = BALLOON_RADIO_MAGIC_0;
  output[1] = BALLOON_RADIO_MAGIC_1;
  output[2] = BALLOON_RADIO_PROTOCOL_VERSION;
  output[3] = (uint8_t)type;
  output[4] = (uint8_t)source;
  output[5] = (uint8_t)(sequence & 0xFFU);
  output[6] = (uint8_t)(sequence >> 8U);
  output[7] = payload_length;
  if (payload_length > 0U)
  {
    memcpy(&output[BALLOON_RADIO_HEADER_SIZE], payload, payload_length);
  }

  crc = BalloonRadio_Crc16(output, frame_length - BALLOON_RADIO_CRC_SIZE);
  output[frame_length - 2U] = (uint8_t)(crc & 0xFFU);
  output[frame_length - 1U] = (uint8_t)(crc >> 8U);
  *output_length = (uint8_t)frame_length;
  return true;
}

bool BalloonRadio_Decode(const uint8_t *data,
                         uint8_t length,
                         BalloonRadioFrame *frame)
{
  uint8_t payload_length;
  size_t expected_length;
  uint16_t expected_crc;
  uint16_t actual_crc;

  if ((data == NULL) || (frame == NULL) ||
      (length < BALLOON_RADIO_FRAME_OVERHEAD))
  {
    return false;
  }

  payload_length = data[7];
  expected_length = BALLOON_RADIO_HEADER_SIZE + payload_length +
                    BALLOON_RADIO_CRC_SIZE;
  if ((data[0] != BALLOON_RADIO_MAGIC_0) ||
      (data[1] != BALLOON_RADIO_MAGIC_1) ||
      (data[2] != BALLOON_RADIO_PROTOCOL_VERSION) ||
      (data[3] < BALLOON_RADIO_TYPE_PING) ||
      (data[3] > BALLOON_RADIO_TYPE_ACK) ||
      ((data[4] != BALLOON_RADIO_SOURCE_FLIGHT) &&
       (data[4] != BALLOON_RADIO_SOURCE_GROUND)) ||
      (payload_length > BALLOON_RADIO_MAX_PAYLOAD) ||
      (expected_length != length))
  {
    return false;
  }

  expected_crc = BalloonRadio_Crc16(data, length - BALLOON_RADIO_CRC_SIZE);
  actual_crc = (uint16_t)data[length - 2U] |
               ((uint16_t)data[length - 1U] << 8U);
  if (expected_crc != actual_crc)
  {
    return false;
  }

  frame->type = (BalloonRadioType)data[3];
  frame->source = (BalloonRadioSource)data[4];
  frame->sequence = (uint16_t)data[5] | ((uint16_t)data[6] << 8U);
  frame->payload_length = payload_length;
  if (payload_length > 0U)
  {
    memcpy(frame->payload, &data[BALLOON_RADIO_HEADER_SIZE], payload_length);
  }
  return true;
}

bool BalloonRadio_EncodeCommandPayload(const BalloonCommandPayload *command,
                                       uint8_t *output,
                                       uint8_t *output_length)
{
  if ((command == NULL) || (output == NULL) || (output_length == NULL))
  {
    return false;
  }

  output[0] = BALLOON_MISSION_PAYLOAD_VERSION;
  output[1] = (uint8_t)command->code;
  BalloonRadio_WriteU16(&output[2], command->mission_id);
  BalloonRadio_WriteU32(&output[4], command->issued_ms);
  BalloonRadio_WriteU16(&output[8], command->ttl_ms);
  output[10] = command->channel;
  output[11] = command->flags;
  BalloonRadio_WriteU16(&output[12], command->value);
  BalloonRadio_WriteU16(&output[14], command->duration_ms);
  *output_length = BALLOON_COMMAND_PAYLOAD_SIZE;
  return true;
}

bool BalloonRadio_DecodeCommandPayload(const uint8_t *data,
                                       uint8_t length,
                                       BalloonCommandPayload *command)
{
  if ((data == NULL) || (command == NULL) ||
      (length != BALLOON_COMMAND_PAYLOAD_SIZE) ||
      (data[0] != BALLOON_MISSION_PAYLOAD_VERSION))
  {
    return false;
  }

  command->code = (BalloonCommandCode)data[1];
  command->mission_id = BalloonRadio_ReadU16(&data[2]);
  command->issued_ms = BalloonRadio_ReadU32(&data[4]);
  command->ttl_ms = BalloonRadio_ReadU16(&data[8]);
  command->channel = data[10];
  command->flags = data[11];
  command->value = BalloonRadio_ReadU16(&data[12]);
  command->duration_ms = BalloonRadio_ReadU16(&data[14]);
  return true;
}

bool BalloonRadio_EncodeAckPayload(const BalloonAckPayload *ack,
                                   uint8_t *output,
                                   uint8_t *output_length)
{
  if ((ack == NULL) || (output == NULL) || (output_length == NULL))
  {
    return false;
  }

  output[0] = BALLOON_MISSION_PAYLOAD_VERSION;
  output[1] = (uint8_t)ack->command;
  output[2] = (uint8_t)ack->stage;
  output[3] = (uint8_t)ack->reason;
  BalloonRadio_WriteU16(&output[4], ack->mission_id);
  BalloonRadio_WriteU16(&output[6], ack->command_sequence);
  output[8] = (uint8_t)ack->system_mode;
  output[9] = ack->action;
  output[10] = ack->channel;
  output[11] = 0U;
  *output_length = BALLOON_ACK_PAYLOAD_SIZE;
  return true;
}

bool BalloonRadio_DecodeAckPayload(const uint8_t *data,
                                   uint8_t length,
                                   BalloonAckPayload *ack)
{
  if ((data == NULL) || (ack == NULL) ||
      (length != BALLOON_ACK_PAYLOAD_SIZE) ||
      (data[0] != BALLOON_MISSION_PAYLOAD_VERSION))
  {
    return false;
  }

  ack->command = (BalloonCommandCode)data[1];
  ack->stage = (BalloonAckStage)data[2];
  ack->reason = (BalloonRejectReason)data[3];
  ack->mission_id = BalloonRadio_ReadU16(&data[4]);
  ack->command_sequence = BalloonRadio_ReadU16(&data[6]);
  ack->system_mode = (BalloonSystemMode)data[8];
  ack->action = data[9];
  ack->channel = data[10];
  return true;
}

bool BalloonRadio_EncodeTelemetryPayload(
    const BalloonTelemetryPayload *telemetry,
    uint8_t *output,
    uint8_t *output_length)
{
  if ((telemetry == NULL) || (output == NULL) || (output_length == NULL))
  {
    return false;
  }

  output[0] = BALLOON_TELEMETRY_PAYLOAD_VERSION;
  output[1] = (uint8_t)telemetry->system_mode;
  BalloonRadio_WriteU16(&output[2], telemetry->mission_id);
  BalloonRadio_WriteU16(&output[4], telemetry->fault_bits);
  BalloonRadio_WriteU32(&output[6], telemetry->timestamp_ms);
  BalloonRadio_WriteU16(&output[10], telemetry->battery_mv);
  BalloonRadio_WriteU16(&output[12], telemetry->adc_raw);
  output[14] = telemetry->imu_who_am_i;
  output[15] = telemetry->imu_valid ? 1U : 0U;
  output[16] = telemetry->sd_present ? 1U : 0U;
  output[17] = telemetry->link_valid ? 1U : 0U;
  output[18] = telemetry->action;
  output[19] = telemetry->action_channel;
  BalloonRadio_WriteU16(&output[20], telemetry->action_value);
  BalloonRadio_WriteU16(&output[22], telemetry->action_remaining_ms);
  BalloonRadio_WriteU16(&output[24], telemetry->radio_rx_count);
  BalloonRadio_WriteU16(&output[26], telemetry->radio_tx_count);
  BalloonRadio_WriteU16(&output[28], telemetry->radio_error_count);
  output[30] = telemetry->action_reverse ? 1U : 0U;
  output[31] = telemetry->log_state;
  *output_length = BALLOON_TELEMETRY_PAYLOAD_SIZE;
  return true;
}

bool BalloonRadio_DecodeTelemetryPayload(const uint8_t *data,
                                         uint8_t length,
                                         BalloonTelemetryPayload *telemetry)
{
  if ((data == NULL) || (telemetry == NULL) ||
      !(((length == BALLOON_TELEMETRY_PAYLOAD_SIZE_V1) &&
         (data[0] == BALLOON_MISSION_PAYLOAD_VERSION)) ||
        ((length == BALLOON_TELEMETRY_PAYLOAD_SIZE) &&
         (data[0] == BALLOON_TELEMETRY_PAYLOAD_VERSION))))
  {
    return false;
  }

  telemetry->system_mode = (BalloonSystemMode)data[1];
  telemetry->mission_id = BalloonRadio_ReadU16(&data[2]);
  telemetry->fault_bits = BalloonRadio_ReadU16(&data[4]);
  telemetry->timestamp_ms = BalloonRadio_ReadU32(&data[6]);
  telemetry->battery_mv = BalloonRadio_ReadU16(&data[10]);
  telemetry->adc_raw = BalloonRadio_ReadU16(&data[12]);
  telemetry->imu_who_am_i = data[14];
  telemetry->imu_valid = data[15] != 0U;
  telemetry->sd_present = data[16] != 0U;
  telemetry->link_valid = data[17] != 0U;
  telemetry->action = data[18];
  telemetry->action_channel = data[19];
  telemetry->action_value = BalloonRadio_ReadU16(&data[20]);
  telemetry->action_remaining_ms = BalloonRadio_ReadU16(&data[22]);
  telemetry->radio_rx_count = BalloonRadio_ReadU16(&data[24]);
  telemetry->radio_tx_count = BalloonRadio_ReadU16(&data[26]);
  telemetry->radio_error_count = BalloonRadio_ReadU16(&data[28]);
  telemetry->payload_version = data[0];
  if (length == BALLOON_TELEMETRY_PAYLOAD_SIZE)
  {
    telemetry->action_reverse = data[30] != 0U;
    telemetry->log_state = data[31];
  }
  else
  {
    telemetry->action_reverse = false;
    telemetry->log_state = BALLOON_LOG_STATE_OFF;
  }
  return true;
}

const char *BalloonRadio_TypeName(BalloonRadioType type)
{
  switch (type)
  {
    case BALLOON_RADIO_TYPE_PING:
      return "ping";
    case BALLOON_RADIO_TYPE_PONG:
      return "pong";
    case BALLOON_RADIO_TYPE_TEXT:
      return "text";
    case BALLOON_RADIO_TYPE_TELEMETRY:
      return "telemetry";
    case BALLOON_RADIO_TYPE_COMMAND:
      return "command";
    case BALLOON_RADIO_TYPE_ACK:
      return "ack";
    default:
      return "unknown";
  }
}

const char *BalloonRadio_SourceName(BalloonRadioSource source)
{
  switch (source)
  {
    case BALLOON_RADIO_SOURCE_FLIGHT:
      return "flight";
    case BALLOON_RADIO_SOURCE_GROUND:
      return "ground";
    default:
      return "unknown";
  }
}

const char *BalloonRadio_SystemModeName(BalloonSystemMode mode)
{
  switch (mode)
  {
    case BALLOON_SYSTEM_MODE_STANDBY:
      return "standby";
    case BALLOON_SYSTEM_MODE_MISSION:
      return "mission";
    case BALLOON_SYSTEM_MODE_FAILSAFE:
      return "failsafe";
    case BALLOON_SYSTEM_MODE_MAINTENANCE:
      return "maintenance";
    default:
      return "unknown";
  }
}

const char *BalloonRadio_CommandName(BalloonCommandCode command)
{
  switch (command)
  {
    case BALLOON_COMMAND_STATUS:
      return "status";
    case BALLOON_COMMAND_STOP:
      return "stop";
    case BALLOON_COMMAND_VALVE:
      return "valve";
    case BALLOON_COMMAND_PUMP:
      return "pump";
    case BALLOON_COMMAND_MOTOR:
      return "motor";
    case BALLOON_COMMAND_SERVO:
      return "servo";
    default:
      return "unknown";
  }
}

const char *BalloonRadio_AckStageName(BalloonAckStage stage)
{
  switch (stage)
  {
    case BALLOON_ACK_REJECTED:
      return "rejected";
    case BALLOON_ACK_ACCEPTED:
      return "accepted";
    case BALLOON_ACK_STARTED:
      return "started";
    case BALLOON_ACK_COMPLETED:
      return "completed";
    case BALLOON_ACK_STOPPED:
      return "stopped";
    default:
      return "unknown";
  }
}

const char *BalloonRadio_RejectReasonName(BalloonRejectReason reason)
{
  switch (reason)
  {
    case BALLOON_REJECT_NONE:
      return "none";
    case BALLOON_REJECT_BAD_PAYLOAD:
      return "bad_payload";
    case BALLOON_REJECT_WRONG_MISSION:
      return "wrong_mission";
    case BALLOON_REJECT_DUPLICATE:
      return "duplicate_or_old";
    case BALLOON_REJECT_NOT_MISSION:
      return "not_mission";
    case BALLOON_REJECT_OUTPUTS_DISARMED:
      return "outputs_disarmed";
    case BALLOON_REJECT_BUSY:
      return "busy";
    case BALLOON_REJECT_RANGE:
      return "range";
    case BALLOON_REJECT_HARDWARE:
      return "hardware";
    case BALLOON_REJECT_LINK_LOST:
      return "link_lost";
    case BALLOON_REJECT_OPERATOR_STOP:
      return "operator_stop";
    case BALLOON_REJECT_EXPIRED:
      return "expired";
    default:
      return "unknown";
  }
}
