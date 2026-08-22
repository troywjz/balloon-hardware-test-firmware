#include "balloon_radio_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void TestTelemetryV3RoundTrip(void)
{
  const BalloonTelemetryPayload input = {
      .system_mode = BALLOON_SYSTEM_MODE_MISSION,
      .mission_id = 0x1234U,
      .fault_bits = 0x0182U,
      .timestamp_ms = 0x89ABCDEFUL,
      .battery_mv = 7400U,
      .adc_raw = 2048U,
      .imu_who_am_i = 0xE9U,
      .imu_valid = true,
      .sd_present = true,
      .link_valid = true,
      .action = 3U,
      .action_channel = 2U,
      .action_value = 30U,
      .action_remaining_ms = 900U,
      .radio_rx_count = 11U,
      .radio_tx_count = 12U,
      .radio_error_count = 13U,
      .action_reverse = true,
      .log_state = BALLOON_LOG_STATE_ACTIVE,
      .imu_accel = {-32768, -123, 32767},
      .imu_gyro = {-30000, 0, 30000},
      .mag_onboard_mg = {-8000, 12, 8000},
      .mag_external_mg = {-456, 789, 1234},
      .sensor_valid_flags = 0x07U,
      .payload_version = BALLOON_TELEMETRY_PAYLOAD_VERSION,
  };
  BalloonTelemetryPayload output;
  uint8_t encoded[BALLOON_TELEMETRY_PAYLOAD_SIZE] = {0U};
  uint8_t encoded_length = 0U;

  assert(BalloonRadio_EncodeTelemetryPayload(&input,
                                             encoded,
                                             &encoded_length));
  assert(encoded_length == BALLOON_TELEMETRY_PAYLOAD_SIZE);
  assert(encoded[0] == BALLOON_TELEMETRY_PAYLOAD_VERSION);
  assert(BalloonRadio_DecodeTelemetryPayload(encoded,
                                             encoded_length,
                                             &output));
  assert(output.payload_version == BALLOON_TELEMETRY_PAYLOAD_VERSION);
  assert(output.mission_id == input.mission_id);
  assert(output.timestamp_ms == input.timestamp_ms);
  assert(output.action_reverse == input.action_reverse);
  assert(output.log_state == input.log_state);
  assert(memcmp(output.imu_accel, input.imu_accel, sizeof(input.imu_accel)) == 0);
  assert(memcmp(output.imu_gyro, input.imu_gyro, sizeof(input.imu_gyro)) == 0);
  assert(memcmp(output.mag_onboard_mg,
                input.mag_onboard_mg,
                sizeof(input.mag_onboard_mg)) == 0);
  assert(memcmp(output.mag_external_mg,
                input.mag_external_mg,
                sizeof(input.mag_external_mg)) == 0);
  assert(output.sensor_valid_flags == input.sensor_valid_flags);
}

static void TestTelemetryV2Compatibility(void)
{
  uint8_t encoded[BALLOON_TELEMETRY_PAYLOAD_SIZE_V2] = {0U};
  BalloonTelemetryPayload output;

  encoded[0] = BALLOON_TELEMETRY_PAYLOAD_VERSION_V2;
  encoded[1] = BALLOON_SYSTEM_MODE_MAINTENANCE;
  encoded[30] = 1U;
  encoded[31] = BALLOON_LOG_STATE_ERROR;
  memset(&output, 0xA5, sizeof(output));

  assert(BalloonRadio_DecodeTelemetryPayload(encoded,
                                             sizeof(encoded),
                                             &output));
  assert(output.payload_version == BALLOON_TELEMETRY_PAYLOAD_VERSION_V2);
  assert(output.action_reverse);
  assert(output.log_state == BALLOON_LOG_STATE_ERROR);
  assert(output.imu_accel[0] == 0);
  assert(output.mag_onboard_mg[0] == 0);
  assert(output.mag_external_mg[0] == 0);
  assert(output.sensor_valid_flags == 0U);
}

static void TestTelemetryV1Compatibility(void)
{
  uint8_t encoded[BALLOON_TELEMETRY_PAYLOAD_SIZE_V1] = {0U};
  BalloonTelemetryPayload output;

  encoded[0] = BALLOON_MISSION_PAYLOAD_VERSION;
  encoded[1] = BALLOON_SYSTEM_MODE_STANDBY;
  memset(&output, 0xA5, sizeof(output));

  assert(BalloonRadio_DecodeTelemetryPayload(encoded,
                                             sizeof(encoded),
                                             &output));
  assert(output.payload_version == BALLOON_MISSION_PAYLOAD_VERSION);
  assert(!output.action_reverse);
  assert(output.log_state == BALLOON_LOG_STATE_OFF);
  assert(output.sensor_valid_flags == 0U);
}

static void TestTelemetryRejectsMismatchedVersionAndLength(void)
{
  uint8_t encoded[BALLOON_TELEMETRY_PAYLOAD_SIZE] = {0U};
  BalloonTelemetryPayload output;

  encoded[0] = BALLOON_TELEMETRY_PAYLOAD_VERSION_V2;
  assert(!BalloonRadio_DecodeTelemetryPayload(encoded,
                                              sizeof(encoded),
                                              &output));
  encoded[0] = BALLOON_TELEMETRY_PAYLOAD_VERSION;
  assert(!BalloonRadio_DecodeTelemetryPayload(
      encoded, BALLOON_TELEMETRY_PAYLOAD_SIZE_V2, &output));
}

int main(void)
{
  TestTelemetryV3RoundTrip();
  TestTelemetryV2Compatibility();
  TestTelemetryV1Compatibility();
  TestTelemetryRejectsMismatchedVersionAndLength();
  puts("balloon_radio_protocol_test: PASS");
  return 0;
}
