#include "flight_board_test.h"

#include "FreeRTOS.h"
#include "adc.h"
#include "balloon_csv_logger.h"
#include "balloon_radio_protocol.h"
#include "cmsis_os.h"
#include "e28_sx1281.h"
#include "fatfs.h"
#include "i2c.h"
#include "main.h"
#include "queue.h"
#include "spi.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define ADC_FULL_SCALE              4095UL
#define ADC_REFERENCE_MV            3300UL
#define ADC_VBAT_DIVIDER_MULTIPLIER 3UL
#define COMMAND_BUFFER_SIZE         128U
#define USB_RX_QUEUE_SIZE           256U
#define RADIO_TX_ARM_TIMEOUT_MS     60000U
#define RADIO_POWER_SAMPLE_MS       100U
#define RADIO_POWER_ENABLE_VBAT_MV  5500UL
#define RADIO_POWER_DISABLE_VBAT_MV 4500UL
#define MISSION_ID                   1U
#define MISSION_TELEMETRY_PERIOD_MS 1000U
#define MISSION_LOG_PERIOD_MS       100U
#define MISSION_LOG_SYNC_PERIOD_MS  1000U
#define MISSION_LOG_QUEUE_LENGTH     32U
#define MISSION_LOG_RESERVED_SLOTS   8U
#define MISSION_LOG_TASK_STACK_WORDS 768U
#define MISSION_HEARTBEAT_TIMEOUT_MS 6000U
#define MISSION_COMMAND_TTL_MIN_MS   100U
#define MISSION_COMMAND_TTL_MAX_MS   5000U
#define MISSION_CLOCK_FUTURE_TOLERANCE_MS 250
#define MISSION_FAULT_ADC             (1U << 0)
#define MISSION_FAULT_IMU             (1U << 1)
#define MISSION_FAULT_SD_MISSING      (1U << 2)
#define MISSION_FAULT_MOTOR           (1U << 3)
#define MISSION_FAULT_RADIO           (1U << 4)
#define MISSION_FAULT_LINK            (1U << 5)
#define MISSION_FAULT_LOG             (1U << 6)
#define ACTUATOR_MIN_DURATION_MS    50U
#define ACTUATOR_MAX_DURATION_MS    30000U
#define MOTOR_TEST_MAX_DUTY_PERCENT 30U
#define SERVO_MIN_PULSE_US          1000U
#define SERVO_MAX_PULSE_US          2000U
#define TCA9548_ADDRESS_8BIT        (0x70U << 1U)
#define BMP3_ADDRESS_LOW_8BIT       (0x76U << 1U)
#define BMP3_ADDRESS_HIGH_8BIT      (0x77U << 1U)
#define BMP3_CHIP_ID_REGISTER       0x00U
#define BMP3_ERROR_REGISTER         0x02U
#define BMP3_STATUS_REGISTER        0x03U
#define BMP3_DATA_REGISTER          0x04U
#define BMP3_POWER_CONTROL_REGISTER 0x1BU
#define BMP3_OSR_REGISTER           0x1CU
#define BMP3_ODR_REGISTER           0x1DU
#define BMP3_CONFIG_REGISTER        0x1FU
#define BMP3_CALIB_REGISTER         0x31U
#define BMP3_COMMAND_REGISTER       0x7EU
#define BMP3_CHIP_ID_BMP388         0x50U
#define BMP3_CHIP_ID_BMP390         0x60U
#define BMP3_COMMAND_READY          0x10U
#define BMP3_DATA_READY             0x60U
#define BMP3_SOFT_RESET_COMMAND     0xB6U
#define BMP3_OSR_PRESS_X8_TEMP_X2   0x0BU
#define BMP3_ODR_25_HZ              0x03U
#define BMP3_IIR_COEFFICIENT_3      0x04U
#define BMP3_PRESS_TEMP_NORMAL_MODE 0x33U
#define BMP3_PRESS_TEMP_SLEEP_MODE  0x03U
#define BMP3_CALIBRATION_LENGTH     21U
#define BMP3_DATA_LENGTH            6U
#define BMP3_DATA_READY_TIMEOUT_MS  250U
#define BMP5_ADDRESS_LOW_8BIT       (0x46U << 1U)
#define BMP5_ADDRESS_HIGH_8BIT      (0x47U << 1U)
#define BMP5_CHIP_ID_REGISTER       0x01U
#define BMP5_STATUS_REGISTER        0x28U
#define BMP5_INT_STATUS_REGISTER    0x27U
#define BMP5_INT_CONFIG_REGISTER    0x14U
#define BMP5_INT_SOURCE_REGISTER    0x15U
#define BMP5_DATA_REGISTER          0x1DU
#define BMP5_OSR_REGISTER           0x36U
#define BMP5_ODR_REGISTER           0x37U
#define BMP5_OSR_EFFECTIVE_REGISTER 0x38U
#define BMP5_COMMAND_REGISTER       0x7EU
#define BMP5_CHIP_ID_PRIMARY        0x50U
#define BMP5_CHIP_ID_SECONDARY      0x51U
#define BMP5_NVM_READY              0x02U
#define BMP5_NVM_ERROR              0x04U
#define BMP5_DATA_READY             0x01U
#define BMP5_RESET_COMPLETE         0x10U
#define BMP5_INTERRUPT_DRDY_CONFIG  0x0AU
#define BMP5_INTERRUPT_DRDY_SOURCE  0x01U
#define BMP5_SOFT_RESET_COMMAND     0xB6U
#define BMP5_OSR_PRESS_X8_TEMP_X2   0x59U
#define BMP5_ODR_10_HZ_NORMAL       0xDDU
#define BMP5_ODR_10_HZ_STANDBY      0xDCU
#define BMP5_DATA_LENGTH            6U
#define BMP5_DATA_READY_TIMEOUT_MS  300U
#define SHT40_ADDRESS_PRIMARY_8BIT  (0x44U << 1U)
#define SHT40_ADDRESS_SECONDARY_8BIT (0x45U << 1U)
#define SHT40_MEASURE_HIGH_PRECISION 0xFDU
#define SHT40_MEASUREMENT_DELAY_MS  10U
#define SHT40_MEASUREMENT_LENGTH    6U
#define I2C_DIAG_STANDARD_CLOCK_HZ  100000U
#define GNSS_CAPTURE_SIZE           160U
#define GNSS_MIN_DURATION_MS        100U
#define GNSS_MAX_DURATION_MS        3000U
#define BOARD_HARDWARE_VERSION      "V1.0.2"
#define BOARD_FIRMWARE_VERSION      "V1.0.2.10"
#define ICM42688_DEVICE_CONFIG_REGISTER 0x11U
#define ICM42688_ACCEL_DATA_X1_REGISTER 0x1FU
#define ICM42688_PWR_MGMT0_REGISTER     0x4EU
#define ICM42688_GYRO_CONFIG0_REGISTER  0x4FU
#define ICM42688_ACCEL_CONFIG0_REGISTER 0x50U
#define ICM42688_WHO_AM_I_REGISTER  0x75U
#define ICM42688_WHO_AM_I_EXPECTED  0x47U
#define ICM42688_REG_BANK_SEL_REGISTER  0x76U
#define ICM42688_BANK0                  0x00U
#define ICM42688_ODR_1KHZ_DEFAULT_FS    0x06U
#define ICM42688_SIX_AXIS_LOW_NOISE     0x0FU
#define ICM42688_RAW_DATA_LENGTH        12U
#define ICM42688_SPI_TRANSFER_MAX       16U
#define SPI_READ_BIT                0x80U
#define SD_TEST_FILE_PATH           "FC_SD.TMP"
#define SD_TEST_BUFFER_SIZE         128U

/*
 * Logical forward/reverse is kept stable in commands and telemetry. Set an
 * individual value to 1 only when the assembled mechanism runs opposite to
 * the intended logical direction. This is a wiring/mechanics calibration and
 * does not require a CubeMX change.
 */
#define PUMP1_FORWARD_INVERTED       0U
#define PUMP2_FORWARD_INVERTED       0U
#define MOTOR1_FORWARD_INVERTED      0U
#define MOTOR2_FORWARD_INVERTED      0U

typedef struct
{
  bool valid;
  BalloonRadioType type;
  uint16_t sequence;
  uint8_t payload_length;
  uint8_t frame_length;
} FlightBoardPendingRadioTx;

typedef struct
{
  bool valid;
  BalloonAckPayload payload;
} FlightBoardPendingAck;

typedef struct
{
  uint16_t par_t1;
  uint16_t par_t2;
  int8_t par_t3;
  int16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int8_t par_p4;
  uint16_t par_p5;
  uint16_t par_p6;
  int8_t par_p7;
  int8_t par_p8;
  int16_t par_p9;
  int8_t par_p10;
  int8_t par_p11;
  int64_t t_lin;
} FlightBoardBmp3Calibration;

typedef struct
{
  bool valid;
  uint8_t channel;
  uint8_t address_7bit;
  uint8_t chip_id;
  int32_t temperature_x100;
  uint32_t pressure_x100;
  const char *model;
} FlightBoardBarometerSample;

static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static uint8_t usb_rx_queue[USB_RX_QUEUE_SIZE];
static char usb_tx_buffer[384];
static bool imu_configured;
static E28Sx1281 radio;
static bool radio_attached;
static bool radio_tx_armed;
static uint32_t radio_tx_arm_deadline;
static uint16_t radio_sequence;
static FlightBoardPendingRadioTx pending_radio_tx;
static bool radio_power_sampled;
static bool radio_power_cached;
static bool radio_bus_forced_off;
static uint32_t radio_power_sample_tick;
static uint8_t radio_power_adc_failures;

typedef enum
{
  BOARD_ACTION_NONE = 0,
  BOARD_ACTION_VALVE,
  BOARD_ACTION_PUMP,
  BOARD_ACTION_MOTOR,
  BOARD_ACTION_SERVO
} BoardActionType;

typedef struct
{
  BoardActionType type;
  uint8_t channel;
  uint32_t value;
  uint32_t deadline;
  bool reverse;
} BoardActionState;

typedef struct
{
  bool active;
  BalloonCommandCode command;
  uint16_t sequence;
} FlightBoardRemoteAction;

typedef enum
{
  FLIGHT_LOG_CONTROL_START = 1,
  FLIGHT_LOG_DATA,
  FLIGHT_LOG_EVENT,
  FLIGHT_LOG_CONTROL_STOP
} FlightBoardLogRecordKind;

typedef enum
{
  FLIGHT_LOG_EVENT_MISSION_START = 1,
  FLIGHT_LOG_EVENT_MISSION_STOP,
  FLIGHT_LOG_EVENT_COMMAND_ACK,
  FLIGHT_LOG_EVENT_FAILSAFE,
  FLIGHT_LOG_EVENT_RADIO_ERROR
} FlightBoardLogEventKind;

typedef struct
{
  uint8_t event;
  uint8_t command;
  uint8_t stage;
  uint8_t reason;
  uint16_t command_sequence;
  uint8_t action;
  uint8_t action_channel;
  uint8_t action_reverse;
  uint16_t action_value;
} FlightBoardLogEvent;

typedef struct
{
  uint8_t kind;
  uint32_t timestamp_ms;
  union
  {
    BalloonTelemetryPayload telemetry;
    FlightBoardLogEvent event;
  } payload;
} FlightBoardLogRecord;

_Static_assert(sizeof(FlightBoardLogRecord) <= 64U,
               "FlightBoardLogRecord must remain queue friendly");

static bool outputs_armed;
static BoardActionState active_action;
static BalloonSystemMode system_mode = BALLOON_SYSTEM_MODE_MAINTENANCE;
static uint32_t mission_last_ground_tick;
static uint32_t mission_next_telemetry_tick;
static uint16_t mission_last_command_sequence;
static uint32_t mission_last_issued_ms;
static bool mission_has_command_history;
static bool mission_clock_synced;
static uint32_t mission_ground_clock_offset;
static bool mission_telemetry_due;
static uint16_t mission_telemetry_sequence;
static bool mission_telemetry_is_response;
static FlightBoardPendingAck mission_pending_ack;
static FlightBoardRemoteAction remote_action;
static BalloonCsvLogger mission_data_log;
static BalloonCsvLogger mission_event_log;
static volatile BalloonLogState mission_log_state = BALLOON_LOG_STATE_OFF;
static bool mission_log_mounted;
static uint32_t mission_next_log_tick;
static uint32_t mission_log_sequence;
static uint32_t mission_event_log_sequence;
static char mission_csv_line[512];
static QueueHandle_t mission_log_queue;
static TaskHandle_t mission_log_task_handle;
static volatile bool mission_log_owns_sd;
static volatile bool mission_log_stop_requested;
static volatile uint32_t mission_log_drop_count;
static volatile FRESULT mission_log_last_result = FR_NOT_READY;

extern volatile uint32_t g_sd_detect;
extern volatile uint32_t g_sd_init_hal;
extern volatile uint32_t g_sd_init_error;
extern volatile uint32_t g_sd_card_state;
extern volatile uint32_t g_sd_hal_state;
extern volatile uint32_t g_sd_cmd_level;
extern volatile uint32_t g_sd_d0_level;
extern volatile uint32_t g_sd_clk_level;

static void FlightBoardTest_SendImuDetails(uint8_t who_am_i,
                                           HAL_StatusTypeDef who_result);
static void FlightBoardTest_Send(const char *format, ...);
static HAL_StatusTypeDef FlightBoardTest_ReadAdc(uint32_t *raw,
                                                 uint32_t *pin_mv);
static bool FlightBoardTest_IsMissionSessionActive(void);
static void FlightBoardTest_StopActuators(void);
static void FlightBoardTest_HandleMissionCommand(const BalloonRadioFrame *frame);
static void FlightBoardTest_ServiceMission(void);
static void FlightBoardTest_ServiceMissionLogging(void);
static void FlightBoardTest_StartMissionLogging(void);
static void FlightBoardTest_StopMissionLogging(void);
static void FlightBoardTest_LogEvent(FlightBoardLogEventKind event,
                                     BalloonCommandCode command,
                                     BalloonAckStage stage,
                                     BalloonRejectReason reason,
                                     uint16_t command_sequence);
static void FlightBoardTest_LogEventForAction(
    FlightBoardLogEventKind event,
    BalloonCommandCode command,
    BalloonAckStage stage,
    BalloonRejectReason reason,
    uint16_t command_sequence,
    const BoardActionState *action);

static void FlightBoardTest_ConfigureOutput(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pins;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &gpio);
}

static void FlightBoardTest_ConfigureAnalog(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pins;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &gpio);
}

static void FlightBoardTest_SetRadioBusEnabled(void *context, bool enabled)
{
  (void)context;

  HAL_GPIO_WritePin(GPIOC, RADIO_RXEN_Pin | RADIO_TXEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(
      SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);

  if (enabled)
  {
    MX_SPI2_Init();
    HAL_GPIO_WritePin(
        SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);
  }
  else
  {
    if ((hspi2.Instance == SPI2) &&
        (hspi2.State != HAL_SPI_STATE_RESET))
    {
      (void)HAL_SPI_DeInit(&hspi2);
    }
    FlightBoardTest_ConfigureAnalog(
        GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  }
}

static bool FlightBoardTest_RadioPowerPresent(void *context)
{
  uint32_t now = HAL_GetTick();
  uint32_t raw = 0U;
  uint32_t pin_mv = 0U;
  uint32_t vbat_mv;

  (void)context;
  if (radio_bus_forced_off)
  {
    return false;
  }
  if (radio_power_sampled &&
      (radio.mode == E28_SX1281_MODE_TX))
  {
    return radio_power_cached;
  }
  if (!radio_power_sampled ||
      ((uint32_t)(now - radio_power_sample_tick) >= RADIO_POWER_SAMPLE_MS))
  {
    radio_power_sampled = true;
    radio_power_sample_tick = now;
    if (FlightBoardTest_ReadAdc(&raw, &pin_mv) == HAL_OK)
    {
      radio_power_adc_failures = 0U;
      vbat_mv = pin_mv * ADC_VBAT_DIVIDER_MULTIPLIER;
      if (!radio_power_cached &&
          (vbat_mv >= RADIO_POWER_ENABLE_VBAT_MV))
      {
        radio_power_cached = true;
      }
      else if (radio_power_cached &&
               (vbat_mv <= RADIO_POWER_DISABLE_VBAT_MV))
      {
        radio_power_cached = false;
      }
    }
    else
    {
      if (radio_power_adc_failures < 2U)
      {
        ++radio_power_adc_failures;
      }
      if (radio_power_adc_failures >= 2U)
      {
        radio_power_cached = false;
      }
    }
  }
  return radio_power_cached;
}

static void FlightBoardTest_ServiceRadioPower(void)
{
  if (radio_attached)
  {
    bool was_present = radio.power_present;
    bool present = E28Sx1281_IsPowerPresent(&radio);

    if (!present)
    {
      pending_radio_tx.valid = false;
      radio_tx_armed = false;
      radio_tx_arm_deadline = 0U;
      if (was_present)
      {
        if (FlightBoardTest_IsMissionSessionActive())
        {
          system_mode = BALLOON_SYSTEM_MODE_FAILSAFE;
          FlightBoardTest_StopActuators();
          outputs_armed = false;
          remote_action.active = false;
        }
        FlightBoardTest_Send(
            "FC radio power_lost bus=high_z controls=low tx_arm=0\r\n");
      }
    }
  }
}

void FlightBoardTest_EarlySafetyInit(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC,
                    VALVE1_Pin | VALVE2_Pin | RADIO_RXEN_Pin | RADIO_TXEN_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_0 | GPIO_PIN_1 | PUMP2_IN2_Pin | PUMP2_IN1_Pin |
                        MOTOR_SLEEP_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | PUMP1_IN1_Pin | RADIO_RST_Pin |
                        PUMP1_IN2_Pin | GPIO_PIN_4 | GPIO_PIN_5,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(
      SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);

  FlightBoardTest_ConfigureOutput(
      GPIOC, VALVE1_Pin | VALVE2_Pin | RADIO_RXEN_Pin | RADIO_TXEN_Pin);
  FlightBoardTest_ConfigureOutput(
      GPIOA,
      GPIO_PIN_0 | GPIO_PIN_1 | PUMP2_IN2_Pin | PUMP2_IN1_Pin |
          SPI1_CS_IMU_Pin | MOTOR_SLEEP_Pin);
  FlightBoardTest_ConfigureOutput(
      GPIOB,
      GPIO_PIN_0 | GPIO_PIN_1 | PUMP1_IN1_Pin | RADIO_RST_Pin |
          SPI2_CS_RADIO_Pin | PUMP1_IN2_Pin | GPIO_PIN_4 | GPIO_PIN_5);
}

static void FlightBoardTest_StopActuators(void)
{
  /* Disable both H-bridge devices before touching their PWM inputs. */
  HAL_GPIO_WritePin(GPIOC, VALVE1_Pin | VALVE2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_0 | GPIO_PIN_1 | PUMP2_IN2_Pin | PUMP2_IN1_Pin |
                        MOTOR_SLEEP_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | PUMP1_IN1_Pin | PUMP1_IN2_Pin |
                        GPIO_PIN_4 | GPIO_PIN_5,
                    GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

  /* TIM2/TIM3 pins are returned to explicit low GPIO outputs after a test. */
  FlightBoardTest_ConfigureOutput(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
  FlightBoardTest_ConfigureOutput(
      GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5,
                    GPIO_PIN_RESET);

  memset(&active_action, 0, sizeof(active_action));
}

static void FlightBoardTest_EnforceRadioLockdown(void)
{
  if (radio_attached)
  {
    E28Sx1281_ForceSafeReset(&radio);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC,
                      RADIO_RXEN_Pin | RADIO_TXEN_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(
        SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);
  }
  radio_tx_armed = false;
  radio_tx_arm_deadline = 0U;
  pending_radio_tx.valid = false;
  mission_pending_ack.valid = false;
  mission_telemetry_due = false;
  mission_telemetry_is_response = false;
}

static void FlightBoardTest_EnforceAllSafety(void)
{
  FlightBoardTest_StopActuators();
  FlightBoardTest_EnforceRadioLockdown();
}

void FlightBoardTest_OnUsbData(const uint8_t *data, uint32_t length)
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

static bool FlightBoardTest_TryReadUsbByte(uint8_t *value)
{
  if ((value == NULL) || (usb_rx_tail == usb_rx_head))
  {
    return false;
  }

  *value = usb_rx_queue[usb_rx_tail];
  usb_rx_tail = (uint16_t)((usb_rx_tail + 1U) % USB_RX_QUEUE_SIZE);
  return true;
}

static void FlightBoardTest_Send(const char *format, ...)
{
  va_list arguments;
  int length;
  uint32_t attempts;

  va_start(arguments, format);
  length = vsnprintf(usb_tx_buffer, sizeof(usb_tx_buffer), format, arguments);
  va_end(arguments);

  if (length <= 0)
  {
    return;
  }
  if ((size_t)length >= sizeof(usb_tx_buffer))
  {
    length = (int)sizeof(usb_tx_buffer) - 1;
  }

  for (attempts = 0U; attempts < 100U; ++attempts)
  {
    if (CDC_Transmit_FS((uint8_t *)usb_tx_buffer, (uint16_t)length) == USBD_OK)
    {
      osDelay(5U);
      return;
    }
    osDelay(1U);
  }
}

static const char *FlightBoardTest_ActionName(BoardActionType type)
{
  switch (type)
  {
    case BOARD_ACTION_VALVE:
      return "valve";
    case BOARD_ACTION_PUMP:
      return "pump";
    case BOARD_ACTION_MOTOR:
      return "motor";
    case BOARD_ACTION_SERVO:
      return "servo";
    default:
      return "none";
  }
}

static const char *FlightBoardTest_ActionDirection(const BoardActionState *action)
{
  if ((action == NULL) ||
      ((action->type != BOARD_ACTION_PUMP) &&
       (action->type != BOARD_ACTION_MOTOR)))
  {
    return "na";
  }
  return action->reverse ? "reverse" : "forward";
}

static bool FlightBoardTest_DirectionIsInverted(BoardActionType type,
                                                 uint32_t channel)
{
  if (type == BOARD_ACTION_PUMP)
  {
    return channel == 1U ? (PUMP1_FORWARD_INVERTED != 0U)
                         : (PUMP2_FORWARD_INVERTED != 0U);
  }
  if (type == BOARD_ACTION_MOTOR)
  {
    return channel == 1U ? (MOTOR1_FORWARD_INVERTED != 0U)
                         : (MOTOR2_FORWARD_INVERTED != 0U);
  }
  return false;
}

static bool FlightBoardTest_DeadlineExpired(uint32_t deadline)
{
  return (int32_t)(HAL_GetTick() - deadline) >= 0;
}

static uint32_t FlightBoardTest_RemainingMs(uint32_t deadline)
{
  uint32_t now = HAL_GetTick();

  if ((deadline == 0U) || ((int32_t)(now - deadline) >= 0))
  {
    return 0U;
  }
  return deadline - now;
}

static bool FlightBoardTest_IsMissionSessionActive(void)
{
  return (system_mode == BALLOON_SYSTEM_MODE_MISSION) ||
         (system_mode == BALLOON_SYSTEM_MODE_FAILSAFE);
}

static void FlightBoardTest_SetPendingAckForAction(
    uint16_t sequence,
    BalloonCommandCode command,
    BalloonAckStage stage,
    BalloonRejectReason reason,
    const BoardActionState *action)
{
  const BoardActionState *ack_action = action != NULL ? action : &active_action;

  mission_pending_ack.valid = true;
  mission_pending_ack.payload.command = command;
  mission_pending_ack.payload.stage = stage;
  mission_pending_ack.payload.reason = reason;
  mission_pending_ack.payload.mission_id = MISSION_ID;
  mission_pending_ack.payload.command_sequence = sequence;
  mission_pending_ack.payload.system_mode = system_mode;
  mission_pending_ack.payload.action = (uint8_t)ack_action->type;
  mission_pending_ack.payload.channel = ack_action->channel;
  FlightBoardTest_LogEventForAction(FLIGHT_LOG_EVENT_COMMAND_ACK,
                                    command,
                                    stage,
                                    reason,
                                    sequence,
                                    ack_action);
}

static void FlightBoardTest_SetPendingAck(uint16_t sequence,
                                          BalloonCommandCode command,
                                          BalloonAckStage stage,
                                          BalloonRejectReason reason)
{
  FlightBoardTest_SetPendingAckForAction(sequence,
                                         command,
                                         stage,
                                         reason,
                                         &active_action);
}

static void FlightBoardTest_ServiceRadioArm(void)
{
  if (!FlightBoardTest_IsMissionSessionActive() && radio_tx_armed &&
      FlightBoardTest_DeadlineExpired(radio_tx_arm_deadline))
  {
    radio_tx_armed = false;
    radio_tx_arm_deadline = 0U;
    E28Sx1281_SetTransmitPermission(&radio, false);
    FlightBoardTest_Send("FC radio tx_arm expired; transmit locked\r\n");
  }
}

static bool FlightBoardTest_IsRadioTxArmed(void)
{
  FlightBoardTest_ServiceRadioArm();
  return radio_tx_armed;
}

static void FlightBoardTest_ProbeRadio(void)
{
  uint8_t chip_status = 0U;
  uint8_t packet_type = 0xFFU;
  HAL_StatusTypeDef result;

  radio_tx_armed = false;
  radio_tx_arm_deadline = 0U;
  pending_radio_tx.valid = false;
  result = E28Sx1281_Probe(&radio, &chip_status, &packet_type);
  FlightBoardTest_Send(
      "FC radio probe power_present=%u hal=%u status=0x%02X packet_type=0x%02X "
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

static HAL_StatusTypeDef FlightBoardTest_InitializeRadio(void)
{
  HAL_StatusTypeDef result;

  radio_tx_armed = false;
  radio_tx_arm_deadline = 0U;
  pending_radio_tx.valid = false;
  result = E28Sx1281_Initialize(&radio);
  FlightBoardTest_Send(
      "FC radio init power_present=%u hal=%u mode=%s freq_hz=%lu chip_power_dbm=%d "
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

static bool FlightBoardTest_QueueRadioFrame(BalloonRadioType type,
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
  if (!FlightBoardTest_IsRadioTxArmed())
  {
    FlightBoardTest_Send(
        "FC radio tx rejected reason=locked command=radio_arm_antenna\r\n");
    return false;
  }
  if (!BalloonRadio_Encode(type,
                           BALLOON_RADIO_SOURCE_FLIGHT,
                           sequence,
                           payload,
                           payload_length,
                           encoded,
                           sizeof(encoded),
                           &encoded_length))
  {
    FlightBoardTest_Send("FC radio tx rejected reason=frame_encode\r\n");
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

  FlightBoardTest_Send("FC radio tx failed type=%s seq=%u hal=%u error=%s\r\n",
                       BalloonRadio_TypeName(type),
                       (unsigned int)sequence,
                       (unsigned int)result,
                       E28Sx1281_ErrorName(radio.last_error));
  return false;
}

static void FlightBoardTest_HandleRadioPacket(const E28Sx1281Packet *packet)
{
  BalloonRadioFrame frame;
  char text[BALLOON_RADIO_MAX_PAYLOAD + 1U];
  uint8_t index;

  if (!BalloonRadio_Decode(packet->data, packet->length, &frame))
  {
    FlightBoardTest_Send(
        "FC radio rx frame_len=%u rssi_dbm=%d snr_db=%d protocol=invalid\r\n",
        (unsigned int)packet->length,
        packet->rssi_dbm,
        packet->snr_db);
    return;
  }
  if (frame.source != BALLOON_RADIO_SOURCE_GROUND)
  {
    FlightBoardTest_Send(
        "FC radio rx source=%s protocol=source_rejected expected=ground\r\n",
        BalloonRadio_SourceName(frame.source));
    return;
  }

  if (FlightBoardTest_IsMissionSessionActive())
  {
    mission_last_ground_tick = HAL_GetTick();
  }

  if (frame.type == BALLOON_RADIO_TYPE_COMMAND)
  {
    FlightBoardTest_Send(
        "FC mission command received seq=%u payload_len=%u rssi_dbm=%d snr_db=%d\r\n",
        (unsigned int)frame.sequence,
        (unsigned int)frame.payload_length,
        packet->rssi_dbm,
        packet->snr_db);
    FlightBoardTest_HandleMissionCommand(&frame);
    return;
  }

  for (index = 0U; index < frame.payload_length; ++index)
  {
    uint8_t value = frame.payload[index];
    text[index] = (value >= 0x20U) && (value <= 0x7EU) ? (char)value : '.';
  }
  text[frame.payload_length] = '\0';
  if (frame.type == BALLOON_RADIO_TYPE_PING)
  {
    if (FlightBoardTest_IsMissionSessionActive() &&
        (frame.payload_length == 4U))
    {
      uint32_t ground_tick = (uint32_t)frame.payload[0] |
                             ((uint32_t)frame.payload[1] << 8U) |
                             ((uint32_t)frame.payload[2] << 16U) |
                             ((uint32_t)frame.payload[3] << 24U);
      mission_ground_clock_offset = HAL_GetTick() - ground_tick;
      mission_clock_synced = true;
    }
    if (active_action.type != BOARD_ACTION_NONE)
    {
      FlightBoardTest_Send(
          "FC radio pong blocked reason=actuator_active\r\n");
    }
    else if (FlightBoardTest_IsRadioTxArmed())
    {
      (void)FlightBoardTest_QueueRadioFrame(BALLOON_RADIO_TYPE_PONG,
                                            frame.sequence,
                                            frame.payload,
                                            frame.payload_length);
    }
    else
    {
      FlightBoardTest_Send(
          "FC radio pong blocked reason=tx_locked; run=radio arm antenna\r\n");
    }
  }
  else
  {
    FlightBoardTest_Send(
        "FC radio rx type=%s source=%s seq=%u payload_len=%u "
        "rssi_dbm=%d snr_db=%d protocol=ok data=%s\r\n",
        BalloonRadio_TypeName(frame.type),
        BalloonRadio_SourceName(frame.source),
        (unsigned int)frame.sequence,
        (unsigned int)frame.payload_length,
        packet->rssi_dbm,
        packet->snr_db,
        frame.payload_length > 0U ? text : "<empty>");
  }
}

static void FlightBoardTest_ServiceRadio(void)
{
  E28Sx1281Packet packet;
  E28Sx1281Event event = E28Sx1281_Service(&radio, &packet);

  switch (event)
  {
    case E28_SX1281_EVENT_TX_DONE:
      if (pending_radio_tx.valid)
      {
        FlightBoardTest_Send(
            "FC radio tx_done type=%s seq=%u payload_len=%u frame_len=%u "
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
        FlightBoardTest_Send("FC radio tx_done mode=%s count=%lu\r\n",
                             E28Sx1281_ModeName(radio.mode),
                             (unsigned long)radio.tx_count);
      }
      pending_radio_tx.valid = false;
      break;
    case E28_SX1281_EVENT_TX_TIMEOUT:
      pending_radio_tx.valid = false;
      radio_tx_armed = false;
      radio_tx_arm_deadline = 0U;
      if (FlightBoardTest_IsMissionSessionActive())
      {
        system_mode = BALLOON_SYSTEM_MODE_FAILSAFE;
        FlightBoardTest_StopActuators();
        outputs_armed = false;
        remote_action.active = false;
        FlightBoardTest_LogEvent(FLIGHT_LOG_EVENT_RADIO_ERROR,
                                 (BalloonCommandCode)0,
                                 (BalloonAckStage)0,
                                 BALLOON_REJECT_HARDWARE,
                                 0U);
      }
      FlightBoardTest_Send("FC radio tx_timeout mode=%s result=FAIL\r\n",
                           E28Sx1281_ModeName(radio.mode));
      break;
    case E28_SX1281_EVENT_RX_DONE:
      FlightBoardTest_HandleRadioPacket(&packet);
      break;
    case E28_SX1281_EVENT_RX_CRC_ERROR:
      FlightBoardTest_Send("FC radio rx_error type=crc restarted=1 mode=%s\r\n",
                           E28Sx1281_ModeName(radio.mode));
      break;
    case E28_SX1281_EVENT_RX_HEADER_ERROR:
      FlightBoardTest_Send(
          "FC radio rx_error type=header restarted=1 mode=%s\r\n",
          E28Sx1281_ModeName(radio.mode));
      break;
    case E28_SX1281_EVENT_ERROR:
      pending_radio_tx.valid = false;
      radio_tx_armed = false;
      radio_tx_arm_deadline = 0U;
      if (FlightBoardTest_IsMissionSessionActive())
      {
        system_mode = BALLOON_SYSTEM_MODE_FAILSAFE;
        FlightBoardTest_StopActuators();
        outputs_armed = false;
        remote_action.active = false;
      }
      FlightBoardTest_Send("FC radio service error=%s mode=%s safe_reset=1\r\n",
                           E28Sx1281_ErrorName(radio.last_error),
                           E28Sx1281_ModeName(radio.mode));
      break;
    default:
      break;
  }
}

static void FlightBoardTest_ConfigureAlternate(GPIO_TypeDef *port,
                                               uint16_t pins,
                                               uint32_t alternate)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pins;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = alternate;
  HAL_GPIO_Init(port, &gpio);
}

static void FlightBoardTest_SetActiveAction(BoardActionType type,
                                            uint8_t channel,
                                            uint32_t value,
                                            uint32_t duration_ms,
                                            bool reverse)
{
  active_action.type = type;
  active_action.channel = channel;
  active_action.value = value;
  active_action.deadline = HAL_GetTick() + duration_ms;
  active_action.reverse = reverse;
}

static void FlightBoardTest_ServiceSafety(void)
{
  if ((active_action.type != BOARD_ACTION_NONE) &&
      FlightBoardTest_DeadlineExpired(active_action.deadline))
  {
    BoardActionState stopped = active_action;
    FlightBoardRemoteAction completed_remote = remote_action;
    FlightBoardTest_StopActuators();
    remote_action.active = false;
    if (completed_remote.active)
    {
      FlightBoardTest_SetPendingAckForAction(completed_remote.sequence,
                                             completed_remote.command,
                                             BALLOON_ACK_COMPLETED,
                                             BALLOON_REJECT_NONE,
                                             &stopped);
      mission_telemetry_due = true;
      mission_telemetry_sequence = completed_remote.sequence;
    }
    FlightBoardTest_Send(
        "FC output auto_stop action=%s channel=%u reason=duration_complete\r\n",
        FlightBoardTest_ActionName(stopped.type),
        (unsigned int)stopped.channel);
  }
}

static bool FlightBoardTest_CheckActionReady(uint32_t duration_ms,
                                             bool uses_motor_driver)
{
  FlightBoardTest_ServiceSafety();
  if (radio.mode == E28_SX1281_MODE_TX)
  {
    FlightBoardTest_Send(
        "FC output rejected reason=radio_tx_active wait=tx_done\r\n");
    return false;
  }
  if (!outputs_armed)
  {
    FlightBoardTest_Send(
        "FC output rejected reason=disarmed command=arm_outputs_first\r\n");
    return false;
  }
  if (active_action.type != BOARD_ACTION_NONE)
  {
    FlightBoardTest_Send(
        "FC output rejected reason=busy action=%s channel=%u\r\n",
        FlightBoardTest_ActionName(active_action.type),
        (unsigned int)active_action.channel);
    return false;
  }
  if ((duration_ms < ACTUATOR_MIN_DURATION_MS) ||
      (duration_ms > ACTUATOR_MAX_DURATION_MS))
  {
    FlightBoardTest_Send(
        "FC output rejected reason=duration_range min=%u max=%u\r\n",
        (unsigned int)ACTUATOR_MIN_DURATION_MS,
        (unsigned int)ACTUATOR_MAX_DURATION_MS);
    return false;
  }
  if (uses_motor_driver &&
      (HAL_GPIO_ReadPin(MOTOR_FAULT_GPIO_Port, MOTOR_FAULT_Pin) == GPIO_PIN_RESET))
  {
    FlightBoardTest_Send(
        "FC output rejected reason=motor_fault_active fault_pin=0\r\n");
    return false;
  }
  return true;
}

static void FlightBoardTest_SendOutputState(void)
{
  const char *direction = "n/a";
  uint32_t action_remaining =
      FlightBoardTest_RemainingMs(active_action.deadline);

  if ((active_action.type == BOARD_ACTION_PUMP) ||
      (active_action.type == BOARD_ACTION_MOTOR))
  {
    direction = active_action.reverse ? "reverse" : "forward";
  }

  FlightBoardTest_Send(
      "FC outputs armed=%u arm_timeout=disabled action=%s channel=%u "
      "value=%lu direction=%s remaining_ms=%lu radio=%s\r\n",
      outputs_armed ? 1U : 0U,
      FlightBoardTest_ActionName(active_action.type),
      (unsigned int)active_action.channel,
      (unsigned long)active_action.value,
      direction,
      (unsigned long)action_remaining,
      E28Sx1281_ModeName(radio.mode));
}

static bool FlightBoardTest_StartValve(uint32_t channel, uint32_t duration_ms)
{
  GPIO_TypeDef *port;
  uint16_t pin;

  if ((channel < 1U) || (channel > 2U))
  {
    FlightBoardTest_Send("FC valve rejected reason=channel_range valid=1..2\r\n");
    return false;
  }
  if (!FlightBoardTest_CheckActionReady(duration_ms, false))
  {
    return false;
  }

  port = (channel == 1U) ? VALVE1_GPIO_Port : VALVE2_GPIO_Port;
  pin = (channel == 1U) ? VALVE1_Pin : VALVE2_Pin;
  FlightBoardTest_SetActiveAction(
      BOARD_ACTION_VALVE, (uint8_t)channel, 1U, duration_ms, false);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  FlightBoardTest_Send("FC valve start channel=%u duration_ms=%lu\r\n",
                      (unsigned int)channel,
                      (unsigned long)duration_ms);
  return true;
}

static bool FlightBoardTest_StartPump(uint32_t channel,
                                     bool reverse,
                                     uint32_t duration_ms)
{
  GPIO_TypeDef *in1_port;
  GPIO_TypeDef *in2_port;
  uint16_t in1_pin;
  uint16_t in2_pin;
  bool drive_reverse;

  if ((channel < 1U) || (channel > 2U))
  {
    FlightBoardTest_Send("FC pump rejected reason=channel_range valid=1..2\r\n");
    return false;
  }
  if (!FlightBoardTest_CheckActionReady(duration_ms, true))
  {
    return false;
  }

  if (channel == 1U)
  {
    in1_port = PUMP1_IN1_GPIO_Port;
    in1_pin = PUMP1_IN1_Pin;
    in2_port = PUMP1_IN2_GPIO_Port;
    in2_pin = PUMP1_IN2_Pin;
  }
  else
  {
    in1_port = PUMP2_IN1_GPIO_Port;
    in1_pin = PUMP2_IN1_Pin;
    in2_port = PUMP2_IN2_GPIO_Port;
    in2_pin = PUMP2_IN2_Pin;
  }

  FlightBoardTest_SetActiveAction(
      BOARD_ACTION_PUMP, (uint8_t)channel, 100U, duration_ms, reverse);
  drive_reverse = reverse ^
                  FlightBoardTest_DirectionIsInverted(BOARD_ACTION_PUMP,
                                                       channel);
  HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_SET);
  osDelay(1U);
  HAL_GPIO_WritePin(in1_port,
                    in1_pin,
                    drive_reverse ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(in2_port,
                    in2_pin,
                    drive_reverse ? GPIO_PIN_SET : GPIO_PIN_RESET);
  FlightBoardTest_Send(
      "FC pump start channel=%u direction=%s drive_direction=%s duration_ms=%lu\r\n",
      (unsigned int)channel,
      reverse ? "reverse" : "forward",
      drive_reverse ? "reverse" : "forward",
      (unsigned long)duration_ms);
  return true;
}

static bool FlightBoardTest_StartMotor(uint32_t channel,
                                      bool reverse,
                                      uint32_t duty_percent,
                                      uint32_t duration_ms)
{
  uint32_t in1_channel;
  uint32_t in2_channel;
  uint32_t active_channel;
  uint32_t compare;
  HAL_StatusTypeDef in1_result;
  HAL_StatusTypeDef in2_result;
  bool drive_reverse;

  if ((channel < 1U) || (channel > 2U))
  {
    FlightBoardTest_Send("FC motor rejected reason=channel_range valid=1..2\r\n");
    return false;
  }
  if ((duty_percent < 1U) || (duty_percent > MOTOR_TEST_MAX_DUTY_PERCENT))
  {
    FlightBoardTest_Send(
        "FC motor rejected reason=duty_range min=1 max=%u\r\n",
        (unsigned int)MOTOR_TEST_MAX_DUTY_PERCENT);
    return false;
  }
  if (!FlightBoardTest_CheckActionReady(duration_ms, true))
  {
    return false;
  }

  if (channel == 1U)
  {
    FlightBoardTest_ConfigureAlternate(
        GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_AF2_TIM3);
    in1_channel = TIM_CHANNEL_3;
    in2_channel = TIM_CHANNEL_4;
  }
  else
  {
    FlightBoardTest_ConfigureAlternate(
        GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_AF2_TIM3);
    in1_channel = TIM_CHANNEL_1;
    in2_channel = TIM_CHANNEL_2;
  }

  __HAL_TIM_SET_COMPARE(&htim3, in1_channel, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, in2_channel, 0U);
  in1_result = HAL_TIM_PWM_Start(&htim3, in1_channel);
  in2_result = HAL_TIM_PWM_Start(&htim3, in2_channel);
  if ((in1_result != HAL_OK) || (in2_result != HAL_OK))
  {
    FlightBoardTest_StopActuators();
    FlightBoardTest_Send("FC motor rejected reason=pwm_start hal=%u/%u\r\n",
                        (unsigned int)in1_result,
                        (unsigned int)in2_result);
    return false;
  }

  compare = ((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U) * duty_percent) / 100U;
  drive_reverse = reverse ^
                  FlightBoardTest_DirectionIsInverted(BOARD_ACTION_MOTOR,
                                                       channel);
  active_channel = drive_reverse ? in2_channel : in1_channel;
  FlightBoardTest_SetActiveAction(
      BOARD_ACTION_MOTOR, (uint8_t)channel, duty_percent, duration_ms, reverse);
  HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_SET);
  osDelay(1U);
  __HAL_TIM_SET_COMPARE(&htim3, active_channel, compare);
  FlightBoardTest_Send(
      "FC motor start channel=%u direction=%s drive_direction=%s duty=%lu duration_ms=%lu\r\n",
      (unsigned int)channel,
      reverse ? "reverse" : "forward",
      drive_reverse ? "reverse" : "forward",
      (unsigned long)duty_percent,
      (unsigned long)duration_ms);
  return true;
}

static bool FlightBoardTest_StartServo(uint32_t channel,
                                      uint32_t pulse_us,
                                      uint32_t duration_ms)
{
  uint32_t timer_channel;
  uint16_t pin;
  HAL_StatusTypeDef result;

  if ((channel < 1U) || (channel > 2U))
  {
    FlightBoardTest_Send("FC servo rejected reason=channel_range valid=1..2\r\n");
    return false;
  }
  if ((pulse_us < SERVO_MIN_PULSE_US) || (pulse_us > SERVO_MAX_PULSE_US))
  {
    FlightBoardTest_Send(
        "FC servo rejected reason=pulse_range min=%u max=%u\r\n",
        (unsigned int)SERVO_MIN_PULSE_US,
        (unsigned int)SERVO_MAX_PULSE_US);
    return false;
  }
  if (!FlightBoardTest_CheckActionReady(duration_ms, false))
  {
    return false;
  }

  timer_channel = (channel == 1U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
  pin = (channel == 1U) ? GPIO_PIN_0 : GPIO_PIN_1;
  FlightBoardTest_ConfigureAlternate(GPIOA, pin, GPIO_AF1_TIM2);
  __HAL_TIM_SET_COMPARE(&htim2, timer_channel, pulse_us);
  result = HAL_TIM_PWM_Start(&htim2, timer_channel);
  if (result != HAL_OK)
  {
    FlightBoardTest_StopActuators();
    FlightBoardTest_Send("FC servo rejected reason=pwm_start hal=%u\r\n",
                        (unsigned int)result);
    return false;
  }

  FlightBoardTest_SetActiveAction(
      BOARD_ACTION_SERVO, (uint8_t)channel, pulse_us, duration_ms, false);
  FlightBoardTest_Send(
      "FC servo start channel=%u pulse_us=%lu duration_ms=%lu\r\n",
      (unsigned int)channel,
      (unsigned long)pulse_us,
      (unsigned long)duration_ms);
  return true;
}

static HAL_StatusTypeDef FlightBoardTest_ReadAdc(uint32_t *raw, uint32_t *pin_mv)
{
  HAL_StatusTypeDef result;

  if ((raw == NULL) || (pin_mv == NULL))
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
    *pin_mv = (*raw * ADC_REFERENCE_MV) / ADC_FULL_SCALE;
  }
  (void)HAL_ADC_Stop(&hadc1);
  return result;
}

static HAL_StatusTypeDef FlightBoardTest_ImuReadRegistersDuplex(
    uint8_t register_address,
    uint8_t *data,
    uint16_t length)
{
  uint8_t tx_data[ICM42688_SPI_TRANSFER_MAX] = {0U};
  uint8_t rx_data[ICM42688_SPI_TRANSFER_MAX] = {0U};
  HAL_StatusTypeDef result;

  if ((data == NULL) || (length == 0U) ||
      ((uint32_t)length + 1U > ICM42688_SPI_TRANSFER_MAX))
  {
    return HAL_ERROR;
  }

  tx_data[0] = register_address | SPI_READ_BIT;
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_RESET);
  result = HAL_SPI_TransmitReceive(
      &hspi1, tx_data, rx_data, (uint16_t)(length + 1U), 100U);
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_SET);

  if (result == HAL_OK)
  {
    memcpy(data, &rx_data[1], length);
  }
  return result;
}

/* This split transaction matches the ICM42688 driver used by the proven
 * RoEx_FC board: keep CS low, send the register address, then clock data in. */
static HAL_StatusTypeDef FlightBoardTest_ImuReadRegistersSplit(
    uint8_t register_address,
    uint8_t *data,
    uint16_t length)
{
  uint8_t command = register_address | SPI_READ_BIT;
  HAL_StatusTypeDef result;

  if ((data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(&hspi1, &command, 1U, 100U);
  if (result == HAL_OK)
  {
    result = HAL_SPI_Receive(&hspi1, data, length, 100U);
  }
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_SET);
  return result;
}

static HAL_StatusTypeDef FlightBoardTest_ImuReadRegisters(uint8_t register_address,
                                                         uint8_t *data,
                                                         uint16_t length)
{
  return FlightBoardTest_ImuReadRegistersSplit(register_address, data, length);
}

static HAL_StatusTypeDef FlightBoardTest_ImuReadRegister(uint8_t register_address,
                                                        uint8_t *value)
{
  return FlightBoardTest_ImuReadRegisters(register_address, value, 1U);
}

static HAL_StatusTypeDef FlightBoardTest_ImuWriteRegister(uint8_t register_address,
                                                         uint8_t value)
{
  uint8_t tx_data[2] = {(uint8_t)(register_address & (uint8_t)~SPI_READ_BIT), value};
  HAL_StatusTypeDef result;

  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_RESET);
  result = HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), 100U);
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_SET);
  return result;
}

static HAL_StatusTypeDef FlightBoardTest_ReadImuWhoAmI(uint8_t *who_am_i)
{
  if (who_am_i == NULL)
  {
    return HAL_ERROR;
  }
  return FlightBoardTest_ImuReadRegister(ICM42688_WHO_AM_I_REGISTER, who_am_i);
}

static int16_t FlightBoardTest_DecodeBigEndianInt16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

static HAL_StatusTypeDef FlightBoardTest_ConfigureImu(void)
{
  HAL_StatusTypeDef result;

  result = FlightBoardTest_ImuWriteRegister(ICM42688_REG_BANK_SEL_REGISTER,
                                            ICM42688_BANK0);
  if (result == HAL_OK)
  {
    result = FlightBoardTest_ImuWriteRegister(ICM42688_GYRO_CONFIG0_REGISTER,
                                              ICM42688_ODR_1KHZ_DEFAULT_FS);
  }
  if (result == HAL_OK)
  {
    result = FlightBoardTest_ImuWriteRegister(ICM42688_ACCEL_CONFIG0_REGISTER,
                                              ICM42688_ODR_1KHZ_DEFAULT_FS);
  }
  if (result == HAL_OK)
  {
    result = FlightBoardTest_ImuWriteRegister(ICM42688_PWR_MGMT0_REGISTER,
                                              ICM42688_SIX_AXIS_LOW_NOISE);
  }
  if (result == HAL_OK)
  {
    osDelay(50U);
  }
  return result;
}

static void FlightBoardTest_ResetAndProbeImu(void)
{
  uint8_t who_am_i = 0U;
  HAL_StatusTypeDef reset_result;
  HAL_StatusTypeDef bank_result = HAL_ERROR;
  HAL_StatusTypeDef who_result = HAL_ERROR;

  imu_configured = false;
  reset_result = FlightBoardTest_ImuWriteRegister(
      ICM42688_DEVICE_CONFIG_REGISTER, 0x01U);
  if (reset_result == HAL_OK)
  {
    osDelay(10U);
    bank_result = FlightBoardTest_ImuWriteRegister(
        ICM42688_REG_BANK_SEL_REGISTER, ICM42688_BANK0);
  }
  if (bank_result == HAL_OK)
  {
    who_result = FlightBoardTest_ReadImuWhoAmI(&who_am_i);
  }

  FlightBoardTest_Send(
      "FC imu legacy_reset reset_hal=%u bank_hal=%u who=0x%02X expected=0x%02X read_hal=%u\r\n",
      (unsigned int)reset_result,
      (unsigned int)bank_result,
      who_am_i,
      ICM42688_WHO_AM_I_EXPECTED,
      (unsigned int)who_result);
  FlightBoardTest_SendImuDetails(who_am_i, who_result);
}

static void FlightBoardTest_CompareImuTransactions(void)
{
  uint8_t duplex_who = 0U;
  uint8_t split_who = 0U;
  HAL_StatusTypeDef duplex_result = FlightBoardTest_ImuReadRegistersDuplex(
      ICM42688_WHO_AM_I_REGISTER, &duplex_who, 1U);
  HAL_StatusTypeDef split_result = FlightBoardTest_ImuReadRegistersSplit(
      ICM42688_WHO_AM_I_REGISTER, &split_who, 1U);

  FlightBoardTest_Send(
      "FC imu compare duplex=0x%02X(%u) split=0x%02X(%u) expected=0x%02X\r\n",
      duplex_who,
      (unsigned int)duplex_result,
      split_who,
      (unsigned int)split_result,
      ICM42688_WHO_AM_I_EXPECTED);
  FlightBoardTest_SendImuDetails(split_who, split_result);
}

static void FlightBoardTest_SendImuProbe(uint8_t who_am_i,
                                        HAL_StatusTypeDef who_result)
{
  uint8_t device_config = 0U;
  uint8_t sensor_config[3] = {0U, 0U, 0U};
  uint8_t bank_select = 0U;
  HAL_StatusTypeDef device_result = FlightBoardTest_ImuReadRegister(
      ICM42688_DEVICE_CONFIG_REGISTER, &device_config);
  HAL_StatusTypeDef config_result = FlightBoardTest_ImuReadRegisters(
      ICM42688_PWR_MGMT0_REGISTER, sensor_config, sizeof(sensor_config));
  HAL_StatusTypeDef bank_result = FlightBoardTest_ImuReadRegister(
      ICM42688_REG_BANK_SEL_REGISTER, &bank_select);
  const char *branch;

  if ((who_result != HAL_OK) || (device_result != HAL_OK) ||
      (config_result != HAL_OK) || (bank_result != HAL_OK))
  {
    branch = "spi_hal_error";
  }
  else if ((device_config == 0x00U) && (sensor_config[0] == 0x00U) &&
           (sensor_config[1] == 0x00U) && (sensor_config[2] == 0x00U) &&
           (who_am_i == 0x00U) && (bank_select == 0x00U))
  {
    branch = "all_zero_miso_low_or_no_response";
  }
  else if ((device_config == 0xFFU) && (sensor_config[0] == 0xFFU) &&
           (sensor_config[1] == 0xFFU) && (sensor_config[2] == 0xFFU) &&
           (who_am_i == 0xFFU) && (bank_select == 0xFFU))
  {
    branch = "all_ff_miso_high_or_open";
  }
  else
  {
    branch = "register_values_present_unexpected_id";
  }

  FlightBoardTest_Send(
      "FC imu probe branch=%s reg11=%02X reg4E=%02X reg4F=%02X "
      "reg50=%02X reg75=%02X reg76=%02X hal=%u/%u/%u/%u\r\n",
      branch,
      device_config,
      sensor_config[0],
      sensor_config[1],
      sensor_config[2],
      who_am_i,
      bank_select,
      (unsigned int)device_result,
      (unsigned int)config_result,
      (unsigned int)who_result,
      (unsigned int)bank_result);
}

static void FlightBoardTest_SendImuDetails(uint8_t who_am_i,
                                          HAL_StatusTypeDef who_result)
{
  uint8_t raw_data[ICM42688_RAW_DATA_LENGTH] = {0U};
  HAL_StatusTypeDef result;

  if ((who_result != HAL_OK) || (who_am_i != ICM42688_WHO_AM_I_EXPECTED))
  {
    imu_configured = false;
    FlightBoardTest_SendImuProbe(who_am_i, who_result);
    return;
  }

  if (!imu_configured)
  {
    result = FlightBoardTest_ConfigureImu();
    FlightBoardTest_Send(
        "FC imu branch=id_ok init bank=0 gyro_cfg=06 accel_cfg=06 pwr=0F hal=%u\r\n",
        (unsigned int)result);
    if (result != HAL_OK)
    {
      return;
    }
    imu_configured = true;
  }

  result = FlightBoardTest_ImuReadRegisters(ICM42688_ACCEL_DATA_X1_REGISTER,
                                            raw_data,
                                            sizeof(raw_data));
  if (result == HAL_OK)
  {
    FlightBoardTest_Send(
        "FC imu raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d hal=%u\r\n",
        (int)FlightBoardTest_DecodeBigEndianInt16(&raw_data[0]),
        (int)FlightBoardTest_DecodeBigEndianInt16(&raw_data[2]),
        (int)FlightBoardTest_DecodeBigEndianInt16(&raw_data[4]),
        (int)FlightBoardTest_DecodeBigEndianInt16(&raw_data[6]),
        (int)FlightBoardTest_DecodeBigEndianInt16(&raw_data[8]),
        (int)FlightBoardTest_DecodeBigEndianInt16(&raw_data[10]),
        (unsigned int)result);
  }
  else
  {
    FlightBoardTest_Send("FC imu raw read_failed hal=%u\r\n",
                        (unsigned int)result);
  }
}

static void FlightBoardTest_SendStatus(void)
{
  uint32_t adc_raw = 0U;
  uint32_t adc_pin_mv = 0U;
  uint8_t imu_who_am_i = 0U;
  HAL_StatusTypeDef adc_result = FlightBoardTest_ReadAdc(&adc_raw, &adc_pin_mv);
  HAL_StatusTypeDef imu_result = FlightBoardTest_ReadImuWhoAmI(&imu_who_am_i);

  FlightBoardTest_Send(
      "FC status hardware=%s firmware=%s mode=%s safe=%u armed=%u action=%s ch=%u direction=%s remain_ms=%lu "
      "radio=%s radio_tx_arm=%u adc=%lu pin_mv=%lu vbat_mv=%lu(%u) "
      "imu=%02X/%02X(%u) sd=%u log=%u log_owner=%u log_files=%s/%s log_result=%u log_drop=%lu "
      "dir_cal=p1:%u,p2:%u,m1:%u,m2:%u tick=%lu\r\n",
      BOARD_HARDWARE_VERSION,
      BOARD_FIRMWARE_VERSION,
      BalloonRadio_SystemModeName(system_mode),
      active_action.type == BOARD_ACTION_NONE ? 1U : 0U,
      outputs_armed ? 1U : 0U,
      FlightBoardTest_ActionName(active_action.type),
      (unsigned int)active_action.channel,
      FlightBoardTest_ActionDirection(&active_action),
      (unsigned long)FlightBoardTest_RemainingMs(active_action.deadline),
      E28Sx1281_ModeName(radio.mode),
      FlightBoardTest_IsRadioTxArmed() ? 1U : 0U,
      (unsigned long)adc_raw,
      (unsigned long)adc_pin_mv,
      (unsigned long)(adc_pin_mv * ADC_VBAT_DIVIDER_MULTIPLIER),
      (unsigned int)adc_result,
      imu_who_am_i,
      ICM42688_WHO_AM_I_EXPECTED,
      (unsigned int)imu_result,
      HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin) == GPIO_PIN_RESET ? 1U : 0U,
      (unsigned int)mission_log_state,
      mission_log_owns_sd ? 1U : 0U,
      mission_data_log.opened ? mission_data_log.path : "-",
      mission_event_log.opened ? mission_event_log.path : "-",
      (unsigned int)mission_log_last_result,
      (unsigned long)mission_log_drop_count,
      (unsigned int)PUMP1_FORWARD_INVERTED,
      (unsigned int)PUMP2_FORWARD_INVERTED,
      (unsigned int)MOTOR1_FORWARD_INVERTED,
      (unsigned int)MOTOR2_FORWARD_INVERTED,
      (unsigned long)HAL_GetTick());
}

static void FlightBoardTest_SendInputs(void)
{
  FlightBoardTest_Send(
      "FC inputs imu_int1=%u imu_int2=%u radio_busy=%u radio_dio1=%u "
      "motor_fault=%u sd_present=%u\r\n",
      HAL_GPIO_ReadPin(IMU_INT1_GPIO_Port, IMU_INT1_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(IMU_INT2_GPIO_Port, IMU_INT2_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(MOTOR_FAULT_GPIO_Port, MOTOR_FAULT_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin) == GPIO_PIN_RESET ? 1U : 0U);
}

static void FlightBoardTest_SendRadioInfo(void)
{
  bool power_present = E28Sx1281_IsPowerPresent(&radio);

  FlightBoardTest_Send(
      "FC radio power_present=%u mode=%s initialized=%u tx_arm=%u arm_remaining_ms=%lu "
      "freq_hz=%lu chip_power_dbm=%d status=0x%02X packet_type=0x%02X "
      "irq=0x%04X tx=%lu rx=%lu errors=%lu last_error=%s "
      "pins(rst/busy/dio1/cs/txen/rxen)=%u/%u/%u/%u/%u/%u\r\n",
      power_present ? 1U : 0U,
      E28Sx1281_ModeName(radio.mode),
      radio.initialized ? 1U : 0U,
      FlightBoardTest_IsRadioTxArmed() ? 1U : 0U,
      (unsigned long)FlightBoardTest_RemainingMs(radio_tx_arm_deadline),
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
      HAL_GPIO_ReadPin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_TXEN_GPIO_Port, RADIO_TXEN_Pin) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(RADIO_RXEN_GPIO_Port, RADIO_RXEN_Pin) == GPIO_PIN_SET ? 1U : 0U);
}

static uint32_t FlightBoardTest_ScanI2cAddresses(bool exclude_mux)
{
  uint16_t address;
  uint32_t count = 0U;

  for (address = 1U; address < 0x7FU; ++address)
  {
    if (exclude_mux && (address == 0x70U))
    {
      continue;
    }
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1U), 1U, 5U) == HAL_OK)
    {
      ++count;
      FlightBoardTest_Send("FC i2c device=0x%02X\r\n", (unsigned int)address);
    }
  }
  return count;
}

static void FlightBoardTest_ScanI2c(void)
{
  uint32_t count;

  FlightBoardTest_Send("FC i2c scan begin bus=upstream\r\n");
  count = FlightBoardTest_ScanI2cAddresses(false);
  FlightBoardTest_Send("FC i2c count=%lu\r\n", (unsigned long)count);
}

static void FlightBoardTest_ScanI2cMuxChannel(uint8_t channel)
{
  uint8_t selection;
  uint8_t disable = 0U;
  uint32_t count;
  HAL_StatusTypeDef result;

  if (channel > 7U)
  {
    FlightBoardTest_Send("FC i2c mux rejected reason=channel_range valid=0..7\r\n");
    return;
  }

  selection = (uint8_t)(1U << channel);
  result = HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &selection, 1U, 50U);
  if (result != HAL_OK)
  {
    FlightBoardTest_Send(
        "FC i2c mux channel=%u select_hal=%u result=select_failed\r\n",
        (unsigned int)channel,
        (unsigned int)result);
    return;
  }

  FlightBoardTest_Send("FC i2c mux channel=%u scan begin\r\n",
                      (unsigned int)channel);
  count = FlightBoardTest_ScanI2cAddresses(true);
  (void)HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &disable, 1U, 50U);
  FlightBoardTest_Send("FC i2c mux channel=%u count=%lu\r\n",
                      (unsigned int)channel,
                      (unsigned long)count);
}

static void FlightBoardTest_ScanAllI2cMuxChannels(void)
{
  uint8_t channel;

  FlightBoardTest_Send("FC i2c mux all begin\r\n");
  for (channel = 0U; channel < 8U; ++channel)
  {
    FlightBoardTest_ScanI2cMuxChannel(channel);
  }
  FlightBoardTest_Send("FC i2c mux all end\r\n");
}

static HAL_StatusTypeDef FlightBoardTest_ReinitializeI2c(uint32_t clock_hz)
{
  HAL_StatusTypeDef result;

  result = HAL_I2C_DeInit(&hi2c1);
  if (result != HAL_OK)
  {
    return result;
  }

  hi2c1.Init.ClockSpeed = clock_hz;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  result = HAL_I2C_Init(&hi2c1);
  if (result == HAL_OK)
  {
    osDelay(2U);
  }
  return result;
}

static uint16_t FlightBoardTest_DecodeLittleEndianUint16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static const char *FlightBoardTest_Bmp3ModelName(uint8_t chip_id)
{
  if (chip_id == BMP3_CHIP_ID_BMP388)
  {
    return "BMP388";
  }
  if (chip_id == BMP3_CHIP_ID_BMP390)
  {
    return "BMP390/BMP390L";
  }
  return "UNKNOWN";
}

static bool FlightBoardTest_IsBmp3ChipId(uint8_t chip_id)
{
  return (chip_id == BMP3_CHIP_ID_BMP388) ||
         (chip_id == BMP3_CHIP_ID_BMP390);
}

static HAL_StatusTypeDef FlightBoardTest_Bmp3ReadRegisters(
    uint16_t address_8bit,
    uint8_t register_address,
    uint8_t *data,
    uint16_t length)
{
  return HAL_I2C_Mem_Read(&hi2c1,
                          address_8bit,
                          register_address,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          length,
                          100U);
}

static HAL_StatusTypeDef FlightBoardTest_Bmp3WriteRegister(
    uint16_t address_8bit,
    uint8_t register_address,
    uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1,
                           address_8bit,
                           register_address,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1U,
                           100U);
}

static void FlightBoardTest_ParseBmp3Calibration(
    const uint8_t *data,
    FlightBoardBmp3Calibration *calibration)
{
  calibration->par_t1 = FlightBoardTest_DecodeLittleEndianUint16(&data[0]);
  calibration->par_t2 = FlightBoardTest_DecodeLittleEndianUint16(&data[2]);
  calibration->par_t3 = (int8_t)data[4];
  calibration->par_p1 =
      (int16_t)FlightBoardTest_DecodeLittleEndianUint16(&data[5]);
  calibration->par_p2 =
      (int16_t)FlightBoardTest_DecodeLittleEndianUint16(&data[7]);
  calibration->par_p3 = (int8_t)data[9];
  calibration->par_p4 = (int8_t)data[10];
  calibration->par_p5 = FlightBoardTest_DecodeLittleEndianUint16(&data[11]);
  calibration->par_p6 = FlightBoardTest_DecodeLittleEndianUint16(&data[13]);
  calibration->par_p7 = (int8_t)data[15];
  calibration->par_p8 = (int8_t)data[16];
  calibration->par_p9 =
      (int16_t)FlightBoardTest_DecodeLittleEndianUint16(&data[17]);
  calibration->par_p10 = (int8_t)data[19];
  calibration->par_p11 = (int8_t)data[20];
  calibration->t_lin = 0;
}

static int32_t FlightBoardTest_CompensateBmp3Temperature(
    int64_t raw_temperature,
    FlightBoardBmp3Calibration *calibration)
{
  int64_t partial_data1;
  int64_t partial_data2;
  int64_t partial_data3;
  int64_t partial_data4;
  int64_t partial_data5;

  partial_data1 =
      raw_temperature - ((int64_t)256 * calibration->par_t1);
  partial_data2 = (int64_t)calibration->par_t2 * partial_data1;
  partial_data3 = partial_data1 * partial_data1;
  partial_data4 = partial_data3 * calibration->par_t3;
  partial_data5 = (partial_data2 * INT64_C(262144)) + partial_data4;
  calibration->t_lin = partial_data5 / INT64_C(4294967296);

  /* Bosch integer compensation returns temperature in 0.01 degree C. */
  return (int32_t)((calibration->t_lin * 25) / 16384);
}

static uint32_t FlightBoardTest_CompensateBmp3Pressure(
    uint64_t raw_pressure,
    const FlightBoardBmp3Calibration *calibration)
{
  int64_t partial_data1;
  int64_t partial_data2;
  int64_t partial_data3;
  int64_t partial_data4;
  int64_t partial_data5;
  int64_t partial_data6;
  int64_t offset;
  int64_t sensitivity;
  uint64_t compensated;

  /* Bosch BMP3 integer compensation. The returned unit is 0.01 Pa. */
  partial_data1 = calibration->t_lin * calibration->t_lin;
  partial_data2 = partial_data1 / 64;
  partial_data3 = (partial_data2 * calibration->t_lin) / 256;
  partial_data4 = (calibration->par_p8 * partial_data3) / 32;
  partial_data5 = (calibration->par_p7 * partial_data1) * 16;
  partial_data6 = (calibration->par_p6 * calibration->t_lin) * INT64_C(4194304);
  offset = ((int64_t)calibration->par_p5 * INT64_C(140737488355328)) +
           partial_data4 + partial_data5 + partial_data6;
  partial_data2 = (calibration->par_p4 * partial_data3) / 32;
  partial_data4 = (calibration->par_p3 * partial_data1) * 4;
  partial_data5 = ((int64_t)calibration->par_p2 - 16384) *
                  calibration->t_lin * INT64_C(2097152);
  sensitivity = (((int64_t)calibration->par_p1 - 16384) *
                 INT64_C(70368744177664)) + partial_data2 +
                partial_data4 + partial_data5;
  partial_data1 = (sensitivity / INT64_C(16777216)) *
                  (int64_t)raw_pressure;
  partial_data2 = calibration->par_p10 * calibration->t_lin;
  partial_data3 = partial_data2 +
                  (INT64_C(65536) * calibration->par_p9);
  partial_data4 = (partial_data3 * (int64_t)raw_pressure) / 8192;
  partial_data5 = ((int64_t)raw_pressure * (partial_data4 / 10)) / 512;
  partial_data5 *= 10;
  partial_data6 = (int64_t)(raw_pressure * raw_pressure);
  partial_data2 = (calibration->par_p11 * partial_data6) / 65536;
  partial_data3 = (partial_data2 * (int64_t)raw_pressure) / 128;
  partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
  compensated = ((uint64_t)partial_data4 * UINT64_C(25)) /
                UINT64_C(1099511627776);

  if ((compensated < UINT64_C(3000000)) ||
      (compensated > UINT64_C(12500000)))
  {
    return 0U;
  }
  return (uint32_t)compensated;
}

static const char *FlightBoardTest_BarometerConnectorName(uint8_t channel)
{
  switch (channel)
  {
    case 0U:
      return "XH7";
    case 1U:
      return "XH8";
    case 2U:
      return "XH9";
    default:
      return "UNMAPPED";
  }
}

static bool FlightBoardTest_IsBmp5ChipId(uint8_t chip_id)
{
  return (chip_id == BMP5_CHIP_ID_PRIMARY) ||
         (chip_id == BMP5_CHIP_ID_SECONDARY);
}

static int32_t FlightBoardTest_DecodeSigned24(const uint8_t *data)
{
  uint32_t value = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8U) |
                   ((uint32_t)data[2] << 16U);

  if ((value & 0x00800000UL) != 0U)
  {
    value |= 0xFF000000UL;
  }
  return (int32_t)value;
}

static HAL_StatusTypeDef FlightBoardTest_TryReadBmp5(
    uint8_t channel,
    FlightBoardBarometerSample *sample,
    bool *detected)
{
  static const uint16_t addresses[] = {
      BMP5_ADDRESS_LOW_8BIT,
      BMP5_ADDRESS_HIGH_8BIT};
  uint8_t raw_data[BMP5_DATA_LENGTH] = {0U};
  uint8_t chip_id = 0xFFU;
  uint8_t status = 0U;
  uint8_t int_status = 0U;
  uint8_t osr_effective = 0U;
  uint16_t address_8bit = 0U;
  uint32_t start_tick;
  uint32_t raw_pressure;
  int32_t raw_temperature;
  uint32_t temperature_magnitude;
  HAL_StatusTypeDef result = HAL_OK;
  const char *step = "chip_id";
  size_t address_index;

  *detected = false;
  for (address_index = 0U;
       address_index < (sizeof(addresses) / sizeof(addresses[0]));
       ++address_index)
  {
    result = FlightBoardTest_Bmp3ReadRegisters(addresses[address_index],
                                                BMP5_CHIP_ID_REGISTER,
                                                &chip_id,
                                                1U);
    if ((result == HAL_OK) && FlightBoardTest_IsBmp5ChipId(chip_id))
    {
      address_8bit = addresses[address_index];
      *detected = true;
      break;
    }
  }
  if (!*detected)
  {
    return HAL_OK;
  }

  sample->channel = channel;
  sample->address_7bit = (uint8_t)(address_8bit >> 1U);
  sample->chip_id = chip_id;
  sample->model = "BMP58x";
  FlightBoardTest_Send(
      "FC baro detected connector=%s channel=%u model=%s address=0x%02X chip_id=0x%02X\r\n",
      FlightBoardTest_BarometerConnectorName(channel),
      (unsigned int)channel,
      sample->model,
      sample->address_7bit,
      sample->chip_id);

  step = "soft_reset";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP5_COMMAND_REGISTER,
                                              BMP5_SOFT_RESET_COMMAND);
  if (result != HAL_OK)
  {
    goto finish;
  }
  osDelay(3U);

  step = "reset_complete";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP5_INT_STATUS_REGISTER,
                                              &int_status,
                                              1U);
  if ((result != HAL_OK) || ((int_status & BMP5_RESET_COMPLETE) == 0U))
  {
    if (result == HAL_OK)
    {
      result = HAL_ERROR;
    }
    goto finish;
  }

  step = "nvm_ready";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP5_STATUS_REGISTER,
                                              &status,
                                              1U);
  if ((result != HAL_OK) || ((status & BMP5_NVM_READY) == 0U) ||
      ((status & BMP5_NVM_ERROR) != 0U))
  {
    if (result == HAL_OK)
    {
      result = HAL_ERROR;
    }
    goto finish;
  }

  step = "standby";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP5_ODR_REGISTER,
                                              BMP5_ODR_10_HZ_STANDBY);
  if (result != HAL_OK)
  {
    goto finish;
  }
  osDelay(3U);

  step = "configure_osr";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP5_OSR_REGISTER,
                                              BMP5_OSR_PRESS_X8_TEMP_X2);
  if (result != HAL_OK)
  {
    goto finish;
  }

  step = "normal_mode";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP5_INT_CONFIG_REGISTER,
                                              BMP5_INTERRUPT_DRDY_CONFIG);
  if (result != HAL_OK)
  {
    step = "interrupt_config";
    goto finish;
  }
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP5_INT_SOURCE_REGISTER,
                                              BMP5_INTERRUPT_DRDY_SOURCE);
  if (result != HAL_OK)
  {
    step = "interrupt_source";
    goto finish;
  }

  step = "normal_mode";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP5_ODR_REGISTER,
                                              BMP5_ODR_10_HZ_NORMAL);
  if (result != HAL_OK)
  {
    goto finish;
  }
  osDelay(3U);
  step = "effective_config";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP5_OSR_EFFECTIVE_REGISTER,
                                              &osr_effective,
                                              1U);
  if (result != HAL_OK)
  {
    goto finish;
  }

  step = "data_ready";
  start_tick = HAL_GetTick();
  do
  {
    result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                                BMP5_INT_STATUS_REGISTER,
                                                &int_status,
                                                1U);
    if ((result == HAL_OK) && ((int_status & BMP5_DATA_READY) != 0U))
    {
      break;
    }
    osDelay(5U);
  } while ((uint32_t)(HAL_GetTick() - start_tick) <
           BMP5_DATA_READY_TIMEOUT_MS);
  if ((result != HAL_OK) || ((int_status & BMP5_DATA_READY) == 0U))
  {
    result = HAL_TIMEOUT;
    goto finish;
  }

  step = "sensor_data";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP5_DATA_REGISTER,
                                              raw_data,
                                              sizeof(raw_data));
  if (result != HAL_OK)
  {
    goto finish;
  }

  raw_temperature = FlightBoardTest_DecodeSigned24(&raw_data[0]);
  raw_pressure = (uint32_t)raw_data[3] |
                 ((uint32_t)raw_data[4] << 8U) |
                 ((uint32_t)raw_data[5] << 16U);
  sample->temperature_x100 =
      (int32_t)(((int64_t)raw_temperature * 100) / 65536);
  sample->pressure_x100 =
      (uint32_t)(((uint64_t)raw_pressure * 100U + 32U) / 64U);
  sample->valid = (sample->temperature_x100 >= -4000) &&
                  (sample->temperature_x100 <= 8500) &&
                  (sample->pressure_x100 >= 3000000U) &&
                  (sample->pressure_x100 <= 12500000U);
  temperature_magnitude = sample->temperature_x100 < 0
                              ? (uint32_t)(-sample->temperature_x100)
                              : (uint32_t)sample->temperature_x100;
  FlightBoardTest_Send(
      "FC baro data connector=%s channel=%u model=%s address=0x%02X "
      "raw_temp=%ld raw_press=%lu temperature_c=%s%lu.%02lu "
      "pressure_pa=%lu.%02lu status=0x%02X int_status=0x%02X "
      "osr_effective=0x%02X result=%s\r\n",
      FlightBoardTest_BarometerConnectorName(channel),
      (unsigned int)channel,
      sample->model,
      sample->address_7bit,
      (long)raw_temperature,
      (unsigned long)raw_pressure,
      sample->temperature_x100 < 0 ? "-" : "",
      (unsigned long)(temperature_magnitude / 100U),
      (unsigned long)(temperature_magnitude % 100U),
      (unsigned long)(sample->pressure_x100 / 100U),
      (unsigned long)(sample->pressure_x100 % 100U),
      status,
      int_status,
      osr_effective,
      sample->valid ? "PASS" : "OUT_OF_RANGE");

finish:
  (void)FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                           BMP5_ODR_REGISTER,
                                           BMP5_ODR_10_HZ_STANDBY);
  if (result != HAL_OK)
  {
    sample->valid = false;
    FlightBoardTest_Send(
        "FC baro failed connector=%s channel=%u model=BMP58x step=%s "
        "hal=%u error=0x%08lX status=0x%02X int_status=0x%02X "
        "osr_effective=0x%02X\r\n",
        FlightBoardTest_BarometerConnectorName(channel),
        (unsigned int)channel,
        step,
        (unsigned int)result,
        (unsigned long)HAL_I2C_GetError(&hi2c1),
        status,
        int_status,
        osr_effective);
  }
  return result;
}

static HAL_StatusTypeDef FlightBoardTest_SelectI2cMuxChannel(uint8_t channel,
                                                             uint8_t *control)
{
  uint8_t selection = (uint8_t)(1U << channel);
  HAL_StatusTypeDef result;

  result = HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &selection, 1U, 50U);
  if (result == HAL_OK)
  {
    result = HAL_I2C_Master_Receive(
        &hi2c1, TCA9548_ADDRESS_8BIT, control, 1U, 50U);
  }
  if ((result == HAL_OK) && (*control != selection))
  {
    result = HAL_ERROR;
  }
  return result;
}

static bool FlightBoardTest_ReadBarometer(uint8_t channel,
                                          FlightBoardBarometerSample *sample)
{
  static const uint16_t addresses[] = {
      BMP3_ADDRESS_LOW_8BIT,
      BMP3_ADDRESS_HIGH_8BIT};
  FlightBoardBmp3Calibration calibration = {0};
  uint8_t calibration_data[BMP3_CALIBRATION_LENGTH] = {0U};
  uint8_t raw_data[BMP3_DATA_LENGTH] = {0U};
  uint8_t control = 0U;
  uint8_t chip_id = 0xFFU;
  uint8_t status = 0U;
  uint8_t error_status = 0U;
  uint8_t disable = 0U;
  uint16_t address_8bit = 0U;
  uint32_t start_tick;
  uint64_t raw_pressure;
  int64_t raw_temperature;
  uint32_t pressure_x100;
  int32_t temperature_x100;
  uint32_t temperature_magnitude;
  HAL_StatusTypeDef result;
  const char *step = "mux_select";
  size_t address_index;
  bool success = false;
  bool bmp5_detected = false;

  memset(sample, 0, sizeof(*sample));
  sample->channel = channel;
  sample->model = "UNKNOWN";

  if (channel > 7U)
  {
    FlightBoardTest_Send(
        "FC baro rejected reason=channel_range valid=0..7\r\n");
    return false;
  }

  result = FlightBoardTest_SelectI2cMuxChannel(channel, &control);
  if (result != HAL_OK)
  {
    goto cleanup;
  }

  result = FlightBoardTest_TryReadBmp5(channel, sample, &bmp5_detected);
  if (bmp5_detected)
  {
    (void)HAL_I2C_Master_Transmit(
        &hi2c1, TCA9548_ADDRESS_8BIT, &disable, 1U, 50U);
    return (result == HAL_OK) && sample->valid;
  }

  step = "chip_id";
  for (address_index = 0U;
       address_index < (sizeof(addresses) / sizeof(addresses[0]));
       ++address_index)
  {
    result = FlightBoardTest_Bmp3ReadRegisters(addresses[address_index],
                                                BMP3_CHIP_ID_REGISTER,
                                                &chip_id,
                                                1U);
    if ((result == HAL_OK) && FlightBoardTest_IsBmp3ChipId(chip_id))
    {
      address_8bit = addresses[address_index];
      break;
    }
  }
  if (address_8bit == 0U)
  {
    result = HAL_ERROR;
    goto cleanup;
  }

  FlightBoardTest_Send(
      "FC baro detected connector=%s channel=%u model=%s address=0x%02X chip_id=0x%02X\r\n",
      FlightBoardTest_BarometerConnectorName(channel),
      (unsigned int)channel,
      FlightBoardTest_Bmp3ModelName(chip_id),
      (unsigned int)(address_8bit >> 1U),
      chip_id);

  step = "command_ready";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP3_STATUS_REGISTER,
                                              &status,
                                              1U);
  if ((result != HAL_OK) || ((status & BMP3_COMMAND_READY) == 0U))
  {
    if (result == HAL_OK)
    {
      result = HAL_ERROR;
    }
    goto cleanup;
  }

  step = "soft_reset";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP3_COMMAND_REGISTER,
                                              BMP3_SOFT_RESET_COMMAND);
  if (result != HAL_OK)
  {
    goto cleanup;
  }
  osDelay(3U);

  step = "reset_error";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP3_ERROR_REGISTER,
                                              &error_status,
                                              1U);
  if ((result != HAL_OK) || (error_status != 0U))
  {
    if (result == HAL_OK)
    {
      result = HAL_ERROR;
    }
    goto cleanup;
  }

  step = "calibration";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP3_CALIB_REGISTER,
                                              calibration_data,
                                              sizeof(calibration_data));
  if (result != HAL_OK)
  {
    goto cleanup;
  }
  FlightBoardTest_ParseBmp3Calibration(calibration_data, &calibration);

  step = "configure_osr";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP3_OSR_REGISTER,
                                              BMP3_OSR_PRESS_X8_TEMP_X2);
  if (result != HAL_OK)
  {
    goto cleanup;
  }
  step = "configure_odr";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP3_ODR_REGISTER,
                                              BMP3_ODR_25_HZ);
  if (result != HAL_OK)
  {
    goto cleanup;
  }
  step = "configure_filter";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP3_CONFIG_REGISTER,
                                              BMP3_IIR_COEFFICIENT_3);
  if (result != HAL_OK)
  {
    goto cleanup;
  }
  step = "normal_mode";
  result = FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                              BMP3_POWER_CONTROL_REGISTER,
                                              BMP3_PRESS_TEMP_NORMAL_MODE);
  if (result != HAL_OK)
  {
    goto cleanup;
  }

  step = "data_ready";
  start_tick = HAL_GetTick();
  do
  {
    result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                                BMP3_STATUS_REGISTER,
                                                &status,
                                                1U);
    if ((result == HAL_OK) &&
        ((status & BMP3_DATA_READY) == BMP3_DATA_READY))
    {
      break;
    }
    osDelay(5U);
  } while ((uint32_t)(HAL_GetTick() - start_tick) <
           BMP3_DATA_READY_TIMEOUT_MS);
  if ((result != HAL_OK) ||
      ((status & BMP3_DATA_READY) != BMP3_DATA_READY))
  {
    result = HAL_TIMEOUT;
    goto cleanup;
  }

  step = "sensor_data";
  result = FlightBoardTest_Bmp3ReadRegisters(address_8bit,
                                              BMP3_DATA_REGISTER,
                                              raw_data,
                                              sizeof(raw_data));
  if (result != HAL_OK)
  {
    goto cleanup;
  }

  raw_pressure = (uint64_t)raw_data[0] |
                 ((uint64_t)raw_data[1] << 8U) |
                 ((uint64_t)raw_data[2] << 16U);
  raw_temperature = (int64_t)((uint32_t)raw_data[3] |
                              ((uint32_t)raw_data[4] << 8U) |
                              ((uint32_t)raw_data[5] << 16U));
  temperature_x100 = FlightBoardTest_CompensateBmp3Temperature(
      raw_temperature, &calibration);
  pressure_x100 = FlightBoardTest_CompensateBmp3Pressure(
      raw_pressure, &calibration);
  temperature_magnitude = temperature_x100 < 0
                              ? (uint32_t)(-temperature_x100)
                              : (uint32_t)temperature_x100;
  success = (temperature_x100 >= -4000) &&
            (temperature_x100 <= 8500) &&
            (pressure_x100 >= 3000000U) &&
             (pressure_x100 <= 12500000U);
  sample->valid = success;
  sample->address_7bit = (uint8_t)(address_8bit >> 1U);
  sample->chip_id = chip_id;
  sample->temperature_x100 = temperature_x100;
  sample->pressure_x100 = pressure_x100;
  sample->model = FlightBoardTest_Bmp3ModelName(chip_id);

  FlightBoardTest_Send(
      "FC baro data connector=%s channel=%u model=%s address=0x%02X "
      "raw_temp=%lu raw_press=%lu temperature_c=%s%lu.%02lu "
      "pressure_pa=%lu.%02lu status=0x%02X result=%s\r\n",
      FlightBoardTest_BarometerConnectorName(channel),
      (unsigned int)channel,
      FlightBoardTest_Bmp3ModelName(chip_id),
      (unsigned int)(address_8bit >> 1U),
      (unsigned long)raw_temperature,
      (unsigned long)raw_pressure,
      temperature_x100 < 0 ? "-" : "",
      (unsigned long)(temperature_magnitude / 100U),
      (unsigned long)(temperature_magnitude % 100U),
      (unsigned long)(pressure_x100 / 100U),
      (unsigned long)(pressure_x100 % 100U),
      status,
      success ? "PASS" : "OUT_OF_RANGE");

cleanup:
  if (address_8bit != 0U)
  {
    (void)FlightBoardTest_Bmp3WriteRegister(address_8bit,
                                             BMP3_POWER_CONTROL_REGISTER,
                                             BMP3_PRESS_TEMP_SLEEP_MODE);
  }
  (void)HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &disable, 1U, 50U);
  if ((result != HAL_OK) || (address_8bit == 0U))
  {
    sample->valid = false;
    FlightBoardTest_Send(
        "FC baro failed connector=%s channel=%u step=%s hal=%u error=0x%08lX "
        "control=0x%02X chip_id=0x%02X\r\n",
        FlightBoardTest_BarometerConnectorName(channel),
        (unsigned int)channel,
        step,
        (unsigned int)result,
        (unsigned long)HAL_I2C_GetError(&hi2c1),
        control,
        chip_id);
  }
  return (result == HAL_OK) && success;
}

static void FlightBoardTest_ReadExpectedBarometers(void)
{
  static const struct
  {
    uint8_t channel;
    const char *connector;
    const char *expected_model;
  } expected[] = {
      {0U, "XH7", "BMP580"},
      {1U, "XH8", "BMP580"},
      {2U, "XH9", "BMP580"}};
  FlightBoardBarometerSample samples[3] = {0};
  bool valid[3] = {false, false, false};
  size_t index;
  uint32_t pass_count = 0U;

  FlightBoardTest_Send(
      "FC baro all begin mapping=XH7/ch0:BMP580,XH8/ch1:BMP580,XH9/ch2:BMP580\r\n");
  for (index = 0U; index < (sizeof(expected) / sizeof(expected[0])); ++index)
  {
    bool measurement_valid = FlightBoardTest_ReadBarometer(
        expected[index].channel, &samples[index]);
    bool expected_model =
        FlightBoardTest_IsBmp5ChipId(samples[index].chip_id) &&
        ((samples[index].address_7bit == 0x46U) ||
         (samples[index].address_7bit == 0x47U));

    valid[index] = measurement_valid && expected_model;
    if (valid[index])
    {
      ++pass_count;
    }
  }

  for (index = 0U; index < (sizeof(expected) / sizeof(expected[0])); ++index)
  {
    if (valid[index] && valid[0])
    {
      int32_t delta_x100 = (int32_t)samples[index].pressure_x100 -
                           (int32_t)samples[0].pressure_x100;
      uint32_t delta_magnitude = delta_x100 < 0
                                     ? (uint32_t)(-delta_x100)
                                     : (uint32_t)delta_x100;

      FlightBoardTest_Send(
          "FC baro summary connector=%s channel=%u expected=%s detected=%s "
          "address=0x%02X pressure_pa=%lu.%02lu delta_vs_xh7_pa=%s%lu.%02lu result=PASS\r\n",
          expected[index].connector,
          (unsigned int)expected[index].channel,
          expected[index].expected_model,
          samples[index].model,
          samples[index].address_7bit,
          (unsigned long)(samples[index].pressure_x100 / 100U),
          (unsigned long)(samples[index].pressure_x100 % 100U),
          delta_x100 < 0 ? "-" : "",
          (unsigned long)(delta_magnitude / 100U),
          (unsigned long)(delta_magnitude % 100U));
    }
    else if (valid[index])
    {
      FlightBoardTest_Send(
          "FC baro summary connector=%s channel=%u expected=%s detected=%s "
          "address=0x%02X pressure_pa=%lu.%02lu delta_vs_xh7_pa=unavailable "
          "result=PASS\r\n",
          expected[index].connector,
          (unsigned int)expected[index].channel,
          expected[index].expected_model,
          samples[index].model,
          samples[index].address_7bit,
          (unsigned long)(samples[index].pressure_x100 / 100U),
          (unsigned long)(samples[index].pressure_x100 % 100U));
    }
    else
    {
      FlightBoardTest_Send(
          "FC baro summary connector=%s channel=%u expected=%s detected=%s "
          "delta_vs_xh7_pa=unavailable result=FAIL\r\n",
          expected[index].connector,
          (unsigned int)expected[index].channel,
          expected[index].expected_model,
          samples[index].model != NULL ? samples[index].model : "UNKNOWN");
    }
  }
  FlightBoardTest_Send(
      "FC baro all end pass=%lu total=%u result=%s\r\n",
      (unsigned long)pass_count,
      (unsigned int)(sizeof(expected) / sizeof(expected[0])),
      pass_count == (sizeof(expected) / sizeof(expected[0])) ? "PASS" : "FAIL");
}

static uint8_t FlightBoardTest_Sht40Crc(const uint8_t *data, size_t length)
{
  uint8_t crc = 0xFFU;
  size_t index;

  for (index = 0U; index < length; ++index)
  {
    uint8_t bit;

    crc ^= data[index];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x80U) != 0U
                ? (uint8_t)((uint8_t)(crc << 1U) ^ 0x31U)
                : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

static bool FlightBoardTest_ReadSht40(uint8_t channel)
{
  static const uint16_t addresses[] = {
      SHT40_ADDRESS_PRIMARY_8BIT,
      SHT40_ADDRESS_SECONDARY_8BIT};
  uint8_t control = 0U;
  uint8_t disable = 0U;
  uint8_t command = SHT40_MEASURE_HIGH_PRECISION;
  uint8_t data[SHT40_MEASUREMENT_LENGTH] = {0U};
  uint16_t address_8bit = 0U;
  uint16_t raw_temperature;
  uint16_t raw_humidity;
  int32_t temperature_x100;
  int32_t humidity_uncropped_x100;
  int32_t humidity_x100;
  uint32_t temperature_magnitude;
  uint32_t humidity_magnitude;
  HAL_StatusTypeDef result;
  const char *step = "mux_select";
  size_t address_index;
  bool crc_valid;
  bool plausible;

  if (channel > 7U)
  {
    FlightBoardTest_Send(
        "FC sht40 rejected reason=channel_range valid=0..7\r\n");
    return false;
  }

  result = FlightBoardTest_SelectI2cMuxChannel(channel, &control);
  if (result != HAL_OK)
  {
    goto cleanup;
  }

  step = "device_ready";
  for (address_index = 0U;
       address_index < (sizeof(addresses) / sizeof(addresses[0]));
       ++address_index)
  {
    result = HAL_I2C_IsDeviceReady(&hi2c1, addresses[address_index], 3U, 20U);
    if (result == HAL_OK)
    {
      address_8bit = addresses[address_index];
      break;
    }
  }
  if (address_8bit == 0U)
  {
    result = HAL_ERROR;
    goto cleanup;
  }

  step = "measure_command";
  result = HAL_I2C_Master_Transmit(
      &hi2c1, address_8bit, &command, 1U, 50U);
  if (result != HAL_OK)
  {
    goto cleanup;
  }
  osDelay(SHT40_MEASUREMENT_DELAY_MS);

  step = "measurement_data";
  result = HAL_I2C_Master_Receive(
      &hi2c1, address_8bit, data, sizeof(data), 50U);
  if (result != HAL_OK)
  {
    goto cleanup;
  }

  crc_valid = (FlightBoardTest_Sht40Crc(&data[0], 2U) == data[2]) &&
              (FlightBoardTest_Sht40Crc(&data[3], 2U) == data[5]);
  raw_temperature = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
  raw_humidity = (uint16_t)(((uint16_t)data[3] << 8U) | data[4]);
  temperature_x100 =
      -4500 + (int32_t)(((int64_t)17500 * raw_temperature + 32767) / 65535);
  humidity_uncropped_x100 =
      -600 + (int32_t)(((int64_t)12500 * raw_humidity + 32767) / 65535);
  humidity_x100 = humidity_uncropped_x100;
  if (humidity_x100 < 0)
  {
    humidity_x100 = 0;
  }
  else if (humidity_x100 > 10000)
  {
    humidity_x100 = 10000;
  }
  plausible = crc_valid &&
              (temperature_x100 >= -4000) &&
              (temperature_x100 <= 12500);
  temperature_magnitude = temperature_x100 < 0
                              ? (uint32_t)(-temperature_x100)
                              : (uint32_t)temperature_x100;
  humidity_magnitude = (uint32_t)humidity_x100;

  FlightBoardTest_Send(
      "FC sht40 data connector=XH10 channel=%u model=SHT40 address=0x%02X "
      "raw_temp=%u raw_rh=%u temperature_c=%s%lu.%02lu humidity_rh=%lu.%02lu "
      "crc=%s rh_clamped=%u result=%s\r\n",
      (unsigned int)channel,
      (unsigned int)(address_8bit >> 1U),
      (unsigned int)raw_temperature,
      (unsigned int)raw_humidity,
      temperature_x100 < 0 ? "-" : "",
      (unsigned long)(temperature_magnitude / 100U),
      (unsigned long)(temperature_magnitude % 100U),
      (unsigned long)(humidity_magnitude / 100U),
      (unsigned long)(humidity_magnitude % 100U),
      crc_valid ? "PASS" : "FAIL",
      humidity_x100 != humidity_uncropped_x100 ? 1U : 0U,
      plausible ? "PASS" : "FAIL");

  (void)HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &disable, 1U, 50U);
  return plausible;

cleanup:
  (void)HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &disable, 1U, 50U);
  FlightBoardTest_Send(
      "FC sht40 failed connector=XH10 channel=%u step=%s hal=%u "
      "error=0x%08lX control=0x%02X result=FAIL\r\n",
      (unsigned int)channel,
      step,
      (unsigned int)result,
      (unsigned long)HAL_I2C_GetError(&hi2c1),
      control);
  return false;
}

static void FlightBoardTest_ReadExpectedEnvironmentalSensors(void)
{
  FlightBoardTest_Send(
      "FC sensors all begin mapping=XH7/ch0:BMP580,XH8/ch1:BMP580,"
      "XH9/ch2:BMP580,XH10/ch3:SHT40\r\n");
  FlightBoardTest_ReadExpectedBarometers();
  (void)FlightBoardTest_ReadSht40(3U);
  FlightBoardTest_Send("FC sensors all end note=review_individual_results\r\n");
}

static void FlightBoardTest_ProbeBmp3Address(uint8_t address_7bit)
{
  HAL_StatusTypeDef ready_result;
  HAL_StatusTypeDef read_result;
  uint32_t ready_error;
  uint32_t read_error;
  uint8_t chip_id = 0xFFU;

  ready_result = HAL_I2C_IsDeviceReady(
      &hi2c1, (uint16_t)(address_7bit << 1U), 5U, 20U);
  ready_error = HAL_I2C_GetError(&hi2c1);
  read_result = HAL_I2C_Mem_Read(&hi2c1,
                                (uint16_t)(address_7bit << 1U),
                                BMP3_CHIP_ID_REGISTER,
                                I2C_MEMADD_SIZE_8BIT,
                                &chip_id,
                                1U,
                                50U);
  read_error = HAL_I2C_GetError(&hi2c1);

  FlightBoardTest_Send(
      "FC i2c diag bmp3 address=0x%02X ready_hal=%u ready_error=0x%08lX "
      "read_hal=%u read_error=0x%08lX chip_id=0x%02X expected=0x50/0x60 "
      "model=%s result=%s\r\n",
      (unsigned int)address_7bit,
      (unsigned int)ready_result,
      (unsigned long)ready_error,
      (unsigned int)read_result,
      (unsigned long)read_error,
      chip_id,
      FlightBoardTest_Bmp3ModelName(chip_id),
      (read_result == HAL_OK) && FlightBoardTest_IsBmp3ChipId(chip_id)
          ? "PASS"
          : (ready_result == HAL_OK ? "BAD_ID_OR_READ" : "NO_ACK"));
}

static void FlightBoardTest_ProbeBmp5Address(uint8_t address_7bit)
{
  HAL_StatusTypeDef ready_result;
  HAL_StatusTypeDef read_result;
  uint32_t ready_error;
  uint32_t read_error;
  uint8_t chip_id = 0xFFU;

  ready_result = HAL_I2C_IsDeviceReady(
      &hi2c1, (uint16_t)(address_7bit << 1U), 5U, 20U);
  ready_error = HAL_I2C_GetError(&hi2c1);
  read_result = HAL_I2C_Mem_Read(&hi2c1,
                                 (uint16_t)(address_7bit << 1U),
                                 BMP5_CHIP_ID_REGISTER,
                                 I2C_MEMADD_SIZE_8BIT,
                                 &chip_id,
                                 1U,
                                 50U);
  read_error = HAL_I2C_GetError(&hi2c1);

  FlightBoardTest_Send(
      "FC i2c diag bmp5 address=0x%02X ready_hal=%u ready_error=0x%08lX "
      "read_hal=%u read_error=0x%08lX chip_id=0x%02X expected=0x50/0x51 "
      "result=%s\r\n",
      (unsigned int)address_7bit,
      (unsigned int)ready_result,
      (unsigned long)ready_error,
      (unsigned int)read_result,
      (unsigned long)read_error,
      chip_id,
      (read_result == HAL_OK) && FlightBoardTest_IsBmp5ChipId(chip_id)
          ? "PASS"
          : (ready_result == HAL_OK ? "BAD_ID_OR_READ" : "NO_ACK"));
}

static void FlightBoardTest_RunI2cDiagnosticPass(uint8_t channel,
                                                 uint32_t clock_hz)
{
  uint8_t selection = (uint8_t)(1U << channel);
  uint8_t control = 0U;
  uint8_t disable = 0U;
  HAL_StatusTypeDef reinit_result;
  HAL_StatusTypeDef select_result = HAL_ERROR;
  HAL_StatusTypeDef readback_result = HAL_ERROR;
  HAL_StatusTypeDef disable_result;
  uint32_t select_error = HAL_I2C_ERROR_NONE;
  uint32_t readback_error = HAL_I2C_ERROR_NONE;

  reinit_result = FlightBoardTest_ReinitializeI2c(clock_hz);
  if (reinit_result == HAL_OK)
  {
    select_result = HAL_I2C_Master_Transmit(
        &hi2c1, TCA9548_ADDRESS_8BIT, &selection, 1U, 50U);
    select_error = HAL_I2C_GetError(&hi2c1);
  }
  if (select_result == HAL_OK)
  {
    readback_result = HAL_I2C_Master_Receive(
        &hi2c1, TCA9548_ADDRESS_8BIT, &control, 1U, 50U);
    readback_error = HAL_I2C_GetError(&hi2c1);
  }

  FlightBoardTest_Send(
      "FC i2c diag clock_hz=%lu reinit_hal=%u mux_select_hal=%u "
      "select_error=0x%08lX mux_read_hal=%u read_error=0x%08lX "
      "control=0x%02X expected=0x%02X scl=%u sda=%u\r\n",
      (unsigned long)clock_hz,
      (unsigned int)reinit_result,
      (unsigned int)select_result,
      (unsigned long)select_error,
      (unsigned int)readback_result,
      (unsigned long)readback_error,
      control,
      selection,
      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET ? 1U : 0U,
      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET ? 1U : 0U);

  if ((reinit_result == HAL_OK) &&
      (select_result == HAL_OK) &&
      (readback_result == HAL_OK) &&
      (control == selection))
  {
    FlightBoardTest_ProbeBmp3Address(BMP3_ADDRESS_LOW_8BIT >> 1U);
    FlightBoardTest_ProbeBmp3Address(BMP3_ADDRESS_HIGH_8BIT >> 1U);
    FlightBoardTest_ProbeBmp5Address(BMP5_ADDRESS_LOW_8BIT >> 1U);
    FlightBoardTest_ProbeBmp5Address(BMP5_ADDRESS_HIGH_8BIT >> 1U);
  }
  else
  {
    FlightBoardTest_Send(
        "FC i2c diag barometers skipped reason=mux_not_confirmed\r\n");
  }

  disable_result = HAL_I2C_Master_Transmit(
      &hi2c1, TCA9548_ADDRESS_8BIT, &disable, 1U, 50U);
  FlightBoardTest_Send(
      "FC i2c diag channel=%u clock_hz=%lu disable_hal=%u final_error=0x%08lX\r\n",
      (unsigned int)channel,
      (unsigned long)clock_hz,
      (unsigned int)disable_result,
      (unsigned long)HAL_I2C_GetError(&hi2c1));
}

static void FlightBoardTest_DiagnoseI2cMuxChannel(uint8_t channel)
{
  uint32_t original_clock_hz;
  HAL_StatusTypeDef restore_result = HAL_OK;

  if (channel > 7U)
  {
    FlightBoardTest_Send(
        "FC i2c diag rejected reason=channel_range valid=0..7\r\n");
    return;
  }

  original_clock_hz = hi2c1.Init.ClockSpeed;
  FlightBoardTest_Send(
      "FC i2c diag begin channel=%u original_clock_hz=%lu "
      "target=bmp3:0x76/0x77,bmp5:0x46/0x47\r\n",
      (unsigned int)channel,
      (unsigned long)original_clock_hz);

  FlightBoardTest_RunI2cDiagnosticPass(channel, original_clock_hz);
  if (original_clock_hz != I2C_DIAG_STANDARD_CLOCK_HZ)
  {
    FlightBoardTest_RunI2cDiagnosticPass(
        channel, I2C_DIAG_STANDARD_CLOCK_HZ);
    restore_result = FlightBoardTest_ReinitializeI2c(original_clock_hz);
  }

  FlightBoardTest_Send(
      "FC i2c diag end channel=%u restored_clock_hz=%lu restore_hal=%u\r\n",
      (unsigned int)channel,
      (unsigned long)original_clock_hz,
      (unsigned int)restore_result);
}

static void FlightBoardTest_CaptureGnss(uint32_t duration_ms)
{
  uint8_t value;
  char captured[GNSS_CAPTURE_SIZE + 1U];
  uint32_t start_tick;
  uint32_t total_bytes = 0U;
  uint32_t captured_bytes = 0U;

  if ((duration_ms < GNSS_MIN_DURATION_MS) ||
      (duration_ms > GNSS_MAX_DURATION_MS))
  {
    FlightBoardTest_Send(
        "FC gnss rejected reason=duration_range min=%u max=%u\r\n",
        (unsigned int)GNSS_MIN_DURATION_MS,
        (unsigned int)GNSS_MAX_DURATION_MS);
    return;
  }

  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < duration_ms)
  {
    if (HAL_UART_Receive(&huart6, &value, 1U, 10U) == HAL_OK)
    {
      ++total_bytes;
      if (captured_bytes < GNSS_CAPTURE_SIZE)
      {
        if ((value == '\r') || (value == '\n'))
        {
          captured[captured_bytes++] = '|';
        }
        else if ((value >= 0x20U) && (value <= 0x7EU))
        {
          captured[captured_bytes++] = (char)value;
        }
        else
        {
          captured[captured_bytes++] = '.';
        }
      }
    }
  }
  captured[captured_bytes] = '\0';

  FlightBoardTest_Send(
      "FC gnss baud=9600 duration_ms=%lu bytes=%lu captured=%lu data=%s\r\n",
      (unsigned long)duration_ms,
      (unsigned long)total_bytes,
      (unsigned long)captured_bytes,
      captured_bytes > 0U ? captured : "<none>");
}

static bool FlightBoardTest_IsSdPresent(void)
{
  return HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin) == GPIO_PIN_RESET;
}

static void FlightBoardTest_SendSdInfo(void)
{
  FATFS *file_system = NULL;
  DWORD free_clusters = 0U;
  uint32_t total_mb;
  uint32_t free_mb;
  FRESULT result;

  if (mission_log_owns_sd)
  {
    FlightBoardTest_Send(
        "FC sd present=%u log=%u log_owner=1 data=%s event=%s fresult=%u dropped=%lu result=logger_owns_sd\r\n",
        FlightBoardTest_IsSdPresent() ? 1U : 0U,
        (unsigned int)mission_log_state,
        mission_data_log.opened ? mission_data_log.path : "-",
        mission_event_log.opened ? mission_event_log.path : "-",
        (unsigned int)mission_log_last_result,
        (unsigned long)mission_log_drop_count);
    return;
  }

  if (!FlightBoardTest_IsSdPresent())
  {
    FlightBoardTest_Send("FC sd present=0 result=no_card\r\n");
    return;
  }
  if (retSD != 0U)
  {
    FlightBoardTest_Send("FC sd present=1 link_driver=%u result=link_failed\r\n",
                        (unsigned int)retSD);
    return;
  }

  result = f_mount(&SDFatFS, SDPath, 1U);
  if (result != FR_OK)
  {
    FlightBoardTest_Send(
        "FC sd present=1 mount=%u detect=%lu init_hal=%lu error=0x%08lX "
        "card_state=%lu hal_state=%lu pins(cmd/d0/clk)=%lu/%lu/%lu "
        "pullup=external47k mode=4bit clk=4MHz result=mount_failed\r\n",
        (unsigned int)result,
        (unsigned long)g_sd_detect,
        (unsigned long)g_sd_init_hal,
        (unsigned long)g_sd_init_error,
        (unsigned long)g_sd_card_state,
        (unsigned long)g_sd_hal_state,
        (unsigned long)g_sd_cmd_level,
        (unsigned long)g_sd_d0_level,
        (unsigned long)g_sd_clk_level);
    return;
  }

  result = f_getfree(SDPath, &free_clusters, &file_system);
  if ((result == FR_OK) && (file_system != NULL))
  {
    total_mb = (uint32_t)(((uint64_t)(file_system->n_fatent - 2U) *
                           file_system->csize) /
                          2048U);
    free_mb = (uint32_t)(((uint64_t)free_clusters * file_system->csize) /
                         2048U);
    FlightBoardTest_Send(
        "FC sd present=1 mount=0 fs_type=%u total_mb=%lu free_mb=%lu result=PASS\r\n",
        (unsigned int)file_system->fs_type,
        (unsigned long)total_mb,
        (unsigned long)free_mb);
  }
  else
  {
    FlightBoardTest_Send("FC sd present=1 mount=0 getfree=%u result=getfree_failed\r\n",
                        (unsigned int)result);
  }
  (void)f_mount(NULL, SDPath, 0U);
}

static void FlightBoardTest_RunSdWriteReadTest(void)
{
  char file_path[24];
  char expected[SD_TEST_BUFFER_SIZE];
  char actual[SD_TEST_BUFFER_SIZE];
  UINT bytes_written = 0U;
  UINT bytes_read = 0U;
  int expected_length;
  FRESULT result;
  FRESULT close_result;
  FRESULT unlink_result = FR_OK;

  if (mission_log_owns_sd)
  {
    FlightBoardTest_Send(
        "FC sdtest result=rejected reason=logger_owns_sd command=mission_stop_first\r\n");
    return;
  }

  if (!FlightBoardTest_IsSdPresent())
  {
    FlightBoardTest_Send("FC sdtest present=0 result=no_card\r\n");
    return;
  }
  if (retSD != 0U)
  {
    FlightBoardTest_Send("FC sdtest link_driver=%u result=link_failed\r\n",
                        (unsigned int)retSD);
    return;
  }

  result = f_mount(&SDFatFS, SDPath, 1U);
  if (result != FR_OK)
  {
    FlightBoardTest_Send(
        "FC sdtest mount=%u detect=%lu init_hal=%lu error=0x%08lX "
        "card_state=%lu hal_state=%lu pins(cmd/d0/clk)=%lu/%lu/%lu "
        "pullup=external47k mode=4bit clk=4MHz result=mount_failed\r\n",
        (unsigned int)result,
        (unsigned long)g_sd_detect,
        (unsigned long)g_sd_init_hal,
        (unsigned long)g_sd_init_error,
        (unsigned long)g_sd_card_state,
        (unsigned long)g_sd_hal_state,
        (unsigned long)g_sd_cmd_level,
        (unsigned long)g_sd_d0_level,
        (unsigned long)g_sd_clk_level);
    return;
  }

  (void)snprintf(file_path, sizeof(file_path), "%s/%s", SDPath,
                 SD_TEST_FILE_PATH);
  expected_length = snprintf(expected, sizeof(expected),
                             "Balloon FCFM SD test tick=%lu safe=1\r\n",
                             (unsigned long)HAL_GetTick());
  if ((expected_length <= 0) || ((size_t)expected_length >= sizeof(expected)))
  {
    FlightBoardTest_Send("FC sdtest result=format_failed\r\n");
    (void)f_mount(NULL, SDPath, 0U);
    return;
  }

  result = f_open(&SDFile, file_path, FA_CREATE_ALWAYS | FA_WRITE);
  if (result == FR_OK)
  {
    result = f_write(&SDFile, expected, (UINT)expected_length, &bytes_written);
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
    FlightBoardTest_Send(
        "FC sdtest write=%u bytes=%u/%u result=write_failed\r\n",
        (unsigned int)result,
        (unsigned int)bytes_written,
        (unsigned int)expected_length);
    (void)f_unlink(file_path);
    (void)f_mount(NULL, SDPath, 0U);
    return;
  }

  memset(actual, 0, sizeof(actual));
  result = f_open(&SDFile, file_path, FA_READ);
  if (result == FR_OK)
  {
    result = f_read(&SDFile, actual, sizeof(actual) - 1U, &bytes_read);
    close_result = f_close(&SDFile);
    if ((result == FR_OK) && (close_result != FR_OK))
    {
      result = close_result;
    }
  }
  unlink_result = f_unlink(file_path);
  (void)f_mount(NULL, SDPath, 0U);

  if ((result == FR_OK) && (bytes_read == (UINT)expected_length) &&
      (memcmp(actual, expected, (size_t)expected_length) == 0) &&
      (unlink_result == FR_OK))
  {
    FlightBoardTest_Send(
        "FC sdtest temp_file=%s bytes=%u verify=match cleanup=deleted result=PASS\r\n",
        SD_TEST_FILE_PATH,
        (unsigned int)bytes_read);
  }
  else
  {
    FlightBoardTest_Send(
        "FC sdtest read=%u bytes=%u/%u unlink=%u verify_or_cleanup=failed result=FAIL\r\n",
        (unsigned int)result,
        (unsigned int)bytes_read,
        (unsigned int)expected_length,
        (unsigned int)unlink_result);
  }
}

static uint16_t FlightBoardTest_SaturateU16(uint32_t value)
{
  return value > 0xFFFFUL ? 0xFFFFU : (uint16_t)value;
}

static bool FlightBoardTest_LinkValid(void)
{
  return FlightBoardTest_IsMissionSessionActive() && mission_clock_synced &&
         ((uint32_t)(HAL_GetTick() - mission_last_ground_tick) <=
          MISSION_HEARTBEAT_TIMEOUT_MS);
}

static void FlightBoardTest_FillTelemetry(BalloonTelemetryPayload *telemetry)
{
  uint32_t adc_raw = 0U;
  uint32_t adc_pin_mv = 0U;
  uint8_t imu_who_am_i = 0U;
  HAL_StatusTypeDef adc_result = FlightBoardTest_ReadAdc(&adc_raw, &adc_pin_mv);
  HAL_StatusTypeDef imu_result = FlightBoardTest_ReadImuWhoAmI(&imu_who_am_i);
  bool sd_present = FlightBoardTest_IsSdPresent();
  bool radio_power = E28Sx1281_IsPowerPresent(&radio);

  memset(telemetry, 0, sizeof(*telemetry));
  telemetry->system_mode = system_mode;
  telemetry->mission_id = MISSION_ID;
  telemetry->timestamp_ms = HAL_GetTick();
  telemetry->battery_mv = FlightBoardTest_SaturateU16(
      adc_pin_mv * ADC_VBAT_DIVIDER_MULTIPLIER);
  telemetry->adc_raw = FlightBoardTest_SaturateU16(adc_raw);
  telemetry->imu_who_am_i = imu_who_am_i;
  telemetry->imu_valid = (imu_result == HAL_OK) &&
                         (imu_who_am_i == ICM42688_WHO_AM_I_EXPECTED);
  telemetry->sd_present = sd_present;
  telemetry->link_valid = FlightBoardTest_LinkValid();
  telemetry->action = (uint8_t)active_action.type;
  telemetry->action_channel = active_action.channel;
  telemetry->action_value = FlightBoardTest_SaturateU16(active_action.value);
  telemetry->action_remaining_ms = FlightBoardTest_SaturateU16(
      FlightBoardTest_RemainingMs(active_action.deadline));
  telemetry->radio_rx_count = FlightBoardTest_SaturateU16(radio.rx_count);
  telemetry->radio_tx_count = FlightBoardTest_SaturateU16(radio.tx_count);
  telemetry->radio_error_count = FlightBoardTest_SaturateU16(radio.error_count);
  telemetry->action_reverse = active_action.reverse &&
                              ((active_action.type == BOARD_ACTION_PUMP) ||
                               (active_action.type == BOARD_ACTION_MOTOR));
  telemetry->log_state = mission_log_state;
  telemetry->payload_version = BALLOON_TELEMETRY_PAYLOAD_VERSION;

  if (adc_result != HAL_OK)
  {
    telemetry->fault_bits |= MISSION_FAULT_ADC;
  }
  if (!telemetry->imu_valid)
  {
    telemetry->fault_bits |= MISSION_FAULT_IMU;
  }
  if (!sd_present)
  {
    telemetry->fault_bits |= MISSION_FAULT_SD_MISSING;
  }
  if (HAL_GPIO_ReadPin(MOTOR_FAULT_GPIO_Port, MOTOR_FAULT_Pin) == GPIO_PIN_RESET)
  {
    telemetry->fault_bits |= MISSION_FAULT_MOTOR;
  }
  if (!radio_power || (radio.last_error != E28_SX1281_ERROR_NONE))
  {
    telemetry->fault_bits |= MISSION_FAULT_RADIO;
  }
  if (!telemetry->link_valid)
  {
    telemetry->fault_bits |= MISSION_FAULT_LINK;
  }
  if (mission_log_state != BALLOON_LOG_STATE_ACTIVE)
  {
    telemetry->fault_bits |= MISSION_FAULT_LOG;
  }
}

static const char *FlightBoardTest_LogEventName(uint8_t event)
{
  switch ((FlightBoardLogEventKind)event)
  {
    case FLIGHT_LOG_EVENT_MISSION_START:
      return "MISSION_START";
    case FLIGHT_LOG_EVENT_MISSION_STOP:
      return "MISSION_STOP";
    case FLIGHT_LOG_EVENT_COMMAND_ACK:
      return "COMMAND_ACK";
    case FLIGHT_LOG_EVENT_FAILSAFE:
      return "FAILSAFE";
    case FLIGHT_LOG_EVENT_RADIO_ERROR:
      return "RADIO_ERROR";
    default:
      return "UNKNOWN";
  }
}

static const char *FlightBoardTest_LogDirection(uint8_t action, bool reverse)
{
  if ((action != (uint8_t)BOARD_ACTION_PUMP) &&
      (action != (uint8_t)BOARD_ACTION_MOTOR))
  {
    return "na";
  }
  return reverse ? "reverse" : "forward";
}

static const char *FlightBoardTest_LogAppliedDirection(uint8_t action,
                                                        uint8_t channel,
                                                        bool reverse)
{
  bool applied_reverse;

  if ((action != (uint8_t)BOARD_ACTION_PUMP) &&
      (action != (uint8_t)BOARD_ACTION_MOTOR))
  {
    return "na";
  }
  applied_reverse = reverse ^ FlightBoardTest_DirectionIsInverted(
                                   (BoardActionType)action,
                                   channel);
  return applied_reverse ? "reverse" : "forward";
}

static bool FlightBoardTest_QueueLogRecord(const FlightBoardLogRecord *record,
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

static void FlightBoardTest_LogEvent(FlightBoardLogEventKind event,
                                     BalloonCommandCode command,
                                     BalloonAckStage stage,
                                     BalloonRejectReason reason,
                                     uint16_t command_sequence)
{
  FlightBoardTest_LogEventForAction(event,
                                    command,
                                    stage,
                                    reason,
                                    command_sequence,
                                    &active_action);
}

static void FlightBoardTest_LogEventForAction(
    FlightBoardLogEventKind event,
    BalloonCommandCode command,
    BalloonAckStage stage,
    BalloonRejectReason reason,
    uint16_t command_sequence,
    const BoardActionState *action)
{
  FlightBoardLogRecord record = {0};
  const BoardActionState *event_action =
      action != NULL ? action : &active_action;

  if (!mission_log_owns_sd)
  {
    return;
  }
  record.kind = FLIGHT_LOG_EVENT;
  record.timestamp_ms = HAL_GetTick();
  record.payload.event.event = (uint8_t)event;
  record.payload.event.command = (uint8_t)command;
  record.payload.event.stage = (uint8_t)stage;
  record.payload.event.reason = (uint8_t)reason;
  record.payload.event.command_sequence = command_sequence;
  record.payload.event.action = (uint8_t)event_action->type;
  record.payload.event.action_channel = event_action->channel;
  record.payload.event.action_reverse = event_action->reverse ? 1U : 0U;
  record.payload.event.action_value =
      FlightBoardTest_SaturateU16(event_action->value);
  (void)FlightBoardTest_QueueLogRecord(&record, false);
}

static bool FlightBoardTest_FindLogPairIndex(unsigned int *sequence)
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
                               "%sFCD%04u.CSV",
                               SDPath,
                               index);
    int event_length = snprintf(event_path,
                                sizeof(event_path),
                                "%sFCE%04u.CSV",
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

static void FlightBoardTest_CloseLogFiles(bool normal_stop)
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
    FRESULT unmount_result = f_mount(NULL, SDPath, 0U);

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

static void FlightBoardTest_LoggerTask(void *argument)
{
  FlightBoardLogRecord record;

  (void)argument;
  BalloonCsvLogger_Reset(&mission_data_log);
  BalloonCsvLogger_Reset(&mission_event_log);
  for (;;)
  {
    bool received = xQueueReceive(mission_log_queue,
                                  &record,
                                  pdMS_TO_TICKS(100U)) == pdPASS;

    if (received && (record.kind == FLIGHT_LOG_CONTROL_START))
    {
      unsigned int file_sequence = 0U;

      mission_log_sequence = 0U;
      mission_event_log_sequence = 0U;
      mission_log_last_result = FR_NOT_READY;
      if (!FlightBoardTest_IsSdPresent() || (retSD != 0U))
      {
        mission_log_state = BALLOON_LOG_STATE_ERROR;
      }
      else
      {
        mission_log_last_result = f_mount(&SDFatFS, SDPath, 1U);
        mission_log_mounted = mission_log_last_result == FR_OK;
        if (mission_log_mounted &&
            FlightBoardTest_FindLogPairIndex(&file_sequence) &&
            BalloonCsvLogger_OpenAt(
                &mission_data_log,
                SDPath,
                "FCD",
                file_sequence,
                "schema,record_seq,mission_id,fc_tick_ms,mode,fault_bits,battery_mv,adc_raw,imu_who_am_i,imu_valid,sd_present,link_valid,action,action_channel,direction_requested,direction_applied,action_value,action_remaining_ms,radio_rx_count,radio_tx_count,radio_error_count,log_state\r\n",
                record.timestamp_ms) &&
            BalloonCsvLogger_OpenAt(
                &mission_event_log,
                SDPath,
                "FCE",
                file_sequence,
                "schema,record_seq,mission_id,fc_tick_ms,event,command,stage,reason,command_seq,action,action_channel,direction_requested,direction_applied,action_value\r\n",
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
          FlightBoardTest_CloseLogFiles(false);
        }
      }
    }
    else if (received && (record.kind == FLIGHT_LOG_DATA) &&
             mission_data_log.opened)
    {
      const BalloonTelemetryPayload *telemetry = &record.payload.telemetry;
      int length;

      ++mission_log_sequence;
      length = snprintf(
          mission_csv_line,
          sizeof(mission_csv_line),
          "1,%lu,%u,%lu,%s,0x%04X,%u,%u,%u,%u,%u,%u,%u,%u,%s,%s,%u,%u,%u,%u,%u,%u\r\n",
          (unsigned long)mission_log_sequence,
          (unsigned int)telemetry->mission_id,
          (unsigned long)telemetry->timestamp_ms,
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
          FlightBoardTest_LogDirection(telemetry->action,
                                       telemetry->action_reverse),
          FlightBoardTest_LogAppliedDirection(telemetry->action,
                                              telemetry->action_channel,
                                              telemetry->action_reverse),
          (unsigned int)telemetry->action_value,
          (unsigned int)telemetry->action_remaining_ms,
          (unsigned int)telemetry->radio_rx_count,
          (unsigned int)telemetry->radio_tx_count,
          (unsigned int)telemetry->radio_error_count,
          (unsigned int)telemetry->log_state);
      if ((length <= 0) || ((size_t)length >= sizeof(mission_csv_line)) ||
          !BalloonCsvLogger_Write(&mission_data_log,
                                  mission_csv_line,
                                  (size_t)length))
      {
        mission_log_last_result = length <= 0 ? FR_INT_ERR
                                               : mission_data_log.last_result;
        FlightBoardTest_CloseLogFiles(false);
      }
    }
    else if (received && (record.kind == FLIGHT_LOG_EVENT) &&
             mission_event_log.opened)
    {
      const FlightBoardLogEvent *event = &record.payload.event;
      int length;

      ++mission_event_log_sequence;
      length = snprintf(
          mission_csv_line,
          sizeof(mission_csv_line),
          "1,%lu,%u,%lu,%s,%s,%s,%s,%u,%u,%u,%s,%s,%u\r\n",
          (unsigned long)mission_event_log_sequence,
          (unsigned int)MISSION_ID,
          (unsigned long)record.timestamp_ms,
          FlightBoardTest_LogEventName(event->event),
          event->command == 0U
              ? "none"
              : BalloonRadio_CommandName((BalloonCommandCode)event->command),
          event->event == (uint8_t)FLIGHT_LOG_EVENT_COMMAND_ACK
              ? BalloonRadio_AckStageName((BalloonAckStage)event->stage)
              : "none",
          BalloonRadio_RejectReasonName((BalloonRejectReason)event->reason),
          (unsigned int)event->command_sequence,
          (unsigned int)event->action,
          (unsigned int)event->action_channel,
          FlightBoardTest_LogDirection(event->action,
                                       event->action_reverse != 0U),
          FlightBoardTest_LogAppliedDirection(event->action,
                                              event->action_channel,
                                              event->action_reverse != 0U),
          (unsigned int)event->action_value);
      if ((length <= 0) || ((size_t)length >= sizeof(mission_csv_line)) ||
          !BalloonCsvLogger_Write(&mission_event_log,
                                  mission_csv_line,
                                  (size_t)length))
      {
        mission_log_last_result = length <= 0 ? FR_INT_ERR
                                               : mission_event_log.last_result;
        FlightBoardTest_CloseLogFiles(false);
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
      FlightBoardTest_CloseLogFiles(false);
    }

    if ((received && (record.kind == FLIGHT_LOG_CONTROL_STOP)) ||
        (mission_log_stop_requested &&
         (uxQueueMessagesWaiting(mission_log_queue) == 0U)))
    {
      FlightBoardTest_CloseLogFiles(true);
    }
  }
}

static void FlightBoardTest_StartMissionLogging(void)
{
  FlightBoardLogRecord record = {0};

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
  mission_next_log_tick = HAL_GetTick();
  record.kind = FLIGHT_LOG_CONTROL_START;
  record.timestamp_ms = mission_next_log_tick;
  if (xQueueSend(mission_log_queue, &record, 0U) != pdPASS)
  {
    mission_log_state = BALLOON_LOG_STATE_ERROR;
    mission_log_last_result = FR_NOT_ENOUGH_CORE;
    mission_log_owns_sd = false;
  }
}

static void FlightBoardTest_StopMissionLogging(void)
{
  FlightBoardLogRecord record = {0};

  if (!mission_log_owns_sd)
  {
    return;
  }
  record.kind = FLIGHT_LOG_CONTROL_STOP;
  record.timestamp_ms = HAL_GetTick();
  mission_log_stop_requested = true;
  (void)xQueueSend(mission_log_queue, &record, 0U);
}

static void FlightBoardTest_ServiceMissionLogging(void)
{
  uint32_t now = HAL_GetTick();
  FlightBoardLogRecord record = {0};

  if (!FlightBoardTest_IsMissionSessionActive() || !mission_log_owns_sd ||
      ((int32_t)(now - mission_next_log_tick) < 0))
  {
    return;
  }
  record.kind = FLIGHT_LOG_DATA;
  record.timestamp_ms = now;
  FlightBoardTest_FillTelemetry(&record.payload.telemetry);
  (void)FlightBoardTest_QueueLogRecord(&record, true);
  mission_next_log_tick = now + MISSION_LOG_PERIOD_MS;
}

static bool FlightBoardTest_SendMissionTelemetry(uint16_t sequence)
{
  BalloonTelemetryPayload telemetry;
  uint8_t payload[BALLOON_TELEMETRY_PAYLOAD_SIZE];
  uint8_t payload_length = 0U;

  FlightBoardTest_FillTelemetry(&telemetry);

  if (!BalloonRadio_EncodeTelemetryPayload(
          &telemetry, payload, &payload_length))
  {
    return false;
  }
  return FlightBoardTest_QueueRadioFrame(BALLOON_RADIO_TYPE_TELEMETRY,
                                         sequence,
                                         payload,
                                         payload_length);
}

static void FlightBoardTest_HandleMissionCommand(const BalloonRadioFrame *frame)
{
  BalloonCommandPayload command;
  bool started = false;
  bool reverse;

  if (!BalloonRadio_DecodeCommandPayload(
          frame->payload, frame->payload_length, &command))
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  (BalloonCommandCode)0,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_BAD_PAYLOAD);
    return;
  }
  if (command.mission_id != MISSION_ID)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_WRONG_MISSION);
    return;
  }
  if ((command.ttl_ms < MISSION_COMMAND_TTL_MIN_MS) ||
      (command.ttl_ms > MISSION_COMMAND_TTL_MAX_MS))
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_BAD_PAYLOAD);
    return;
  }
  if (mission_has_command_history &&
      (((int16_t)(frame->sequence - mission_last_command_sequence) <= 0) ||
       ((int32_t)(command.issued_ms - mission_last_issued_ms) <= 0)))
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_DUPLICATE);
    return;
  }
  if ((command.code != BALLOON_COMMAND_STOP) && !mission_clock_synced)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_LINK_LOST);
    return;
  }
  if ((command.code != BALLOON_COMMAND_STOP) && mission_clock_synced)
  {
    uint32_t estimated_issue_tick = command.issued_ms +
                                    mission_ground_clock_offset;
    int32_t command_age_ms = (int32_t)(HAL_GetTick() - estimated_issue_tick);

    if ((command_age_ms < -MISSION_CLOCK_FUTURE_TOLERANCE_MS) ||
        (command_age_ms > (int32_t)command.ttl_ms))
    {
      FlightBoardTest_SetPendingAck(frame->sequence,
                                    command.code,
                                    BALLOON_ACK_REJECTED,
                                    BALLOON_REJECT_EXPIRED);
      return;
    }
  }
  mission_has_command_history = true;
  mission_last_command_sequence = frame->sequence;
  mission_last_issued_ms = command.issued_ms;

  if (command.code == BALLOON_COMMAND_STATUS)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_COMPLETED,
                                  BALLOON_REJECT_NONE);
    mission_telemetry_due = true;
    mission_telemetry_is_response = true;
    mission_telemetry_sequence = frame->sequence;
    return;
  }
  if (command.code == BALLOON_COMMAND_STOP)
  {
    BoardActionState stopped = active_action;

    FlightBoardTest_StopActuators();
    outputs_armed = false;
    remote_action.active = false;
    FlightBoardTest_SetPendingAckForAction(frame->sequence,
                                           command.code,
                                           BALLOON_ACK_COMPLETED,
                                           BALLOON_REJECT_OPERATOR_STOP,
                                           &stopped);
    mission_telemetry_due = true;
    mission_telemetry_is_response = true;
    mission_telemetry_sequence = frame->sequence;
    FlightBoardTest_Send("FC mission emergency_stop result=safe\r\n");
    return;
  }
  if (system_mode != BALLOON_SYSTEM_MODE_MISSION)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  system_mode == BALLOON_SYSTEM_MODE_FAILSAFE
                                      ? BALLOON_REJECT_LINK_LOST
                                      : BALLOON_REJECT_NOT_MISSION);
    return;
  }
  if (!outputs_armed)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_OUTPUTS_DISARMED);
    return;
  }
  if (active_action.type != BOARD_ACTION_NONE)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_BUSY);
    return;
  }
  if ((command.flags & 0xFEU) != 0U)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_BAD_PAYLOAD);
    return;
  }

  reverse = (command.flags & 0x01U) != 0U;
  switch (command.code)
  {
    case BALLOON_COMMAND_VALVE:
      started = FlightBoardTest_StartValve(command.channel,
                                            command.duration_ms);
      break;
    case BALLOON_COMMAND_PUMP:
      started = FlightBoardTest_StartPump(command.channel,
                                           reverse,
                                           command.duration_ms);
      break;
    case BALLOON_COMMAND_MOTOR:
      started = FlightBoardTest_StartMotor(command.channel,
                                            reverse,
                                            command.value,
                                            command.duration_ms);
      break;
    case BALLOON_COMMAND_SERVO:
      started = FlightBoardTest_StartServo(command.channel,
                                            command.value,
                                            command.duration_ms);
      break;
    default:
      FlightBoardTest_SetPendingAck(frame->sequence,
                                    command.code,
                                    BALLOON_ACK_REJECTED,
                                    BALLOON_REJECT_BAD_PAYLOAD);
      return;
  }

  if (!started)
  {
    FlightBoardTest_SetPendingAck(frame->sequence,
                                  command.code,
                                  BALLOON_ACK_REJECTED,
                                  BALLOON_REJECT_RANGE);
    return;
  }

  remote_action.active = true;
  remote_action.command = command.code;
  remote_action.sequence = frame->sequence;
  FlightBoardTest_SetPendingAck(frame->sequence,
                                command.code,
                                BALLOON_ACK_STARTED,
                                BALLOON_REJECT_NONE);
}

static void FlightBoardTest_ServiceMission(void)
{
  uint32_t now = HAL_GetTick();

  if (!FlightBoardTest_IsMissionSessionActive())
  {
    return;
  }
  if (!radio_tx_armed || !radio.initialized)
  {
    return;
  }

  if ((system_mode == BALLOON_SYSTEM_MODE_MISSION) &&
      ((uint32_t)(now - mission_last_ground_tick) >
       MISSION_HEARTBEAT_TIMEOUT_MS))
  {
    FlightBoardTest_StopActuators();
    outputs_armed = false;
    remote_action.active = false;
    system_mode = BALLOON_SYSTEM_MODE_FAILSAFE;
    mission_telemetry_due = true;
    mission_telemetry_is_response = false;
    FlightBoardTest_LogEvent(FLIGHT_LOG_EVENT_FAILSAFE,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_LINK_LOST,
                             0U);
    FlightBoardTest_Send(
        "FC mission failsafe reason=link_timeout timeout_ms=%u outputs=safe\r\n",
        (unsigned int)MISSION_HEARTBEAT_TIMEOUT_MS);
  }

  if (((int32_t)(now - mission_next_telemetry_tick) >= 0) &&
      !mission_telemetry_due)
  {
    mission_telemetry_due = true;
    mission_telemetry_is_response = false;
    mission_next_telemetry_tick = now + MISSION_TELEMETRY_PERIOD_MS;
  }

  if (pending_radio_tx.valid || (radio.mode == E28_SX1281_MODE_TX))
  {
    return;
  }

  if (mission_pending_ack.valid)
  {
    uint8_t payload[BALLOON_ACK_PAYLOAD_SIZE];
    uint8_t payload_length = 0U;

    mission_pending_ack.payload.system_mode = system_mode;
    if (BalloonRadio_EncodeAckPayload(
            &mission_pending_ack.payload, payload, &payload_length) &&
        FlightBoardTest_QueueRadioFrame(
            BALLOON_RADIO_TYPE_ACK,
            mission_pending_ack.payload.command_sequence,
            payload,
            payload_length))
    {
      mission_pending_ack.valid = false;
    }
    return;
  }

  if (mission_telemetry_due)
  {
    uint16_t sequence;

    if (mission_telemetry_is_response)
    {
      sequence = mission_telemetry_sequence;
    }
    else
    {
      ++radio_sequence;
      sequence = radio_sequence;
    }
    if (FlightBoardTest_SendMissionTelemetry(sequence))
    {
      mission_telemetry_due = false;
      mission_telemetry_is_response = false;
    }
  }
}

static void FlightBoardTest_StartMission(void)
{
  HAL_StatusTypeDef result;
  uint32_t now;

  if (FlightBoardTest_IsMissionSessionActive())
  {
    FlightBoardTest_Send("FC mission start rejected reason=already_active\r\n");
    return;
  }
  if (mission_log_owns_sd)
  {
    FlightBoardTest_Send(
        "FC mission start rejected reason=log_flushing wait=status_log_owner_0\r\n");
    return;
  }

  if (active_action.type != BOARD_ACTION_NONE)
  {
    FlightBoardTest_Send("FC mission start rejected reason=action_active\r\n");
    return;
  }

  outputs_armed = false;
  FlightBoardTest_StopActuators();
  radio_bus_forced_off = false;
  radio_power_sampled = false;
  radio_power_adc_failures = 0U;
  result = FlightBoardTest_InitializeRadio();
  if (result == HAL_OK)
  {
    result = E28Sx1281_StartReceive(&radio);
  }
  if (result != HAL_OK)
  {
    system_mode = BALLOON_SYSTEM_MODE_MAINTENANCE;
    FlightBoardTest_EnforceRadioLockdown();
    FlightBoardTest_Send(
        "FC mission start result=FAIL error=%s\r\n",
        E28Sx1281_ErrorName(radio.last_error));
    return;
  }

  system_mode = BALLOON_SYSTEM_MODE_MISSION;
  radio_tx_armed = true;
  radio_tx_arm_deadline = 0U;
  E28Sx1281_SetTransmitPermission(&radio, true);
  now = HAL_GetTick();
  mission_last_ground_tick = now;
  mission_next_telemetry_tick = now;
  mission_has_command_history = false;
  mission_clock_synced = false;
  mission_pending_ack.valid = false;
  mission_telemetry_due = true;
  mission_telemetry_is_response = false;
  remote_action.active = false;
  FlightBoardTest_StartMissionLogging();
  FlightBoardTest_LogEvent(FLIGHT_LOG_EVENT_MISSION_START,
                           (BalloonCommandCode)0,
                           (BalloonAckStage)0,
                           BALLOON_REJECT_NONE,
                           0U);
  FlightBoardTest_Send(
      "FC mission start result=PASS mission_id=%u mode=mission telemetry_ms=%u "
      "log_ms=%u link_timeout_ms=%u outputs_armed=0 radio_tx=session log=requested\r\n",
      (unsigned int)MISSION_ID,
      (unsigned int)MISSION_TELEMETRY_PERIOD_MS,
      (unsigned int)MISSION_LOG_PERIOD_MS,
      (unsigned int)MISSION_HEARTBEAT_TIMEOUT_MS);
}

static void FlightBoardTest_StopMission(void)
{
  FlightBoardTest_StopActuators();
  outputs_armed = false;
  system_mode = BALLOON_SYSTEM_MODE_STANDBY;
  mission_pending_ack.valid = false;
  mission_telemetry_due = false;
  mission_telemetry_is_response = false;
  mission_clock_synced = false;
  remote_action.active = false;
  FlightBoardTest_LogEvent(FLIGHT_LOG_EVENT_MISSION_STOP,
                           (BalloonCommandCode)0,
                           (BalloonAckStage)0,
                           BALLOON_REJECT_OPERATOR_STOP,
                           0U);
  FlightBoardTest_StopMissionLogging();
  FlightBoardTest_EnforceRadioLockdown();
  FlightBoardTest_Send(
      "FC mission stop result=safe mode=standby outputs=locked radio=reset log=flushing\r\n");
}

static void FlightBoardTest_RunAllStatusTests(void)
{
  if (mission_log_owns_sd)
  {
    FlightBoardTest_LogEvent(FLIGHT_LOG_EVENT_MISSION_STOP,
                             (BalloonCommandCode)0,
                             (BalloonAckStage)0,
                             BALLOON_REJECT_OPERATOR_STOP,
                             0U);
    FlightBoardTest_StopMissionLogging();
  }
  outputs_armed = false;
  system_mode = BALLOON_SYSTEM_MODE_MAINTENANCE;
  mission_pending_ack.valid = false;
  mission_telemetry_due = false;
  mission_clock_synced = false;
  remote_action.active = false;
  FlightBoardTest_EnforceAllSafety();

  FlightBoardTest_Send(
      "FC test begin hardware=%s firmware=%s actuators=locked radio_tx=disabled\r\n",
      BOARD_HARDWARE_VERSION,
      BOARD_FIRMWARE_VERSION);
  FlightBoardTest_Send("FC test step=usb_cdc result=PASS\r\n");
  FlightBoardTest_Send("FC test step=status_adc\r\n");
  FlightBoardTest_SendStatus();
  FlightBoardTest_Send("FC test step=digital_inputs\r\n");
  FlightBoardTest_SendInputs();
  FlightBoardTest_Send("FC test step=radio_safe_state\r\n");
  FlightBoardTest_SendRadioInfo();
  FlightBoardTest_Send("FC test step=radio_probe_no_tx\r\n");
  FlightBoardTest_ProbeRadio();
  FlightBoardTest_EnforceRadioLockdown();
  FlightBoardTest_Send("FC test step=imu\r\n");
  FlightBoardTest_ResetAndProbeImu();
  FlightBoardTest_Send("FC test step=i2c_upstream\r\n");
  FlightBoardTest_ScanI2c();
  FlightBoardTest_Send("FC test step=i2c_mux_all\r\n");
  FlightBoardTest_ScanAllI2cMuxChannels();
  FlightBoardTest_Send("FC test step=environmental_sensors_expected\r\n");
  FlightBoardTest_ReadExpectedEnvironmentalSensors();
  FlightBoardTest_Send("FC test step=gnss duration_ms=1000\r\n");
  FlightBoardTest_CaptureGnss(1000U);
  FlightBoardTest_Send("FC test step=sd_mount\r\n");
  FlightBoardTest_SendSdInfo();
  FlightBoardTest_Send("FC test step=sd_write_read file=%s\r\n",
                      SD_TEST_FILE_PATH);
  FlightBoardTest_RunSdWriteReadTest();
  FlightBoardTest_Send(
      "FC test end note=review_each_step_result actuators=locked radio=reset\r\n");
}

static bool FlightBoardTest_CommandStartsWithNoCase(const char *command,
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

static bool FlightBoardTest_IsRadioEmergencyCommand(const char *command)
{
  return (FlightBoardTest_CommandStartsWithNoCase(command, "radio disarm") &&
          (command[12] == '\0')) ||
         (FlightBoardTest_CommandStartsWithNoCase(command, "radio reset") &&
          (command[11] == '\0')) ||
         (FlightBoardTest_CommandStartsWithNoCase(command, "radio power off") &&
          (command[15] == '\0')) ||
         (FlightBoardTest_CommandStartsWithNoCase(command, "mission stop") &&
          (command[12] == '\0')) ||
         (FlightBoardTest_CommandStartsWithNoCase(command, "actuator stop") &&
          (command[13] == '\0')) ||
         (FlightBoardTest_CommandStartsWithNoCase(command, "stop") &&
          (command[4] == '\0'));
}

static void FlightBoardTest_HandleCommand(char *command)
{
  size_t index;
  unsigned int channel = 0U;
  unsigned int value = 0U;
  unsigned long duration_ms = 0UL;
  char direction[8] = {0};
  char extra = '\0';
  int fields;
  bool reverse;

  if (FlightBoardTest_CommandStartsWithNoCase(command, "radio send "))
  {
    const char *payload = &command[11];
    size_t payload_length = strlen(payload);

    FlightBoardTest_ServiceSafety();
    if (active_action.type != BOARD_ACTION_NONE)
    {
      FlightBoardTest_Send(
          "FC radio tx rejected reason=actuator_active allowed=stop\r\n");
    }
    else if ((payload_length == 0U) ||
        (payload_length > BALLOON_RADIO_MAX_PAYLOAD))
    {
      FlightBoardTest_Send(
          "FC radio tx rejected reason=text_length valid=1..%u\r\n",
          (unsigned int)BALLOON_RADIO_MAX_PAYLOAD);
    }
    else
    {
      ++radio_sequence;
      (void)FlightBoardTest_QueueRadioFrame(BALLOON_RADIO_TYPE_TEXT,
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

  FlightBoardTest_ServiceSafety();

  if ((strcmp(command, "help") == 0) || (strcmp(command, "?") == 0))
  {
    FlightBoardTest_Send(
        "FC commands: version | status | test | mission start antenna | mission stop\r\n");
    FlightBoardTest_Send(
        "FC actuator: actuator arm | actuator status | actuator stop | actuator disarm\r\n");
    FlightBoardTest_Send(
        "FC actuator: actuator valve <1|2> <ms> | actuator pump <1|2> <fwd|rev> <ms>\r\n");
    FlightBoardTest_Send(
        "FC actuator: actuator motor <1|2> <fwd|rev> <duty1..30> <ms>\r\n");
    FlightBoardTest_Send(
        "FC actuator: actuator servo <1|2> <pulse1000..2000us> <ms>; duration=50..30000ms\r\n");
    FlightBoardTest_Send(
        "FC sensors: i2c | i2call | i2c mux <0..7> | i2c diag <0..7> | baro <0..7> | baro all | sht40 | sensors all\r\n");
  }
  else if (strcmp(command, "version") == 0)
  {
    FlightBoardTest_Send(
        "FC hardware=%s firmware=%s mode=%s command_payload_v=%u telemetry_payload_v=%u radio_tx=locked_default\r\n",
        BOARD_HARDWARE_VERSION,
        BOARD_FIRMWARE_VERSION,
        BalloonRadio_SystemModeName(system_mode),
        (unsigned int)BALLOON_MISSION_PAYLOAD_VERSION,
        (unsigned int)BALLOON_TELEMETRY_PAYLOAD_VERSION);
  }
  else if (strcmp(command, "status") == 0)
  {
    FlightBoardTest_SendStatus();
  }
  else if ((strcmp(command, "outputs") == 0) ||
           (strcmp(command, "actuator status") == 0))
  {
    FlightBoardTest_SendOutputState();
  }
  else if ((strcmp(command, "arm outputs") == 0) ||
           (strcmp(command, "actuator arm") == 0))
  {
    if (active_action.type != BOARD_ACTION_NONE)
    {
      FlightBoardTest_Send("FC arm rejected reason=action_active\r\n");
    }
    else
    {
      outputs_armed = true;
      FlightBoardTest_Send(
          "FC outputs armed=1 timeout=disabled lock=actuator_stop_or_disarm\r\n");
    }
  }
  else if ((strcmp(command, "stop") == 0) ||
           (strcmp(command, "disarm") == 0) ||
           (strcmp(command, "actuator stop") == 0) ||
           (strcmp(command, "actuator disarm") == 0))
  {
    FlightBoardTest_StopActuators();
    outputs_armed = false;
    remote_action.active = false;
    FlightBoardTest_Send("FC outputs armed=0 action=none result=safe\r\n");
  }
  else if (strcmp(command, "mission start antenna") == 0)
  {
    FlightBoardTest_StartMission();
  }
  else if (strcmp(command, "mission stop") == 0)
  {
    FlightBoardTest_StopMission();
  }
  else if (active_action.type != BOARD_ACTION_NONE)
  {
    FlightBoardTest_Send(
        "FC command rejected reason=action_active allowed=status,outputs,stop\r\n");
  }
  else if (strcmp(command, "adc") == 0)
  {
    uint32_t raw = 0U;
    uint32_t pin_mv = 0U;
    HAL_StatusTypeDef result = FlightBoardTest_ReadAdc(&raw, &pin_mv);
    FlightBoardTest_Send(
                        "FC adc raw=%lu pin_mv=%lu vbat_mv=%lu hal=%u\r\n",
                        (unsigned long)raw,
                        (unsigned long)pin_mv,
                        (unsigned long)(pin_mv * ADC_VBAT_DIVIDER_MULTIPLIER),
                        (unsigned int)result);
  }
  else if (strcmp(command, "inputs") == 0)
  {
    FlightBoardTest_SendInputs();
  }
  else if (strcmp(command, "imu") == 0)
  {
    FlightBoardTest_CompareImuTransactions();
  }
  else if (strcmp(command, "imureset") == 0)
  {
    FlightBoardTest_ResetAndProbeImu();
  }
  else if ((strcmp(command, "radio") == 0) ||
           (strcmp(command, "radio status") == 0))
  {
    FlightBoardTest_SendRadioInfo();
  }
  else if (strcmp(command, "radio probe") == 0)
  {
    FlightBoardTest_ProbeRadio();
  }
  else if (strcmp(command, "radio power off") == 0)
  {
    radio_bus_forced_off = true;
    FlightBoardTest_StopActuators();
    outputs_armed = false;
    system_mode = BALLOON_SYSTEM_MODE_STANDBY;
    remote_action.active = false;
    FlightBoardTest_EnforceRadioLockdown();
    FlightBoardTest_Send(
        "FC radio power_control=off bus=high_z controls=low result=safe_to_remove_battery\r\n");
  }
  else if (strcmp(command, "radio power on") == 0)
  {
    radio_bus_forced_off = false;
    radio_power_sampled = false;
    radio_power_adc_failures = 0U;
    FlightBoardTest_Send(
        "FC radio power_control=on power_present=%u note=run_radio_init\r\n",
        E28Sx1281_IsPowerPresent(&radio) ? 1U : 0U);
  }
  else if (strcmp(command, "radio init") == 0)
  {
    FlightBoardTest_InitializeRadio();
  }
  else if (strcmp(command, "radio rx") == 0)
  {
    HAL_StatusTypeDef result = E28Sx1281_StartReceive(&radio);
    FlightBoardTest_Send(
        "FC radio rx hal=%u mode=%s error=%s result=%s\r\n",
        (unsigned int)result,
        E28Sx1281_ModeName(radio.mode),
        E28Sx1281_ErrorName(radio.last_error),
        result == HAL_OK ? "PASS" : "FAIL");
  }
  else if (strcmp(command, "radio arm antenna") == 0)
  {
    if (!E28Sx1281_IsPowerPresent(&radio))
    {
      FlightBoardTest_Send(
          "FC radio arm rejected reason=power_off connect_battery_side_power\r\n");
    }
    else if (!radio.initialized)
    {
      FlightBoardTest_Send(
          "FC radio arm rejected reason=not_initialized command=radio_init_first\r\n");
    }
    else
    {
      radio_tx_armed = true;
      radio_tx_arm_deadline = HAL_GetTick() + RADIO_TX_ARM_TIMEOUT_MS;
      E28Sx1281_SetTransmitPermission(&radio, true);
      FlightBoardTest_Send(
          "FC radio tx_arm=1 expires_ms=%u confirmation=antenna_installed "
          "chip_power_dbm=%d\r\n",
          (unsigned int)RADIO_TX_ARM_TIMEOUT_MS,
          E28_SX1281_MIN_CHIP_POWER_DBM);
    }
  }
  else if ((strcmp(command, "radio disarm") == 0) ||
           (strcmp(command, "radio reset") == 0))
  {
    system_mode = BALLOON_SYSTEM_MODE_STANDBY;
    remote_action.active = false;
    FlightBoardTest_EnforceRadioLockdown();
    FlightBoardTest_Send(
        "FC radio mode=reset tx_arm=0 txen=0 rxen=0 result=safe "
        "note=run_radio_init_before_reuse\r\n");
  }
  else if (strcmp(command, "radio ping") == 0)
  {
    static const uint8_t ping_payload[] = "flight";

    ++radio_sequence;
    (void)FlightBoardTest_QueueRadioFrame(BALLOON_RADIO_TYPE_PING,
                                          radio_sequence,
                                          ping_payload,
                                          sizeof(ping_payload) - 1U);
  }
  else if (strcmp(command, "i2c") == 0)
  {
    FlightBoardTest_ScanI2c();
  }
  else if (strcmp(command, "i2call") == 0)
  {
    FlightBoardTest_ScanAllI2cMuxChannels();
  }
  else if (strcmp(command, "sd") == 0)
  {
    FlightBoardTest_SendSdInfo();
  }
  else if (strcmp(command, "sdtest") == 0)
  {
    FlightBoardTest_RunSdWriteReadTest();
  }
  else if (strcmp(command, "test") == 0)
  {
    FlightBoardTest_RunAllStatusTests();
  }
  else if (strcmp(command, "baro all") == 0)
  {
    FlightBoardTest_ReadExpectedBarometers();
  }
  else if (strcmp(command, "sht40") == 0)
  {
    (void)FlightBoardTest_ReadSht40(3U);
  }
  else if (strcmp(command, "sensors all") == 0)
  {
    FlightBoardTest_ReadExpectedEnvironmentalSensors();
  }
  else if ((fields = sscanf(command,
                            "baro %u %c",
                            &channel,
                            &extra)) == 1)
  {
    if (channel > 7U)
    {
      FlightBoardTest_Send(
          "FC baro rejected reason=channel_range valid=0..7\r\n");
    }
    else
    {
      FlightBoardBarometerSample sample;

      (void)FlightBoardTest_ReadBarometer((uint8_t)channel, &sample);
    }
  }
  else if ((fields = sscanf(command,
                            "i2c diag %u %c",
                            &channel,
                            &extra)) == 1)
  {
    if (channel > 7U)
    {
      FlightBoardTest_Send(
          "FC i2c diag rejected reason=channel_range valid=0..7\r\n");
    }
    else
    {
      FlightBoardTest_DiagnoseI2cMuxChannel((uint8_t)channel);
    }
  }
  else if ((fields = sscanf(command,
                            "i2c mux %u %c",
                            &channel,
                            &extra)) == 1)
  {
    if (channel > 7U)
    {
      FlightBoardTest_Send(
          "FC i2c mux rejected reason=channel_range valid=0..7\r\n");
    }
    else
    {
      FlightBoardTest_ScanI2cMuxChannel((uint8_t)channel);
    }
  }
  else if ((fields = sscanf(command,
                            "gnss %lu %c",
                            &duration_ms,
                            &extra)) == 1)
  {
    FlightBoardTest_CaptureGnss((uint32_t)duration_ms);
  }
  else if ((fields = sscanf(command,
                            "actuator valve %u %lu %c",
                            &channel,
                            &duration_ms,
                            &extra)) == 2)
  {
    FlightBoardTest_StartValve(channel, (uint32_t)duration_ms);
  }
  else if ((fields = sscanf(command,
                            "actuator pump %u %7s %lu %c",
                            &channel,
                            direction,
                            &duration_ms,
                            &extra)) == 3)
  {
    reverse = (strcmp(direction, "rev") == 0) ||
              (strcmp(direction, "reverse") == 0);
    if (!reverse && (strcmp(direction, "fwd") != 0) &&
        (strcmp(direction, "forward") != 0))
    {
      FlightBoardTest_Send("FC pump rejected reason=direction use=fwd|rev\r\n");
    }
    else
    {
      FlightBoardTest_StartPump(
          channel, reverse, (uint32_t)duration_ms);
    }
  }
  else if ((fields = sscanf(command,
                            "actuator motor %u %7s %u %lu %c",
                            &channel,
                            direction,
                            &value,
                            &duration_ms,
                            &extra)) == 4)
  {
    reverse = (strcmp(direction, "rev") == 0) ||
              (strcmp(direction, "reverse") == 0);
    if (!reverse && (strcmp(direction, "fwd") != 0) &&
        (strcmp(direction, "forward") != 0))
    {
      FlightBoardTest_Send("FC motor rejected reason=direction use=fwd|rev\r\n");
    }
    else
    {
      FlightBoardTest_StartMotor(
          channel, reverse, value, (uint32_t)duration_ms);
    }
  }
  else if ((fields = sscanf(command,
                            "actuator servo %u %u %lu %c",
                            &channel,
                            &value,
                            &duration_ms,
                            &extra)) == 3)
  {
    FlightBoardTest_StartServo(
        channel, value, (uint32_t)duration_ms);
  }
  else if (command[0] != '\0')
  {
    FlightBoardTest_Send(
        "FC error unknown_or_bad_syntax command=%s; type=help\r\n",
        command);
  }
}

static void FlightBoardTest_ProcessUsbCommands(void)
{
  static char command[COMMAND_BUFFER_SIZE];
  static size_t command_length;
  static bool command_ready;
  uint8_t value;

  if (command_ready)
  {
    if ((radio.mode == E28_SX1281_MODE_TX) &&
        !FlightBoardTest_IsRadioEmergencyCommand(command))
    {
      command_length = 0U;
      command_ready = false;
    }
    else
    {
      FlightBoardTest_HandleCommand(command);
      command_length = 0U;
      command_ready = false;
      if (radio.mode == E28_SX1281_MODE_TX)
      {
        return;
      }
    }
  }

  while (FlightBoardTest_TryReadUsbByte(&value))
  {
    if ((value == '\r') || (value == '\n'))
    {
      if (command_length > 0U)
      {
        command[command_length] = '\0';
        command_ready = true;
        if ((radio.mode == E28_SX1281_MODE_TX) &&
            !FlightBoardTest_IsRadioEmergencyCommand(command))
        {
          command_length = 0U;
          command_ready = false;
          continue;
        }
        FlightBoardTest_HandleCommand(command);
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
        FlightBoardTest_Send("FC error command_too_long\r\n");
      }
      else
      {
        continue;
      }
    }
  }
}

void FlightBoardTest_Run(void)
{
  const E28Sx1281Pins radio_pins = {
      .spi = &hspi2,
      .nss_port = SPI2_CS_RADIO_GPIO_Port,
      .nss_pin = SPI2_CS_RADIO_Pin,
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
      .power_present = FlightBoardTest_RadioPowerPresent,
      .set_bus_enabled = FlightBoardTest_SetRadioBusEnabled,
      .callback_context = NULL,
  };

  E28Sx1281_Construct(
      &radio, &radio_pins, E28_SX1281_DEFAULT_FREQUENCY_HZ);
  radio_attached = true;
  mission_log_queue = xQueueCreate(MISSION_LOG_QUEUE_LENGTH,
                                   sizeof(FlightBoardLogRecord));
  if ((mission_log_queue == NULL) ||
      (xTaskCreate(FlightBoardTest_LoggerTask,
                   "FCLogger",
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
  system_mode = BALLOON_SYSTEM_MODE_MAINTENANCE;
  FlightBoardTest_EnforceAllSafety();
  outputs_armed = false;

  for (;;)
  {
    FlightBoardTest_ServiceRadio();
    FlightBoardTest_ServiceRadioPower();
    if (radio.mode != E28_SX1281_MODE_TX)
    {
      FlightBoardTest_ServiceSafety();
      FlightBoardTest_ServiceRadioArm();
      FlightBoardTest_ServiceMission();
      FlightBoardTest_ServiceMissionLogging();
    }
    FlightBoardTest_ProcessUsbCommands();
    osDelay(5U);
  }
}
