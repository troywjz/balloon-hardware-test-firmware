#ifndef FC_BOARD_CONFIG_H
#define FC_BOARD_CONFIG_H

/* Shared flight-controller board configuration. */
/* 飞控组件共用的板级配置。 */

/* Identifies the flight-controller board revision. */
/* 标识飞控板卡修订版本。 */
#define FC_BOARD_HARDWARE_VERSION "V1.0.5"

/* Firmware version reported by the version command. */
/* 版本命令输出的固件版本。 */
#define FC_FIRMWARE_VERSION       "V1.0.5.9"

/* Minimum allowed duration of one actuator action, in milliseconds. */
/* 单次执行器动作允许的最短时长，单位为毫秒。 */
#define FC_ACTUATOR_MIN_DURATION_MS 50U

/* Maximum allowed duration of one actuator action, in milliseconds. */
/* 单次执行器动作允许的最长时长，单位为毫秒。 */
#define FC_ACTUATOR_MAX_DURATION_MS 30000U

/* I2C (Inter-Integrated Circuit) multiplexer channel indices. */
/* I2C（集成电路间总线）多路复用器通道索引。 */

/* SH1 ambient-pressure BMP580 sensor channel. */
/* SH1 外界大气压 BMP580 传感器通道。 */
#define FC_I2C_MUX_CHANNEL_BARO_1       0U

/* SH2 helium-lift-bag BMP580 sensor channel. */
/* SH2 氦气升力囊 BMP580 传感器通道。 */
#define FC_I2C_MUX_CHANNEL_BARO_2       1U

/* SH3 air-ballast-bag BMP580 sensor channel. */
/* SH3 空气压载囊 BMP580 传感器通道。 */
#define FC_I2C_MUX_CHANNEL_BARO_3       2U

/* SH4 SHT40 temperature and humidity sensor channel. */
/* SH4 SHT40 温湿度传感器通道。 */
#define FC_I2C_MUX_CHANNEL_SHT40        3U

/* Onboard MMC5983MA magnetometer channel. */
/* 飞控板载 MMC5983MA 磁力计通道。 */
#define FC_I2C_MUX_CHANNEL_MAG_ONBOARD  4U

/* External SH7 MMC5983MA magnetometer channel. */
/* 外接 SH7 MMC5983MA 磁力计通道。 */
#define FC_I2C_MUX_CHANNEL_MAG_EXTERNAL 5U

/* Marks E28 DIO3 as grounded and unavailable on board V1.0.5. */
/* 标记 V1.0.5 板卡上的 E28 DIO3 已接地且不可用。 */
#define FC_RADIO_DIO3_AVAILABLE 0U

#endif
