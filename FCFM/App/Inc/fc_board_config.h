#ifndef FC_BOARD_CONFIG_H
#define FC_BOARD_CONFIG_H

/*
 * 飞控板级编译配置。印制电路板（Printed Circuit Board，PCB）版本、执行器边界、
 * 传感器通道和射频引脚约束在此统一定义。
 * Flight-controller board configuration. It defines the board revision, actuator limits,
 * sensor channels, and radio-pin constraints shared by firmware components.
 */

/* 硬件版本标识飞控板卡修订；固件版本标识此板运行的软件，并用于版本命令输出。
 * The hardware version identifies the flight-controller board revision. The firmware version
 * identifies the software running on the board and is reported by the version command.
 */
#define FC_BOARD_HARDWARE_VERSION "V1.0.5"
#define FC_FIRMWARE_VERSION       "V1.0.5.9"

/*
 * 单次执行器动作的允许时长，单位为毫秒；最小值和最大值都包含在有效范围内。
 * 这组范围由飞控本地命令、飞控远程命令和地面站的前置校验、帮助与拒绝提示共同使用。
 * The allowed duration of one actuator action, in milliseconds; both endpoints are inclusive.
 * The flight controller uses these limits for local and remote commands, and the ground
 * station uses the same values for preflight validation, help, and rejection messages.
 *
 * 当前范围为 50～30000 毫秒（50 毫秒～30 秒）。远程协议中的 duration_ms 字段是
 * 16 位无符号整数，因此最大值不得超过 65535；同时最小值不得大于最大值。
 * The current range is 50 to 30000 milliseconds (50 milliseconds to 30 seconds). The remote
 * protocol encodes duration_ms as a 16-bit unsigned integer, so the maximum must not exceed
 * 65535, and the minimum must not be greater than the maximum.
 *
 * actuator arm 解锁当前没有自动超时；必须由 actuator stop/disarm、任务停止、失联保护、
 * 复位或断电等现有操作撤销。此处配置的是单次动作时长，不是解锁保持时长。
 * The actuator arm state currently has no automatic timeout. It is cleared by actuator
 * stop/disarm, mission stop, link-loss protection, reset, or power loss. These values limit
 * each action; they do not set how long the actuator remains armed.
 */
#define FC_ACTUATOR_MIN_DURATION_MS 50U
#define FC_ACTUATOR_MAX_DURATION_MS 30000U

/*
 * I²C（Inter-Integrated Circuit，集成电路间总线）多路复用器通道索引。通道号对应板上
 * 接口和传感器布置，不是传感器的 I²C 地址。
 * I²C (Inter-Integrated Circuit) multiplexer channel indices. These values map to the board
 * connectors and sensor placement; they are not I²C device addresses.
 */
/* SH1：外界大气压 BMP580，作为压差参考。
 * SH1: Ambient-pressure BMP580, used as the pressure-difference reference.
 */
#define FC_I2C_MUX_CHANNEL_BARO_1       0U
/* SH2：氦气升力囊 BMP580。
 * SH2: BMP580 for the helium lift bag.
 */
#define FC_I2C_MUX_CHANNEL_BARO_2       1U
/* SH3：空气压载囊 BMP580。
 * SH3: BMP580 for the air ballast bag.
 */
#define FC_I2C_MUX_CHANNEL_BARO_3       2U
/* SH4：温湿度传感器 SHT40。
 * SH4: SHT40 temperature and humidity sensor.
 */
#define FC_I2C_MUX_CHANNEL_SHT40        3U
/* 飞控板载 MMC5983MA 磁力计。
 * Onboard MMC5983MA magnetometer.
 */
#define FC_I2C_MUX_CHANNEL_MAG_ONBOARD  4U
/* 外接 SH7 MMC5983MA 磁力计。
 * External SH7 MMC5983MA magnetometer.
 */
#define FC_I2C_MUX_CHANNEL_MAG_EXTERNAL 5U

/*
 * 当前 V1.0.5 板卡的 E28 DIO3（Digital Input/Output 3，数字输入/输出 3）引脚接地，
 * 因此设为 0（不可用），不得将 DIO3 配置为中断、路由或采样。当前固件还有编译期断言
 * 要求此值为 0；只有硬件连线和固件实现都更新后，才能在对应板卡版本中启用。
 * On the current V1.0.5 board, the E28 DIO3 (Digital Input/Output 3) pin is tied to ground,
 * so this must be 0 (unavailable). Do not configure, route, or sample DIO3 as an interrupt.
 * A compile-time assertion requires 0 for this board revision. Update the board wiring,
 * firmware routing, and assertion before enabling DIO3 on a compatible board revision.
 */
#define FC_RADIO_DIO3_AVAILABLE 0U

#endif
