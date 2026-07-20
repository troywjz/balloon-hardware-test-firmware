# FCFM 飞控检测与任务固件

- 飞控硬件版本：`V1.0.2`
- 飞控固件版本：`V1.0.2.9`
- MCU：STM32F405RGT6

固件继续保留 ADC、IMU、I²C/TCA9548、GNSS、SDIO、两个阀、两个泵、两个
空心杯电机和两个舵机的原板级测试能力，并增加 E28-2G4M20SX（SX1281）正式
任务通信。射频功能没有替换或删除原来的传感器、存储和执行器功能。

上电默认进入 `maintenance`（维护）模式：执行器锁定、E28 硬件复位、不主动发射，
也不周期刷屏。USB CDC 命令使用 CRLF 结尾。

## 简化命令

```text
version
status
test
mission start antenna
mission stop
```

- `version`：显示硬件版本、固件版本和当前模式。
- `status`：只返回一条当前状态，不执行完整测试。
- `test`：执行全部非执行器状态测试，并包含 E28 非发射探测。
- `mission start antenna`：确认两端天线已经安装，初始化 E28 并进入正式任务模式。
- `mission stop`：停止执行器、锁定输出、复位 E28，并同步关闭任务日志。

## 一个命令完成全部状态测试

发送：

```text
test
```

固件会先锁定全部执行器和射频发射，然后依次完成：

1. USB CDC 响应。
2. 当前模式、ADC 原始值、ADC 引脚电压和电池估算电压。
3. IMU/射频/电机故障/SD 卡检测等数字输入。
4. E28 安全状态和无发射 SPI 探测。
5. ICM-42688 复位、WHO_AM_I 和原始数据探测。
6. I²C 上游及 TCA9548 八个下游通道扫描。
7. 按原理图映射读取 XH7、XH8、XH9 上的三颗 BMP580，计算相对 XH7 的
   压差，并读取 XH10 上的 SHT40 温湿度。
8. USART6 GNSS 1000 ms 数据捕获。
9. SD 卡挂载和临时文件写入/读回/删除验证。

`test` 不会启动任何泵、阀、电机或舵机，不会格式化 SD 卡。SD 验证使用
`FC_SD.TMP`，读回一致后立即删除，不在卡中遗留测试文件。没有天线时可以执行
`test`，因为射频步骤不会进入 TX。

I²C/BMP3xx 定向诊断使用：

```text
i2c diag <0..7>
```

该命令会读回 TCA9548 通道控制字，分别在当前速率和 100 kHz 下探测
BMP3 系列地址 `0x76`/`0x77` 和 BMP5 系列地址 `0x46`/`0x47`，并读取芯片 ID；
结束后恢复原 I²C
速率。命令不会驱动执行器或射频发射。

读取自动识别的 BMP388/BMP390/BMP390L/BMP58x 实际温度和气压数据：

```text
baro <0..7>
baro all
sht40
sensors all
```

`baro <0..7>` 会自动识别两代传感器。BMP3 系列读取出厂校准参数并使用 Bosch
整数补偿；BMP58x 输出已经在芯片内部线性化，固件按数据手册比例换算。命令打印
24 位原始值、摄氏温度和 Pa 气压，随后让传感器回到待机/睡眠模式并关闭 TCA9548
下游通道。`baro all` 固定按 `XH7/ch0=BMP580`、`XH8/ch1=BMP580`、
`XH9/ch2=BMP580` 依次测试，并输出相对 XH7 的压差。

`sht40` 固定读取 `XH10/ch3` 上地址 `0x44` 或 `0x45` 的 SHT40，使用无加热高精度
测量命令，校验两段 CRC-8（Cyclic Redundancy Check，循环冗余校验）后输出温度和
相对湿度。`sensors all` 会依次执行三颗 BMP580 和一颗 SHT40 的集中测试；`test`
中的环境传感器步骤也使用相同映射。

## 执行器测试

执行器必须先在飞控 USB 串口本地解锁：

```text
actuator arm
```

解锁有效期 60 秒，一次只允许一个执行器动作，每次动作限制为 50～3000 ms，
空心杯电机占空比限制为 1%～30%。

```text
actuator status
actuator valve 1 300
actuator valve 2 300
actuator pump 1 fwd 300
actuator pump 1 rev 300
actuator pump 2 fwd 300
actuator pump 2 rev 300
actuator motor 1 fwd 10 300
actuator motor 1 rev 10 300
actuator motor 2 fwd 10 300
actuator motor 2 rev 10 300
actuator servo 1 1500 500
actuator servo 2 1500 500
actuator stop
```

首次测试必须卸除机械负载、固定泵和电机，并从短时、低占空比开始。

`fwd` 是 forward（正向）的缩写，`rev` 是 reverse（反向）的缩写。固件中的逻辑
正向默认对应驱动器 IN1 有效，但这不等于机械安装后的实际充气方向或推力方向。
首次测试每一路后，如果实际方向相反，只把 `flight_board_test.c` 顶部对应的
`PUMP1_FORWARD_INVERTED`、`PUMP2_FORWARD_INVERTED`、
`MOTOR1_FORWARD_INVERTED` 或 `MOTOR2_FORWARD_INVERTED` 从 `0` 改为 `1`。
固件会执行“请求方向 XOR（异或）通道反相”，不需要改 CubeMX 引脚。

## 正式任务模式

正式模式使用 2405 MHz、LoRa SF7、812.5 kHz、CR 4/5、私有同步字 `0x12`。
SX1281 芯片功率设置为最低档 `-18 dBm`，但 E28 外部 PA 后的实际天线口功率仍需
实测。没有安装两端 2.4 GHz 天线时，严禁执行 `mission start antenna`。

操作顺序：

1. 两块板断电并安装匹配的 2.4 GHz 天线。
2. 先在地面站执行 `mission start antenna`。
3. 再在飞控执行 `mission start antenna`。
4. 地面站开始显示飞控每 1000 ms 回传的遥测。
5. 如需远程执行器测试，飞控本地执行 `actuator arm` 后，再从地面站发送 `fc ...`。
6. 结束时地面站先发送 `fc stop`，确认完成 ACK；随后两端执行 `mission stop`。

正式模式支持的地面指令：

```text
fc status
fc stop
fc valve <1|2> <50..3000ms>
fc pump <1|2> <fwd|rev> <50..3000ms>
fc motor <1|2> <fwd|rev> <1..30%> <50..3000ms>
fc servo <1|2> <1000..2000us> <50..3000ms>
```

每条指令包含任务号、序号、地面发送时刻、有效期和 CRC16。飞控拒绝重复/旧序号、
错误任务号、非法范围、未进入任务模式、未本地解锁或执行器正忙的指令。地面站会
显示 `started`、`completed`、`stopped` 或 `rejected` 及拒绝原因。

飞控遥测当前每秒回传：

- 任务号、系统模式、机载毫秒时间戳和故障位。
- 电池估算电压、ADC 原始值。
- IMU WHO_AM_I 与有效性、SD 卡存在状态。
- 链路有效性、当前执行器类型/通道/方向/值/剩余时间。
- 射频收包、发包和错误计数，以及地面端测得的 RSSI/SNR。
- 飞控任务日志状态。

GNSS 位置、压力、温度和电流等字段要在对应传感器驱动、单位和有效性规则冻结后再
加入正式遥测；本版不会把尚未验证的数据伪装成有效值。

地面站每 2000 ms 发送心跳。飞控连续 6000 ms 未收到有效地面帧时进入
`failsafe`，停止执行器并取消输出解锁；恢复链路后也不会自动补执行旧指令，需本地
执行 `mission stop` 后重新进入任务模式。

## CSV 任务日志

`mission start antenna` 成功后，低优先级日志任务会独占 FatFs 文件系统；控制任务
只向固定队列非阻塞投递记录，因此慢卡或坏卡不会阻塞执行器定时和失联保护。
每秒同步一次，`mission stop` 时按“先停止输出，再写停止事件并同步关闭文件”的顺序
收尾。日志失败不会阻止射频任务，但遥测的日志状态会变为 `2`，故障位 bit6 置位。

文件名满足 FAT 8.3 限制，且不会覆盖旧实验：

- `FCD0001.CSV`：FCD = Flight Controller Data（飞控周期数据），记录频率 10 Hz。
- `FCE0001.CSV`：FCE = Flight Controller Events（飞控事件），记录任务、指令应答、
  失联和射频错误等离散事件。

执行器记录同时保存 `direction_requested`（逻辑请求方向）和
`direction_applied`（应用逐通道反相后的电气驱动方向），便于首轮校准追溯。

每次任务自动选择下一组相同编号。任务期间 `sd` 只查询记录器状态，`sdtest` 会被
拒绝，必须先执行 `mission stop`，等待 `status` 显示 `log_owner=0` 后再拔卡或运行
`sdtest`。`log=0/1/2` 分别表示关闭、正常记录、记录错误；即使错误会话关闭后仍保留
`log=2` 供追查，此时以 `log_owner=0` 作为文件已关闭、可以拔卡的判据，并检查
`log_result` 和 `log_drop`。CSV 可直接用 Excel 打开，也可按逗号分隔导入其他分析工具。

## 通道映射

- 阀1/阀2：PC13、PC14。
- 泵1：DRV8833 B 通道，PB2/PB3。
- 泵2：DRV8833 D 通道，PA3/PA2。
- 空心杯电机1：TIM3 CH3/CH4，PB0/PB1。
- 空心杯电机2：TIM3 CH1/CH2，PB4/PB5。
- 舵机1/舵机2：TIM2 CH1/CH2，PA0/PA1，50 Hz。

## 供电安全

只连接 USB 时，MCU 和 `3V3_SYS` 可以工作，但 `3V3_RADIO`、`VBAT+5V` 和
`5V_ACT` 需要电池侧供电。测试 E28、SD 卡和执行器前，先用万用表确认相应电源轨。

飞控仍连接 USB 时如需拔掉电池，应先执行 `mission stop`；确认射频为 reset、
TXEN/RXEN 为低后再断开电池。不要把 `3V3_SYS` 与 `3V3_RADIO` 直接短接。

## 构建与烧录文件

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

烧录文件：

```text
build/Debug/FCFM_BOARD_TEST_V1.0.2.9.hex
```

如果使用 CubeMX 重新生成代码，必须确认 PB12 `SPI2_CS_RADIO` 初始输出为低，且
`MX_SPI2_Init` 不在 `main()` 中自动调用；SPI2 只在飞控确认电池侧射频电源存在后
动态初始化，以降低 USB 单独供电时的反向供电风险。同时复查
`FATFS/Target/sd_diskio.c` 的未对齐写入慢路径：`BSP_SD_WriteBlocks_DMA`
之后必须等待 `WRITE_CPLT_MSG`，不能等待 `READ_CPLT_MSG`。
