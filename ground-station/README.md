# 地面站检测与任务固件

- 地面站硬件版本：`V1.1.0`
- 地面站固件版本：`V1.1.0.9`
- MCU：STM32F405RGT6

固件保留电源 ADC、W25Q128、SD 卡和 USB CDC 板级检测，并增加与飞控
`V1.0.5.0` 配套的 E28-2G4M20SX（SX1281）任务通信和 CSV 任务记录。
地面站可同时解码飞控 V1（30 字节）、V2（32 字节）和 V3（57 字节）遥测；
V2 增加执行器方向和飞控日志状态，V3 增加 ICM-45686 六轴原始值、板载/外接
MMC5983MA 三轴磁场值和传感器有效位。

上电默认复位 E28、关闭 TXEN/RXEN 且不主动发射。

## 简化命令

```text
version
status
test
mission start antenna
mission stop
```

- `test`：一次完成地面站板级状态、SD 临时文件写入/读回/删除和 E28 无发射探测。
- `mission start antenna`：确认已经安装天线，初始化射频并开始任务心跳。
- `mission stop`：停止任务心跳并复位射频。

## 正式任务操作

两端断电安装匹配的 2.4 GHz 天线后：

1. 地面站执行 `mission start antenna`。
2. 飞控执行 `mission start antenna`。
3. 等待地面站出现 `GS telemetry ... link=1`。
4. 使用以下命令查看状态或控制飞控。

```text
fc status
fc stop
fc valve 1 300
fc pump 1 fwd 300
fc pump 1 rev 300
fc motor 1 fwd 10 300
fc motor 1 rev 10 300
fc servo 1 1500 500
```

远程执行器指令只有在飞控本地执行过 `actuator arm` 后才会被接受；解锁不按时间
自动失效，保持到显式上锁、任务停止、失联保护、复位或断电。单次动作时长范围为
50～30000 ms。地面站只负责发送请求，最终范围、模式、故障、互锁和动作定时由飞控完成。

每条指令会收到 ACK：

- `started`：指令有效，动作已经开始。
- `completed`：动作按时结束，或状态/急停请求已经完成。
- `stopped`：动作被安全层提前停止。
- `rejected`：拒绝，并显示 `expired`、`duplicate_or_old`、`outputs_disarmed`、
  `busy`、`range`、`link_lost` 等原因。

遥测每秒显示任务号、模式、机载时间戳、故障位、电池电压、ADC、IMU 有效性、
SD 卡、链路状态、执行器方向与状态、飞控日志状态、射频计数及 RSSI/SNR；收到
V3 时另输出 ICM-45686 六轴原始值、两颗 MMC5983MA 的 mG 值和有效位，并写入
地面站 CSV。

地面站每 2000 ms 发送带本机时间戳的心跳，飞控用它判断链路和指令有效期。连续
6000 ms 没有有效地面帧时，飞控进入失联安全状态并停止执行器。

结束任务时：

1. 地面站发送 `fc stop` 并等待 `completed` ACK。
2. 飞控执行 `mission stop`。
3. 地面站执行 `mission stop`。

## CSV 任务日志

地面站用独立低优先级日志任务写卡，射频任务只做非阻塞投递。任务期间生成同编号的
两份文件，文件名满足 FAT 8.3 限制且不会覆盖旧实验：

- `GSD0001.CSV`：GSD = Ground Station Data（地面站接收数据），每收到一帧有效
  飞控遥测记录一行，同时保存地面接收时刻、飞控时刻、RSSI（Received Signal
  Strength Indicator，接收信号强度指示）和 SNR（Signal-to-Noise Ratio，
  信噪比）。`telemetry_version` 表示遥测负载版本，`direction_requested` 表示飞控
  收到的逻辑请求方向；旧版遥测缺失的方向和日志状态记录为 `unknown`（未知）。
- `GSE0001.CSV`：GSE = Ground Station Events（地面站事件），记录任务启停、
  指令排队、发送完成/超时、ACK（Acknowledgement，确认应答）和协议错误；只有
  指令排队事件携带 `direction_requested`，ACK 本身不携带方向，记录为 `na`
  （not applicable，不适用）。

`mission stop` 后日志会异步同步并关闭。等待 `status` 显示 `log_owner=0` 后再拔卡。
`log=0/1/2` 分别表示关闭、正常记录、记录错误；错误会话关闭后仍保留 `log=2` 作为
诊断证据，此时检查 `log_result` 和 `log_drop`。任务期间 `sd` 只查询当前文件、
FatFs 结果和丢记录计数；日志异常不阻止安全控制。

## 无天线阶段

没有安装两端天线时，可以执行 `version`、`status`、`test`、`adc`、`flash`、
`sd` 和 `sdtest` 等不发射命令。严禁执行
`mission start antenna` 或任何 `fc ...` 指令。SX1281 芯片设置为最低档
`-18 dBm` 不能代替天线，也不能代表 E28 外部 PA 后的实际天线口功率。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

烧录文件：

```text
build/Debug/GroundStation.hex
build/Debug/GROUND_STATION_BOARD_TEST_V1.1.0.9.hex
```

## 硬件 V1.1.0 的 TF 卡检测与只读诊断

当前 PCB 的卡座 CD（Card Detect，插卡检测）网络没有实际连接到 STM32 的 PC4。
固件看到 `raw_cd=1` 时，会自动执行与 `sd force on` 等效的回退，并固定使用
SDIO 4-bit 尝试挂载：

```text
GS sd auto_fallback raw_cd=1 force=1 mounted=1 fresult=0 result=PASS
```

挂载成功后保持 `force=1` 并继续供板级测试和任务记录使用；挂载失败则每 2 秒重试，
但警告最多每 30 秒输出一次。执行 `sd force off` 会卸载文件系统、关闭强制检测并暂停
自动回退，适合安全取卡；执行 `sd force on` 可重新启用并立即尝试挂载。手动状态仅保存在
内存中，复位后恢复“允许自动回退”。

随后可用以下只读命令比较 SDIO 1-bit 和 4-bit 数据读取，不会修改卡内数据：

```text
sd raw 1
sd raw 4
sd mount 1
sd mount 4
```

`sd raw` 输出卡容量、第 0 扇区、MBR（Master Boot Record，主引导记录）分区项和
首分区启动扇区；`sd mount` 只尝试挂载。只有 `mounted=1 fresult=0` 后才执行
`sdtest` 写入—读回测试。长期硬件修复方案是把 R35 的 CD 信号侧连接到 U11 的
PC4（物理 24 脚）。

## CubeMX 中需要保持的 SDIO 配置

- SDIO（Secure Digital Input Output，安全数字输入输出接口）模式保持
  `SD 4 bits Wide bus`；PC8～PC11 为 D0～D3、PC12 为 CLK、PD2 为 CMD。
- RX 使用 DMA2 Stream3/Channel4，TX 使用 DMA2 Stream6/Channel4；两个 DMA 和
  SDIO 中断优先级均为 5。
- 首轮板级测试把 `SDIOCLK clock divide factor` 设为 `10`，数据阶段约 4 MHz。
  本工程 `.ioc` 和生成代码已经同步为该值。
- `sdio.c` 初始化时显示 `BusWide = 1B` 是正常的：SD 卡先以 1-bit 初始化，BSP
  随后切换为 4-bit，不要手工改成初始 4-bit。

CubeMX 重新生成代码后，还要复查 `FATFS/Target/sd_diskio.c` 的未对齐写入慢路径：
`BSP_SD_WriteBlocks_DMA` 之后必须等待 `WRITE_CPLT_MSG`，不能等待
`READ_CPLT_MSG`。新版生成器若再次覆盖为后者，需要重新改正。
