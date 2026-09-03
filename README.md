# Balloon 硬件测试固件

本仓库保存浮力气球 Demo 的飞控板与地面站板硬件测试固件、公共通信协议和可直接
烧录的固件文件。它用于板级上电、传感器、存储、射频和执行器验证，是正式飞控开发前
的硬件验收基线；其中的测试任务模式不等同于正式飞行控制软件。

## 当前版本

- 飞控硬件：`V1.0.5`；飞控固件：`V1.0.5.4`。
- 地面站硬件：`V1.1.0`；地面站固件：`V1.1.0.9`。
- MCU（Microcontroller Unit，微控制器）：STM32F405RGT6。
- 串口：USB CDC（Communications Device Class，通信设备类），115200、8-N-1、CRLF。

当前代码覆盖安全初始状态、USB 串口命令、传感器检测、TF 卡、E28-2G4M20SX/SX1281
通信、任务指令、遥测、失联保护、执行器测试和 CSV（Comma-Separated Values，
逗号分隔值）日志。姿态解算、状态估计、闭环高度控制和正式任务状态机仍属于后续开发。

## 目录与说明

- `FCFM/`：飞控工程，目录名沿用现有工程名称。
- `ground-station/`：地面站工程。
- `common/`：公共射频驱动、通信协议和日志代码。
- `E28_SX1281_TEST_GUIDE.md`：双板射频通信逐步测试指南。
- [Releases](https://github.com/troywjz/balloon-hardware-test-firmware/releases)：版本化烧录文件。

更详细的板级说明见 `FCFM/README.md` 和 `ground-station/README.md`。

飞控 `V1.0.5.4` 增加了 `imu stream` 连续诊断命令、分组气囊执行器命令、软件 I²C 诊断命令和磁力计连续输出命令。该命令以 10 Hz 输出两个候选
身份寄存器、原始六轴数据和相邻采样变化数，用于区分 ICM-45686 通信异常、错料和
焊接问题；使用 `imu stop` 停止。诊断模式不绕过正式功能的器件身份校验，也不驱动
执行器或启用射频发射。

## 重要安全规则

1. 两端没有安装匹配的 2.4 GHz 天线时，禁止执行任何会发射的命令，包括
   `radio arm antenna`、`radio ping`、`radio send ...` 和 `mission start antenna`。
2. SX1281 设置为最低 `-18 dBm` 只是射频芯片参数，不能代表 E28 外置功率放大器后的
   实际天线口功率。
3. 飞控只接 USB 时不能完整供电给 E28、TF 卡和执行器；射频、存储和执行器联调需要
   正确接入电池侧电源。
4. 执行器上电后一次只测试一路，首次测试卸除机械负载并从短时间、低占空比开始。
5. 飞控执行器本地解锁没有自动超时，但任务停止、失联保护、复位和断电都会撤销解锁。
6. 单次执行器动作允许 `50～30000 ms`；空心杯电机占空比仅允许 `1%～30%`。
7. `test` 是无执行器、无射频发射的综合检测命令。

## 可直接烧录的文件

飞控：

```text
FCFM/build/Debug/FCFM_BOARD_TEST_V1.0.5.4.hex
FCFM/build/Debug/FCFM_BOARD_TEST_V1.0.5.4.bin
```

地面站：

```text
ground-station/build/Debug/GROUND_STATION_BOARD_TEST_V1.1.0.9.hex
ground-station/build/Debug/GROUND_STATION_BOARD_TEST_V1.1.0.9.bin
```

同一目录保留不带版本号的通用别名。HEX（Intel HEX，英特尔十六进制）适合通过
STM32CubeProgrammer 烧录；BIN（Binary，二进制）供明确指定 Flash 起始地址的工具使用。

## 编译与烧录

分别进入两个工程目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

使用 STM32CubeProgrammer 通过 USB DFU（Device Firmware Upgrade，设备固件升级）
模式烧录对应 HEX。烧录后将 BOOT0 恢复低电平并重新上电，再打开 USB 串口监视器。

## 飞控本地命令

命令在飞控 USB 串口中发送，不区分大小写；末尾使用 CRLF。

### 基础状态与板级检测

| 命令 | 作用 |
|---|---|
| `help` 或 `?` | 显示主要命令提示 |
| `version` | 显示硬件、固件和通信负载版本 |
| `status` | 显示系统模式、电源、IMU、TF 卡、日志、射频和执行器状态 |
| `test` | 依次检测 USB、ADC、输入、射频无发射探测、IMU、I²C、GNSS 和 TF 卡读写 |
| `adc` | 读取电池电压采样的 ADC（Analog-to-Digital Converter，模数转换器）原始值和估算电压 |
| `inputs` | 读取 IMU 中断、射频 BUSY/DIO1、电机故障和 TF 卡检测输入 |
| `imu` | 比较 ICM-42688 不同 SPI（Serial Peripheral Interface，串行外设接口）读取事务并输出探测结果 |
| `imureset` | 执行 ICM-42688 复位和 WHO_AM_I 探测 |
| `gnss <100..3000ms>` | 在指定时间内捕获 GNSS（Global Navigation Satellite System，全球卫星导航系统）串口数据 |
| `sd` | 查看 TF 卡挂载、容量、日志占用和错误状态 |
| `sdtest` | 写入、读回、校验并删除临时测试文件；任务日志占用 TF 卡时会拒绝 |

### I²C 与环境传感器

I²C 是 Inter-Integrated Circuit，两线串行总线。飞控通过 TCA9548 多路复用器连接外部
传感器。

| 命令 | 作用 |
|---|---|
| `i2c` | 扫描上游 I²C，总线上应能看到 TCA9548 地址 `0x70` |
| `i2call` | 依次扫描 TCA9548 的 0～7 通道 |
| `i2c mux <0..7>` | 选择并扫描指定通道 |
| `i2c diag <0..7>` | 对指定通道执行多速率、总线电平和气压计地址诊断 |
| `baro <0..7>` | 自动识别并读取指定通道的 BMP3/BMP58x 气压计 |
| `baro all` | 读取 SH1、SH2、SH3 三颗 BMP580，并计算 SH2/SH3 相对 SH1 的压差 |
| `sht40` | 读取 SH4 上的 SHT40 温度和相对湿度 |
| `mag onboard` / `mag external` / `mag all` | 读取 TCA9548 ch4 板载和 ch5/SH7 外接 MMC5983MA 的 Product ID 与三轴磁场 |
| `mag stream [onboard|external|all]` | 每500 ms输出板载/外接磁力计缓存的原始值和 mG 值，`mag stop`停止 |
| `imu i2c diag` | 临时将 SPI1 引脚切为软件I²C，测试ICM-45686的0x68/0x69地址和WHO_AM_I；结束后恢复SPI |
| `sensors all` | 集中执行三颗 BMP580 和一颗 SHT40 的读取 |

当前固定映射：

| 接口 | 通道 | 模块 | 建议对象 |
|---|---:|---|---|
| SH1 | 0 | BMP580 | 外界大气，压差基准 |
| SH2 | 1 | BMP580 | 氦气升力囊 |
| SH3 | 2 | BMP580 | 空气压载囊 |
| SH4 | 3 | SHT40 | 外界温湿度 |

### 执行器命令

先解锁：

```text
actuator arm
```

| 命令 | 参数和作用 |
|---|---|
| `actuator status` | 查看是否解锁、当前动作和剩余时间 |
| `actuator valve <通道> <时长ms>` | 通道为 1 或 2；打开指定电磁阀 50～30000 ms，到时自动关闭 |
| `actuator group <1\|2> <intake\|exhaust> <时长ms>` | 组1=XH1泵1+XH3阀2；组2=XH2泵2+XH4阀1；同时开阀并正/反向运行泵 |
| `actuator pump <通道> <方向> <时长ms>` | 通道为 1 或 2；方向为 `fwd` 或 `rev`；运行 50～30000 ms |
| `actuator motor <通道> <方向> <占空比%> <时长ms>` | 通道为 1 或 2；方向为 `fwd` 或 `rev`；占空比 1%～30%；运行 50～30000 ms |
| `actuator servo <通道> <脉宽us> <时长ms>` | 通道为 1 或 2；脉宽 1000～2000 us；保持 50～30000 ms |
| `actuator stop` | 立即停止当前动作并上锁 |
| `actuator disarm` | 与 `actuator stop` 相同，立即停止并上锁 |

分组命令通过飞控板 USB CDC 串口发送。`intake`/`exhaust` 也可写成
`inhale`/`exhale` 或 `fwd`/`rev`。动作结束、超时或执行 `actuator group stop` 时，
泵停止并关闭电磁阀。

这里的 `intake`/`exhaust` 是泵电机 H 桥正反极性的逻辑标签，不保证所用泵能够物理
反向输送。当前水路观察表明泵大概率为单向泵，本命令暂按电气和泵阀联动测试使用；正式
充排气能力需更换双向泵或增加气路切换后再验收。额定介质仅为空气的泵不得继续用水测试。

`fwd` 是 forward（正向），`rev` 是 reverse（反向）。兼容别名包括 `outputs`、
`arm outputs`、`stop` 和 `disarm`，建议新测试统一使用 `actuator ...` 形式。

例如，让 1 号电磁阀打开 3 秒：

```text
actuator arm
actuator valve 1 3000
```

通道对应：阀1/2、泵1/2、空心杯电机1/2和舵机1/2。当前动作未结束时，其他动作命令
会因 `busy`（忙碌）被拒绝。

### 飞控射频与任务命令

| 命令 | 作用 |
|---|---|
| `radio` 或 `radio status` | 查看 E28 电源、模式、授权、引脚、IRQ 和收发计数 |
| `radio probe` | 无发射读取 SX1281 状态和分组类型 |
| `radio power off` | 复位射频、三态化总线并关闭控制输出，供安全拆除电池侧电源 |
| `radio power on` | 重新允许射频总线，之后还要执行 `radio init` |
| `radio init` | 初始化 SX1281，默认不授予发射权限 |
| `radio rx` | 进入接收模式 |
| `radio arm antenna` | 确认已安装天线，临时授予 60 秒手动发射权限 |
| `radio ping` | 发送一帧飞控 PING |
| `radio send <文本>` | 发送一帧文本，负载不超过公共协议允许长度 |
| `radio disarm` 或 `radio reset` | 取消发射授权并复位射频 |
| `mission start antenna` | 确认天线后进入任务通信并开始飞控 CSV 日志 |
| `mission stop` | 停止执行器、上锁、复位射频并异步同步关闭日志 |

手动 `radio arm antenna` 的 60 秒限制没有取消；只有执行器本地解锁改为无时间限制。
任务模式的射频授权随任务会话保持，直到 `mission stop`、射频错误或安全状态触发。

## 地面站本地命令

命令在地面站 USB 串口中发送。

### 基础、存储和 Flash

| 命令 | 作用 |
|---|---|
| `help` 或 `?` | 显示主要命令提示 |
| `version` | 显示地面站硬件、固件和协议版本 |
| `status` | 显示射频、供电、Flash、TF 卡、日志、内存和任务状态 |
| `test` | 执行板级状态、TF 卡临时文件读写和射频无发射探测 |
| `adc` | 读取 USB 供电采样和估算电压 |
| `flash` | 读取 W25Q128 的 JEDEC（Joint Electron Device Engineering Council，联合电子器件工程委员会）ID |
| `sd` | 查看卡检测回退、挂载、日志文件和丢记录计数 |
| `sdtest` | 写入、读回、校验并删除临时文件 |
| `sd force on` | 忽略硬件卡检测问题并强制尝试使用 TF 卡 |
| `sd force off` | 卸载文件系统、停止自动回退，供安全取卡 |
| `sd raw <总线位宽>` | 总线位宽为 1 或 4；使用指定 SDIO（Secure Digital Input Output，安全数字输入输出）位宽做底层诊断 |
| `sd mount <总线位宽>` | 总线位宽为 1 或 4；使用指定总线位宽尝试挂载文件系统 |

地面站硬件 V1.1.0 的卡检测网络没有实际接到 MCU，因此正常使用时固件会自动采用与
`sd force on` 等效的回退；正式配置保持 SDIO 4 位。

### 地面站射频与任务命令

| 命令 | 作用 |
|---|---|
| `radio` 或 `radio status` | 查看 E28/SX1281 状态 |
| `radio probe` | 无发射探测 SX1281 |
| `radio init` | 初始化射频但保持发射锁定 |
| `radio rx` | 进入接收模式 |
| `radio arm antenna` | 确认天线后授予 60 秒手动发射权限 |
| `radio ping` | 发送一帧地面站 PING |
| `radio send <文本>` | 发送文本帧 |
| `radio disarm` 或 `radio reset` | 取消发射授权并复位射频 |
| `mission start antenna` | 启动任务心跳、遥测接收和地面站 CSV 日志 |
| `mission stop` | 停止任务通信并异步同步关闭日志 |

## 地面站远程控制飞控

### 启动顺序

1. 两块板断电，分别安装匹配的 2.4 GHz 天线。
2. 飞控接入电池侧电源，地面站接入 USB；确认两端 `status` 无关键电源故障。
3. 在地面站发送 `mission start antenna`。
4. 在飞控发送 `mission start antenna`。
5. 等待地面站出现 `GS telemetry ... link=1`。
6. 如需控制执行器，在飞控本地 USB 串口发送 `actuator arm`。
7. 在地面站 USB 串口发送下面的 `fc ...` 命令。

### 可远程发送的全部飞控命令

| 地面站命令 | 飞控动作 |
|---|---|
| `fc status` | 请求一帧飞控状态遥测，不要求执行器解锁 |
| `fc stop` | 远程急停当前执行器并撤销飞控本地解锁 |
| `fc valve <通道> <时长ms>` | 通道为 1 或 2；远程打开指定电磁阀 50～30000 ms |
| `fc pump <通道> <方向> <时长ms>` | 通道为 1 或 2；方向为 `fwd` 或 `rev`；远程运行 50～30000 ms |
| `fc motor <通道> <方向> <占空比%> <时长ms>` | 通道为 1 或 2；方向为 `fwd` 或 `rev`；占空比 1%～30%；远程运行 50～30000 ms |
| `fc servo <通道> <脉宽us> <时长ms>` | 通道为 1 或 2；脉宽 1000～2000 us；远程保持 50～30000 ms |

示例：

```text
fc status
fc valve 1 500
fc pump 1 fwd 1000
fc motor 1 fwd 10 1000
fc servo 1 1500 1000
fc stop
```

除 `fc status` 和 `fc stop` 外，远程执行器命令要求：两端处于同一任务会话、飞控本地
已解锁、参数合法、链路与时间同步有效、当前没有其他动作。地面指令有效期用于判断命令
是否过期，不限制已经开始的最长 30 秒动作；链路丢失仍会立即触发 failsafe（失联安全
状态）并停止执行器。

每条远程命令会收到 ACK（Acknowledgement，确认应答）：

- `started`：动作已经开始。
- `completed`：动作按计划结束，或状态/停止请求完成。
- `stopped`：动作被安全逻辑提前停止。
- `rejected`：命令被拒绝，并附带 `outputs_disarmed`、`busy`、`range`、`expired`、
  `duplicate_or_old` 或 `link_lost` 等原因。

`fc stop` 会让飞控重新上锁；若还要继续测试执行器，必须回到飞控本地再次执行
`actuator arm`。

## 任务日志与安全下电

进入任务模式后：

- 飞控生成 `FCDxxxx.CSV`（Flight Controller Data，飞控周期数据）和
  `FCExxxx.CSV`（Flight Controller Events，飞控事件）。
- 地面站生成 `GSDxxxx.CSV`（Ground Station Data，地面站接收数据）和
  `GSExxxx.CSV`（Ground Station Events，地面站事件）。

结束任务建议依次执行：

```text
地面站：fc stop
飞控：mission stop
地面站：mission stop
```

然后分别执行 `status`，等待两端均显示 `log_owner=0`，再拔 TF 卡或断电。维护模式下
没有运行任务日志时，飞控先执行 `actuator disarm` 并确认 `armed=0 action=none`；随后
先断开电池，再断开 USB。上锁不是电气断电的必要条件，但应作为固定安全操作规程。

## 开发与发布约定

- CubeMX 重新生成代码前确认用户代码区和安全初始电平不会被覆盖。
- 新版本必须执行 `cmake --build --preset Debug --clean-first`。
- 仓库提交 `build/Debug` 顶层当前 `.hex/.bin`，不提交缓存、`.elf`、`.map` 和中间文件。
- 硬件版本与软件修订号分开，例如硬件 `V1.0.2` 对应固件 `V1.0.2.10`。
- 正式控制功能必须先定义故障处理和安全边界，再接入执行器输出。
