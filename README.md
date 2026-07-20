# Balloon Hardware Test Firmware

本仓库保存浮力气球 Demo 的飞控板与地面站板硬件测试固件，以及两端共用的嵌入式
代码。它用于板级上电、传感器、存储、射频和执行器验证，并作为后续正式飞控固件
开发前的硬件验证基线；不应把本仓库中的测试任务模式视为正式飞行控制软件。

## 当前版本

- 飞控硬件：V1.0.2；飞控固件：V1.0.2.9。
- 地面站硬件：V1.1.0；地面站固件：V1.1.0.7。
- MCU（Microcontroller Unit，微控制器）：STM32F405RGT6。

当前代码已经覆盖板级安全状态、USB 串口、传感器检测、SD 卡、E28/SX1281
射频通信、任务指令、遥测、失联保护和 CSV（Comma-Separated Values，逗号分隔值）
日志。姿态解算、状态估计、闭环高度控制和正式任务状态机仍需在后续开发中逐步完成。

## 目录

- `FCFM/`：飞控工程；`FCFM` 为沿用的现有工程目录名。
- `ground-station/`：地面站工程。
- `common/`：两套固件共用的射频协议、E28/SX1281 驱动和日志代码。
- `E28_SX1281_TEST_GUIDE.md`：双板射频通信测试说明。

## 仓库内烧录文件

仓库直接保留当前版本编译得到的 HEX（Intel HEX，英特尔十六进制）和 BIN
（Binary，二进制）烧录文件，便于不重新编译就进行硬件测试：

- `FCFM/build/Debug/FCFM_BOARD_TEST_V1.0.2.9.hex`
- `FCFM/build/Debug/FCFM_BOARD_TEST_V1.0.2.9.bin`
- `ground-station/build/Debug/GROUND_STATION_BOARD_TEST_V1.1.0.7.hex`
- `ground-station/build/Debug/GROUND_STATION_BOARD_TEST_V1.1.0.7.bin`

同一目录还保留不带版本号的通用名和工程默认输出名。CMake 缓存、中间目标文件、
ELF（Executable and Linkable Format，可执行与可链接格式）和 MAP（链接映射）文件
仍不纳入版本管理。

## 编译

工程使用 STM32CubeMX 生成的 HAL（Hardware Abstraction Layer，硬件抽象层）代码，
通过 CMake、Ninja 和 GNU Arm Embedded Toolchain 编译。

飞控：

```powershell
cd FCFM
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

烧录文件：

```text
FCFM/build/Debug/FCFM_BOARD_TEST_V1.0.2.9.hex
```

地面站：

```powershell
cd ground-station
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

烧录文件：

```text
ground-station/build/Debug/GROUND_STATION_BOARD_TEST_V1.1.0.7.hex
```

## 烧录与串口

使用 STM32CubeProgrammer，通过 USB DFU（Device Firmware Upgrade，设备固件升级）
模式烧录对应 HEX 文件。烧录完成后将 BOOT0 恢复为低电平并重新上电。

USB CDC（Communications Device Class，通信设备类）串口参数：

```text
115200 baud, 8 data bits, no parity, 1 stop bit, CRLF
```

飞控常用检测命令：

```text
version
test
sensors all
sdtest
actuator status
```

执行器必须先在飞控本地解锁，解锁有效期为 60 秒：

```text
actuator arm
actuator valve 1 500
actuator pump 1 fwd 500
actuator motor 1 fwd 10 500
actuator servo 1 1500 1000
actuator stop
```

`fwd` 是 forward（正向），`rev` 是 reverse（反向）。一次只测试一路执行器，
首次测试应卸除机械负载，并从短时间、低占空比开始。

地面站常用检测命令：

```text
version
test
sdtest
```

## 射频安全

两端安装匹配的 2.4 GHz 天线后，先在地面站、再在飞控执行：

```text
mission start antenna
```

结束时先停止飞控任务，再停止两端任务模式。没有安装天线时，严禁初始化发射或进入
任务通信模式；芯片设置为最低功率不等于天线口没有射频功率。

## 开发约定

- CubeMX 重新生成代码前先确认用户代码区和安全初始电平不会被覆盖。
- 新固件发布前必须执行 `--clean-first` 全量编译。
- 提交 `build/Debug` 顶层的 `.hex` 和 `.bin` 烧录文件，但不提交其他构建缓存和中间文件。
- 硬件版本和固件版本分别维护，例如硬件 V1.0.2 对应固件 V1.0.2.9。
- 正式控制功能必须先定义故障处理和安全边界，再接入执行器输出。
