# 飞控 V1.0.2.10 与 E28 双板测试指南

适用工程：

```text
飞控：D:\code\Balloon\firmware\FCFM
地面站：D:\code\Balloon\firmware\ground-station
```

## 1. 版本与功能边界

- 飞控硬件：`V1.0.2`，飞控固件：`V1.0.2.10`。
- 地面站硬件：`V1.1.0`，地面站固件：`V1.1.0.8`。
- 飞控原有 ADC、IMU、I²C、GNSS、SD、泵、阀、电机和舵机功能继续保留。
- 新增 E28-2G4M20SX（SX1281）链路、正式指令、ACK、遥测和失联保护。

## 2. 编译与烧录

分别用 VS Code 打开两个工程目录，点击：

```text
终端 → 运行生成任务
```

成功后烧录：

```text
飞控：build\Debug\FCFM_BOARD_TEST_V1.0.2.10.hex
地面站：build\Debug\GROUND_STATION_BOARD_TEST_V1.1.0.8.hex
```

使用 STM32CubeProgrammer：连接方式选“USB”→“连接”→“擦除和编程”→选择 HEX →
勾选“校验编程”和“烧录后运行”→“开始烧录”。完成后将 BOOT0 恢复为低并重新上电。

串口监视器使用 115200、8-N-1、CRLF。

## 3. 无天线测试

没有安装两端天线时，严禁执行 `mission start antenna` 和任何发射命令。

飞控发送：

```text
version
test
```

`test` 会一次完成非执行器状态测试和 E28 非发射探测。飞控只插 USB 时，E28
通常没有电，射频步骤出现 `power_present=0`/`power_off` 是正常结果；要探测 E28，
需要接入电池侧电源并用万用表确认 `3V3_RADIO` 约为 3.3 V。

地面站发送：

```text
version
test
```

E28 正常探测的关键结果为：

```text
packet_type=0x01 error=none result=PASS
```

测试结束后射频保持 reset，TXEN/RXEN 均为低。

## 4. 飞控本地执行器测试

先断开螺旋桨或其他危险机械负载，固定泵和电机，然后：

```text
actuator arm
actuator valve 1 300
actuator pump 1 fwd 300
actuator motor 1 fwd 10 300
actuator servo 1 1500 500
actuator stop
```

按同样方式测试第 2 通道和反向。每次动作结束应看到自动停止日志；任何异常立即发送
`actuator stop`。

## 5. 安装天线后的正式任务测试

1. 两板完全断电。
2. 两端安装匹配的 2.4 GHz 天线，不要带电插拔天线。
3. 初次相距约 1～3 m，上电并打开两个 USB 串口。
4. 地面站发送：

```text
mission start antenna
```

5. 飞控发送：

```text
mission start antenna
```

6. 等待地面站出现：

```text
GS telemetry ... mode=mission ... link=1 ...
```

7. 地面站查询飞控：

```text
fc status
```

应先看到状态指令 `completed` ACK，随后收到同序号遥测。

8. 如需验证远程执行器，先在飞控本地发送：

```text
actuator arm
```

再在地面站逐项发送，并等待上一条指令完成后再发下一条：

```text
fc valve 1 300
fc pump 1 fwd 300
fc motor 1 fwd 10 300
fc servo 1 1500 500
```

正常动作指令先返回 `started`，动作到时关闭后返回 `completed`。未本地解锁、范围
错误、重复、过期、链路失效或执行器忙时返回 `rejected` 和明确原因。
飞控本地解锁不按时间自动失效，单次动作允许 50～30000 ms；结束测试应执行
`fc stop` 或飞控本地 `actuator disarm`，使输出停止并重新上锁。

9. 结束任务：

```text
地面站：fc stop
飞控：mission stop
地面站：mission stop
```

## 6. 任务模式数据

飞控每 1000 ms 回传任务号、模式、时间戳、故障位、电池电压、ADC、IMU 有效性、
SD 卡、链路、执行器方向与状态、飞控日志状态和射频计数；地面站同时添加
RSSI（Received Signal Strength Indicator，接收信号强度指示）和
SNR（Signal-to-Noise Ratio，信噪比）。

地面站每 2000 ms 发送时间心跳。飞控 6000 ms 未收到有效地面帧时进入 failsafe、
停止执行器并禁止继续执行控制指令。重连不会自动执行断链期间的旧指令。

任务停止后，等待两端 `status` 均显示 `log_owner=0` 再拔卡；如果 `log=2`，说明
本次记录出现错误，应同时保存 `log_result` 和 `log_drop` 供排查。飞控卡应出现
`FCDnnnn.CSV`（Flight Controller Data，飞控周期数据）和
`FCEnnnn.CSV`（Flight Controller Events，飞控事件）；地面站卡应出现
`GSDnnnn.CSV`（Ground Station Data，地面站接收数据）和
`GSEnnnn.CSV`（Ground Station Events，地面站事件）。同一板上的两份文件编号
应一致，且再次启动任务会使用新编号，不覆盖旧数据。

## 7. 供电与停止

飞控的 `3V3_RADIO` 来自电池侧，只有 USB 时只能运行 MCU/USB 测试。USB 仍连接时
要拔电池，先执行 `mission stop`，确认射频 reset、TXEN/RXEN 为低后再拔。

SX1281 芯片设置 `-18 dBm` 只是芯片最低档；E28 外部 PA 后的实际输出必须实测，
不能据此允许无天线发射。
