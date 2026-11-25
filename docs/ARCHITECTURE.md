# 架构说明

重构后的代码严格分为三层，驱动逻辑里没有任何板级硬编码。

```
┌─────────────────────────────────────────────────────────────────────┐
│                        机载电脑 / ROS 2 (CM5)                        │
│        px4_msgs: fmu/out/distance_sensor (uXRCE-DDS)                │
└───────────────────────────────▲─────────────────────────────────────┘
                                │ UXRCE-DDS (TELEM 2, 921600)
┌───────────────────────────────┴─────────────────────────────────────┐
│ PX4 飞控侧                                                          │
│                                                                     │
│  rc.serial (自动生成)                                                │
│    SENS_TOFMS_CFG=103 ──> tof_sense_ms start -i 0 -d /dev/ttyS4 ...  │
│                                                                     │
│  ┌──────────────────────── 驱动调度层 ──────────────────────────┐   │
│  │ TOFSenseMS : ModuleBase/ScheduledWorkItem                    │   │
│  │   4 ms 轮询、串口 I/O、超时重同步、聚合、统计、发布           │   │
│  └───────────────────────────┬──────────────────────────────────┘   │
│                              │ Frame (纯 C++ 结构体)                 │
│  ┌───────────────────────────▼──────────────────────────────────┐   │
│  │ 协议解析层 tof_sense_ms_parser (无 PX4 依赖)                  │   │
│  │   流式状态机、校验和、单点/4x4/8x8 帧、错误分类                │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                              │ distance_sensor uORB                 │
│  ┌───────────────────────────▼──────────────────────────────────┐   │
│  │ EKF2 / 避障 / logger / MAVLink                               │   │
│  │   EKF2_RNG_CTRL, EKF2_HGT_REF, EKF2_RNG_NOISE ...            │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 1. 协议解析层（纯 C++）

文件：`tof_sense_ms_parser.{h,cpp}`

- 只依赖 `<cstdint>`，不包含任何 PX4 头文件；
- 可在 PC 上编译运行单元测试（`test/`），也可直接移植到 Arduino / STM32 HAL；
- 四状态流式状态机：`Sync → Function → Header → Payload`；
- 支持单点 16 字节帧与 4×4 / 8×8 面阵帧（含/不含 6 字节尾部）；
- 错误分类：`header_errors` / `length_errors` / `checksum_errors`；
- 缓冲固定 400 字节，写入前检查上限，不动态分配内存。

数据流：`uint8_t` → `Parser::push()` → 校验通过的 `Frame`（含最多 64 个
`Pixel`）。

---

## 2. 驱动调度层（PX4）

文件：`TOFSenseMS.{hpp,cpp}`、`tof_sense_ms_main.cpp`、`module.yaml`、
`CMakeLists.txt`、`Kconfig`。

职责：

| 职责 | 实现 |
|---|---|
| 串口打开/配置 | `open(O_RDWR|O_NOCTTY|O_NONBLOCK)` + termios 8N1，921600 默认 |
| 调度 | `ScheduledWorkItem`，4 ms 周期，单次最多读 16×256 字节 |
| 解析 | 逐字节喂 `Parser::push()` |
| 过滤 | `status == 0` 且 `[min, max]`，单点盲区 0.3 m |
| 聚合 | 中心区均值 / 全图中位数 / 全图均值 |
| 发布 | `PX4Rangefinder::update()` → `distance_sensor` |
| 超时 | 1 s 无有效帧 → `timeouts++`、reset parser |
| 统计 | 每实例独立的帧数、帧率、发布数、无效帧、超量程、解析错误、超时 |
| 多实例 | `g_dev[0..1]`，参数、状态、perf 计数器完全隔离 |

性能特征（STM32H7，8×8 10 Hz）：

- 每帧 400 字节，10 Hz → 4 kB/s；
- 聚合是 64 个元素的插入排序/求和，单帧耗时可忽略；
- 每实例固定 RAM：一个 `Frame`（约 64×8 B）+ 400 B 解析缓冲 + 状态；
- 不使用动态内存（除对象本身），不使用浮点排序。

### 2.1 多实例

| 实例 | 参数 |
|---|---|
| 0 | `SENS_TOFMS_CFG`、`SENS_TOFMS_ROT`、`SENS_TOFMS_AGG` |
| 1 | `SENS_TOFMS_CFG2`、`SENS_TOFMS_ROT2`、`SENS_TOFMS_AGG2` |

`SENS_TOFMS_MIN/MAX` 描述传感器型号，两个实例共享。每个实例：

- 自己的串口 fd、解析器、帧缓冲、统计计数；
- 自己的 `PX4Rangefinder` 与 `device_id`（`bus` = ttyS 编号）；
- 自己的 perf 计数器（`tof_sense_ms:N read` / `com_err`）；
- 同一个物理串口不能被两个实例同时占用（`start` 拒绝）。

---

## 3. 板级配置解耦

驱动目录里没有任何 `boards/...` 引用。板级只做两件事：

1. `default.px4board` 打开 `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y`；
2. `init/rc.board_defaults` 用 `param set-default` 预置端口/波特率/安装方向。

实际启动由 PX4 自动生成的 `rc.serial` 完成：

```sh
set PRT SENS_TOFMS_CFG
. $PRT_F
if [ $SERIAL_DEV != none ]; then
	tof_sense_ms start -i 0 -d ${SERIAL_DEV} -b p:${BAUD_PARAM} \
		-R p:SENS_TOFMS_ROT -a p:SENS_TOFMS_AGG \
		-m p:SENS_TOFMS_MIN -M p:SENS_TOFMS_MAX
fi
```

换板子只需要在 `module.yaml` 之外提供目标板的 `CONFIG_BOARD_SERIAL_*`
（PX4 已经声明），TOF 不需要任何板级代码。示例见
[corvon_743v2.md](corvon_743v2.md)。

---

## 4. 飞控 / 机载电脑边界

飞控端保持轻量：

- 只发布聚合后的**单点** `distance_sensor`；
- 不实现多区深度图、点云、避障决策、ROS 2 通信；
- 不引入额外动态内存或长耗时计算。

CM5 侧通过 uXRCE-DDS 订阅 `fmu/out/distance_sensor`，按 `orientation` 区分
朝下（25）与朝前（0）实例，后续的深度图、避障、地形跟随算法全部在机载电脑
实现。这样飞控固件保持可预测的 CPU/RAM 占用，机载电脑可以独立升级算法。

---

## 5. 与 PX4 原生测距仪的一致性

| 特性 | tfmini | mb12xx | tof_sense_ms |
|---|---|---|---|
| 标准 `SENS_*_CFG` 串口选择 | ✅ | ✅ | ✅ |
| `rc.serial` 自动拉起 | ✅ | ✅ | ✅ |
| 标准 `distance_sensor` uORB | ✅ | ✅ | ✅ |
| 多实例 | ❌ | ❌ | ✅（2 路） |
| 纯 C++ 解析层单测 | ❌ | ❌ | ✅ |

驱动使用的公共设施：`ScheduledWorkItem`、`PX4Rangefinder`、
`perf_counter`、`px4_get_parameter_value()`、`module.yaml` 参数生成。
