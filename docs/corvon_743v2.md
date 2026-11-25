# corvon_743v2 板级集成

本文件说明如何把 `tof_sense_ms` 集成到 `boards/corvon/743v2/`，并给出
**树莓派 CM5（ROS 2 / uXRCE-DDS）** 的端口与参数预留。驱动代码本身不包含
任何 corvon 板信息，换板只需改这里的两个文件。

---

## 1. 板级文件改动

### 1.1 `boards/corvon/743v2/default.px4board`

```diff
+CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y
 CONFIG_MODULES_UXRCE_DDS_CLIENT=y
```

`UXRCE_DDS_CLIENT` 在 corvon_743v2 上本来就是 `y`，这里列出来是为了让这份
集成说明自包含。

### 1.2 `boards/corvon/743v2/init/rc.board_defaults`

只设置 **默认值**（`param set-default`），用户已有的参数不会被覆盖：

```sh
# >>> tof_sense_ms >>>
# Nooploop TOFSense on TELEM 3 (/dev/ttyS4), 921600 8N1, downward facing.
param set-default SENS_TOFMS_CFG   103
param set-default SENS_TOFMS_ROT   25
param set-default SENS_TOFMS_AGG   0
param set-default SENS_TOFMS_MIN   0.15
param set-default SENS_TOFMS_MAX   4.0
param set-default SER_TEL3_BAUD    921600

# Raspberry Pi CM5 companion link on TELEM 2 (/dev/ttyS3), uXRCE-DDS at 921600.
param set-default MAV_1_CONFIG      0
param set-default UXRCE_DDS_CFG     102
param set-default UXRCE_DDS_FLCTRL  0
param set-default SER_TEL2_BAUD     921600
# <<< tof_sense_ms <<<
```

仓库里对应 `boards/corvon/743v2/default.px4board.append` 与
`boards/corvon/743v2/init/rc.board_defaults.append`，也可以用脚本自动插入：

```bash
python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --board corvon/743v2
```

---

## 2. 串口映射

corvon_743v2 的 `default.px4board` 里已经声明了标准逻辑串口：

| QGC 名称 | 索引 | 设备 | 本集成用途 |
|---|---:|---|---|
| TELEM 1 | 101 | `/dev/ttyS0` | 常规遥测（MAVLink 实例 0） |
| **TELEM 2** | **102** | `/dev/ttyS3` | **树莓派 CM5 / uXRCE-DDS** |
| **TELEM 3** | **103** | `/dev/ttyS4` | **TOFSense（本驱动）** |
| TELEM/SERIAL 4 | 104 | `/dev/ttyS7` | 板载蓝牙 MAVLink（不要占用） |
| GPS 1 | 201 | `/dev/ttyS2` | GPS |
| Radio Controller | 300 | `/dev/ttyS5` | RC |

> 接线的关键：TOFSense 只发不收，传感器 **TX → 飞控 RX**、GND、5 V。
> 不需要飞控 TX。具体连接器针脚以 corvon 板丝印/原理图为准；确认 `ttyS4`
> 就是 TELEM 3 的 UART 即可。

---

## 3. 为什么 TOF 用 TELEM 3 而不是 TELEM 2

- TELEM 2 被规划给 CM5 的 uXRCE-DDS（102），需要独占；
- TELEM 4 已被 corvon 板级默认值用于板载蓝牙 MAVLink（`MAV_2_CONFIG=104`，
  `SER_TEL4_BAUD=115200`），不要和 TOF 抢；
- TELEM 1 留给主遥测；
- TELEM 3 空闲，且 921600 波特率完全够 400 字节 × 10 Hz 的面阵帧。

如果实际接线不同，只需在 QGC 里把 `SENS_TOFMS_CFG` 改成对应逻辑串口，并把
该端口的 `SER_xxx_BAUD` 设为 921600，**不需要改驱动或重编固件**。

---

## 4. CM5 / uXRCE-DDS 边界

飞控侧只做一件事：把 TOF 聚合后的单点距离发布到标准 `distance_sensor`
uORB 话题。多区深度图、避障策略、ROS 2 节点全部放到 CM5 侧，飞控驱动不
参与。

建议参数：

| 参数 | 值 | 说明 |
|---|---|---|
| `UXRCE_DDS_CFG` | `102` | 在 TELEM 2 上前台启动 uXRCE-DDS client |
| `SER_TEL2_BAUD` | `921600` | CM5 串口波特率（与 CM5 侧一致） |
| `MAV_1_CONFIG` | `0` | 释放 TELEM 2，避免 MAVLink 和 DDS 抢同一个 tty |
| `UXRCE_DDS_FLCTRL` | `0` | 本版本 PX4 没有 `SER_TEL2_FLOW` 参数；DDS 的硬件流控用 `UXRCE_DDS_FLCTRL`（0=关闭） |

> 注意：任务书里写的 `SER_TEL2_FLOW` 在 PX4 v1.18 并不存在；串口的流控
> 由具体协议参数控制（MAVLink 用 `MAV_x_FLOW_CTRL`，uXRCE-DDS 用
> `UXRCE_DDS_FLCTRL`）。这里用实际存在的参数，避免写入一个无效参数。

CM5 侧通过 `px4_msgs` 订阅：

```
fmu/out/distance_sensor     # px4_msgs/msg/DistanceSensor
```

两个实例会以不同的 `orientation`（25 朝下 / 0 朝前）同时出现，CM5 可以按
`orientation` 区分定高与测障数据。

---

## 5. 编译与烧录

```bash
# 1. 打补丁（幂等）
cd ~/px4/PX4-Autopilot
python3 /path/to/moonpilot-tof/tools/apply_to_px4.py \
    --px4-root . --board corvon/743v2

# 2. 编译
make corvon_743v2_default -j"$(nproc)"

# 3. 烧录
#    QGC -> 固件 -> 选择 build/corvon_743v2_default/corvon_743v2_default.px4
#    或者：make corvon_743v2_default upload
```

编译产物：

```
build/corvon_743v2_default/corvon_743v2_default.px4
build/corvon_743v2_default/corvon_743v2_default.bin
```

本仓库在 PX4 `v1.18.0-beta1-248-gae5ebc8a80` 上实测（驱动 +
`UXRCE_DDS_CLIENT`）：

| 项 | 数值 |
|---|---|
| Flash | 1,693,736 B / 1,835,008 B = **92.30%** |
| AXI_SRAM | 69,144 B / 512 KB = **13.19%** |
| `arm-none-eabi-size` | text 1,689,576 / data 4,160 / bss 65,304 |
| 编译 | 无报错、无 TOF 相关 warning |

---

## 6. 上电自检

QGC MAVLink Console：

```sh
tof_sense_ms status
listener distance_sensor
param show SENS_TOFMS_CFG
param show SER_TEL3_BAUD
```

正常情况下 `SENS_TOFMS_CFG = 103`、`SER_TEL3_BAUD = 921600`，驱动在
`/dev/ttyS4` 上以 10 Hz 收帧，`orientation = 25`。详细判据和 EKF2 参数见
[QGC_SETUP.md](QGC_SETUP.md)。

---

## 7. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 编译报 `DRV_DIST_DEVTYPE_TOFMS` 未定义 | 没执行 `apply_to_px4.py` 的 `drv_sensor.h` 步骤 | 重新跑脚本，或手工加 `#define DRV_DIST_DEVTYPE_TOFMS 0x9D` |
| QGC 里看不到 `SENS_TOFMS_CFG` | 板级没打开驱动 / 没重新生成参数 | 检查 `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y`，清理 `build/corvon_743v2_default` 后重编 |
| 驱动起了但帧率 0 | 选错 tty / 波特率 | `tof_sense_ms status` 看端口；确认 `SER_TEL3_BAUD=921600` |
| DDS 和 MAVLink 抢 TELEM 2 | `MAV_1_CONFIG` 仍为 102 | 设为 0（或改到别的端口），重启 |
| `SENS_TOFMS_CFG2` 和 `SENS_TOFMS_CFG` 选了同一串口 | `rc.serial` 冲突检测 | 改成两个不同物理串口；日志会出现 `Conflicting config` |
