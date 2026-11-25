# QGC 配置与实机验证清单

本文按 PX4 原生测距仪（`tfmini` / `mb12xx` / `lightware` 等）的配置方式进行，
所有步骤都可以在 **QGroundControl** 内完成，不需要改板级脚本、不需要手工改
`/dev/ttySx`。

> 目标：QGC 参数里选一个逻辑串口 → 重启 → 驱动自动拉起 → `distance_sensor`
> 正常发布 → EKF2 融合测距。

---

## 0. 前置条件

- 固件已打开 `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y` 并烧录（本文以
  `corvon_743v2_default` 为例，板级默认值见 [corvon_743v2.md](corvon_743v2.md)）。
- 传感器已接好：TOFSense **TX → 飞控 RX**、GND、5 V（模块峰值电流较大，建议
  独立 5 V 或加大电容）。
- 模块保持出厂默认：**921600 8N1**、主动上报（如改过，用厂商 NAssistant
  恢复，或在 QGC 里把对应 `SER_xxx_BAUD` 改成模块当前的波特率）。

---

## 1. QGC 串口绑定（标准 serial_config）

1. QGC 连接飞控 → **Parameters** → 搜索 `SENS_TOFMS`。
2. 设置：

   | 参数 | 示例值 | 说明 |
   |---|---|---|
   | `SENS_TOFMS_CFG` | `TELEM 3`（=103） | 第一路 TOF 接的逻辑串口；`Disabled` 表示不启动 |
   | `SENS_TOFMS_ROT` | `25` | 朝下安装（定高）；朝前测障填 `0` |
   | `SENS_TOFMS_AGG` | `0` | 中心区均值（定高推荐）；`1` 中位数、`2` 均值 |
   | `SENS_TOFMS_MIN` | `0.15` | 最小距离 [m] |
   | `SENS_TOFMS_MAX` | `4.0` | 最大距离 [m] |
   | `SER_TEL3_BAUD` | `921600` | 所选端口的波特率，必须与传感器一致 |

   > 选哪个逻辑串口，就改哪个 `SER_xxx_BAUD`。例如选 `TELEM 1` 就是
   > `SER_TEL1_BAUD`，选 `GPS 1` 就是 `SER_GPS1_BAUD`。`corvon_743v2` 的板级
   > 默认值已经把 `SENS_TOFMS_CFG=103` 和 `SER_TEL3_BAUD=921600` 预置好。
3. **重启飞控**（参数在启动时读取）：QGC → **Tools → Reboot Flight Controller**，
   或重新上电。
4. 如果 QGC 参数列表里没有 `SENS_TOFMS_CFG`，说明固件没打开驱动或没有重新
   生成参数，回到 [corvon_743v2.md](corvon_743v2.md) 检查编译配置。

底层机制（供排障参考）：`module.yaml` 里的 `serial_config` 会在编译时生成
`SENS_TOFMS_CFG` 参数和 `rc.serial` 启动项，开机时 `rc.serial` 根据参数值把
逻辑串口翻译成 `/dev/ttySx`，然后把 `-d /dev/ttySx -b p:SER_xxx_BAUD` 传给
驱动。板级脚本里没有任何 TOF 专用分支。

---

## 2. 底层验证（QGC MAVLink Console）

QGC → **Tools → MAVLink Console**：

```sh
# 驱动是否起来、帧率是否正常、错误计数是否为零
tof_sense_ms status

# 标准 uORB 消息是否正确
listener distance_sensor

# 确认 EKF2 测距融合已打开
param show EKF2_RNG_CTRL
```

期望输出（单实例、朝下安装）：

```
INFO  [tof_sense_ms] instance 0: /dev/ttyS4 @ 921600 Bd, orientation 25, aggregation center, range 0.15..4.00 m
INFO  [tof_sense_ms]   frames: 547 total, 10 Hz, published 547, no-valid-zone 0, out-of-range 0
INFO  [tof_sense_ms]   parser: header/length errors 0, checksum errors 0
INFO  [tof_sense_ms]   link: timeouts 0, last frame 31 ms ago, last distance 2.501 m (type 1, 64 zones)
```

```
TOPIC: distance_sensor
 distance_sensor
    timestamp: ...
    device_id: ...
    min_distance: 0.1500
    max_distance: 4.0000
    current_distance: 2.5010     <-- 随遮挡物变化
    type: 0                      <-- MAV_DISTANCE_SENSOR_LASER
    h_fov: 1.1000
    v_fov: 1.1000
    orientation: 25              <-- 朝下
    variance: ...
    signal_quality: 87
```

判定标准：

- `frames` 每秒增长，`Hz` ≈ 传感器上报频率（默认 10 Hz，可在 NAssistant 改）；
- `parser: header/length errors 0, checksum errors 0`；
- `timeouts 0`（拔线后 1 s 会 +1，重新接上后恢复）；
- `current_distance` 与卷尺/地面距离一致，`orientation = 25`。

---

## 3. EKF2 定高参数

QGC → **Parameters**，按用途二选一：

| 参数 | 值 | 说明 |
|---|---|---|
| `EKF2_RNG_CTRL` | `1` | **Conditional**：仅在测距与气压计一致时融合，常规/户外推荐 |
| | `2` | **Always enabled**：室内全程在量程内时推荐 |
| `EKF2_HGT_REF` | `0` | 气压计为主，测距做低空辅助（默认） |
| | `2` | Range sensor 测距为主，室内无 GPS 贴地定高 |
| `EKF2_RNG_NOISE` | `0.1` | ToF 推荐 0.1 m，按实测抖动微调 |
| `EKF2_RNG_A_HMAX` | `3.5` | 略小于传感器最大可靠量程（4 m 模块取 3.5） |

注意事项：

- `EKF2_RNG_CTRL` 必须非 0，否则 EKF2 **完全不用**测距；
- `EKF2_HGT_REF` 是 **reboot required**，改完要重启；
- 气压计（`EKF2_BARO_CTRL`）保持开启，不要只留测距：测距没有绝对基准，
  单靠它无法完成起飞前的高度估计；
- 测距量程内的地面（例如 0.3–3 m）最适合融合，贴地 <0.15 m 会进入盲区。

验证融合状态：

```sh
listener estimator_status
# control_mode_flags 的 bit 10（rng）应为 1
listener estimator_aid_source1d
# 查看 ranging 相关的 innovation / test ratio
```

---

## 4. 双实例（朝下定高 + 朝前测障）

第二路 TOF 接另一个串口，例如 SERIAL 4：

| 参数 | 值 |
|---|---|
| `SENS_TOFMS_CFG2` | `SERIAL 4`（=104） |
| `SENS_TOFMS_ROT2` | `0`（朝前） |
| `SENS_TOFMS_AGG2` | `1`（中位数，抗单点干扰） |
| `SER_TEL4_BAUD` | `921600` |

重启后 `tof_sense_ms status` 会打印两个实例，`listener distance_sensor` 会看到
两个 `instance`，`orientation` 分别为 25 和 0。两路实例的帧计数、错误计数、
超时计数完全独立。

> 朝前的 `distance_sensor` 默认不参与 EKF2 高度融合，可用于避障（`CP_DIST`
> 等）或后续机载电脑的避障逻辑。飞控端只负责稳定发布，不在这里耦合复杂
> 避障算法。

---

## 5. 排障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `SENS_TOFMS_CFG` 不存在 | 固件未打开驱动 / 参数未重新生成；或者当前连的是 SITL | 检查 `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y`，重新编译烧录。SITL 没有 `CONFIG_BOARD_SERIAL_*`，本来就不会生成 `SENS_TOFMS_CFG`，只能用 `tof_sense_ms start -d /dev/pts/N` |
| `frames` 不涨，`parser errors` 猛涨 | 波特率不对、TX/RX 接反、未共地 | 确认 `SER_xxx_BAUD=921600`；传感器 TX → 飞控 RX；共地 |
| `timeouts` 持续增长 | 模块没在主动上报 / 线断 | 用 NAssistant 确认模块为主动上报；检查线序 |
| `checksum errors` 少量增长 | 偶发干扰 | 一般无需处理；比例高则降波特率或换屏蔽线 |
| `no-valid-zone` 持续增长 | 贴太近（<0.15 m）、超量程、黑面/斜面反射弱 | 调整安装高度/角度，检查目标面 |
| `out-of-range` 持续增长 | 距离超出 `SENS_TOFMS_MIN/MAX` | 放宽量程或检查安装 |
| `listener distance_sensor` 没数据 | 驱动没启动 / 端口冲突 | `tof_sense_ms status`；确认该串口没被 MAVLink/UXRCE 占用 |
| EKF 不融合 | `EKF2_RNG_CTRL=0` 或测距被判定异常 | 设为 1/2；看 `estimator_status` 的 `CS_RNG_FAULT` |
| 两个实例只有一个起来 | 两个 `SENS_TOFMS_CFG` 选了同一个物理串口 | `rc.serial` 会打印 `Conflicting config`，改成不同串口 |

---

## 6. 验收勾选

- [ ] 固件包含 `tof_sense_ms`，QGC 参数里能看到 `SENS_TOFMS_CFG`
- [ ] `SENS_TOFMS_CFG` 选择实际串口，`SER_xxx_BAUD=921600`，重启
- [ ] `tof_sense_ms status`：帧率正常、错误计数 0、无持续 timeout
- [ ] `listener distance_sensor`：`current_distance` 正确、`orientation=25`
- [ ] `EKF2_RNG_CTRL` 非 0，`EKF2_HGT_REF` 按场景设置并重启
- [ ] QGC 中能看到高度/测距数据，遮挡时数值实时变化
- [ ] （可选）第二实例：`SENS_TOFMS_CFG2` 生效，两个实例统计独立
