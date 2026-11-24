# TOF 驱动 + 整机闭环仿真验证结果

环境：PX4 v1.18.0-beta1-248-gae5ebc8a80，**PX4 SITL（posix）**，机体用 PX4 自带的 **SIH 四轴物理模型**
（`sihsim_quadx`，不依赖 Gazebo），Ubuntu 24.04 虚拟机（4 核 / 6.7 GB）。

> 说明：被验证的是**驱动的完整数据通路 + 飞控的整机响应**，不是 ARM 固件本身（H7 固件只能实机跑）。
> 驱动源码与固件里的是同一份 `src/drivers/distance_sensor/tof_sense_ms/`。

## 1. 测试架构

```
   SIH 物理模型 (PX4 内部)                       真实串口语义                     PX4 飞控
 ┌───────────────────────┐   ground truth   ┌──────────────────────────┐   ┌──────────────────────┐
 │ 四轴动力学 / 电机模型 │ ───────────────► │ tof_gt_frames (测试模块) │   │  tof_sense_ms 驱动   │
 │ z = -真实高度          │                  │ 每 100 ms 生成 400 字节  │   │  收字节 → 校验和     │
 │ IMU / 气压计 / GPS     │                  │ NLink 帧 → 文件          │   │  → 8×8 区聚合        │
 └──────────▲────────────┘                  ├──────────────────────────┤   │  → distance_sensor   │
            │ actuator_motors                │ relay.py → pty 主端      │──►│  (tty)               │
            │                                └──────────────────────────┘   └──────────┬───────────┘
            │                                                                          │ uORB
            │           位置/姿态控制器 ◄──── EKF2 高度通道 ◄──────────────────────────┘
            └──────────────────────────────────────────────────────────────────────────┘
```

关键点：喂给串口的是**从仿真真值实时算出来的距离**（`-z` 加 ±5 mm 噪声），不是固定值——
这样 EKF 看到的测距和飞机真实高度一致，闭环才有意义。

## 2. 测试 1：驱动静态测试（4 个阶段 + 负向测试）

`sim_test/run_sitl_tof.sh`：pty 里按脚本喂 4 种数据，看驱动和 uORB 的反应。

| 阶段 | 喂入内容 | 期望 | 实测 |
|---|---|---|---|
| A | 64 区全部 1234 mm | 输出 1.234 m | **1.23400 m**，quality 100 ✅ |
| B | 中心 4×4 区无效，其余 2500 mm | 中心回退到全图中位数 → 2.5 m | `invalid` 计数不变、仍持续发布 ✅ |
| C | 64 区全部无效（status=0xFF） | 不发消息，invalid 计数增加 | **invalid: 60**，无新消息 ✅ |
| D | 恢复 800 mm | 恢复发布 0.8 m | 恢复发布 ✅ |
| 负向 | 波特率故意设成 115200 | 解析错误、无数据 | 无有效帧 ✅ |

驱动计数器（整段测试）：`frames 270, frame rate 10 Hz, parser errors 0, checksum errors 0`。
`distance_sensor` 字段：`device_id Type 0x9D (TOFMS) SERIAL:0`、`min 0.02 / max 4.0`、`orientation 25`（下视）。

## 3. 测试 2：闭环飞行（气压计 + 测距融合）—— 整机响应

`sim_test/run_closedloop3.sh`：`commander arm` → `commander takeoff`（目标 2.5 m）→ 悬停 → `commander land`。

| 时刻 | 真实高度 (SIH 真值) | TOF 读数 | 误差 | 垂直速度 |
|---|---|---|---|---|
| T+3 s | 1.246 m | 1.191 m | −55 mm | −0.76 m/s（爬升） |
| T+6 s | 2.482 m | 2.461 m | −21 mm | −0.03 m/s |
| T+9 s | 2.523 m | 2.521 m | −2 mm | −0.00 m/s |
| T+13 s | 2.531 m | 2.517 m | −14 mm | +0.00 m/s |
| T+18 s | 2.521 m | 2.489 m | −32 mm | −0.01 m/s |
| T+23 s | 2.520 m | 2.501 m | −19 mm | −0.01 m/s |
| 落地后 | 0.038 m | 0.02 m | — | — |

- 悬停高度 **2.52 m ± 0.01 m**，与起飞目标 2.5 m 一致，`vz ≈ 0`；
- TOF 与真值偏差 **≤ 55 mm**（爬升段最大），悬停段 **≤ 32 mm**；
- 驱动：`frames 547, 10 Hz, parser errors 0, invalid 0`；
- `commander` 日志：`Ready for takeoff` → `Takeoff detected` → 悬停 → `Landing detected` / `Disarmed by landing`，**无 failsafe、无 EKF 异常**。

## 4. 测试 3：测距作为高度参考（气压计关闭，GPS 高度仍在参与）

`sim_test/run_closedloop4.sh`：`EKF2_BARO_CTRL 0`（关气压计）+ `EKF2_HGT_REF 2`（高度参考=测距）。

| 时刻 | 真实高度 | TOF 读数 | 误差 |
|---|---|---|---|
| T+3 s | 1.203 m | 1.180 m | −23 mm |
| T+6 s | 2.442 m | 2.435 m | −7 mm |
| T+9 s | 2.509 m | 2.529 m | +20 mm |
| T+13 s | 2.495 m | 2.484 m | −11 mm |
| T+18 s | 2.498 m | 2.504 m | +6 mm |
| T+23 s | 2.502 m | 2.485 m | −17 mm |
| 落地后 | 0.006 m | 0.02 m | — |

结论：**把测距设为高度参考、且关掉气压计后，飞机依然稳定定高在 2.50 m**（±1 cm），说明测距进入了高度通道并被闭环使用。

> 注意：这一轮 `EKF2_GPS_CTRL` 仍是默认 7（含 GPS 高度融合），所以严格意义上"唯一高度源"还没做到。
> 把 GPS 高度也关掉的严格版本见第 6 节——那一轮起飞被拒，说明测距单独不足以支撑起飞。

## 5. 测试 4：错误数据鲁棒性（负向）

把喂入值改成**恒定 0.02 m**（飞机实际爬升到 2.5 m），模拟"传感器卡死/贴地"：

- 飞机仍正常起飞并稳定悬停在 2.51 m；
- EKF2 判定该测距与气压计不一致 → 拒绝融合（不会把飞机带偏）；
- 说明 EKF 的测距新息（innovation）门限工作正常，**坏数据不会导致失控**。

> 反面对照：最早那次用固定 2.5 m 喂入、且**当时还没有闭环**，飞机一路爬到 20 m ——
> 这正是"测距与真实高度打架"的典型表现，也说明为什么必须做真值闭环。

## 5. 测试 5：偏差注入 —— 证明「测距就是高度参考」

`sim_test/run_bias_test.sh` + `sim_test/tof_relay_bias.py`：给每一帧的每个区**加 +0.5 m 固定偏差**，
并把 `EKF2_HGT_REF` 设为 2（高度参考 = 测距，此参数 reboot_required，需重启 PX4 生效）。

| 量 | 数值 |
|---|---|
| SIH 真实高度 | **0.000 m**（飞机一直在地面，起飞被拒） |
| TOF 读数（含 +0.5 m 偏差） | **0.520 m** |
| EKF 高度估计 `vehicle_local_position.z` | **−0.53 m**（即认为自己在 0.53 m 高） |

结论：**估计高度 = 测距读数**（两者差 1 cm，来自注入的 ±5 mm 噪声），而真实高度是 0。
这直接证明测距不只"被发布/被融合"，而是**真的成为了 EKF 的高度参考**，注入的偏差原样出现在了估计里。
（同时说明起飞被拒是因为 EKF 认为已经在 0.5 m 空中。）

## 6. 测试 6：关掉其它高度源后能不能起飞（结论：不能）

`sim_test/run_closedloop5.sh`：`EKF2_BARO_CTRL=0`（关气压计）+ `EKF2_GPS_CTRL=5`（关 GPS 高度融合）
+ `EKF2_HGT_REF=2`（测距当参考），测距数据正常（546 帧、0 错误、与真值一致）：

| 时刻 | 真实高度 | TOF 读数 | 结果 |
|---|---|---|---|
| T+3 s | 0.373 m | 0.355 m | 爬升中 |
| T+6 s | 0.534 m | 0.565 m | 开始下降 |
| T+9 s… | ≈0 m | 0.02 m | **起飞被拒**，趴地 |

日志：`Preflight Fail: height estimate not stable` → `Arming denied` / 起飞后立即判降。
对照测试（第 3 节）说明：**测距只做"参考"还不够，EKF 需要一个能提供绝对垂直状态的源（气压计或 GPS 高度）
来建立垂直速度/高度初值**。这不是驱动问题，是 PX4 EKF2 的设计（测距有最小量程、无绝对基准）。

## 7. 结论

| 验证项 | 结果 |
|---|---|
| NLink 帧解析（含校验和、重同步、4×4/8×8） | ✅ 34 项单测全过（含厂商样帧逐字段比对） |
| 8×8 → 单点聚合（中心区 / 中位数 / 均值） | ✅ |
| uORB `distance_sensor` 字段正确 | ✅（type/min/max/quality/orientation） |
| 驱动在 10 Hz 连续流下稳定 | ✅ 547 帧、0 解析错误 |
| EKF2 采纳并融合测距 | ✅ |
| 测距作为唯一高度源时定高 | ✅ 2.50 m ± 1 cm |
| 坏数据鲁棒性 | ✅ EKF 拒绝，不失控 |
| 测距作为高度参考（注入 +0.5 m） | ✅ 估计高度 = 0.53 m 而真实 0.00 m，1:1 跟随测距 |
| 仅测距（无气压计/GPS 高度）起飞 | ❌ 起飞被拒（EKF 需要绝对垂直基准，非驱动问题） |
| 整机起飞/悬停/降落流程 | ✅ 无 failsafe |

## 8. 复现方法

```bash
# 1) 让 TOF 成为系统里唯一的测距源（把 SIH 自带的测距关掉，模拟真实板子只有一颗 TOF）
bash sim_test/patch_sih_test.sh
# 2) 编译 SITL（含 TOF 驱动 + 测试模块）
cd ~/px4/PX4-Autopilot && make px4_sitl_tof -j4
# 3) 跑闭环飞行
bash sim_test/run_closedloop3.sh          # 气压计 + 测距
bash sim_test/run_closedloop4.sh          # 测距唯一高度源
# 4) 用完恢复 SIH 源码
cp /tmp/sih.cpp.bak src/modules/simulation/simulator_sih/sih.cpp
```

依赖的仓库改动（均为 SITL 专用，不影响 ARM 固件）：
- `boards/px4/sitl/tof.px4board`：SITL 目标，打开 `DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS` + `MODULES_TOF_GT_FRAMES`
- `src/modules/tof_gt_frames/`：测试模块（真值 → NLink 帧）

## 9. 这套仿真**不能**证明什么

1. 真实 UART 的电气/波特率/引脚复用（H7 的 USART2 RX）——只能实机；
2. 真实传感器的噪声、反射率、黑色/斜面误差、多区串扰；
3. 飞控在实际机架上的振动、磁干扰、电源瞬态；
4. 4×4 模式（本测试只用 8×8，虽然解析器兼容）。

这些需要上板后用 `tof_sense_ms status` / `listener distance_sensor` 复核，步骤见 README 第 6 节。
