# MoonPilot TOF 固件（`moonpilot-tof`）— TOFSense 串口测距接入 PX4

飞机（机架）名称：**moonpilot-juier** ｜ 固件目标：**moonpilot_tof_default** ｜ 驱动模块：**tof_sense_ms**

**文档索引**：[驱动 README](drivers/tof_sense_ms/README.md) ｜ [移植说明](PORTING.md) ｜ [仿真结果](SIM_RESULTS.md) ｜ [引脚对照表](docs/moonpilot_px4_pinmap.md)

本目录是已在本机 PX4 仓库（v1.18.0-beta1-248-gae5ebc8a80）里编译通过的完整改动集，
固件产物已放进 `firmware/`，可审阅的代码补丁在 `patches/moonpilot_tof.patch`。

---

## 1. 结论先说

| 项 | 结果 |
|---|---|
| 器件协议 | Nooploop **NLink**：TOFSense-M/MS 为 **400 字节定长帧**，校验和 = 前 399 字节按字节相加取低 8 位 |
| PX4 原生支持 | **没有**（`src/drivers/distance_sensor/` 下无 nlink/tofsense 驱动），必须新写 |
| 本次交付 | 新驱动 `tof_sense_ms` + 新板级目标 `boards/moonpilot/tof` + 新机架 `4002_moonpilot-juier` |
| 编译结果 | **成功**，`moonpilot_tof_default.px4` 1.66 MB（上限 1.92 MB，占用 90%） |
| 离线单测 | **34 项检查 0 失败**，含厂商文档里那帧 400 字节原始数据的逐字段比对 |
| 实机验证 | **未做**（没有实机与传感器），需要你上机跑 `tof_sense_ms status` / `listener distance_sensor` 确认 |

---

## 2. 协议（已核对厂商文档 + 校验和验证）

来源：<https://support.nooploop.com/tofsense/protocol/>、<https://support.nooploop.com/tofsense/example-code/>
默认 **921600 8N1**，模块主动上报；小端。

### 2.1 TOFSense / -F / -S（单点，16 字节）

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 1 | `0x57` 帧头 |
| 1 | 1 | `0x00` 功能字（单点） |
| 2 | 1 | `0xFF` |
| 3 | 1 | id |
| 4 | 4 | system_time [ms] |
| 8 | 3 | 距离，**有符号 24 位，单位 mm**（可为负，如 -1044 = -1.044 m） |
| 11 | 1 | 距离状态（0 = 有效） |
| 12 | 2 | 信号强度 |
| 14 | 1 | 精度（预留） |
| 15 | 1 | 校验和 |

### 2.2 TOFSense-M / **MS**（面阵，8×8=64 区，400 字节）

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 1 | `0x57` |
| 1 | 1 | `0x01` 功能字（面阵） |
| 2 | 1 | `0xFF` |
| 3 | 1 | id |
| 4 | 4 | system_time [ms] |
| 8 | 1 | **zone_map**（像素数：64 = 8×8，16 = 4×4） |
| 9+6·i | 3 | 第 i 区距离，**无符号 24 位，单位 µm**（492000 → 492 mm） |
| 12+6·i | 1 | 第 i 区状态（0 = 有效，0xFF = 无效） |
| 13+6·i | 2 | 第 i 区信号强度 |
| … | 6 | 尾部 6 字节（8×8 模式下全是 `0xFF`） |
| 399 | 1 | 校验和 |

> 8×8 时长度：`9 + 64×6 + 6 + 1 = 400`。驱动对「有无这 6 字节尾部」两种排布都做了兼容
> （解析器在 394 / 400 两个长度上都验校验和），所以 4×4 模式（16 区）也能用。

**校验和**：`sum(frame[0 .. n-2]) & 0xFF == frame[n-1]`

**查询模式**（模块配置成非主动上报时）：`57 10 FF FF <id> FF FF <chk>`（8 字节）。

---

## 3. 接线（引脚按网表定义的 UART 口）

| 用途 | MCU 引脚 | 球位 | 板上位置 | NuttX 设备 |
|---|---|---|---|---|
| **TOF（默认）** | PD6 = UART2_RX | C6 | **U11 第 6 脚** | `/dev/ttyS1` |
| TOF（备选，全双工） | PB9/PB8 = UART4_TX/RX | A4/B4 | U11 第 3/4 脚 | `/dev/ttyS3` |
| RC 接收机 | PC6/PC7 = USART6 | F10/E10 | U15 | `/dev/ttyS4` |

TOFSense 只发不收，所以默认挂在 **UART2_RX（U11.6）**：与 GPS（UART4，U11.3/4）共用同一个 6 芯座，
不占用 GPS 串口，也不需要 TX。

> 改到 UART4：把 `init/rc.board_sensors` 里改成 `tof_sense_ms start -d /dev/ttyS3 -b 921600` 即可
> （此时 GPS1 要让位，`default.px4board` 里的 `CONFIG_BOARD_SERIAL_GPS1` 需改到别的口）。

---

## 4. 软件结构

```
drivers/tof_sense_ms/                 -> src/drivers/distance_sensor/tof_sense_ms/
  README.md                           驱动说明（协议/用法/聚合算法/EKF 配合/排障）
  tof_sense_ms_parser.{h,cpp}         纯 C++ 解析器（不依赖 PX4，可在 PC 上单测）
  TOFSenseMS.{hpp,cpp}                Serial + ScheduledWorkItem，聚合后发 distance_sensor
  tof_sense_ms_main.cpp               start/stop/status 命令行
  Kconfig / CMakeLists.txt            CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS

board/                                -> boards/moonpilot/tof/（从 boards/matek/h743 派生）
  default.px4board                    串口映射 + 打开 TOF 驱动
  board_config.h                      8 路 PWM / 6 路 ADC / LED 高电平点亮 / 无 VBUS 检测
  timer_config.cpp                    TIM3(CH3/4) + TIM5(CH1-4) + TIM4(CH1/2)
  spi.cpp                             SPI1/SPI4 两片 ICM-42688-P、SPI2 OSD、SPI3 外扩
  hw_config.h                         board type / 无 BOARD_VBUS
  rc.board_sensors                    启动 IMU/气压/磁力计 + tof_sense_ms
  rc.board_extras                     AT7456E OSD + ESC 遥测注释

airframes/4002_moonpilot-juier        -> ROMFS/px4fmu_common/init.d/airframes/
tools/apply_patch.py                  一键把上面所有改动落进 PX4 仓库（幂等）
tools/test_parser.cpp                 PC 端解析器单测（含厂商参考帧）
tools/gen_sample_frame.py             从厂商文档抓参考帧生成 tof_sample_frame.h
firmware/                             编译产物（.px4 / .bin）
patches/moonpilot_tof.patch           本次全部改动的 git diff
```

### 4.1 驱动行为

- 4 ms 轮询串口（`ScheduledWorkItem`），流式状态机解析，逐字节喂入，容错重同步；
- 每帧验校验和，校验失败计入 `checksum_err` 并丢弃该帧；
- **8×8 区 → 单点距离**（`-a` 可选，默认 0）：
  - `0` 中心区均值（8×8 取中间 4×4，4×4 取中间 2×2），中心全无效时退回全图中位数；
  - `1` 全图有效区中位数；`2` 全图有效区均值；
  - 只统计 `status == 0` 且落在 `[min,max]` 的距离；无有效区时不发消息（EKF 自然超时）；
- 发布 `distance_sensor`（type = LASER、orientation = 下视 25、min/max 可配、fov 8×8 按 63°）；
- 自动识别单点/面阵两种帧：单点帧会把 fov 切成 2.3°、min 切 0.3 m。

### 4.2 命令行

```
tof_sense_ms start [-d /dev/ttyS1] [-b 921600] [-a 0|1|2] [-R 25] [-m 0.02] [-M 4.0]
tof_sense_ms status
tof_sense_ms stop
```

`status` 会打印：端口/波特率/聚合模式、已解析帧数、帧率、解析错误数与校验错误数、最后帧的区数。

---

## 5. 构建 & 烧录

```bash
# 在虚拟机里
cd ~/px4/PX4-Autopilot
python3 ~/px4_patch/moonpilot_tof/tools/apply_patch.py     # 或直接 git apply patches/moonpilot_tof.patch
make moonpilot_tof_default -j4
# 产物：build/moonpilot_tof_default/moonpilot_tof_default.px4
```

烧录：
- QGC → 固件 → 选 `firmware/moonpilot_tof_default.px4`；
- 或 `make moonpilot_tof_default upload`（需要板子已跑 PX4 bootloader）。

机架选择：QGC 里选 **MoonPilot Juier**（SYS_AUTOSTART = 4002）。

> 板子是新目标，`make moonpilot_tof_bootloader` 也可以生成，但 **bootloader 未验证**：
> 本板没有 VBUS 检测脚，bootloader 的 USB 插拔判断需要另行处理。建议先沿用 MatekH743 的
> bootloader（`hw_config.h` 里 BOARD_TYPE 故意保持 139，就是为了兼容它）。

---

## 6. 上机自检步骤（建议顺序）

```bash
# 1) 看驱动有没有起来、收到多少帧
tof_sense_ms status
#    正常：frames 每秒增长、frame rate ≈ 传感器上报频率（默认 10 Hz）、checksum 0

# 2) 看 uORB 数据
listener distance_sensor
#    正常：current_distance 随遮挡物变化，orientation = 25（下视）

# 3) 确认 EKF 用上了
param show EKF2_RNG_CTRL      # 应为 1
#    QGC 里 "Distance Sensor" 面板应显示高度

# 4) 若没有数据
#    - 波特率：模块出厂 921600，若改过用 -b 115200 试
#    - 接线：U11.6 是 RX，传感器 TX → 板子 RX，共地
#    - 上行：tof_sense_ms status 的 parser errors 若很大 → 波特率/线序问题
```

室内定高（把测距当主高度源）：

```
param set EKF2_HGT_REF 2      # 2 = Range finder（默认是气压计）
param set EKF2_RNG_CTRL 1     # 机架脚本已默认设置
```

要 DShot（默认是 PWM 400 Hz）：

```
param set PWM_MAIN_TIM0 -3    # TIM3 组（M1/M2）：-3 = DShot600
param set PWM_MAIN_TIM1 -3    # TIM5 组（M3..M6）
param set PWM_MAIN_TIM2 -3    # TIM4 组（M7/M8）
```

---

## 7. 验证记录

### 7.1 解析器单测（PC 上编译运行，非 PX4 环境）

```
$ g++ -std=c++17 -O1 -Wall -Wextra -o test_parser test_parser.cpp ../drivers/tof_sense_ms/tof_sense_ms_parser.cpp -I.
$ ./test_parser
[1] reference 8x8 frame from the vendor documentation
[2] single point frame
[3] single point negative distance
[4] synthetic 8x8 frame
[5] 4x4 frame, no trailer
[6] 4x4 frame with trailer
[7] corrupted frame + resync
[8] partial frame
34 checks, 0 failures
```

其中 `[1]` 用的是厂商文档里那帧真实数据（400 字节），逐字段断言：
zone0 = **492 mm**、zone1 = **214 mm**、zone63 = **26 mm**、zone0 信号 = **159** —— 与文档给出的解析结果完全一致。

### 7.2 固件编译

```
$ make moonpilot_tof_default -j4
[1129/1129] Creating .../moonpilot_tof_default.px4

$ arm-none-eabi-size moonpilot_tof_default.elf
   text    data     bss     dec     hex filename
1730144    3728   54052 1787924  1b4814 moonpilot_tof_default.elf

.px4 清单：board_id=1099  description="Firmware for the MoonPilot flight controller (STM32H743VIH6 + TOFSense-MS)"
          image_size=1733872  image_maxsize=1966080
符号：    tof_sense_ms_main / TOFSenseMS::init() 均在 ELF 中
ROMFS：   etc/init.d/airframes/4002_moonpilot-juier（含 PWM_MAIN_FUNC1..4 = 101..104、EKF2_RNG_CTRL 1）
          etc/init.d/rc.board_sensors（含 tof_sense_ms start -d /dev/ttyS1 -b 921600 -a 0）
```

---

## 8. 已知限制 / 待你确认

1. **没有实机验证**：驱动逻辑、协议、板级配置目前到"编译通过 + 解析器单测通过 + SITL 闭环仿真通过"，
   第一上电仍请按第 6 节自检（真实 UART 电气、传感器噪声只能实机看）。仿真结果见 **[SIM_RESULTS.md](SIM_RESULTS.md)**：

   | 仿真项 | 结果 |
   |---|---|
   | 驱动 10 Hz 连续流 | 547 帧、0 解析错误 |
   | TOF 读数 vs 真值 | 悬停段误差 ≤ 32 mm |
   | 整机定高（气压计+测距） | 2.52 m ± 0.01 m，无 failsafe |
   | 测距当**唯一**高度源 | 2.50 m ± 1 cm |
   | 喂错误数据（卡死 0.02 m） | EKF 拒绝，飞机不受影响 |

2. **USB 无 VBUS 检测**：本板 VBUS 只喂 5V 轨，没有检测脚，所以 `board_read_VBUS_state()` 直接返回 0
   （"始终已连接"，与 Radiolink PIX6 / 3DR ctrl-n1 的做法一致）。副作用：不插 USB 时系统也认为 USB 在线。
3. **bootloader 未编译验证**：`BOARD_TYPE` 保留 MatekH743 的 139，可直接用 Matek 的 bootloader 刷本固件。
4. **TOF 4×4 模式未实测**：解析器兼容 106/112 两种长度，但手上只有 8×8 的厂商样帧。
5. **电流标定**：`BOARD_BATTERY*_A_PER_V` 暂填 Matek 参考值 40.0，需要按你的四合一电调实测改。
6. **PPM 不可用**（上一份引脚表已说明）：RC 走 USART6 串口（SBUS/CRSF）。

---

## 9. 改动清单（相对原仓库）

| 文件/目录 | 说明 |
|---|---|
| `src/drivers/distance_sensor/tof_sense_ms/` | 新驱动模块 `drivers__tof_sense_ms`（7 个文件，含 README.md 与 module.yaml） |
| `src/drivers/drv_sensor.h` | 追加 `DRV_DIST_DEVTYPE_TOFMS 0x9D` |
| `boards/moonpilot/tof/` | 新板级目标（由 matek/h743 派生，21 个文件） |
| `ROMFS/px4fmu_common/init.d/airframes/4002_moonpilot-juier` | 新机架 |
| `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` | 注册机架 |

`patches/moonpilot_tof.patch` 是上述全部改动的 diff（含新增文件），可直接 `git apply`。
