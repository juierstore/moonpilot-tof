# moonpilot-tof — Nooploop TOFSense 串口测距驱动（PX4 v1.18+）

把 Nooploop **TOFSense / TOFSense-M / TOFSense-MS**（NLink 协议）测距模块接入
PX4，输出标准 `distance_sensor` uORB 话题，供 EKF2 做定高/地形融合，也可供
机载电脑（树莓派 CM5 / ROS 2）通过 uXRCE-DDS 订阅。

本仓库是 **PX4 源码覆盖层（overlay）**：`src/` 与 PX4 源码树同构，板级集成
只提供示例片段，驱动本身不包含任何飞控板硬编码。

```
QGC 选逻辑串口 (SENS_TOFMS_CFG) → 重启 → rc.serial 自动拉起驱动
   → NLink 解析/校验 → 8×8 区聚合 → distance_sensor → EKF2
```

---

## 1. 验收结果

| 项目 | 结果 |
|---|---|
| 协议解析层 | 纯 C++，无 PX4 依赖，可在 PC 上独立编译 |
| PC 端单元测试 | **77 checks, 0 failures**（含厂商 400 字节参考帧、单点帧、4×4/8×8、残帧重同步、校验和错误、缓冲越界） |
| 飞控固件编译 | `corvon_743v2_default` **编译通过**（驱动 + `UXRCE_DDS_CLIENT`） |
| Flash 占用 | 1,693,736 B / 1,792 KB = **92.30%**（image_maxsize 1,835,008 B） |
| RAM 占用 | AXI_SRAM 69,144 B / 512 KB = **13.19%** |
| QGC 串口选择 | 标准 `SENS_TOFMS_CFG` 下拉框（TELEM 1/2/3、SERIAL 4、GPS 1、UART 6 等） |
| 多实例 | 2 路（`SENS_TOFMS_CFG` / `SENS_TOFMS_CFG2`），状态/统计/参数隔离 |
| SITL 运行验证 | 双实例 1.234 m / 3.000 m 同时发布，`orientation` 25 / 0，10 Hz、0 错误；无效区、校验和错误、超时计数均验证通过，详见 [SITL_TESTS.md](docs/SITL_TESTS.md) |

> 硬件实测仍需上机确认（真实 UART 电气、传感器噪声、安装角度）。

---

## 2. 本次重构做了什么

| 旧方案（v1） | 新方案（本仓库） |
|---|---|
| `TOFMS_CFG` 自定义枚举 `0/1/2`，板级 `rc.board_sensors` 里 `if param compare` 硬编码 `/dev/ttyS1` / `/dev/ttyS3` | **标准 `serial_config`**：`module.yaml` 声明 `SENS_TOFMS_CFG`，PX4 自动生成参数与 `rc.serial` 启动项，QGC 直接选逻辑串口 |
| 单实例全局 `g_dev` | `g_dev[0..1]` 双实例，每实例独立解析器、统计、perf 计数、device id |
| 状态只有 frames / parser errors / invalid | 分类统计：接收帧、帧率、发布数、传感器无效区、超量程、帧头/长度错误、校验和错误、通信超时 |
| 绑定 `boards/moonpilot/tof` | 驱动零板级依赖，提供 `boards/corvon/743v2` 集成示例；MoonPilot 板级降级为可选扩展 |
| 解析器与驱动混在一起 | `tof_sense_ms_parser.{h,cpp}` 纯 C++，`test/` 下 77 项检查 |
| 无多实例 | `SENS_TOFMS_CFG2` + `SENS_TOFMS_ROT2`/`AGG2`，支持朝下定高 + 朝前测障 |

---

## 3. 仓库结构

```
src/drivers/distance_sensor/tof_sense_ms/   驱动模块（覆盖到 PX4 同名路径）
  module.yaml          serial_config + SENS_TOFMS_* 参数
  TOFSenseMS.{hpp,cpp} 驱动调度：串口 I/O、聚合、超时、统计、发布
  tof_sense_ms_main.cpp start/stop/status（支持 -i 实例、p:<param>）
  tof_sense_ms_parser.{h,cpp}  纯 C++ NLink 流式解析
  CMakeLists.txt Kconfig README.md
test/
  test_parser.cpp      解析器单测（厂商参考帧 + 异常帧）
  tof_sample_frame.h   厂商文档里的 400 字节原始帧
  Makefile             make test
  sitl/                SITL 虚拟串口喂帧 + 双实例冒烟测试
boards/
  corvon/743v2/        板级集成片段（default.px4board / rc.board_defaults）
  px4/sitl/tof.px4board  SITL 变体
tools/
  apply_to_px4.py      幂等安装：拷驱动 + 注册 devtype + 板级片段
  gen_sample_frame.py  从厂商页面重新抓取参考帧
docs/
  QGC_SETUP.md         QGC 配置 + EKF2 + 排障清单
  corvon_743v2.md      corvon 板集成（含 CM5 / uXRCE-DDS）
  PROTOCOL.md          NLink 协议与解析规则
  ARCHITECTURE.md      三层架构、多实例、性能、边界
  PORTING.md           移植到其它板 / 其它 PX4 版本
  SITL_TESTS.md        SITL 双实例运行结果
extras/moonpilot/      历史 MoonPilot 板级目标（可选，驱动不依赖）
patches/
  tof_sense_ms.patch   相对 PX4 v1.18 基线（ae5ebc8a80）的完整补丁
  corvon_743v2_tof.patch  板级启用补丁
```

---

## 4. 快速开始

### 4.1 安装到 PX4 源码树

```bash
git clone https://github.com/juierstore/moonpilot-tof.git
cd moonpilot-tof
python3 tools/apply_to_px4.py \
    --px4-root ~/px4/PX4-Autopilot \
    --board corvon/743v2
```

脚本会：

1. 复制 `src/drivers/distance_sensor/tof_sense_ms/`；
2. 在 `src/drivers/drv_sensor.h` 注册 `DRV_DIST_DEVTYPE_TOFMS 0x9D`；
3. 在 `boards/corvon/743v2/default.px4board` 打开驱动；
4. 在 `boards/corvon/743v2/init/rc.board_defaults` 追加 TOF 与 CM5 默认参数。

检查安装：

```bash
python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --board corvon/743v2 --check
```

也可以直接用补丁（相对 PX4 v1.18 基线生成，已 `git apply --check` 验证）：

```bash
cd ~/px4/PX4-Autopilot
git apply /path/to/moonpilot-tof/patches/tof_sense_ms.patch
git apply /path/to/moonpilot-tof/patches/corvon_743v2_tof.patch
```

### 4.2 编译

```bash
cd ~/px4/PX4-Autopilot
make corvon_743v2_default -j"$(nproc)"
```

产物：`build/corvon_743v2_default/corvon_743v2_default.px4`。

### 4.3 烧录 + QGC 配置

1. QGC → 固件 → 选择 `.px4` 文件；
2. QGC → Parameters → `SENS_TOFMS_CFG` 选择实际串口（示例板默认 `TELEM 3`）；
3. 确认所选端口 `SER_xxx_BAUD = 921600`；
4. 重启飞控；
5. MAVLink Console：

```sh
tof_sense_ms status
listener distance_sensor
```

完整步骤、EKF2 参数与排障见 **[docs/QGC_SETUP.md](docs/QGC_SETUP.md)**。

### 4.4 单元测试

```bash
cd test
make test
# 77 checks, 0 failures
```

---

## 5. 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `SENS_TOFMS_CFG` | Disabled | 实例 0 的逻辑串口（101 TELEM1 / 102 TELEM2 / 103 TELEM3 / 104 SERIAL4 / 201 GPS1 / 6 UART6） |
| `SENS_TOFMS_CFG2` | Disabled | 可选实例 1 |
| `SENS_TOFMS_ROT` / `ROT2` | 25 | 安装方向（25 朝下、0 朝前） |
| `SENS_TOFMS_AGG` / `AGG2` | 0 | 0 中心区均值 / 1 全图中位数 / 2 全图均值 |
| `SENS_TOFMS_MIN` / `MAX` | 0.15 / 4.0 m | 量程过滤，同时写入 `distance_sensor` |

EKF2：`EKF2_RNG_CTRL=1`（或 2）、`EKF2_HGT_REF=0`（或 2）、
`EKF2_RNG_NOISE=0.1`、`EKF2_RNG_A_HMAX=3.5`。

---

## 6. 文档

- **[QGC 配置与验证](docs/QGC_SETUP.md)** — 从参数下拉框到 EKF2 定高，含排障表
- **[corvon_743v2 集成](docs/corvon_743v2.md)** — 串口映射、板级默认值、CM5/uXRCE-DDS
- **[协议与解析](docs/PROTOCOL.md)** — 帧格式、校验和、单位、重同步
- **[架构](docs/ARCHITECTURE.md)** — 三层拆分、多实例、CPU/RAM、飞控/机载电脑边界
- **[移植说明](docs/PORTING.md)** — 换板、换 PX4 版本、只复用解析器
- **[SITL 测试](docs/SITL_TESTS.md)** — 双实例运行结果
- **驱动 README** — `src/drivers/distance_sensor/tof_sense_ms/README.md`

---

## 7. 已知限制

- 无实机数据：协议、编译、SITL 已通过，真实 UART 电气与传感器噪声需上机确认；
- 只接收主动上报，不实现查询模式（`57 10 FF ..`）与 NAssistant 配置；
- CAN 版本 TOFSense 不支持（协议不同）；
- 4×4 模式解析器兼容，但未在 4×4 实机上验证；
- 聚合是稳健统计（中心均值/中位数/均值），没有平面拟合或倾斜补偿；
- `signal_quality` 只是信号强度的粗略 0..100 映射。

---

## 8. 许可

BSD-3-Clause，与 PX4 一致。
