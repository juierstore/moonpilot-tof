# 移植说明

分四种场景：

| 场景 | 目标 | 工作量 |
|---|---|---|
| **A** | 同一 PX4 版本，换一块飞控板 | 改 2 个板级配置 + 选串口 |
| **B** | 换 PX4 版本 | 改 3~5 处 API |
| **C** | 不移植 PX4，只用解析器 | 只拷 2 个文件 |
| **D** | 换物理接口（CAN / I2C / 查询模式 / 4×4） | 另写收包层 |

驱动目录：`src/drivers/distance_sensor/tof_sense_ms/`。

---

## 场景 A：换一块 PX4 板子（推荐路径）

### A.1 安装驱动

```bash
cd <本仓库>
python3 tools/apply_to_px4.py --px4-root <PX4>
```

如果不用脚本，手工拷贝：

```bash
cp -r src/drivers/distance_sensor/tof_sense_ms \
      <PX4>/src/drivers/distance_sensor/
```

再在 `<PX4>/src/drivers/drv_sensor.h` 里加一行（编号必须唯一）：

```c
#define DRV_DIST_DEVTYPE_TOFMS         0x9D
```

> `0x9D` 在 PX4 v1.18 未被占用；换版本时先查重：
> `grep -n 'DRV_DIST_DEVTYPE' src/drivers/drv_sensor.h | sort -t x -k2`。

### A.2 打开驱动

在目标板 `boards/<vendor>/<board>/default.px4board` 加：

```
CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y
```

### A.3 串口选择

本驱动使用 PX4 标准 `serial_config`，**不需要写板级启动脚本**。目标板必须在
`default.px4board` 里声明了要用的逻辑串口，例如：

```
CONFIG_BOARD_SERIAL_TEL3="/dev/ttyS4"
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS2"
```

编译后 QGC 的 `SENS_TOFMS_CFG` 下拉框会自动出现这些端口（`TELEM 3`、`GPS 1`、
`SERIAL 4` 等），选择后重启即可。

如果要给板子预置默认值，在 `boards/<vendor>/<board>/init/rc.board_defaults` 加：

```sh
param set-default SENS_TOFMS_CFG 103
param set-default SER_TEL3_BAUD  921600
param set-default SENS_TOFMS_ROT 25
param set-default SENS_TOFMS_MAX 4.0
```

`param set-default` 不会覆盖用户已经改过的参数。

如果板子只有非标准 UART（没有 `CONFIG_BOARD_SERIAL_*`），有两个选择：

1. 在板级 `default.px4board` 里把它声明成标准标签（推荐，能进 QGC 下拉框）；
2. 退化为 CLI 启动：`rc.board_sensors` 里 `tof_sense_ms start -d /dev/ttySx -b 921600`，
   但这样就没有 QGC 串口选择，属于旧方案。

### A.4 NuttX 注意事项

**① UART 必须在 NuttX 里使能**

```bash
grep -n 'STM32H7_UART\|STM32H7_USART' boards/<vendor>/<board>/nuttx-config/nsh/defconfig
```

没有就加，例如 `CONFIG_STM32H7_USART2=y`。

**② RX 缓冲区要按帧率和波特率算**

一帧 400 字节：

```
每秒字节数 = 400 × 上报频率
10 Hz  → 4 kB/s   → CONFIG_USARTx_RXBUFSIZE=2048 足够
100 Hz → 40 kB/s  → 建议 4096 以上
```

**③ NuttX 的 ttyS 编号**

console 固定占 `/dev/ttyS0`，其余按 UART 序号递增。务必用
`tof_sense_ms start -d /dev/ttySx` + `status` 实测确认。

**④ 921600 可行性**

NuttX 提供 `B921600`；长线/无屏蔽线误码率高时，可在 NAssistant 把模块降到
115200/460800，再把 `SER_xxx_BAUD` 同步改掉。

### A.5 编译与自检

```bash
make <vendor>_<board>_default -j"$(nproc)"
```

上板：

```sh
tof_sense_ms status
listener distance_sensor
```

判据：`frames` 增长、`frame rate ≈ 上报频率`、解析错误 0、`orientation=25`。

### A.6 检查表

- [ ] 驱动目录已拷贝
- [ ] `drv_sensor.h` 注册了 `DRV_DIST_DEVTYPE_TOFMS`，编号不冲突
- [ ] `default.px4board` 打开 `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS`
- [ ] 目标 UART 在 NuttX 里使能，RXBUFSIZE 够大
- [ ] `CONFIG_BOARD_SERIAL_*` 声明了目标逻辑串口
- [ ] `SENS_TOFMS_CFG` 在 QGC 下拉框里可见
- [ ] 所选端口 `SER_xxx_BAUD = 921600`
- [ ] 启动路径只有一处（用了 `serial_config` 就不要在 `rc.board_sensors` 再写死）
- [ ] `tof_sense_ms status` 错误计数为 0
- [ ] `EKF2_RNG_CTRL` 非 0

---

## 场景 B：换 PX4 版本

先做 30 秒体检：

```bash
cd <PX4>
grep -n -A3 'void update(' src/lib/drivers/rangefinder/PX4Rangefinder.hpp
grep -rn 'serial_port_to_wq' platforms/common/px4_work_queue/ | head -2
grep -n 'rsource' src/drivers/distance_sensor/Kconfig
grep -n 'B921600' platforms/nuttx/NuttX/nuttx/include/termios.h
ls src/lib/drivers/rangefinder/CMakeLists.txt
grep -n 'serial_config' src/drivers/distance_sensor/tfmini/module.yaml
```

对照修改：

| 差异点 | 老版本情况 | 改法 |
|---|---|---|
| `PX4Rangefinder::update()` | v1.13 之前是 `update(distance, timestamp)` | 调换参数顺序，`quality` 不支持就删 |
| `px4::serial_port_to_wq()` | v1.13 之前没有 | 改用 `px4::wq_configurations::hp_default` |
| Kconfig 自动 `rsource "*/Kconfig"` | 老版本没有 | 手动在 `src/drivers/distance_sensor/Kconfig` 加 `source` |
| `B921600` | 部分 NuttX 移植没有 | 改用 `B460800`/`B115200` |
| `serial_config` | v1.14 之前不支持 | 删除 `module.yaml` 的 `serial_config`，只能显式启动（失去 QGC 串口选择） |
| `px4_get_parameter_value()` | 老版本可能没有 | 用 `atoi`/`strtof`，同时把 `rc.serial` 命令里的 `p:` 去掉 |
| `MODULE_CONFIG` | 老版本不识别 | 删掉这两行，只丢模块元数据 |

改完先编 SITL（快、报错一样）：

```bash
make px4_sitl_default -j"$(nproc)"
cp boards/px4/sitl/tof.px4board <PX4>/boards/px4/sitl/   # 可选
make px4_sitl_tof -j"$(nproc)"
```

SITL 通过再交叉编译目标板。

---

## 场景 C：不移植 PX4，只复用协议解析器

`src/drivers/distance_sensor/tof_sense_ms/tof_sense_ms_parser.{h,cpp}` 只依赖
`<cstdint>`，可以直接拷进 Arduino / STM32 HAL / ESP-IDF：

```cpp
#include "tof_sense_ms_parser.h"

tof_sense_ms::Parser parser;
tof_sense_ms::Frame frame;

void on_uart_byte(uint8_t b)
{
	if (parser.push(b, frame)) {
		// frame.type: 0 = 单点, 1 = 面阵
		// frame.pixels[i].distance_mm / status / signal
	}
}
```

单位换算解析器已经做了（单点 mm、面阵 µm→mm），外面不要再乘除 1000。

自己写收包层只需：串口 921600 8N1、逐字节 `push()`、过滤 `status != 0` 后
按需求聚合。

---

## 场景 D：换接口

### D.1 CAN 版 TOFSense

CAN 帧格式不同：

- 标准帧 ID = `0x200 + id`；
- 8 字节数据：`dis(3B, mm) + status(1) + signal(2) + 保留(2)`，小端；
- 面阵型号连发 64 帧拼一帧图像；
- 默认 1 M 波特率。

PX4 里要走 DroneCAN/UAVCAN 节点，工作量大；简单用法是在飞控侧写一个 CAN
收包线程 + 复用同样的聚合和 `distance_sensor` 发布。

### D.2 UART 查询模式

模块配成被动上报时，发 8 字节查询帧：

```
57 10 FF FF <id> FF FF <checksum>
```

在 `Run()` 里定时 `write()` 这 8 字节即可；当前驱动只读。

### D.3 4×4 模式

解析器已兼容（`zone_map=16`，106/112 字节两种帧长），中心区聚合会自动取
中间 2×2。未在 4×4 实机验证，使用前先用 `listener distance_sensor` 确认。

---

## 常见编译/运行错误

| 报错/现象 | 原因 | 处理 |
|---|---|---|
| `DRV_DIST_DEVTYPE_TOFMS` 未定义 | 没改 `drv_sensor.h` | 重新跑 `apply_to_px4.py` |
| `undefined reference to ... TOFSenseMS` | CMakeLists 没被收录 | 确认 Kconfig/CMakeLists 在驱动目录，清 build 重编 |
| `SENS_TOFMS_CFG` 不在 QGC 里 | 驱动没打开 / 参数没重新生成 | 检查 `default.px4board`，删除 `build/<target>` 重编 |
| `rc.serial` 里没有 TOF 启动行 | 模块没编进固件 | 同上；检查 `generated_params/serial_params.c` |
| `parser errors` 一直涨 | 波特率错 / 接反 / 没共地 | 确认 `SER_xxx_BAUD=921600`，传感器 TX→飞控 RX |
| `Conflicting config for /dev/ttySx` | 两个驱动选了同一物理串口 | 改不同串口 |
| EKF 不用测距 | `EKF2_RNG_CTRL=0` | 设为 1 或 2 |
