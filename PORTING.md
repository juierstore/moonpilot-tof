# TOF 驱动移植说明

本文档讲**怎么把这套东西搬到别处**，分四种场景：

| 场景 | 目标 | 主要工作量 |
|---|---|---|
| **A** | 同一个 PX4 版本，换一块板子 | 改 2 个配置 + 选串口 |
| **B** | 换一个 PX4 版本（老版本/更新版本） | 改 3~5 处 API |
| **C** | 不用 PX4（Arduino / STM32 HAL / 自研） | 只复用解析器 |
| **D** | 换物理接口（CAN / I2C / 查询模式 / 4×4 模式） | 另写收包层 |

板级目标 `moonpilot/tof` 本身的移植（从 `matek/h743` 派生）见第 5 节。

---

## 场景 A：移植到另一块 PX4 板子（推荐路径）

前提：目标板已经在 PX4 里有自己的 `boards/<vendor>/<board>/` 目标，且能编译通过。

### A.1 复制驱动目录

```bash
cp -r <本仓库>/drivers/tof_sense_ms \
      <PX4>/src/drivers/distance_sensor/tof_sense_ms
```

目录里 8 个文件都要（`README.md` 可留可删）：

```
Kconfig  CMakeLists.txt  module.yaml
tof_sense_ms_parser.h  tof_sense_ms_parser.cpp
TOFSenseMS.hpp  TOFSenseMS.cpp  tof_sense_ms_main.cpp
```

Kconfig 会被 `src/drivers/distance_sensor/Kconfig` 里的 `rsource "*/Kconfig"` 自动收进来，
`CMakeLists.txt` 会被模块扫描自动收进来，**不需要**改任何上层 CMakeLists。

### A.2 注册设备类型

`src/drivers/drv_sensor.h` 里加一行（编号必须唯一）：

```c
#define DRV_DIST_DEVTYPE_TOFMS         0x9D
```

查重/挑空号：

```bash
grep -n 'DRV_DIST_DEVTYPE' src/drivers/drv_sensor.h | sort -t x -k2
```

> 如果目标 PX4 版本已经用掉了 0x9D，随便换一个未被占用的值即可，驱动里只引用宏名。

### A.3 打开驱动

在 `boards/<vendor>/<board>/default.px4board` 里加：

```
CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y
```

### A.4 选串口（最关键的一步）

TOFSense **只发不收**，所以只需要一根 RX；但要注意三件事：

**① 目标 UART 必须已经在 NuttX 里使能**

```bash
grep -n 'STM32H7_UART\|STM32H7_USART' boards/<vendor>/<board>/nuttx-config/nsh/defconfig
```

没有的话加上（以 USART2 为例）：`CONFIG_STM32H7_USART2=y`。

**② RX 缓冲区要按帧率和波特率算**

面阵型号一帧 400 字节：

```
所需字节/秒 = 400 × 上报频率
例如 10 Hz  → 4 kB/s   → RXBUFSIZE 600 够
     100 Hz → 40 kB/s  → 建议 2048 以上，并缩短驱动轮询周期
```

`boards/<vendor>/<board>/nuttx-config/nsh/defconfig`：

```
CONFIG_USART2_RXBUFSIZE=2048
```

**③ 确认这个 `/dev/ttySx` 没被别人占用**

NuttX 的命名规则：**console 那个 UART 固定占 `/dev/ttyS0`**，其余按 UART 序号递增。
所以「USART1 当 console」的板子上，USART2 是 `ttyS1`、USART3 是 `ttyS2`、UART4 是 `ttyS3`……
换 console 会让整串编号平移，务必 `printf` 一遍确认：

```sh
tof_sense_ms start -d /dev/ttyS1 -b 921600   # 起不来就换 ttyS2/ttyS3 试
tof_sense_ms status                          # 看 frames 是否在涨
```

**④ 921600 能不能跑**

- NuttX 的 `termios.h` 提供 `B921600`（本仓库已确认：`#define B921600 0010007`）；
- 但高波特率对 UART 时钟分频误差敏感，长线 + 无屏蔽容易误码；
- 模块出厂默认 921600，**如果误码率高，用厂商 NAssistant 把模块降到 115200/460800**，
  驱动侧 `-b` 跟着改即可（115200 下 10 Hz 面阵只占 35% 带宽，完全够）。

### A.5 启动方式（三选一）

| 方式 | 放哪里 | 适合 | 注意 |
|---|---|---|---|
| **板级固定**（推荐） | `boards/<v>/<b>/init/rc.board_sensors` | 传感器焊在板上，端口固定 | 最简单，和 IMU/气压计一起起 |
| 机型脚本 | `ROMFS/.../airframes/<id>_<name>` | 同一块板不同机型接法不同 | 换机型才生效 |
| `serial_config` 参数 | 驱动 `module.yaml` | 让用户在 QGC 里选端口 | 需要额外写 yaml，且**别和上面两种重复启动** |

板级固定的写法（就是本仓库 MoonPilot 的做法）：

```sh
# boards/<vendor>/<board>/init/rc.board_sensors
tof_sense_ms start -d /dev/ttyS1 -b 921600 -a 0
```

想做成"用户可在 QGC 选端口"（PX4 会自动生成 `SENS_TOFMS_CFG` 参数和启动逻辑），
在驱动的 `module.yaml` 里加：

```yaml
module_name: Nooploop TOFSense Rangefinder
serial_config:
    - command: tof_sense_ms start -d ${SERIAL_DEV}
      port_config_param:
        name: SENS_TOFMS_CFG
        group: Sensors
```

⚠️ 用了 `serial_config` 就**不要**再在 `rc.board_sensors` 里写死，否则两个启动路径打架
（驱动第二次 `start` 会打印 "already started"，不致命但会误导）。

**第四种：自定义参数（本仓库 MoonPilot 板采用）**

当想要的端口不在标准列表里（例如只引出 RX 的 UART2），绕开 `serial_config`，
自己定义一个参数 + 在板级脚本里分支：

```yaml
# 驱动目录 module.yaml —— 只负责"参数存在且有可选项"
parameters:
    - group: Sensors
      definitions:
        TOFMS_CFG:
            description: {short: Serial port for the Nooploop TOFSense rangefinder}
            type: enum
            values: {0: Disabled, 1: UART2 RX (/dev/ttyS1), 2: UART4 (/dev/ttyS3)}
            default: 1
            reboot_required: true
```

```sh
# boards/<v>/<b>/init/rc.board_sensors —— 真正决定跑在哪个口
if param compare TOFMS_CFG 1
then
	tof_sense_ms start -d /dev/ttyS1 -b 921600 -a 0
fi
if param compare TOFMS_CFG 2
then
	tof_sense_ms start -d /dev/ttyS3 -b 921600 -a 0
fi
```

优点：端口可任意映射（不受 `CONFIG_BOARD_SERIAL_*` 限制）、QGC 里可见可改、**不动驱动代码**，
加端口只是再加一段 `if`。注意它同样是"开机读参数"，改完要重启。

### A.6 编译 + 上板自检

```bash
make <vendor>_<board>_default -j4
# 刷机后：
tof_sense_ms status          # frames 在涨、frame rate ≈ 上报频率、parser errors 0
listener distance_sensor     # current_distance 跟手变化，orientation = 25
```

### A.7 移植检查表

- [ ] 驱动目录拷过去（8 个文件）
- [ ] `drv_sensor.h` 加了 devtype，编号不冲突
- [ ] `default.px4board` 打开 `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS`
- [ ] 目标 UART 在 NuttX 里使能，AF 定义对（`GPIO_USARTn_RX`）
- [ ] RXBUFSIZE 按帧率算过
- [ ] ttyS 编号确认（console 占 ttyS0）
- [ ] 启动脚本只有一处
- [ ] `tof_sense_ms status` 里 `parser errors = 0`
- [ ] `EKF2_RNG_CTRL` 已设为 1（否则 EKF 不用）
- [ ] 气压计或 GPS 高度**没有被关掉**（见 README 第 7 节）

---

## 场景 B：移植到另一个 PX4 版本

驱动只用了 PX4 的公共设施，但不同版本的 API 有出入。**先做 30 秒体检**：

```bash
cd <PX4>
grep -n -A3 'void update(' src/lib/drivers/rangefinder/PX4Rangefinder.hpp   # ① update 签名
grep -rn 'serial_port_to_wq' platforms/common/px4_work_queue/ | head -2     # ② 工作队列选择
grep -c 'rsource' src/drivers/distance_sensor/Kconfig                      # ③ Kconfig 自动收录
grep -n 'B921600' platforms/nuttx/NuttX/nuttx/include/termios.h            # ④ 高波特率常量
ls src/lib/drivers/rangefinder/CMakeLists.txt                              # ⑤ 依赖库存在
```

对照下表改：

| # | 差异点 | 老版本情况 | 改法 |
|---|---|---|---|
| ① | `PX4Rangefinder::update()` | 旧版是 `update(distance, timestamp)`（约 v1.13 及以前） | 参数顺序调过来；`quality` 参数老版本没有就删掉 |
| ② | `px4::serial_port_to_wq()` | v1.13 前没有 | 构造时用 `ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)` |
| ③ | Kconfig 自动 `rsource` | 老版本要手动在 `Kconfig` 里加 `source "*/Kconfig"` | 手动加一行 |
| ④ | `B921600` | 部分 NuttX 移植没有 | 用 `B460800` 或 `B115200`，同时把模块波特率降下来 |
| ⑤ | `drivers_rangefinder` 库 | 老版本可能没有独立库 | 把 `PX4Rangefinder.cpp/.hpp` 一起拷进驱动目录，或从 `DEPENDS` 里去掉并在 CMake 里加源码 |
| ⑥ | `module.yaml` 的 `serial_config` | v1.14 前不支持 | 删掉 `serial_config`，只用显式启动 |
| ⑦ | `MODULE_CONFIG` | 老版本若不识别该参数 | 删掉 `MODULE_CONFIG module.yaml` 两行即可（只是丢掉模块元数据，不影响编译） |

改完先编 **SITL**（比交叉编译快得多、报错信息一样）：

```bash
make px4_sitl_default -j4
```

SITL 通过 = 代码层面没问题；再编目标板。

> 小技巧：SITL 里可以直接用第 3 节的方法接虚拟串口跑，不用等硬件。

---

## 场景 C：不移植 PX4，只复用协议解析器

`tof_sense_ms_parser.{h,cpp}` **只依赖 `<cstdint>`**，没有任何 PX4 头文件，
可以直接拷进 Arduino / STM32 HAL / ESP-IDF 工程。

```cpp
#include "tof_sense_ms_parser.h"

tof_sense_ms::Parser parser;
tof_sense_ms::Frame  frame;

// 串口每收到一个字节就喂进去
void on_uart_byte(uint8_t b)
{
    if (parser.push(b, frame)) {
        // frame.type: 0 = 单点系列, 1 = 面阵系列
        // frame.pixels[i].distance_mm / .status / .signal
        // 面阵取中心区（8x8 时行列 2..5）
    }
}
```

**单位坑（最容易错）**：

| 帧类型 | 原始单位 | 解析器输出 |
|---|---|---|
| 单点（`0x00`） | **mm，有符号 24 位** | `distance_mm`，可能为负（表示无效/超量程） |
| 面阵（`0x01`） | **µm，无符号 24 位** | 已换算成 `distance_mm`（内部 `/1000`） |

解析器已经按帧类型分别处理，**不要**在外面再乘/除 1000。

自己写收包层时要做的只有三件事：

1. 串口配置 **921600 8N1**（或模块改过的波特率）；
2. 每字节喂 `push()`；
3. `status != 0` 的区丢掉，再按需求聚合。

---

## 场景 D：换接口

### D.1 CAN 版 TOFSense

帧格式完全不同，不能直接复用现在的串口解析：

- 标准帧 ID = `0x200 + id`（如 id=1 → `0x201`）；
- 8 字节数据：`dis(3 字节, mm) + status(1) + signal(2) + 保留(2)`，小端；
- 面阵型号会连发 64 帧（8×8）拼成一帧图像；
- 波特率默认 1 M。

PX4 里要走 UAVCAN 需要写 DroneCAN 节点，工作量大；如果只是自己用，
建议在飞控上加一个 `can` 收包 + 同样的聚合逻辑，再发布 `distance_sensor`。

### D.2 UART 查询模式（模块不主动上报）

发 8 字节查询帧，模块回一帧：

```
57 10 FF FF <id> FF FF <checksum>
```

在驱动里 `Run()` 里定时 `write()` 这 8 字节即可（当前驱动只读，未实现发送）。

### D.3 4×4 模式（16 区）

解析器已兼容（`zone_map = 16`，帧长 106 或 112 都接受），
但**没有实测过**——如果要用，先用 `listener distance_sensor` 确认数值，
并把 `-a` 聚合的"中心区"逻辑验一遍（4×4 时中心是中间 2×2）。

---

## 5. 板级目标本身的移植（`boards/moonpilot/tof`）

如果连飞控硬件也换了，从 `boards/matek/h743` 派生最省事——两边都是 STM32H743VIH6 / TFBGA100，
球位分配几乎逐脚相同。

```bash
cp -r boards/matek/h743 boards/<vendor>/<board>
```

必改的 9 处（详细对照见 `docs/moonpilot_px4_pinmap.md`）：

| # | 文件 | 改什么 | 不改会怎样 |
|---|---|---|---|
| 1 | `default.px4board` | 串口映射、打开的驱动、`CONFIG_BOARD_SERIAL_*` | 串口功能错位 |
| 2 | `firmware.prototype` | `board_id`（全局唯一）、`summary`、`description` | QGC 认错固件 |
| 3 | `src/hw_config.h` | `BOARD_TYPE`、`APP_LOAD_ADDRESS`、flash 大小、**删 `BOARD_VBUS`**（无 VBUS 检测脚时） | bootloader 不认 / 链接报错 |
| 4 | `src/board_config.h` | PWM 数量、ADC 通道、LED 极性（`BOARD_LED_ON`）、`BOARD_NUM_IO_TIMERS` | 电机不转 / LED 反向 |
| 5 | `src/timer_config.cpp` | 定时器组和通道，与原理图对齐 | PWM 输出到错误引脚 |
| 6 | `src/spi.cpp` | 总线/片选/DRDY | IMU、OSD 找不到 |
| 7 | `src/led.c` | 若 LED 是高电平点亮，把 `!state` 的取反去掉 | LED 常亮/常灭 |
| 8 | `nuttx-config/{nsh,bootloader}/defconfig` | `CONFIG_ARCH_BOARD_CUSTOM_DIR` 改成新路径 | NuttX 找不到 board.h |
| 9 | `src/usb.c` | 无 VBUS 检测脚时补 `board_read_VBUS_state()` 返回 0 | 链接报 `undefined reference` |

**常见编译/链接错误对照**：

| 报错 | 原因 |
|---|---|
| `undefined reference to board_read_VBUS_state` | 删了 `GPIO_OTGFS_VBUS` 但没提供该函数（改法见上表第 9 条） |
| `undefined reference to px4_spi_buses` / `io_timers` | `spi.cpp` / `timer_config.cpp` 没加进 `src/CMakeLists.txt` |
| `region FLASH overflowed` | 关掉用不到的驱动（`CONFIG_MODULES_*`）或减板级功能 |
| `#error "Unknown CONFIG_ARCH_BOARD_*"` | `nuttx-config` 的 `CONFIG_ARCH_BOARD_CUSTOM_DIR` 路径还是旧的 |
| 编译过但 QGC 不显示机架 | 忘了把 airframe 文件名加进 `ROMFS/.../airframes/CMakeLists.txt` |

---

## 6. 移植完成后的回归验证

按成本从低到高，**不要跳步**：

1. **解析器单测**（PC，1 秒）

   ```bash
   g++ -std=c++17 -O2 -o test_parser test_parser.cpp tof_sense_ms_parser.cpp -I.
   ./test_parser          # 期望 34 checks, 0 failures
   ```

2. **SITL 静态测试**（虚拟串口喂 4 种数据：正常/中心遮挡/全无效/恢复）

   见本仓库 `sim_test/run_sitl_tof.sh`，判据：`frames` 增长、`parser errors 0`、
   `invalid` 只在全无效阶段增加、`distance_sensor` 数值正确。

3. **上板自检**

   ```sh
   tof_sense_ms status
   listener distance_sensor
   listener estimator_status      # control_mode_flags 的 bit10（测距融合）应为 1
   ```

4. **飞行前确认**（很多人栽在这）

   - `EKF2_RNG_CTRL = 1`；
   - 气压计或 GPS 高度**没有被关掉**（测距不能当唯一高度源，会拒绝起飞）；
   - `EKF2_HGT_REF` 想用测距当参考就设 `2`，但**这个参数 reboot_required，改完要重启**。

---

## 7. 相关文档

- 驱动用法与协议细节：`src/drivers/distance_sensor/tof_sense_ms/README.md`
- 仿真验证数据（6 组）：`SIM_RESULTS.md`
- 整机交付说明（固件/烧录/自检）：`README.md`
