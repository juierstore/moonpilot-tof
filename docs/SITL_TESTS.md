# SITL 运行验证

用 PX4 SITL + 虚拟串口（pty）对驱动做运行级验证，不依赖硬件。测试脚本在
`test/sitl/`。

环境：

- PX4 `v1.18.0-beta1-248-gae5ebc8a80`
- `px4_sitl_tof`（`boards/px4/sitl/tof.px4board`，打开
  `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y`）
- 喂帧脚本：`test/sitl/tof_feeder.py`，10 Hz，400 字节 8×8 帧
- 编译：`make px4_sitl_tof -j4`，无报错

---

## 1. 正常双实例

```bash
./test/sitl/run_sitl_tof.sh
```

实例 0：1.234 m，`orientation=25`（朝下），中心区均值；
实例 1：3.000 m，`orientation=0`（朝前），全图中位数。

`tof_sense_ms status`：

```
INFO  [tof_sense_ms] instance 0: /dev/pts/1 @ 921600 Bd, orientation 25, aggregation center, range 0.15..4.00 m
INFO  [tof_sense_ms]   frames: 50 total, 10 Hz, published 50, no-valid-zone 0, out-of-range 0
INFO  [tof_sense_ms]   parser: header/length errors 0, checksum errors 0
INFO  [tof_sense_ms]   link: timeouts 0, last frame 84 ms ago, last distance 1.234 m (type 1, 64 zones)
INFO  [tof_sense_ms] instance 1: /dev/pts/2 @ 921600 Bd, orientation 0, aggregation median, range 0.15..4.00 m
INFO  [tof_sense_ms]   frames: 50 total, 10 Hz, published 50, no-valid-zone 0, out-of-range 0
INFO  [tof_sense_ms]   parser: header/length errors 0, checksum errors 0
INFO  [tof_sense_ms]   link: timeouts 0, last frame 84 ms ago, last distance 3.000 m (type 1, 64 zones)
```

`listener distance_sensor -n 1`：

```
TOPIC: distance_sensor 3 instances

Instance 0:   current_distance: -0.00015  orientation: 25   <- PX4 SIH 自带的模拟测距
Instance 1:   current_distance:  1.23400  orientation: 25   <- tof_sense_ms 实例 0
Instance 2:   current_distance:  3.00000  orientation: 0    <- tof_sense_ms 实例 1
```

> SITL 的 SIH 模型自带一个 `distance_sensor`（实例 0），所以驱动的两个实例在
> SITL 里是 1/2；真机上没有 SIH 测距时驱动实例就是 0/1。按 `orientation` 和
> `device_id` 区分即可。

`listener distance_sensor -i 1 -n 3` / `-i 2 -n 3` 连续 3 帧数值稳定为
1.23400 / 3.00000，`orientation` 分别为 25 / 0。

停止实例 0 后，实例 1 继续运行（`frames: 80`），证明实例状态隔离。

---

## 2. 异常数据与超时

```bash
./test/sitl/run_sitl_errors.sh
```

实例 0 喂 **全部区 status=0xFF** 的帧；实例 1 喂 **校验和损坏** 的帧；
9 s 后停掉两个喂帧脚本。

第一次 `status`（异常数据阶段）：

```
INFO  [tof_sense_ms] instance 0: ...
INFO  [tof_sense_ms]   frames: 31 total, 10 Hz, published 0, no-valid-zone 31, out-of-range 0
INFO  [tof_sense_ms]   parser: header/length errors 0, checksum errors 0
INFO  [tof_sense_ms]   link: timeouts 0, last frame 80 ms ago, last distance 0.000 m (type 1, 64 zones)
INFO  [tof_sense_ms] instance 1: ...
INFO  [tof_sense_ms]   frames: 1 total, 0 Hz, published 1, no-valid-zone 0, out-of-range 0
INFO  [tof_sense_ms]   parser: header/length errors 1, checksum errors 30
INFO  [tof_sense_ms]   link: timeouts 1, last frame 176 ms ago, last distance 1.234 m (type 1, 64 zones)
```

第二次 `status`（约 8 s 后）：

```
instance 0: frames 81, published 0, no-valid-zone 81, checksum errors 0
instance 1: frames 2, published 2, checksum errors 79, header/length errors 1, timeouts 2
WARN  [tof_sense_ms] instance 1: no valid frame on /dev/pts/2 for 1004 ms (check baudrate and wiring)
```

结论：

- 传感器标记无效（`status != 0`）的帧 **不会发布**，只累加 `no-valid-zone`；
- 校验和错误计入 `checksum errors`，并触发解析器重同步；
- 超过 1 s 没有有效帧会累加 `timeouts` 并打印 WARN，同时 `parser.reset()`
  丢弃可能的半帧；
- 两路实例的计数器互不影响。

> 注意：校验和损坏的流里仍有极小概率出现「碰巧校验通过」的帧（NLink 校验
> 只有 8 位），本测试 80 帧里出现 2 帧。EKF2 的 `EKF2_RNG_CTRL=1`（Conditional）
> 会做一致性检查；如现场干扰严重，优先降低波特率/换屏蔽线。

---

## 3. 复现步骤

```bash
# 1. 安装 SITL 变体
python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --sitl

# 2. 编译
cd ~/px4/PX4-Autopilot
make px4_sitl_tof -j"$(nproc)"

# 3. 正常双实例
cd <repo>
./test/sitl/run_sitl_tof.sh

# 4. 异常帧 + 超时
./test/sitl/run_sitl_errors.sh
```

脚本默认使用 `~/px4/PX4-Autopilot/build/px4_sitl_tof`，也可以把 build 目录
作为第一个参数传入。
