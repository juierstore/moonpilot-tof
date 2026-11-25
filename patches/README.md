# PX4 补丁

两个补丁都是相对 PX4 `v1.18.0-beta1-248-gae5ebc8a80` 生成的，可直接
`git apply`。

| 补丁 | 内容 | 应用位置 |
|---|---|---|
| `tof_sense_ms.patch` | 新增 `src/drivers/distance_sensor/tof_sense_ms/`（9 个文件）+ `src/drivers/drv_sensor.h` 增加 `DRV_DIST_DEVTYPE_TOFMS 0x9D` | PX4 源码根目录 |
| `corvon_743v2_tof.patch` | `boards/corvon/743v2/default.px4board` 打开驱动；`init/rc.board_defaults` 预置 TOF + CM5/uXRCE-DDS 参数 | PX4 源码根目录 |

```bash
cd ~/px4/PX4-Autopilot

git apply --check /path/to/patches/tof_sense_ms.patch
git apply         /path/to/patches/tof_sense_ms.patch

git apply --check /path/to/patches/corvon_743v2_tof.patch
git apply         /path/to/patches/corvon_743v2_tof.patch
```

> 如果目标 PX4 版本的 `drv_sensor.h` 已经占用了 `0x9D`，或者
> `corvon_743v2` 的文件上下文已经变化，补丁可能不适用；此时用
> `tools/apply_to_px4.py`（幂等、按锚点插入）更稳妥。

应用后：

```bash
python3 tools/apply_to_px4.py --px4-root . --board corvon/743v2 --check
make corvon_743v2_default -j"$(nproc)"
```
