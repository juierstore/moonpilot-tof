# extras/moonpilot — 历史 MoonPilot 板级目标（可选）

这里保留 v1 仓库里的 MoonPilot（STM32H743VI, `boards/moonpilot/tof`）板级
文件和机架 `4002_moonpilot-juier`，仅作历史参考。

**新驱动不依赖这个目录。** `tof_sense_ms` 已经与板级解耦，任何在
`default.px4board` 里声明了 `CONFIG_BOARD_SERIAL_*` 的 PX4 板都能直接使用
`SENS_TOFMS_CFG` 在 QGC 里选串口。MoonPilot 板只有 GPS1（`/dev/ttyS3`）在标准
串口表里，UART2 RX（`/dev/ttyS1`）需要先声明为 `CONFIG_BOARD_SERIAL_TEL1`
才能在 QGC 下拉框里出现；否则只能用命令行临时启动，见
`board/rc.board_sensors` 里的说明。

目录内容：

```
board/                       boards/moonpilot/tof/ 的板级文件
airframes/4002_moonpilot-juier
moonpilot_px4_pinmap.md      引脚对照表
moonpilot_px4_pinmap.xlsx
```

使用方式（可选）：把 `board/` 拷成 `<PX4>/boards/moonpilot/tof/`，把机架文件
拷进 `<PX4>/ROMFS/px4fmu_common/init.d/airframes/` 并注册到该目录的
`CMakeLists.txt`。该板级目标未在 PX4 v1.18 上重新验证。
