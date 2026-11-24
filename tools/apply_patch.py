#!/usr/bin/env python3
"""Apply the MoonPilot (moonpilot_tof) changes to a PX4-Autopilot checkout.

Run on the build machine:

    python3 apply_patch.py [--repo /path/to/PX4-Autopilot]

The script is idempotent: it recreates boards/moonpilot/tof from
boards/matek/h743, copies the staged files, patches the files that are
copied from the reference board and registers the new airframe and driver.
"""

import argparse
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STAGE = os.path.dirname(HERE)          # px4_moonpilot_tof/
BOARD = "moonpilot/tof"
DRIVER_DIR = "src/drivers/distance_sensor/tof_sense_ms"
AIRFRAME = "4002_moonpilot-juier"

changed = []


def note(msg):
    changed.append(msg)
    print("  " + msg)


def copy_file(src, dst, repo, note_msg):
    dst_abs = os.path.join(repo, dst)
    os.makedirs(os.path.dirname(dst_abs), exist_ok=True)
    shutil.copy2(src, dst_abs)
    note(note_msg)


def patch_text(path, old, new, count=1, label=None):
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()

    if new in text and old not in text:
        note("already patched: " + (label or path))

    if old not in text:
        raise SystemExit("PATCH FAILED (pattern not found): %s\n  %r" % (label or path, old[:120]))

    text = text.replace(old, new, count)

    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)

    note(label or os.path.relpath(path))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=os.path.expanduser("~/px4/PX4-Autopilot"))
    args = ap.parse_args()
    repo = os.path.abspath(args.repo)

    if not os.path.isfile(os.path.join(repo, "Makefile")):
        raise SystemExit("not a PX4 checkout: %s" % repo)

    board_dir = os.path.join(repo, "boards", BOARD)
    ref_dir = os.path.join(repo, "boards", "matek", "h743")

    print("== 1. create board target from boards/matek/h743")
    if os.path.isdir(board_dir):
        shutil.rmtree(board_dir)
    shutil.copytree(ref_dir, board_dir)
    shutil.rmtree(os.path.join(board_dir, "extras"), ignore_errors=True)
    note("boards/%s (copied from boards/matek/h743)" % BOARD)

    print("== 2. board files")
    copy_file(os.path.join(STAGE, "board", "default.px4board"),
              "boards/%s/default.px4board" % BOARD, repo, "default.px4board (serials, TOF driver)")
    copy_file(os.path.join(STAGE, "board", "firmware.prototype"),
              "boards/%s/firmware.prototype" % BOARD, repo, "firmware.prototype (board_id 1099)")
    copy_file(os.path.join(STAGE, "board", "hw_config.h"),
              "boards/%s/src/hw_config.h" % BOARD, repo, "src/hw_config.h (no BOARD_VBUS)")
    copy_file(os.path.join(STAGE, "board", "board_config.h"),
              "boards/%s/src/board_config.h" % BOARD, repo, "src/board_config.h (8 PWM, LED polarity, no VBUS)")
    copy_file(os.path.join(STAGE, "board", "timer_config.cpp"),
              "boards/%s/src/timer_config.cpp" % BOARD, repo, "src/timer_config.cpp (TIM3/TIM5/TIM4, 8 channels)")
    copy_file(os.path.join(STAGE, "board", "spi.cpp"),
              "boards/%s/src/spi.cpp" % BOARD, repo, "src/spi.cpp (ICM42688 x2, AT7456E, external SPI3)")
    copy_file(os.path.join(STAGE, "board", "rc.board_sensors"),
              "boards/%s/init/rc.board_sensors" % BOARD, repo, "init/rc.board_sensors (+ tof_sense_ms)")
    copy_file(os.path.join(STAGE, "board", "rc.board_extras"),
              "boards/%s/init/rc.board_extras" % BOARD, repo, "init/rc.board_extras")

    print("== 3. nuttx configs")
    for rel in ("nuttx-config/nsh/defconfig", "nuttx-config/bootloader/defconfig"):
        path = os.path.join(board_dir, rel)
        patch_text(path,
                   '../../../../boards/matek/h743/nuttx-config',
                   '../../../../boards/moonpilot/tof/nuttx-config',
                   label="%s (CONFIG_ARCH_BOARD_CUSTOM_DIR)" % rel)

    usart2 = os.path.join(board_dir, "nuttx-config", "nsh", "defconfig")
    with open(usart2, "r", encoding="utf-8") as fh:
        text = fh.read()

    if "CONFIG_USART2_RXBUFSIZE=600" in text:
        patch_text(usart2, "CONFIG_USART2_RXBUFSIZE=600", "CONFIG_USART2_RXBUFSIZE=2048",
                   label="CONFIG_USART2_RXBUFSIZE=2048 (TOF stream on ttyS1)")

    print("== 4. led.c: active high LEDs")
    led = os.path.join(board_dir, "src", "led.c")
    patch_text(led, "stm32_gpiowrite(g_ledmap[led], !state);", "stm32_gpiowrite(g_ledmap[led], state);",
               label="led.c (drive high to switch on)")
    patch_text(led, "return !stm32_gpioread(g_ledmap[led]);", "return stm32_gpioread(g_ledmap[led]);",
               label="led.c (read high = on)")

    print("== 5. driver: %s" % DRIVER_DIR)
    drv_dst = os.path.join(repo, DRIVER_DIR)
    if os.path.isdir(drv_dst):
        shutil.rmtree(drv_dst)
    shutil.copytree(os.path.join(STAGE, "drivers", "tof_sense_ms"), drv_dst)
    note(DRIVER_DIR + " (new module drivers__tof_sense_ms)")

    print("== 6. device type")
    drv_sensor = os.path.join(repo, "src", "drivers", "drv_sensor.h")
    with open(drv_sensor, "r", encoding="utf-8") as fh:
        text = fh.read()

    if "DRV_DIST_DEVTYPE_TOFMS" not in text:
        anchor = "#define DRV_DIST_DEVTYPE_GY_US42       0x9C"
        if anchor not in text:
            raise SystemExit("PATCH FAILED: devtype anchor not found in src/drivers/drv_sensor.h")
        patch_text(drv_sensor, anchor,
                   anchor + "\n#define DRV_DIST_DEVTYPE_TOFMS         0x9D",
                   label="src/drivers/drv_sensor.h (+DRV_DIST_DEVTYPE_TOFMS)")

    print("== 7. usb.c: VBUS state stub")
    usb_c = os.path.join(board_dir, "src", "usb.c")
    with open(usb_c, "r", encoding="utf-8") as fh:
        text = fh.read()

    if "board_read_VBUS_state" not in text:
        with open(usb_c, "a", encoding="utf-8") as fh:
            fh.write("""
/****************************************************************************
 * Name: board_read_VBUS_state
 *
 * Description:
 *   This board has no VBUS sense pin - VBUS is only used to feed the 5V rail,
 *   so the OTG core cannot detect the cable. Report "connected" so that the
 *   USB console / MAVLink come up as soon as the cable is plugged in.
 *
 ****************************************************************************/

__EXPORT int board_read_VBUS_state(void)
{
	return 0;
}
""")
        note("src/usb.c (+board_read_VBUS_state, no VBUS pin)")

    print("== 8. airframe %s" % AIRFRAME)
    copy_file(os.path.join(STAGE, "airframes", AIRFRAME),
              "ROMFS/px4fmu_common/init.d/airframes/" + AIRFRAME, repo,
              "ROMFS airframe " + AIRFRAME)

    romfs_list = os.path.join(repo, "ROMFS", "px4fmu_common", "init.d", "airframes", "CMakeLists.txt")
    with open(romfs_list, "r", encoding="utf-8") as fh:
        text = fh.read()

    if AIRFRAME not in text:
        patch_text(romfs_list, "\t4001_quad_x\n",
                   "\t4001_quad_x\n\t%s\n" % AIRFRAME,
                   label="airframes/CMakeLists.txt (+%s)" % AIRFRAME)

    print("\n== done, %d changes" % len(changed))


if __name__ == "__main__":
    sys.exit(main())
