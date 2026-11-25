#!/usr/bin/env python3
"""Install the Nooploop TOFSense driver overlay into a PX4-Autopilot tree.

The repository mirrors the PX4 source layout, so installing it is a copy of
``src/drivers/distance_sensor/tof_sense_ms`` plus a one line device type
registration. Optionally the corvon_743v2 board integration and the SITL
board variant are applied as well.

Examples
--------
    python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot
    python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --board corvon/743v2
    python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --sitl
    python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --check

The script is idempotent: running it twice does not duplicate any line.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DRIVER_REL = Path("src/drivers/distance_sensor/tof_sense_ms")
DEVTYPE_LINE = "#define DRV_DIST_DEVTYPE_TOFMS         0x9D"
DEVTYPE_ANCHOR = "DRV_DIST_DEVTYPE_GY_US42"


def fail(message: str) -> None:
	print(f"error: {message}", file=sys.stderr)
	sys.exit(1)


def read(path: Path) -> str:
	return path.read_text(encoding="utf-8")


def write(path: Path, text: str) -> None:
	path.parent.mkdir(parents=True, exist_ok=True)
	path.write_text(text, encoding="utf-8")


def install_driver(px4_root: Path) -> None:
	source = REPO_ROOT / DRIVER_REL
	if not source.is_dir():
		fail(f"driver overlay not found: {source}")

	target = px4_root / DRIVER_REL
	if target.exists():
		shutil.rmtree(target)

	shutil.copytree(source, target)
	print(f"driver: copied {DRIVER_REL} -> {px4_root}")


def register_device_type(px4_root: Path) -> None:
	path = px4_root / "src/drivers/drv_sensor.h"
	if not path.is_file():
		fail(f"not a PX4 tree, missing {path}")

	text = read(path)

	if "DRV_DIST_DEVTYPE_TOFMS" in text:
		print("drv_sensor.h: DRV_DIST_DEVTYPE_TOFMS already present")
		return

	lines = text.splitlines(keepends=True)
	for index, line in enumerate(lines):
		if DEVTYPE_ANCHOR in line:
			lines.insert(index + 1, DEVTYPE_LINE + "\n")
			write(path, "".join(lines))
			print("drv_sensor.h: added DRV_DIST_DEVTYPE_TOFMS 0x9D")
			return

	fail(f"could not find {DEVTYPE_ANCHOR} in {path}")


def append_config_lines(px4_root: Path, board: str) -> None:
	"""Add the CONFIG_ lines from boards/<board>/default.px4board.append."""
	snippet = REPO_ROOT / "boards" / board / "default.px4board.append"
	target = px4_root / "boards" / board / "default.px4board"

	if not snippet.is_file():
		fail(f"board snippet not found: {snippet}")

	if not target.is_file():
		fail(f"board not found in this PX4 tree: {target}")

	lines = [line.strip() for line in read(snippet).splitlines()]
	configs = [line for line in lines if line.startswith("CONFIG_")]

	current = read(target)
	added = []

	for config in configs:
		key = config.split("=", 1)[0]

		if f"{key}=" in current:
			continue

		added.append(config)

	if added:
		with target.open("a", encoding="utf-8") as handle:
			handle.write("\n# Nooploop TOFSense rangefinder\n")
			handle.write("\n".join(added) + "\n")

		print(f"{target.relative_to(px4_root)}: added {len(added)} config line(s)")
	else:
		print(f"{target.relative_to(px4_root)}: already integrated")


def append_rc_defaults(px4_root: Path, board: str) -> None:
	"""Append the block between the tof_sense_ms markers to rc.board_defaults."""
	snippet = REPO_ROOT / "boards" / board / "init" / "rc.board_defaults.append"
	target = px4_root / "boards" / board / "init" / "rc.board_defaults"

	if not snippet.is_file():
		fail(f"board rc snippet not found: {snippet}")

	lines = read(snippet).splitlines()
	try:
		start = lines.index("# >>> tof_sense_ms >>>")
		end = lines.index("# <<< tof_sense_ms <<<")
	except ValueError:
		fail(f"missing tof_sense_ms markers in {snippet}")

	block = "\n".join(lines[start:end + 1]) + "\n"

	if target.is_file():
		current = read(target)
	else:
		current = "#!/bin/sh\n# Board specific defaults.\n"

	if "# >>> tof_sense_ms >>>" in current:
		print(f"{target.relative_to(px4_root)}: already integrated")
		return

	if not current.endswith("\n"):
		current += "\n"

	write(target, current + "\n" + block)
	print(f"{target.relative_to(px4_root)}: added TOFSense / uXRCE-DDS defaults")


def install_sitl(px4_root: Path) -> None:
	source = REPO_ROOT / "boards" / "px4" / "sitl" / "tof.px4board"
	target = px4_root / "boards" / "px4" / "sitl" / "tof.px4board"

	if not source.is_file():
		fail(f"SITL board variant not found: {source}")

	if not target.parent.is_dir():
		fail(f"not a PX4 tree, missing {target.parent}")

	shutil.copyfile(source, target)
	print(f"boards/px4/sitl/tof.px4board: installed ({target})")


def check(px4_root: Path, board: str | None) -> int:
	problems = 0

	driver = px4_root / DRIVER_REL
	if not driver.is_dir():
		print(f"missing driver: {driver}")
		problems += 1
	else:
		print(f"driver: {driver}")

	sensor_header = px4_root / "src/drivers/drv_sensor.h"
	if not sensor_header.is_file():
		print(f"missing: {sensor_header}")
		problems += 1
	elif "DRV_DIST_DEVTYPE_TOFMS" in read(sensor_header):
		print("drv_sensor.h: DRV_DIST_DEVTYPE_TOFMS present")
	else:
		print("drv_sensor.h: DRV_DIST_DEVTYPE_TOFMS MISSING")
		problems += 1

	if board:
		board_config = px4_root / "boards" / board / "default.px4board"
		rc_defaults = px4_root / "boards" / board / "init" / "rc.board_defaults"

		if board_config.is_file() and "CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y" in read(board_config):
			print(f"{board}: driver enabled in default.px4board")
		else:
			print(f"{board}: driver NOT enabled in default.px4board")
			problems += 1

		if rc_defaults.is_file() and "# >>> tof_sense_ms >>>" in read(rc_defaults):
			print(f"{board}: TOFSense defaults present in rc.board_defaults")
		else:
			print(f"{board}: TOFSense defaults MISSING in rc.board_defaults")
			problems += 1

	return 1 if problems else 0


def main() -> int:
	parser = argparse.ArgumentParser(description=__doc__,
					 formatter_class=argparse.RawDescriptionHelpFormatter)
	parser.add_argument("--px4-root", type=Path, required=True,
			    help="path to the PX4-Autopilot source tree")
	parser.add_argument("--board", action="append", default=[],
			    help="board integration to apply, e.g. corvon/743v2 (can be repeated)")
	parser.add_argument("--sitl", action="store_true",
			    help="install the boards/px4/sitl/tof.px4board variant")
	parser.add_argument("--check", action="store_true",
			    help="only verify the installation, do not modify anything")
	args = parser.parse_args()

	px4_root = args.px4_root.expanduser().resolve()

	if not (px4_root / "src/drivers/drv_sensor.h").is_file():
		fail(f"{px4_root} does not look like a PX4-Autopilot tree")

	if args.check:
		board = args.board[0] if args.board else None
		return check(px4_root, board)

	install_driver(px4_root)
	register_device_type(px4_root)

	for board in args.board:
		append_config_lines(px4_root, board)
		append_rc_defaults(px4_root, board)

	if args.sitl:
		install_sitl(px4_root)

	print("\nDone. Build with:")
	print(f"  cd {px4_root}")
	print("  make corvon_743v2_default -j$(nproc)   # or your board target")

	return 0


if __name__ == "__main__":
	sys.exit(main())
