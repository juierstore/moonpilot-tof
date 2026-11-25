#!/bin/bash
# Runtime smoke test for tof_sense_ms in PX4 SITL.
#
# Two virtual sensors are connected to two driver instances:
#   instance 0: 1.234 m, orientation 25 (down)
#   instance 1: 3.000 m, orientation 0  (forward)
#
# Usage:
#   ./test/sitl/run_sitl_tof.sh [path/to/build/px4_sitl_tof]
#
# Requires a PX4 SITL build with the driver enabled:
#   cp boards/px4/sitl/tof.px4board <PX4>/boards/px4/sitl/
#   make px4_sitl_tof

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
BUILD=${1:-"$HOME/px4/PX4-Autopilot/build/px4_sitl_tof"}
FEEDER="$HERE/tof_feeder.py"
LOG=${LOG:-/tmp/px4_sitl_tof.log}
FLOG=${FLOG:-/tmp/tof_feeder_sitl.log}
PTY0=/tmp/tof_pty0
PTY1=/tmp/tof_pty1

if [ ! -x "$BUILD/bin/px4" ]; then
	echo "error: PX4 SITL binary not found: $BUILD/bin/px4" >&2
	exit 1
fi

rm -f "$PTY0" "$PTY1" "$LOG" "$FLOG"

echo "===== start two virtual TOF sensors ====="
python3 "$FEEDER" --pty-file "$PTY0" --distance 1234 --mode normal > "$FLOG" 2>&1 &
FEEDER0=$!
python3 "$FEEDER" --pty-file "$PTY1" --distance 3000 --mode normal >> "$FLOG" 2>&1 &
FEEDER1=$!

cleanup() {
	kill "$FEEDER0" "$FEEDER1" 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 1 50); do
	if [ -s "$PTY0" ] && [ -s "$PTY1" ]; then
		break
	fi
	sleep 0.1
done

DEV0=$(cat "$PTY0" 2>/dev/null)
DEV1=$(cat "$PTY1" 2>/dev/null)

if [ -z "$DEV0" ] || [ -z "$DEV1" ]; then
	echo "error: pty devices were not created" >&2
	exit 1
fi

echo "instance 0 pty: $DEV0 (1.234 m, orientation 25)"
echo "instance 1 pty: $DEV1 (3.000 m, orientation 0)"
echo

cd "$BUILD" || exit 1

echo "===== run PX4 SITL + driver commands ====="
{
	echo "tof_sense_ms start -i 0 -d $DEV0 -b 921600 -R 25 -a 0"
	echo "tof_sense_ms start -i 1 -d $DEV1 -b 921600 -R 0 -a 1"
	sleep 6
	echo "tof_sense_ms status"
	echo "listener distance_sensor -n 1"
	echo "listener distance_sensor -i 1 -n 3"
	echo "listener distance_sensor -i 2 -n 3"
	sleep 2
	echo "tof_sense_ms stop -i 0"
	sleep 1
	echo "tof_sense_ms status -i 1"
	echo "tof_sense_ms stop"
	sleep 1
} | timeout 60 ./bin/px4 > "$LOG" 2>&1

echo "px4 exit: $? (124 = expected timeout)"
echo
echo "===== driver output ====="
grep -nE 'tof_sense_ms|distance_sensor|current_distance|orientation' "$LOG" | head -80

echo
echo "===== feeder output ====="
tail -5 "$FLOG"
