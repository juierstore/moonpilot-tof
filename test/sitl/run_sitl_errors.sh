#!/bin/bash
# Error-path smoke test for tof_sense_ms in PX4 SITL.
#
#   instance 0: all zones marked invalid -> no-valid-zone counter grows,
#               distance_sensor must not be published
#   instance 1: corrupted checksums      -> checksum error counter grows
#   both feeders stopped                 -> timeout counter grows after 1 s
#
# Usage:
#   ./test/sitl/run_sitl_errors.sh [path/to/build/px4_sitl_tof]

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
BUILD=${1:-"$HOME/px4/PX4-Autopilot/build/px4_sitl_tof"}
FEEDER="$HERE/tof_feeder.py"
LOG=${LOG:-/tmp/px4_sitl_tof_errors.log}
FLOG=${FLOG:-/tmp/tof_feeder_errors.log}
PTY0=/tmp/tof_err_pty0
PTY1=/tmp/tof_err_pty1

if [ ! -x "$BUILD/bin/px4" ]; then
	echo "error: PX4 SITL binary not found: $BUILD/bin/px4" >&2
	exit 1
fi

rm -f "$PTY0" "$PTY1" "$LOG" "$FLOG"

echo "===== start invalid-all and corrupt feeders ====="
python3 "$FEEDER" --pty-file "$PTY0" --mode invalid-all > "$FLOG" 2>&1 &
FEEDER0=$!
python3 "$FEEDER" --pty-file "$PTY1" --mode corrupt >> "$FLOG" 2>&1 &
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

echo "instance 0 pty: $DEV0 (all zones invalid)"
echo "instance 1 pty: $DEV1 (corrupted checksum)"
echo

cd "$BUILD" || exit 1

# Stop the feeders from a background shell while PX4 is running.
(
	sleep 9
	kill "$FEEDER0" "$FEEDER1" 2>/dev/null
) &

echo "===== run PX4 SITL + driver commands ====="
{
	echo "tof_sense_ms start -i 0 -d $DEV0 -b 921600 -R 25 -a 0"
	echo "tof_sense_ms start -i 1 -d $DEV1 -b 921600 -R 0 -a 1"
	sleep 4
	echo "tof_sense_ms status"
	sleep 5
	echo "tof_sense_ms status"
	echo "tof_sense_ms stop"
	sleep 1
} | timeout 45 ./bin/px4 > "$LOG" 2>&1

echo "px4 exit: $? (124 = expected timeout)"
echo
echo "===== relevant output ====="
grep -nE 'tof_sense_ms|timeouts|checksum|no-valid-zone|published|frames:' "$LOG" | tail -40

echo
echo "===== feeder output ====="
tail -5 "$FLOG"
