#!/usr/bin/env bash
# Run the TOF driver inside PX4 SITL with a virtual serial port fed by tof_feeder.py
set -u

BUILD=/home/juier/px4/PX4-Autopilot/build/px4_sitl_tof
FEEDER=/home/juier/sitl_tools/tof_feeder.py
LOG=/tmp/px4_tof.log
FLOG=/tmp/tof_feeder.log

rm -f /tmp/tof_pty "$LOG" "$FLOG"

echo "===== 1. start virtual TOF on a pty ====="
python3 "$FEEDER" > "$FLOG" 2>&1 &
FEEDER_PID=$!
trap 'kill $FEEDER_PID 2>/dev/null' EXIT

for _ in $(seq 1 50); do
  [ -s /tmp/tof_pty ] && break
  sleep 0.1
done
PTY=$(cat /tmp/tof_pty)
echo "pty: $PTY"
head -2 "$FLOG"

echo
echo "===== 2. px4 binary ====="
ls -la $BUILD/bin/px4

cd $BUILD || exit 1

echo
echo "===== 3. run PX4 SITL + driver test ====="
{
  echo "tof_sense_ms start -d $PTY -b 921600 -a 0"
  sleep 6
  echo "tof_sense_ms status"
  echo "listener distance_sensor -n 3"
  sleep 8
  echo "=== phase B ==="
  echo "tof_sense_ms status"
  echo "listener distance_sensor -n 3"
  sleep 8
  echo "=== phase C (all zones invalid) ==="
  echo "tof_sense_ms status"
  sleep 6
  echo "=== phase D ==="
  echo "tof_sense_ms status"
  echo "listener distance_sensor -n 3"
  sleep 3
  echo "=== negative test: wrong baudrate ==="
  echo "tof_sense_ms stop"
  echo "tof_sense_ms start -d $PTY -b 115200 -a 0"
  sleep 4
  echo "tof_sense_ms status"
  echo "tof_sense_ms stop"
  sleep 1
} | timeout 90 ./bin/px4 > "$LOG" 2>&1

echo "px4 exit: $? (124 = timeout)"
echo
echo "===== 4. driver / listener output ====="
grep -nE 'tof_sense_ms|distance_sensor|phase|=== ' "$LOG" | head -80
