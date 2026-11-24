#!/usr/bin/env bash
# Find out which distance_sensor instances exist and which one carries our data
set -u
BUILD=/home/juier/px4/PX4-Autopilot/build/px4_sitl_tof
FEEDER=/home/juier/sitl_tools/tof_feeder.py
cd $BUILD || exit 1

rm -f /tmp/tof_pty
python3 "$FEEDER" > /tmp/tof_feeder.log 2>&1 &
FEEDER_PID=$!
trap 'kill $FEEDER_PID 2>/dev/null' EXIT

for _ in $(seq 1 50); do [ -s /tmp/tof_pty ] && break; sleep 0.1; done
PTY=$(cat /tmp/tof_pty)
echo "pty=$PTY"

{
  echo "tof_sense_ms start -d $PTY -b 921600 -a 0"
  sleep 4
  echo "listener distance_sensor -n 2"
  sleep 2
  echo "listener distance_sensor -i 1 -n 2"
  sleep 2
  echo "listener distance_sensor -i 2 -n 2"
  sleep 2
  echo "tof_sense_ms status"
  sleep 1
} | timeout 40 ./bin/px4 2>&1 | sed -e 's/\x1b\[[0-9;]*[A-Za-z]//g' -e 's/\r/\n/g' | grep -vE '^\s*$' > /tmp/inst_test.log

echo "=== instance 0 ==="
awk '/listener distance_sensor -n 2$/,/listener distance_sensor -i 1/' /tmp/inst_test.log | grep -E 'instance|current_distance|min_distance|max_distance|signal_quality|device_id' | head -20
echo "=== instance 1 ==="
awk '/listener distance_sensor -i 1/,/listener distance_sensor -i 2/' /tmp/inst_test.log | grep -E 'instance|current_distance|min_distance|max_distance|signal_quality|device_id' | head -20
echo "=== instance 2 ==="
awk '/listener distance_sensor -i 2/,/tof_sense_ms status/' /tmp/inst_test.log | grep -E 'instance|current_distance|device_id|ERROR' | head -10
