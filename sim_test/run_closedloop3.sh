#!/usr/bin/env bash
# TRUE closed loop flight test:
#   SIH ground truth -> tof_gt_frames (PX4 test module) -> /tmp/tof_frames.bin
#   -> relay -> pty -> tof_sense_ms -> distance_sensor -> EKF2 -> controller -> SIH
set -u

BUILD=/home/juier/px4/PX4-Autopilot/build/px4_sitl_tof
TOOLS=/home/juier/sitl_tools
cd $BUILD || exit 1

rm -f /tmp/tof_pty /tmp/tof_frames.bin /tmp/px4_daemon.log /tmp/tof_relay.log

cleanup() {
  kill $RELAY_PID 2>/dev/null
  pkill -f 'bin/px4 -d' 2>/dev/null
  sleep 1
}
trap cleanup EXIT

echo "=== start PX4 daemon (SIH quad) ==="
PX4_SIM_MODEL=sihsim_quadx nohup ./bin/px4 -d -s etc/init.d-posix/rcS > /tmp/px4_daemon.log 2>&1 &
sleep 14

echo "=== start ground-truth frame generator ==="
./bin/px4-tof_gt_frames start -f /tmp/tof_frames.bin
sleep 2
echo "frame file: $(stat -c%s /tmp/tof_frames.bin 2>/dev/null || echo 0) bytes"

echo "=== start relay ==="
python3 $TOOLS/tof_relay.py > /tmp/tof_relay.log 2>&1 &
RELAY_PID=$!
for _ in $(seq 1 50); do [ -s /tmp/tof_pty ] && break; sleep 0.1; done
PTY=$(cat /tmp/tof_pty)
echo "pty=$PTY"

echo "=== start TOF driver ==="
./bin/px4-tof_sense_ms start -d "$PTY" -b 921600 -a 0
sleep 8
./bin/px4-battery_simulator stop 2>/dev/null || true
./bin/px4-param set COM_LOW_BAT_ACT 0
./bin/px4-param set EKF2_BARO_CTRL 1
./bin/px4-param set EKF2_HGT_REF 0
./bin/px4-param set EKF2_RNG_CTRL 1
./bin/px4-tof_gt_frames status
./bin/px4-listener distance_sensor -n 1 2>&1 | grep -E 'device_id|current_distance|min_distance|max_distance|signal_quality|orientation'

echo "=== arm + takeoff ==="
./bin/px4-commander arm 2>&1 | tail -1
sleep 2
./bin/px4-commander takeoff 2>&1 | tail -1

for t in 3 6 9 13 18 23; do
  sleep 5
  echo "--- T+~${t}s ---"
  ./bin/px4-listener vehicle_local_position -n 1 2>&1 | grep -E '^\s+(z|vz):' | head -2
  ./bin/px4-listener distance_sensor -n 1 2>&1 | grep -E 'current_distance'
done

echo "=== hover: attitude + status ==="
./bin/px4-listener vehicle_attitude -n 1 2>&1 | grep -E '^\s+(roll|pitch|yaw):' | head -3
./bin/px4-tof_sense_ms status 2>&1 | grep -E 'frames|invalid'

echo "=== land ==="
./bin/px4-commander land 2>&1 | tail -1
sleep 16
echo "--- after landing ---"
./bin/px4-listener vehicle_local_position -n 1 2>&1 | grep -E '^\s+(z|vz):' | head -2
./bin/px4-tof_gt_frames status
./bin/px4-tof_sense_ms status 2>&1 | grep -E 'frames|invalid'

echo "=== relay log tail ==="
tail -4 /tmp/tof_relay.log
