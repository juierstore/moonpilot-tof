#!/usr/bin/env bash
# Check whether the PX4 posix daemon/client model works (client stdout goes to bash)
set -u
BUILD=/home/juier/px4/PX4-Autopilot/build/px4_sitl_tof
cd $BUILD || exit 1

rm -f /tmp/gt_test.txt /tmp/px4_daemon.log

echo "=== start daemon ==="
PX4_SIM_MODEL=sihsim_quadx nohup ./bin/px4 -d -s etc/init.d-posix/rcS > /tmp/px4_daemon.log 2>&1 &
sleep 12
pgrep -f 'bin/px4 -d' >/dev/null && echo "daemon running" || echo "daemon NOT running"

echo "=== client: single sample ==="
timeout 10 ./bin/px4 listener vehicle_local_position_groundtruth -n 1 2>&1 | head -20

echo "=== client: background redirect (5 s) ==="
timeout 5 ./bin/px4 listener vehicle_local_position_groundtruth > /tmp/gt_test.txt 2>&1 &
BG=$!
sleep 6
kill $BG 2>/dev/null
echo "gt_test.txt: $(wc -l < /tmp/gt_test.txt 2>/dev/null || echo missing) lines"
head -8 /tmp/gt_test.txt 2>/dev/null
echo "..."
tail -4 /tmp/gt_test.txt 2>/dev/null

echo "=== stop daemon ==="
timeout 10 ./bin/px4 shutdown 2>&1 | head -3
sleep 3
pgrep -f 'bin/px4 -d' >/dev/null && echo "still running" || echo "stopped"
