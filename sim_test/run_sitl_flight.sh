#!/usr/bin/env bash
# Full SITL flight test: SIH quad + external TOF driver feeding distance_sensor.
set -u

BUILD=/home/juier/px4/PX4-Autopilot/build/px4_sitl_tof
FEEDER=/home/juier/sitl_tools/tof_feeder_flight.py
LOG=/tmp/px4_flight.log
FLOG=/tmp/tof_flight_feeder.log

rm -f /tmp/tof_pty "$LOG" "$FLOG"

echo "===== start TOF feeder ====="
python3 "$FEEDER" > "$FLOG" 2>&1 &
FEEDER_PID=$!
trap 'kill $FEEDER_PID 2>/dev/null' EXIT
for _ in $(seq 1 50); do [ -s /tmp/tof_pty ] && break; sleep 0.1; done
PTY=$(cat /tmp/tof_pty)
echo "pty=$PTY"

cd $BUILD || exit 1

echo "===== run SITL flight ====="
{
  echo "tof_sense_ms start -d $PTY -b 921600 -a 0"
  sleep 12
  echo "param set EKF2_RNG_CTRL 1"
  echo "listener distance_sensor -n 1"
  echo "commander status"
  sleep 1
  echo "commander arm"
  sleep 2
  echo "commander takeoff"
  sleep 3
  echo "=== T+3 ==="
  echo "listener vehicle_local_position -n 1"
  sleep 3
  echo "=== T+6 ==="
  echo "listener vehicle_local_position -n 1"
  sleep 3
  echo "=== T+9 ==="
  echo "listener vehicle_local_position -n 1"
  sleep 3
  echo "=== T+12 ==="
  echo "listener vehicle_local_position -n 1"
  echo "listener vehicle_attitude -n 1"
  echo "listener actuator_motors -n 1"
  sleep 4
  echo "=== T+16 (hover) ==="
  echo "listener vehicle_local_position -n 1"
  echo "tof_sense_ms status"
  sleep 4
  echo "=== T+20 (hover) ==="
  echo "listener vehicle_local_position -n 1"
  echo "listener vehicle_attitude -n 1"
  sleep 3
  echo "commander land"
  sleep 12
  echo "=== after landing ==="
  echo "listener vehicle_local_position -n 1"
  echo "tof_sense_ms status"
  echo "tof_sense_ms stop"
  sleep 1
} | PX4_SIM_MODEL=sihsim_quadx timeout 120 ./bin/px4 -s etc/init.d-posix/rcS > "$LOG" 2>&1

echo "px4 exit: $? (124 = timeout)"

echo "===== feeder log ====="
tail -12 "$FLOG"

echo "===== key output ====="
sed -e 's/\x1b\[[0-9;]*[A-Za-z]//g' -e 's/\r/\n/g' "$LOG" > /tmp/px4_flight_clean.log
grep -nE 'commander|Armed|Takeoff|landed|distance_sensor|tof_sense_ms|=== T|=== after' /tmp/px4_flight_clean.log | head -60
