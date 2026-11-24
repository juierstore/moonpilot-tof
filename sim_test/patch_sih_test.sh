#!/usr/bin/env bash
# TEST ONLY: disable the SIH built-in rangefinder so the external TOF driver is the
# only distance_sensor in the system (emulates the real board, which has no SIH sensor).
set -e
R=/home/juier/px4/PX4-Autopilot
F=$R/src/modules/simulation/simulator_sih/sih.cpp

cp -f "$F" /tmp/sih.cpp.bak

python3 - "$F" <<'PY'
import sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
old = "\t_px4_rangefinder.update(hrt_absolute_time(), current_distance);"
new = ("\t// TEST ONLY (moonpilot): built-in SIH rangefinder disabled so that the\n"
       "\t// external tof_sense_ms driver is the only distance_sensor in the system.\n"
       "\t// _px4_rangefinder.update(hrt_absolute_time(), current_distance);")
if old not in text:
    raise SystemExit("anchor not found")
open(path, "w", encoding="utf-8").write(text.replace(old, new, 1))
print("patched", path)
PY

grep -n "TEST ONLY" "$F" | head -3
