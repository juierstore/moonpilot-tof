#!/usr/bin/env python3
"""Summarise the SITL flight test log (altitude trace + sensor + attitude)."""
import re

LOG = "/tmp/px4_flight_clean.log"
text = open(LOG, encoding="utf-8", errors="replace").read()
lines = text.splitlines()

marker = "init"
samples = []          # (marker, z, vz, x, y)
att = []
motors = []
ds = []
cmds = []
tof_status = []

cur = None      # current topic block being parsed
for line in lines:
    m = re.search(r"=== (T\+\d+|after landing|.+?) ===", line)
    if m:
        marker = m.group(1)

    if line.strip().startswith("TOPIC: vehicle_local_position"):
        cur = "lp"
        continue
    if line.strip().startswith("TOPIC: vehicle_attitude"):
        cur = "att"
        continue
    if line.strip().startswith("TOPIC: actuator_motors"):
        cur = "mot"
        continue
    if line.strip().startswith("TOPIC: distance_sensor"):
        cur = "ds"
        continue
    if line.strip().startswith("TOPIC:"):
        cur = None
        continue

    if line.startswith("INFO  [commander]"):
        cmds.append(line.strip())
    if "INFO  [tof_sense_ms]" in line or line.startswith("tof_sense_ms:"):
        tof_status.append(line.strip())

    if cur == "lp":
        mm = re.match(r"\s+(z|vz|x|y|timestamp):\s*(-?[\d.]+)", line)
        if mm:
            cur_data = samples[-1] if samples and samples[-1][0] == marker and not samples[-1][1].get("_done") else None
            if cur_data is None or "z" in cur_data[1]:
                cur_data = (marker, {})
                samples.append(cur_data)
            cur_data[1][mm.group(1)] = float(mm.group(2))
    elif cur == "att" and re.match(r"\s+(roll|pitch|yaw|timestamp):", line):
        mm = re.match(r"\s+(roll|pitch|yaw|timestamp):\s*(-?[\d.]+)", line)
        if mm:
            if not att or att[-1][0] != marker:
                att.append((marker, {}))
            att[-1][1][mm.group(1)] = float(mm.group(2))
    elif cur == "mot":
        mm = re.match(r"\s+timestamp:\s*(-?[\d.]+)", line)
        if mm and (not motors or motors[-1][0] != marker):
            motors.append((marker, mm.group(1)))
    elif cur == "ds":
        mm = re.match(r"\s+(current_distance|min_distance|max_distance|signal_quality|orientation|device_id):\s*(-?[\d.]+)", line)
        if mm:
            if not ds or ds[-1][0] != marker:
                ds.append((marker, {}))
            ds[-1][1][mm.group(1)] = mm.group(2)

print("--- commander ---")
for c in cmds:
    print(" ", c)

print("\n--- altitude trace (vehicle_local_position, NED: z negative = up) ---")
print("%-14s %9s %9s %9s" % ("marker", "z [m]", "alt [m]", "vz [m/s]"))
for mk, d in samples:
    z = d.get("z")
    print("%-14s %9s %9s %9s" % (mk, z, ("%.2f" % -z) if z is not None else "-", d.get("vz")))

print("\n--- distance_sensor ---")
for mk, d in ds:
    print(" ", mk, d)

print("\n--- attitude (rad) ---")
for mk, d in att:
    print(" ", mk, {k: round(v, 3) for k, v in d.items() if k in ("roll", "pitch", "yaw")})

print("\n--- motor output messages ---")
print(" ", motors)

print("\n--- tof_sense_ms ---")
for s in tof_status:
    print(" ", s)
