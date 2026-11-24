#!/usr/bin/env python3
"""Summarise the PX4 SITL TOF test log."""
import re
import sys

LOG = "/tmp/px4_tof.log"
text = open(LOG, encoding="utf-8", errors="replace").read()
text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text)   # strip ANSI (nsh echo)

# split into sections by the "=== ... ===" markers we echo
sections = []
current = "start"
for line in text.splitlines():
    m = re.search(r"=== (.+?) ===", line)
    if m:
        current = m.group(1)
        continue
    sections.append((current, line))

blocks = []
cur = None
for sec, line in sections:
    if "TOPIC: distance_sensor" in line:
        if cur:
            blocks.append(cur)
        cur = {"section": sec, "fields": {}}
        continue
    if cur is not None:
        m = re.match(r"\s+([a-z_0-9]+):\s*(-?[\d.]+)", line)
        if m:
            cur["fields"][m.group(1)] = m.group(2)
if cur:
    blocks.append(cur)

print("distance_sensor messages captured: %d\n" % len(blocks))
print("%-22s %10s %10s %8s %8s %10s %10s" %
      ("section", "distance[m]", "min", "max", "qual", "orient", "timestamp"))
for b in blocks:
    f = b["fields"]
    print("%-22s %10s %10s %8s %8s %10s %10s" % (
        b["section"][:22],
        f.get("current_distance", "-"),
        f.get("min_distance", "-"),
        f.get("max_distance", "-"),
        f.get("signal_quality", "-"),
        f.get("orientation", "-"),
        f.get("timestamp", "-")))

print("\n--- tof_sense_ms status blocks ---")
for line in text.splitlines():
    if re.search(r"\[tof_sense_ms\]|tof_sense_ms: ", line):
        print(line.strip())

print("\n--- ekf2 distance_sensor selection lines ---")
for line in text.splitlines():
    if "selected distance_sensor" in line or "distance_sensor" in line and "ekf2" in line:
        print(line.strip()[:160])

print("\n--- other publishers? (uORB advertise) ---")
for line in text.splitlines():
    if "advertised" in line:
        print(line.strip()[:160])
