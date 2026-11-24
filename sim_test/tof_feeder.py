#!/usr/bin/env python3
"""Feed Nooploop TOFSense-MS (NLink) frames into a virtual serial port.

Creates a pty pair, writes 400 byte 8x8 frames at 10 Hz and prints the slave
device path (also written to /tmp/tof_pty) so the PX4 driver can be started on it.

Distance profile (phase seconds):
    0 -  8 s : all zones 1234 mm          -> expect ~1.234 m
    8 - 16 s : center 4x4 invalid, others 2500 mm -> expect ~2.500 m (fallback)
   16 - 22 s : all zones invalid          -> expect no new distance_sensor
   22 - 30 s : all zones 800 mm           -> expect ~0.800 m
"""
import os
import pty
import select
import sys
import time
import tty

RATE_HZ = 10
PTY_FILE = "/tmp/tof_pty"


def checksum(buf):
    return sum(buf) & 0xFF


def make_frame(distances_mm, statuses=None, frame_id=0, t_ms=0):
    body = bytearray()
    body += bytes([0x57, 0x01, 0xFF, frame_id])
    body += int(t_ms).to_bytes(4, "little")
    body += bytes([len(distances_mm)])

    for i, d in enumerate(distances_mm):
        um = max(0, int(d)) * 1000
        body += (um & 0xFFFFFF).to_bytes(3, "little")
        body.append(0 if statuses is None else statuses[i])
        body += (100 + (i % 40)).to_bytes(2, "little")

    body += b"\xff" * 6                      # trailer seen on the M series
    body.append(checksum(body))

    assert len(body) == 400, len(body)
    return bytes(body)


def phase_profile(elapsed):
    """Return (name, distances, statuses)."""
    if elapsed < 8:
        return "A: all zones 1234 mm", [1234] * 64, [0] * 64

    if elapsed < 16:
        distances = [2500] * 64
        statuses = [0] * 64
        for row in range(2, 6):              # center 4x4 invalid
            for col in range(2, 6):
                idx = row * 8 + col
                distances[idx] = 0
                statuses[idx] = 0xFF
        return "B: center invalid, others 2500 mm", distances, statuses

    if elapsed < 22:
        return "C: all zones invalid", [0] * 64, [0xFF] * 64

    return "D: all zones 800 mm", [800] * 64, [0] * 64


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    tty.setraw(slave_fd)

    with open(PTY_FILE, "w") as fh:
        fh.write(slave_name)

    os.set_blocking(master_fd, False)
    print("TOF feeder: pty %s, %d Hz, 400 byte frames" % (slave_name, RATE_HZ), flush=True)

    start = time.monotonic()
    next_frame = start
    frame_id = 0
    last_phase = None
    sent = 0
    dropped = 0

    while True:
        now = time.monotonic()
        elapsed = now - start
        name, distances, statuses = phase_profile(elapsed)

        if name != last_phase:
            print("  [%6.2f s] phase -> %s" % (elapsed, name), flush=True)
            last_phase = name

        frame = make_frame(distances, statuses, frame_id & 0xFF, int(elapsed * 1000))
        frame_id += 1

        try:
            os.write(master_fd, frame)
            sent += 1

        except BlockingIOError:
            dropped += 1

        if sent % 50 == 0:
            print("  [%6.2f s] frames sent=%d dropped=%d" % (elapsed, sent, dropped), flush=True)

        next_frame += 1.0 / RATE_HZ
        sleep = next_frame - time.monotonic()

        if sleep > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
