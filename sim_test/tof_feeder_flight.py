#!/usr/bin/env python3
"""Simulated Nooploop TOFSense-MS for a scripted SITL takeoff/land profile.

Distance profile (seconds since start):
    0 - 10 : 0.02 m        (on the ground)
   10 - 18 : ramp to 2.50 m (takeoff)
   18 - 40 : 2.50 m +- 3 cm (hover)
   40 - 47 : ramp down to 0.02 m (landing)
    > 47   : 0.02 m
"""
import os
import pty
import sys
import time
import tty

RATE_HZ = 10
PTY_FILE = "/tmp/tof_pty"


def checksum(buf):
    return sum(buf) & 0xFF


def make_frame(distances_mm, frame_id=0, t_ms=0):
    body = bytearray()
    body += bytes([0x57, 0x01, 0xFF, frame_id & 0xFF])
    body += int(t_ms).to_bytes(4, "little")
    body += bytes([64])

    for i, d in enumerate(distances_mm):
        um = max(0, int(d)) * 1000
        body += (um & 0xFFFFFF).to_bytes(3, "little")
        body.append(0)
        body += (100 + (i % 40)).to_bytes(2, "little")

    body += b"\xff" * 6
    body.append(checksum(body))
    assert len(body) == 400
    return bytes(body)


def distance_mm(elapsed):
    if elapsed < 10:
        return 20

    if elapsed < 18:
        return 20 + (2500 - 20) * (elapsed - 10) / 8.0

    if elapsed < 40:
        # small noise so the estimator sees a real sensor
        noise = ((int(elapsed * 10) % 7) - 3) * 10
        return 2500 + noise

    if elapsed < 47:
        return 2500 - (2500 - 20) * (elapsed - 40) / 7.0

    return 20


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    tty.setraw(slave_fd)
    open(PTY_FILE, "w").write(slave_name)
    os.set_blocking(master_fd, False)

    print("TOF flight feeder: pty %s, %d Hz" % (slave_name, RATE_HZ), flush=True)

    start = time.monotonic()
    next_tick = start
    frame_id = 0

    while True:
        elapsed = time.monotonic() - start
        d = distance_mm(elapsed)
        frame = make_frame([int(d)] * 64, frame_id, int(elapsed * 1000))
        frame_id += 1

        try:
            os.write(master_fd, frame)
        except BlockingIOError:
            pass

        if frame_id % 50 == 0:
            print("  [%6.2f s] distance = %6.1f mm" % (elapsed, d), flush=True)

        next_tick += 1.0 / RATE_HZ
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
