#!/usr/bin/env python3
"""Closed-loop TOFSense-MS simulator for PX4 SITL.

Reads the true altitude from PX4 (SIH ground truth, dumped to /tmp/gt.txt by
`listener vehicle_local_position_groundtruth > /tmp/gt.txt &`) and streams
NLink frames whose distance is the real height above ground.

distance = clamp(-z_truth + noise, 0.02 ... 4.0) m
"""
import os
import pty
import re
import sys
import time
import tty

RATE_HZ = 10
PTY_FILE = "/tmp/tof_pty"
GT_FILE = "/tmp/gt.txt"

_re_z = re.compile(r"^\s+z:\s*(-?[\d.]+)", re.MULTILINE)


def checksum(buf):
    return sum(buf) & 0xFF


def make_frame(distances_mm, frame_id=0, t_ms=0):
    body = bytearray([0x57, 0x01, 0xFF, frame_id & 0xFF])
    body += int(t_ms).to_bytes(4, "little")
    body += bytes([64])

    for i, d in enumerate(distances_mm):
        body += (max(0, int(d)) * 1000 & 0xFFFFFF).to_bytes(3, "little")
        body.append(0)
        body += (100 + (i % 40)).to_bytes(2, "little")

    body += b"\xff" * 6
    body.append(checksum(body))
    assert len(body) == 400
    return bytes(body)


class GroundTruth:
    def __init__(self, path):
        self.path = path
        self.pos = 0
        self.z = 0.0
        self.messages = 0

    def update(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return

        if size < self.pos:          # file was truncated
            self.pos = 0
        if size == self.pos:
            return

        with open(self.path, "r", errors="replace") as fh:
            if size - self.pos > 200_000:      # fell too far behind: resync near EOF
                self.pos = size - 20_000
            fh.seek(self.pos)
            chunk = fh.read()
            self.pos = fh.tell()

        matches = _re_z.findall(chunk)
        if matches:
            self.z = float(matches[-1])
            self.messages += len(matches)


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    tty.setraw(slave_fd)
    open(PTY_FILE, "w").write(slave_name)
    os.set_blocking(master_fd, False)

    gt = GroundTruth(GT_FILE)
    print("closed loop TOF: pty %s, ground truth %s" % (slave_name, GT_FILE), flush=True)

    start = time.monotonic()
    tick = start
    frame_id = 0
    noise = [0, 10, -10, 5, -5, 0, 15, -15]

    while True:
        gt.update()
        elapsed = time.monotonic() - start

        alt_m = max(0.0, -gt.z)
        d_mm = alt_m * 1000.0 + noise[frame_id % len(noise)]
        d_mm = min(max(d_mm, 20.0), 4000.0)

        frame = make_frame([int(d_mm)] * 64, frame_id, int(elapsed * 1000))
        frame_id += 1

        try:
            os.write(master_fd, frame)
        except BlockingIOError:
            pass

        if frame_id % 50 == 0:
            print("  [%6.2f s] truth z=%7.2f m -> distance %6.1f mm (gt msgs %d)"
                  % (elapsed, gt.z, d_mm, gt.messages), flush=True)

        tick += 1.0 / RATE_HZ
        sleep = tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
