#!/usr/bin/env python3
"""Feed Nooploop TOFSense-M (NLink) frames into a virtual serial port.

Creates a pty pair, writes NLink frames at the requested rate and prints the
slave device path (also written to --pty-file) so that the PX4 driver can be
started on it, e.g. in SITL:

    python3 test/sitl/tof_feeder.py --pty-file /tmp/tof_pty0 --distance 1234 &
    build/px4_sitl_tof/bin/px4
    pxh> tof_sense_ms start -d /tmp/tof_pty0_or_slave_path -b 921600

Modes:
    normal          all zones valid, distance --distance
    invalid-center  center 4x4 zones invalid, other zones --distance
                    (driver should fall back to the median)
    invalid-all     every zone marked invalid (driver must not publish)
    point           single point frame (16 byte, unit mm)
"""

import argparse
import os
import pty
import time
import tty


def checksum(buf: bytes) -> int:
    return sum(buf) & 0xFF


def make_pixel_frame(distances_mm, statuses, frame_id: int, t_ms: int) -> bytes:
    body = bytearray()
    body += bytes([0x57, 0x01, 0xFF, frame_id & 0xFF])
    body += int(t_ms).to_bytes(4, "little")
    body += bytes([len(distances_mm)])

    for i, distance in enumerate(distances_mm):
        um = max(0, int(distance)) * 1000
        body += (um & 0xFFFFFF).to_bytes(3, "little")
        body.append(statuses[i])
        body += (100 + (i % 40)).to_bytes(2, "little")

    body += b"\xff" * 6                      # trailer seen on the M series
    body.append(checksum(body))
    return bytes(body)


def make_point_frame(distance_mm: int, status: int, frame_id: int, t_ms: int) -> bytes:
    body = bytearray()
    body += bytes([0x57, 0x00, 0xFF, frame_id & 0xFF])
    body += int(t_ms).to_bytes(4, "little")
    body += (int(distance_mm) & 0xFFFFFF).to_bytes(3, "little")
    body.append(status)
    body += (3).to_bytes(2, "little")
    body.append(0x06)
    body.append(checksum(body))
    return bytes(body)


def build_profile(mode: str, distance: int):
    zones = 64
    distances = [distance] * zones
    statuses = [0] * zones

    if mode == "invalid-center":
        for row in range(2, 6):
            for col in range(2, 6):
                statuses[row * 8 + col] = 0xFF
                distances[row * 8 + col] = 0

    elif mode == "invalid-all":
        statuses = [0xFF] * zones
        distances = [0] * zones

    return distances, statuses


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pty-file", default="/tmp/tof_pty", help="file that receives the pty slave path")
    parser.add_argument("--distance", type=int, default=1234, help="distance in mm")
    parser.add_argument("--rate", type=float, default=10.0, help="frame rate in Hz")
    parser.add_argument("--mode", default="normal",
                        choices=["normal", "invalid-center", "invalid-all", "point", "corrupt"])
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    args = parser.parse_args()

    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    tty.setraw(slave_fd)

    with open(args.pty_file, "w", encoding="utf-8") as handle:
        handle.write(slave_name)

    os.set_blocking(master_fd, False)
    print(f"TOF feeder: pty {slave_name}, {args.rate} Hz, mode {args.mode}", flush=True)

    distances, statuses = build_profile(args.mode, args.distance)
    start = time.monotonic()
    next_frame = start
    frame_id = 0
    sent = 0

    while True:
        now = time.monotonic()

        if args.duration > 0 and (now - start) >= args.duration:
            break

        t_ms = int((now - start) * 1000)

        if args.mode == "point":
            frame = make_point_frame(args.distance, 0, frame_id, t_ms)
        else:
            frame = make_pixel_frame(distances, statuses, frame_id, t_ms)

            if args.mode == "corrupt":
                # Break the checksum to exercise the parser error counter.
                frame = bytearray(frame)
                frame[100] ^= 0xFF
                frame = bytes(frame)

        frame_id += 1

        try:
            os.write(master_fd, frame)
            sent += 1

        except BlockingIOError:
            pass

        next_frame += 1.0 / args.rate
        delay = next_frame - time.monotonic()

        if delay > 0:
            time.sleep(delay)

    os.close(master_fd)
    os.close(slave_fd)
    print(f"TOF feeder: sent {sent} frames", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
