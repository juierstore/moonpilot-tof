#!/usr/bin/env python3
"""Relay TOFSense frames produced by the PX4 test module into a pty.

The PX4 module `tof_gt_frames` writes 400 byte NLink frames (built from the SIH
ground truth altitude) into /tmp/tof_frames.bin. This script copies those bytes
verbatim into the master side of a pty, so the real tof_sense_ms driver reads
them from a proper tty.
"""
import os
import pty
import time
import tty

PTY_FILE = "/tmp/tof_pty"
FRAME_FILE = "/tmp/tof_frames.bin"


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    tty.setraw(slave_fd)
    open(PTY_FILE, "w").write(slave_name)
    os.set_blocking(master_fd, False)

    print("relay: pty %s <- %s" % (slave_name, FRAME_FILE), flush=True)

    pos = 0
    fed = 0
    waiting = False

    while True:
        try:
            size = os.path.getsize(FRAME_FILE)
        except OSError:
            size = 0

        if size < pos:                 # recreated file
            pos = 0

        if size > pos:
            with open(FRAME_FILE, "rb") as fh:
                fh.seek(pos)
                chunk = fh.read(size - pos)
                pos = fh.tell()

            if waiting:
                print("  frame stream started", flush=True)
                waiting = False

            # keep frame alignment: only forward whole 400 byte frames
            whole = len(chunk) - (len(chunk) % 400)
            if whole:
                data = chunk[:whole]
                try:
                    os.write(master_fd, data)
                    fed += len(data)
                except BlockingIOError:
                    pass
                pos -= (len(chunk) - whole)

        elif not waiting:
            waiting = True
            print("  waiting for frames from PX4 ...", flush=True)

        if fed and fed % 40000 < 400:
            print("  relayed %d frames" % (fed // 400), flush=True)

        time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
