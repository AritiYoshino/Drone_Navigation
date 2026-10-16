#!/usr/bin/env python3
"""Stop only processes carrying one headless drone-nav session's unique marker."""

import os
from pathlib import Path
import signal
import sys
import time


def members(token):
    marker = ("DRONE_SESSION_ID=" + token).encode()
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            if marker not in (entry / "environ").read_bytes().split(b"\0"):
                continue
            # Zombies have stopped; their parent or init will reap them.
            if (entry / "stat").read_text().rsplit(") ", 1)[1].split()[0] == "Z":
                continue
            found.append(int(entry.name))
        except (OSError, IndexError):
            continue
    return found


def stop(token, grace=8.0):
    for pid in members(token):
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + grace
    while members(token) and time.monotonic() < deadline:
        time.sleep(0.2)
    for pid in members(token):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 1.0
    while members(token) and time.monotonic() < deadline:
        time.sleep(0.1)
    remaining = members(token)
    if remaining:
        print("Session processes still stopping: {}".format(remaining), file=sys.stderr)
    return 1 if remaining else 0


if __name__ == "__main__":
    if len(sys.argv) != 2 or not sys.argv[1].startswith("drone-nav."):
        raise SystemExit("Usage: stop_session.py drone-nav.SESSION_ID")
    sys.exit(stop(sys.argv[1]))
