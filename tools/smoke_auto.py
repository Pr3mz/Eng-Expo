"""Run the real argos.py loop on a still camera photo with a fake robot link.

Checks the whole auto path (detection -> planner -> pilot -> wheel commands)
without hardware: it prints the commands the program would send.

    .venv/bin/python tools/smoke_auto.py
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "host"))

import argos  # noqa: E402
import vision  # noqa: E402

FRAME = ROOT / "data" / "captures" / "raw" / "frame_1790803038.jpg"
CORNERS = [[6, 28], [512, 30], [514, 458], [11, 468]]
# Drop circles seen in that photo (raw pixels): red and green. Two sites for this test.
RAW_SITES = {"crimson": (465, 110), "lime": (250, 128)}


class FakeSock:
    def __init__(self, log): self.log = log
    def sendto(self, data, addr): self.log.append(data.decode())
    def close(self): pass


class FakeLink:
    log: list[str] = []

    def __init__(self, *a, **k):
        self.robot_ip = "10.0.0.2"; self.port = 4217; self.status = "fake robot"
        self.sock = FakeSock(FakeLink.log); self.is_online = True; self.last_command = ""
    def discover(self): pass
    def poll(self): pass
    def send(self, command, keepalive=False): FakeLink.log.append(command)
    def drive(self, command, speed=200): FakeLink.log.append(f"drive {command} {speed}")
    def drive_mix(self, forward, turn_left):
        from robot_link import RobotLink
        sent = []
        helper = RobotLink.__new__(RobotLink); helper.send = lambda c, keepalive=False: sent.append(c)
        helper.drive_mix(forward, turn_left)
        FakeLink.log.append(f"{sent[0]}   (fwd {forward:.0f}, turn {turn_left:+.0f})")
        return 0, 0
    def stop(self): FakeLink.log.append("STOP")
    def close(self): pass


class FakeCap:
    def __init__(self):
        self.frame = cv2.imread(str(FRAME)); self.rng = np.random.default_rng(0)
    def read(self):
        noise = self.rng.integers(-3, 4, self.frame.shape, dtype=np.int16)
        return True, np.clip(self.frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    def set(self, *a): return True
    def get(self, *a): return 0.0
    def release(self): pass
    def isOpened(self): return True


def main() -> int:
    tmp = Path(tempfile.mkdtemp())
    homography = vision.make_homography(CORNERS, argos.WARP_W, argos.WARP_H)
    zones = {}
    for color, (x, y) in RAW_SITES.items():
        wx, wy = cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float32), homography)[0, 0]
        zones[color] = [int(wx), int(wy)]
    calibration = tmp / "calibration.json"
    calibration.write_text(json.dumps({"version": 1, "camera_index": 0, "warp_h": argos.WARP_H,
                                       "corners": CORNERS, "zones": zones, "home_center": None}))
    keys = {40: ord(" "), 260: ord("q")}
    state = {"calls": 0}

    def wait_key(_ms=1):
        state["calls"] += 1
        return keys.get(state["calls"], 255)

    with mock.patch.object(argos, "RobotLink", FakeLink), \
         mock.patch.object(argos, "open_camera", lambda *a, **k: FakeCap()), \
         mock.patch.object(argos, "CALIBRATION_PATH", calibration), \
         mock.patch.object(argos, "CAPTURE_DIR", tmp / "captures"), \
         mock.patch.object(argos, "GestureController", None), \
         mock.patch.object(argos, "acquire_single_instance", lambda: True), \
         mock.patch.object(cv2, "imshow", lambda *a, **k: None), \
         mock.patch.object(cv2, "namedWindow", lambda *a, **k: None), \
         mock.patch.object(cv2, "setMouseCallback", lambda *a, **k: None), \
         mock.patch.object(cv2, "getWindowProperty", lambda *a, **k: 1), \
         mock.patch.object(cv2, "destroyAllWindows", lambda *a, **k: None), \
         mock.patch.object(cv2, "waitKey", wait_key), \
         mock.patch.object(sys, "argv", ["argos.py", "--sites", "2", "--no-gripper", "--mm-per-pixel", "2.5"]):
        result = argos.main()

    commands = [c for c in FakeLink.log if c.startswith("M ")]
    print(f"\nexit code {result}; {len(FakeLink.log)} messages, {len(commands)} wheel commands")
    print("first wheel commands:")
    for line in commands[:6]: print("  ", line)
    print("distinct message kinds:", sorted({c.split()[0] for c in FakeLink.log}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
