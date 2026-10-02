"""Headless check of manual mode: scripted gestures and key presses, fake robot.

Runs the real argos.py loop on a still photo and prints the wheel commands the
program sends for backward (gesture and keyboard), turning, stop, and the
gripper keys. No camera or robot needed.

    .venv/bin/python tools/smoke_manual.py
"""

from __future__ import annotations

import sys
import tempfile
import json
from pathlib import Path
from unittest import mock

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "host"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import argos  # noqa: E402
import smoke_auto as sa  # noqa: E402

FRAMES_PER_STEP = 1  # one camera frame per loop pass


class ScriptedGestures:
    """Stands in for GestureController: returns one scripted command per frame."""
    script: list[str] = []
    index = 0

    def __init__(self):
        self.debug_text = ""
        self.fist_stop = False
        self.min_hand_span = 0.0

    def process_frame(self, frame, grabbed):
        cmd = ScriptedGestures.script[min(ScriptedGestures.index, len(ScriptedGestures.script) - 1)]
        ScriptedGestures.index += 1
        self.fist_stop = cmd == "FIST"
        return ("S" if cmd == "FIST" else cmd), grabbed


def run(script, keys, label):
    ScriptedGestures.script, ScriptedGestures.index = script, 0
    sa.FakeLink.log = []
    sa.FakeLink.log.clear()
    tmp = Path(tempfile.mkdtemp())
    homography_ok = tmp / "calibration.json"
    homography_ok.write_text(json.dumps({"version": 1, "camera_index": 0, "warp_h": argos.WARP_H, "corners": sa.CORNERS,
                                         "zones": {}, "home_center": None}))
    keys = dict(keys)
    keys[2] = ord("m")                     # turn Manual on
    keys[len(script) + 12] = ord("q")
    state = {"calls": 0}

    def wait_key(_ms=1):
        import time
        time.sleep(0.03)                   # about 30 camera frames per second
        state["calls"] += 1
        return keys.get(state["calls"], 255)

    class SlowCap(sa.FakeCap):
        def read(self):
            import time
            time.sleep(0.02)
            return super().read()

    with mock.patch.object(argos, "RobotLink", sa.FakeLink), \
         mock.patch.object(argos, "open_camera", lambda *a, **k: SlowCap()), \
         mock.patch.object(argos, "CALIBRATION_PATH", homography_ok), \
         mock.patch.object(argos, "CAPTURE_DIR", tmp / "captures"), \
         mock.patch.object(argos, "GestureController", ScriptedGestures), \
         mock.patch.object(argos, "acquire_single_instance", lambda: True), \
         mock.patch.object(cv2, "imshow", lambda *a, **k: None), \
         mock.patch.object(cv2, "namedWindow", lambda *a, **k: None), \
         mock.patch.object(cv2, "setMouseCallback", lambda *a, **k: None), \
         mock.patch.object(cv2, "getWindowProperty", lambda *a, **k: 1), \
         mock.patch.object(cv2, "destroyAllWindows", lambda *a, **k: None), \
         mock.patch.object(cv2, "waitKey", wait_key), \
         mock.patch.object(sys, "argv", ["argos.py"]):
        argos.CFG["servo_test_deg"] = argos.CFG["servo_open_deg"]
        argos.main()
    log = sa.FakeLink.log
    moves = [c for c in log if c.startswith("drive ")]
    summary = {}
    for m in moves:
        summary[m] = summary.get(m, 0) + 1
    stops = sum(1 for c in log if c == "STOP")
    print(f"{label}: {summary or 'no drive commands'}  STOPs sent: {stops}  gripper: {[c for c in log if c.startswith('SERVO')][1:]}")
    return log


def main() -> int:
    b = "B"
    run([b] * 40, {}, "gesture BACKWARD held 40 frames   ")
    run([b] * 4 + ["S"] * 3 + [b] * 40, {}, "backward with a flicker mid-hold    ")
    run(["S"] * 30, {ord_i: ord("s") for ord_i in range(4, 30, 2)}, "keyboard S (backward) held       ")
    run(["L"] * 40, {}, "gesture TURN LEFT                  ")
    run(["F"] * 20 + ["FIST"] * 5 + ["S"] * 5, {}, "forward then fist (instant stop)   ")
    run(["S"] * 12, {4: ord("c"), 8: ord("o")}, "gripper keys c then o               ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
