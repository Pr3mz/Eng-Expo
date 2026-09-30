"""Latest-frame camera reader with a freshness watchdog."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np

WARMUP_FRAME_COUNT = 5
MAX_STARTUP_FRAME_COUNT = 30


class LatestFrameCamera:
    def __init__(self, capture, *, startup_timeout_s: float = 8.0, frozen_after_frames: int = 90):
        self.capture = capture
        self.frozen_after_frames = frozen_after_frames
        self._lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._frame = None
        self._captured_at = 0.0
        self._last_small = None
        self._unchanged_frames = 0
        self.startup_frame_number = 0
        self._error = ""
        self._thread = threading.Thread(target=self._reader, name="ExpoRedCrObot-camera", daemon=True)
        self._thread.start()
        if not self._ready.wait(startup_timeout_s):
            self.release()
            raise RuntimeError("Camera opened but no usable frame arrived within the startup timeout")
        if self._frame is None:
            error = self._error or "camera did not provide a usable startup image"
            self.release()
            raise RuntimeError(error)

    def _reader(self):
        while not self._stop.is_set():
            try:
                with self._capture_lock:
                    ok, frame = self.capture.read()
            except cv2.error as exc:
                with self._lock:
                    self._error = str(exc)
                self._ready.set()
                time.sleep(0.05)
                continue
            if not ok or frame is None:
                with self._lock:
                    self._error = "camera read failed"
                time.sleep(0.03)
                continue

            self.startup_frame_number += 1
            if self.startup_frame_number < WARMUP_FRAME_COUNT:
                continue
            if float(frame.mean()) < 3.0 and float(frame.std()) < 3.0:
                if self.startup_frame_number >= MAX_STARTUP_FRAME_COUNT:
                    with self._lock:
                        self._error = f"camera is black at startup frame {self.startup_frame_number}; adjust exposure"
                        self._frame = frame.copy()
                        self._captured_at = time.monotonic()
                        self._ready.set()
                time.sleep(0.02)
                continue

            small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (32, 24), interpolation=cv2.INTER_AREA)
            with self._lock:
                if self._last_small is not None and np.array_equal(small, self._last_small):
                    self._unchanged_frames += 1
                else:
                    self._unchanged_frames = 0
                self._last_small = small
                self._frame = frame.copy()
                self._captured_at = time.monotonic()
                self._error = ""
                self._ready.set()

    def read_latest(self):
        with self._lock:
            frame = self._frame.copy() if self._frame is not None else None
            age = time.monotonic() - self._captured_at if self._captured_at else float("inf")
            frozen = self._unchanged_frames >= self.frozen_after_frames
            error = self._error
        return frame, age, frozen, error

    def set_exposure(self, value: float) -> tuple[bool, float]:
        """Change exposure between frame reads to avoid racing the capture thread."""
        with self._capture_lock:
            try:
                supported = bool(self.capture.set(cv2.CAP_PROP_EXPOSURE, float(value)))
                actual = float(self.capture.get(cv2.CAP_PROP_EXPOSURE))
            except cv2.error:
                return False, float("nan")
        return supported, actual

    def release(self):
        self._stop.set()
        self.capture.release()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
