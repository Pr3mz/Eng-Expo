"""Slider window for robot geometry and tuning (press G in argos.py).

Changes apply live to the running program and are saved to
robot_settings.json about a second after the last slider move.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

import robot_settings

WINDOW = "ARGOS | robot setup"
SAVE_DELAY_S = 0.8


class SetupPanel:
    def __init__(self, values: dict[str, float]):
        self.values = values          # shared dict, edited in place
        self.is_open = False
        self._dirty_at: float | None = None

    def toggle(self) -> None:
        self.close() if self.is_open else self.open()

    def open(self) -> None:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 520, 1040)
        for field in robot_settings.FIELDS:
            position = int(round(self.values[field.key] * field.scale))
            cv2.createTrackbar(field.label, WINDOW, min(max(position, field.low), field.high),
                               field.high, lambda _value: None)
            cv2.setTrackbarMin(field.label, WINDOW, field.low)
        self.is_open = True
        self._draw()
        print("Robot setup open: drag sliders; values save automatically. Press G again to close.")

    def close(self) -> None:
        self._save_now()
        if self.is_open:
            try:
                cv2.destroyWindow(WINDOW)
            except cv2.error:
                pass
        self.is_open = False

    def poll(self) -> bool:
        """Read the sliders; return True when any value changed this frame."""
        if not self.is_open:
            return False
        try:
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                self.is_open = False
                self._save_now()
                return False
        except cv2.error:
            self.is_open = False
            return False

        changed = False
        for field in robot_settings.FIELDS:
            value = cv2.getTrackbarPos(field.label, WINDOW) / field.scale
            if value != self.values[field.key]:
                self.values[field.key] = value
                changed = True
        now = time.monotonic()
        if changed:
            self._dirty_at = now
            self._draw()
        elif self._dirty_at is not None and now - self._dirty_at >= SAVE_DELAY_S:
            self._save_now()
        return changed

    def _save_now(self) -> None:
        if self._dirty_at is not None:
            robot_settings.save(self.values)
            self._dirty_at = None
            print(f"Saved robot settings to {robot_settings.SETTINGS_PATH.name}")
            if self.is_open:
                self._draw()

    def _draw(self) -> None:
        scale = self.values["mm_per_pixel"] or None
        lines = [("ROBOT SETUP", (0, 255, 255))]
        for field in robot_settings.FIELDS:
            value = self.values[field.key]
            text = f"{field.label.replace(' x100', '')}: {value:g}"
            if field.key == "mm_per_pixel":
                text = f"mm per px: {value:.2f}" if scale else "mm per px: NOT SET (Auto blocked)"
            elif field.key.endswith("_px") and scale:
                text += f"  (= {value * scale:.0f} mm)"
            lines.append((text, (230, 230, 230)))
        lines += [
            ("", None),
            ("Green circle = gripper (length, size)", (0, 255, 0)),
            ("Orange circle = robot size, dot = rear", (0, 165, 255)),
            ("mm per px = arena width in mm / 800", (180, 180, 180)),
            ("Servo TEST angle moves the gripper live: find", (255, 200, 0)),
            ("the angle, then copy it into OPEN / CLOSE", (255, 200, 0)),
            ("Saved" if self._dirty_at is None else "Saving...", (0, 255, 0) if self._dirty_at is None else (0, 200, 255)),
        ]
        image = np.full((32 + 26 * len(lines), 520, 3), 30, dtype=np.uint8)
        for index, (text, color) in enumerate(lines):
            if text:
                cv2.putText(image, text, (14, 30 + 26 * index), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6 if index == 0 else 0.5, color, 2 if index == 0 else 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, image)
