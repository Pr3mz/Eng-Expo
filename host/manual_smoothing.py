"""Debounce for gesture drive commands.

A hand-tracking frame is sometimes wrong for a single frame (a finger lost,
the hand briefly out of view). Acting on every frame makes the robot stutter,
so a new drive command must be seen on a few frames in a row, and a lost
gesture keeps the last command alive for a short grace period before stopping.
"""

from __future__ import annotations

DRIVE_COMMANDS = "FBLR"


class GestureSmoother:
    def __init__(self, confirm_frames: int = 2):
        self.confirm_frames = max(1, confirm_frames)
        self.reset()

    def reset(self) -> None:
        self.active = "S"
        self._candidate: str | None = None
        self._count = 0
        self._last_seen = 0.0

    def update(self, command: str, now: float, hold_s: float, stop_now: bool = False) -> str:
        """Return the drive command to act on: one of F, B, L, R or S.

        `stop_now` (a closed fist) stops immediately, skipping the hold time.
        """
        if stop_now:
            self.reset()
            return self.active
        if command in DRIVE_COMMANDS:
            if command == self.active:
                self._candidate, self._count = None, 0
                self._last_seen = now
                return self.active
            if command == self._candidate:
                self._count += 1
            else:
                self._candidate, self._count = command, 1
            if self._count >= self.confirm_frames:
                self.active = command
                self._candidate, self._count = None, 0
                self._last_seen = now
            return self.active

        # Stop / no hands / gripper gesture: keep going briefly, then stop.
        self._candidate, self._count = None, 0
        if self.active != "S" and now - self._last_seen > hold_s:
            self.active = "S"
        return self.active
