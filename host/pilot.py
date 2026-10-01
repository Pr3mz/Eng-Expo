"""Smooth continuous steering for the autonomous drive.

Replaces the old stop-and-go pulses. Every camera frame it turns the
heading error and the distance left into one forward power and one turn power:

* far off heading  -> spin on the spot, slower as the heading gets close
* roughly aligned  -> drive and steer in one smooth arc, slowing down as the
                      target gets close so it arrives at creep speed
* target just behind us and close -> back up instead of turning around

The wheels do not move below about 140 PWM on this rover, so every nonzero
output is kept at or above `min_pwm` instead of fading into a dead zone.
Output changes are rate-limited so starts and slow-downs stay gentle.

Sign conventions: error > 0 means the target is clockwise of the heading
(the planner's "R"); `turn` > 0 means counter-clockwise ("L"), so a positive
error produces a negative turn.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class PilotConfig:
    min_pwm: float = 160.0         # slowest power that still moves the rover
    cruise_pwm: float = 210.0      # fastest straight-line power
    spin_max_pwm: float = 180.0    # spin power for a large heading error
    slow_range_mm: float = 260.0    # start slowing this far from the stop point
    creep_range_mm: float = 45.0    # creep at min_pwm inside this distance
    spin_enter: float = math.radians(48)   # leave arc driving above this error
    spin_exit: float = math.radians(14)    # resume arc driving below this error
    full_spin_error: float = math.radians(100)
    steer_gain: float = 40.0       # turn PWM per radian of error while driving
    max_steer: float = 30.0
    reverse_error: float = math.radians(110)  # target this far behind the gripper and close: back up
    reverse_range_mm: float = 330.0           # ...within this distance (about the gripper's reach)
    reverse_center_error: float = math.radians(100)  # ...but only if it is still ahead of the robot centre
    slew_per_s: float = 900.0       # largest output change per second
    heading_slow: float = 0.55      # how much heading error cuts forward speed
    near_mm: float = 220.0          # below this distance the steering gain fades...
    near_floor: float = 0.4        # ...down to this fraction (angles blow up close in)


class Pilot:
    def __init__(self, config: PilotConfig | None = None):
        self.cfg = config or PilotConfig()
        self.reset()

    def reset(self) -> None:
        self.mode = "drive"
        self._prev_mode = "drive"
        self._spin_dir = 0.0
        self._forward = 0.0
        self._turn = 0.0
        self._last_time: float | None = None

    @staticmethod
    def _slew(previous: float, wanted: float, limit: float, rest_floor: float) -> float:
        if wanted == 0.0:
            return 0.0                        # stopping is never delayed
        if previous == 0.0:
            previous = math.copysign(rest_floor, wanted)   # kick past the dead zone
        if math.copysign(1.0, previous) != math.copysign(1.0, wanted):
            return math.copysign(rest_floor, wanted)
        return previous + max(-limit, min(limit, wanted - previous))

    def update(self, error: float, distance_mm: float, stop_mm: float, now: float,
               gripper_error: float | None = None) -> tuple[float, float, str]:
        """Return (forward, turn_left, mode) for one frame.

        `error` is the target's bearing from the robot CENTRE relative to its
        heading (used for steering: a target near the robot can never be lined
        up from the gripper tip, which orbits the centre as the robot turns).
        `gripper_error` is the same bearing from the gripper tip, used to tell
        that the target lies between the centre and the tip, so the robot
        should back up. `distance_mm` is gripper tip to target and `stop_mm`
        the distance at which the planner considers it arrived.
        """
        if gripper_error is None:
            gripper_error = error
        c = self.cfg
        dt = 0.033 if self._last_time is None else max(0.001, min(0.2, now - self._last_time))
        self._last_time = now
        e = abs(error)
        remaining = max(0.0, distance_mm - stop_mm)

        if self.mode == "drive" and e > c.spin_enter:
            self.mode = "spin"
        elif self.mode == "spin" and e < c.spin_exit:
            self.mode = "drive"
        if self.mode != "spin":
            self._spin_dir = 0.0

        reverse = (abs(gripper_error) > c.reverse_error and remaining < c.reverse_range_mm
                   and e < c.reverse_center_error)
        if reverse:
            mode = "reverse"
            # Back straight until the gripper reaches the target; steer on the rear error.
            rear_error = math.copysign(math.pi - abs(gripper_error), -gripper_error)
            forward = -c.min_pwm
            turn = -c.steer_gain * 0.6 * rear_error
            turn = max(-c.max_steer, min(c.max_steer, turn))
        elif self.mode == "spin":
            mode = "spin"
            span = max(1e-6, c.full_spin_error - c.spin_exit)
            fraction = max(0.0, min(1.0, (e - c.spin_exit) / span))
            power = c.min_pwm + (c.spin_max_pwm - c.min_pwm) * fraction
            forward = 0.0
            # A target almost exactly behind flips the error between +180 and -180
            # degrees; hold one spin direction until the error is clearly one-sided.
            if e > math.radians(140):
                if self._spin_dir == 0.0:
                    self._spin_dir = math.copysign(1.0, error)
            else:
                self._spin_dir = math.copysign(1.0, error)
            turn = -self._spin_dir * power
        else:
            mode = "drive"
            if remaining <= c.creep_range_mm:
                speed = c.min_pwm
            else:
                fraction = min(1.0, (remaining - c.creep_range_mm) / max(1.0, c.slow_range_mm - c.creep_range_mm))
                speed = c.min_pwm + (c.cruise_pwm - c.min_pwm) * fraction
            speed = max(c.min_pwm, speed * (1.0 - c.heading_slow * min(1.0, e / c.spin_enter)))
            forward = speed
            gain = c.steer_gain * max(c.near_floor, min(1.0, distance_mm / c.near_mm))
            turn = max(-c.max_steer, min(c.max_steer, -gain * error))

        if mode == "drive" and self._prev_mode in ("spin", "reverse"):
            self._turn = 0.0                  # do not carry a spin's turn power into the arc
        self._prev_mode = mode
        limit = c.slew_per_s * dt
        forward = self._slew(self._forward, forward, limit, c.min_pwm)
        turn = self._turn + max(-limit, min(limit, turn - self._turn)) if mode != "spin" else turn
        self._forward, self._turn = forward, turn
        return forward, turn, mode
