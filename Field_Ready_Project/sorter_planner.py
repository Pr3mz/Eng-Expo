"""Pure, hardware-independent decisions for one gemstone sorting cycle."""

from __future__ import annotations

from dataclasses import dataclass
import math

from navigation_math import heading_error, steering_command
from vision import PALETTE, Gem, RobotPose


@dataclass(frozen=True)
class PlanDecision:
    command: str
    reason: str
    action: str | None = None
    target: tuple[int, int] | None = None
    phase: str = "search"


class SorterPlanner:
    """Choose a target and advance search/grip/deliver/drop states.

    This class has no camera, socket, or motor dependency, so the same
    decisions can be tested offline before enabling the hardware loop.
    """

    def __init__(self, *, grip_dwell_s: float = 1.0, drop_dwell_s: float = 1.0, retreat_dwell_s: float = 0.8):
        self.grip_dwell_s = grip_dwell_s
        self.drop_dwell_s = drop_dwell_s
        self.retreat_dwell_s = retreat_dwell_s
        self.reset()

    def reset(self) -> None:
        self.phase = "search"
        self.carrying_color: str | None = None
        self.active_gem: Gem | None = None
        self.action_deadline = 0.0
        self.home_center: tuple[int, int] | None = None

    def begin_return_home(self, center: tuple[int, int]) -> bool:
        """Temporarily route to the saved center, then resume sorting safely."""
        if self.phase in ("grip", "drop"):
            return False
        carried_color = self.carrying_color if self.phase == "deliver" else None
        self.phase = "home"
        self.home_center = (int(center[0]), int(center[1]))
        self.active_gem = None
        self.carrying_color = carried_color
        self.action_deadline = 0.0
        return True

    @staticmethod
    def select_gem(pose: RobotPose, gems: list[Gem]) -> Gem | None:
        """Prefer the nearest gem in front, with a penalty for gems behind."""
        if not gems:
            return None
        heading_x = math.cos(pose.heading)
        heading_y = math.sin(pose.heading)

        def cost(gem: Gem) -> float:
            dx, dy = gem.x - pose.x, gem.y - pose.y
            behind_penalty = 500.0 if dx * heading_x + dy * heading_y < 0 else 0.0
            return math.hypot(dx, dy) + behind_penalty

        return min(gems, key=cost)

    def step(
        self,
        *,
        pose: RobotPose | None,
        approach_point: tuple[int, int] | None = None,
        gems: list[Gem],
        zones: dict[str, tuple[int, int]],
        mm_per_pixel: float | None,
        pickup_distance_mm: float,
        drop_distance_mm: float,
        now: float,
        home_distance_mm: float = 70.0,
        angle_threshold: float = 0.30,
    ) -> PlanDecision:
        if pose is None:
            return PlanDecision("S", "STOP: robot marker not visible", phase=self.phase)
        if any(color not in zones for color in PALETTE):
            return PlanDecision("S", f"STOP: set drop circles ({len(zones)}/6)", phase=self.phase)
        if mm_per_pixel is None or not math.isfinite(mm_per_pixel) or mm_per_pixel <= 0:
            return PlanDecision("S", "STOP: measured scale not configured", phase=self.phase)

        if self.phase == "grip":
            if now < self.action_deadline:
                return PlanDecision("S", "GRIP: holding position", phase=self.phase)
            self.phase = "deliver"
        elif self.phase == "drop":
            if now < self.action_deadline:
                return PlanDecision("S", "DROP: releasing at destination", phase=self.phase)
            self.phase = "retreat"
            self.action_deadline = now + self.retreat_dwell_s
            return PlanDecision("B", "RETREAT: backing up from drop zone", phase=self.phase)
            
        elif self.phase == "retreat":
            if now < self.action_deadline:
                return PlanDecision("B", "RETREAT: backing up from drop zone", phase=self.phase)
            self.reset()
            return PlanDecision("S", "RETREAT: complete", phase=self.phase)

        if self.phase == "search":
            # Stones vanish from detection as the robot gets close because its
            # own marker/gripper is masked out. Keep the chosen stone's last
            # position until pickup instead of switching targets every frame.
            if self.active_gem is None:
                self.active_gem = self.select_gem(pose, gems)
            else:
                matches = [
                    gem for gem in gems
                    if gem.color == self.active_gem.color
                    and math.hypot(gem.x - self.active_gem.x, gem.y - self.active_gem.y) <= 25
                ]
                if matches:
                    self.active_gem = min(
                        matches,
                        key=lambda gem: math.hypot(gem.x - self.active_gem.x, gem.y - self.active_gem.y),
                    )
            self.carrying_color = None
            if self.active_gem is None:
                return PlanDecision("S", "STOP: no visible gemstone", phase=self.phase)
            target = (self.active_gem.x, self.active_gem.y)
            threshold = pickup_distance_mm
        elif self.phase == "home":
            if self.home_center is None:
                self.reset()
                return PlanDecision("S", "STOP: home center not configured", phase=self.phase)
            target = self.home_center
            threshold = home_distance_mm
        elif self.phase == "deliver":
            if self.carrying_color not in zones:
                return PlanDecision("S", "STOP: destination not configured", phase=self.phase)
            target = zones[self.carrying_color]
            threshold = drop_distance_mm
        else:
            self.reset()
            return PlanDecision("S", "STOP: planner reset", phase=self.phase)

        reference = (pose.x, pose.y) if self.phase == "home" else (approach_point or (pose.x, pose.y))
        distance_px = math.hypot(target[0] - reference[0], target[1] - reference[1])
        distance_mm = distance_px * mm_per_pixel
        if distance_mm <= threshold:
            if self.phase == "home":
                self.home_center = None
                self.active_gem = None
                if self.carrying_color is not None:
                    self.phase = "deliver"
                    return PlanDecision(
                        "S", "HOME: reached center; resuming delivery",
                        target=target, phase=self.phase,
                    )
                self.reset()
                return PlanDecision(
                    "S", "HOME: reached center; restarting search",
                    target=target, phase=self.phase,
                )
            if self.phase == "search" and self.active_gem is not None:
                self.carrying_color = self.active_gem.color
                self.phase = "grip"
                self.action_deadline = now + self.grip_dwell_s
                return PlanDecision(
                    "S", f"STOP: search range {distance_mm:.0f} mm",
                    action="CLOSE", target=target, phase=self.phase,
                )
            if self.phase == "deliver":
                self.phase = "drop"
                self.action_deadline = now + self.drop_dwell_s
                return PlanDecision(
                    "S", f"STOP: deliver range {distance_mm:.0f} mm",
                    action="OPEN", target=target, phase=self.phase,
                )

        error = heading_error(reference, pose.heading, target)
        command = steering_command(error, angle_threshold)
        return PlanDecision(
            command, f"{self.phase.upper()} {command} | {distance_mm:.0f} mm | error {math.degrees(error):+.0f} deg",
            target=target, phase=self.phase,
        )
