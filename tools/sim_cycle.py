"""Offline simulation of whole sorting cycles: planner + pilot + a simulated rover.

Runs the same planner/pilot calls the real program makes, with no gripper
(each stone is visited once), and reports how precisely it stops at stones and
drop circles.

    .venv/bin/python tools/sim_cycle.py
"""

from __future__ import annotations

import math
import random
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import sim_pilot as sp  # noqa: E402
from navigation_math import heading_error, wrap_angle  # noqa: E402
from pilot import Pilot  # noqa: E402
from sorter_planner import SorterPlanner  # noqa: E402
from vision import Gem, RobotPose  # noqa: E402

MM_PER_PX = 2.2
PICKUP_MM = 20 * MM_PER_PX
DROP_MM = 60.0


def main(seed: int = 1, seconds: float = 150.0) -> int:
    rng = random.Random(seed)
    zones = {"crimson": (90, 90), "violet": (710, 90), "lime": (90, 510), "cyan": (710, 510)}
    colors = list(zones)
    gems = [Gem(int(rng.uniform(300, 500)), int(rng.uniform(220, 380)), colors[i % 4], 300.0) for i in range(8)]
    x, y, heading = 400.0, 300.0, rng.uniform(-math.pi, math.pi)
    planner = SorterPlanner(grip_dwell_s=0.6, drop_dwell_s=0.6, skip_picked=True)
    pilot = Pilot()
    vl = vr = 0.0
    cmd_l = cmd_r = 0.0
    sim_dt, frame_dt = 1 / 240, 1 / 30
    history: deque = deque(maxlen=int(sp.LATENCY_S / sim_dt) + 1)
    t, next_frame = 0.0, 0.0
    events = []
    last_phase = planner.phase
    while t < seconds:
        if t >= next_frame:
            next_frame += frame_dt
            px, py, ph = history[0] if history else (x, y, heading)
            px += rng.gauss(0, 1.0); py += rng.gauss(0, 1.0); ph += rng.gauss(0, math.radians(0.8))
            pose = RobotPose(int(px), int(py), ph, (int(px + 50 * math.cos(ph)), int(py + 50 * math.sin(ph))))
            grip_pt = (int(px + sp.GRIPPER_PX * math.cos(ph)), int(py + sp.GRIPPER_PX * math.sin(ph)))
            d = planner.step(pose=pose, approach_point=grip_pt, gems=gems, zones=zones, mm_per_pixel=MM_PER_PX,
                             pickup_distance_mm=PICKUP_MM, drop_distance_mm=DROP_MM, now=t)
            if d.phase != last_phase:
                true_grip = (x + sp.GRIPPER_PX * math.cos(heading), y + sp.GRIPPER_PX * math.sin(heading))
                if d.phase == "grip" and planner.active_gem:
                    events.append((t, "PICK", planner.active_gem.color, math.dist(true_grip, (planner.active_gem.x, planner.active_gem.y)) * MM_PER_PX))
                if d.phase == "drop":
                    events.append((t, "DROP", planner.carrying_color, math.dist(true_grip, zones[planner.carrying_color]) * MM_PER_PX))
                last_phase = d.phase
            forward = turn = 0.0
            if d.target is not None and d.command in "FLR":
                stop_mm = DROP_MM if planner.phase == "deliver" else PICKUP_MM
                err = heading_error((px, py), ph, d.target)
                forward, turn, _ = pilot.update(err, math.dist(grip_pt, d.target) * MM_PER_PX, stop_mm, t,
                                                gripper_error=heading_error(grip_pt, ph, d.target))
            else:
                pilot.reset()
                if d.command == "B":
                    forward = -160.0
            cmd_l, cmd_r = forward - turn, forward + turn
            peak = max(abs(cmd_l), abs(cmd_r), 255.0)
            cmd_l, cmd_r = cmd_l * 255 / peak, cmd_r * 255 / peak
        vl += (sp.wheel_speed(cmd_l) - vl) * sim_dt / sp.MOTOR_TAU
        vr += (sp.wheel_speed(cmd_r) - vr) * sim_dt / sp.MOTOR_TAU
        if cmd_l == 0 and cmd_r == 0:
            vl = vr = 0.0   # STOP is a hard stop on the robot
        heading = wrap_angle(heading - (vr - vl) / sp.TRACK_PX * sim_dt)
        x += (vl + vr) / 2 * math.cos(heading) * sim_dt
        y += (vl + vr) / 2 * math.sin(heading) * sim_dt
        history.append((x, y, heading))
        t += sim_dt
    for t, kind, color, err in events:
        print(f"{t:6.1f}s {kind} {color:8s} stopped {err:5.0f} mm from the {'stone' if kind == 'PICK' else 'circle centre'}")
    picks = [e[3] for e in events if e[1] == "PICK"]
    drops = [e[3] for e in events if e[1] == "DROP"]
    print(f"cycles: {len(drops)} drops in {seconds:.0f}s | pick error avg {sum(picks)/max(1,len(picks)):.0f} mm | drop error avg {sum(drops)/max(1,len(drops)):.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
