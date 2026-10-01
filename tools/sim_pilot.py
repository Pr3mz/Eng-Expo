"""Offline simulation of the autonomous pilot (no camera, no robot).

Drives a simulated differential-drive rover with a dead zone, motor lag,
camera latency and measurement noise toward random targets and reports how
close it stops and how smooth the path is.

    .venv/bin/python tools/sim_pilot.py
"""

from __future__ import annotations

import math
import random
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))

from navigation_math import heading_error, wrap_angle  # noqa: E402
from pilot import Pilot, PilotConfig  # noqa: E402

DEAD_PWM = 135.0        # wheels do not turn below this
PX_PER_S_PER_PWM = 1.6  # speed above the dead zone
TRACK_PX = 80.0
MOTOR_TAU = 0.12
LATENCY_S = 0.10
GRIPPER_PX = 110.0
MM_PER_PX = 2.2


def wheel_speed(pwm: float) -> float:
    return math.copysign(max(0.0, abs(pwm) - DEAD_PWM) * PX_PER_S_PER_PWM, pwm)


def run(seed: int, fps: float = 30.0, stop_mm: float = 40.0, noise: bool = True, verbose: bool = False, config: PilotConfig | None = None):
    rng = random.Random(seed)
    x, y = rng.uniform(80, 720), rng.uniform(80, 520)
    heading = rng.uniform(-math.pi, math.pi)
    while True:
        target = (rng.uniform(80, 720), rng.uniform(80, 520))
        if math.dist((x, y), target) > 150:
            break
    pilot = Pilot(config)
    vl = vr = 0.0
    cmd_l = cmd_r = 0.0
    dt = 1.0 / fps
    history: deque = deque(maxlen=max(1, int(LATENCY_S / (1 / 240)) + 1))
    t = 0.0
    sim_dt = 1 / 240
    next_frame = 0.0
    stopped_at = None
    turn_flips = 0
    last_turn_sign = 0
    path_len = 0.0
    prev = (x, y)
    arrived_t = None
    while t < 40.0:
        # camera frame
        if t >= next_frame:
            next_frame += dt
            px, py, ph = history[0] if history else (x, y, heading)
            if noise:
                px += rng.gauss(0, 1.0)
                py += rng.gauss(0, 1.0)
                ph += rng.gauss(0, math.radians(0.8))
            ref = (px + GRIPPER_PX * math.cos(ph), py + GRIPPER_PX * math.sin(ph))
            distance_mm = math.dist(ref, target) * MM_PER_PX
            if distance_mm <= stop_mm:
                arrived_t = t
                cmd_l = cmd_r = 0.0
                pilot.update(0.0, distance_mm, stop_mm, t)
                break
            error = heading_error((px, py), ph, target)
            forward, turn, mode = pilot.update(error, distance_mm, stop_mm, t, gripper_error=heading_error(ref, ph, target))
            cmd_l, cmd_r = forward - turn, forward + turn
            peak = max(abs(cmd_l), abs(cmd_r), 255.0)
            cmd_l, cmd_r = cmd_l * 255 / peak, cmd_r * 255 / peak
            sign = (turn > 5) - (turn < -5)
            if sign and last_turn_sign and sign != last_turn_sign and mode == "drive":
                turn_flips += 1
            if sign:
                last_turn_sign = sign
            if verbose:
                print(f"t={t:5.2f} {mode:7s} err={math.degrees(error):7.1f} dist={distance_mm:6.1f} fwd={forward:6.1f} turn={turn:6.1f}")
        # physics
        vl += (wheel_speed(cmd_l) - vl) * sim_dt / MOTOR_TAU
        vr += (wheel_speed(cmd_r) - vr) * sim_dt / MOTOR_TAU
        v = (vl + vr) / 2
        heading = wrap_angle(heading - (vr - vl) / TRACK_PX * sim_dt)
        x += v * math.cos(heading) * sim_dt
        y += v * math.sin(heading) * sim_dt
        path_len += math.dist((x, y), prev)
        prev = (x, y)
        history.append((x, y, heading))
        t += sim_dt
    # coast after the STOP command (hard stop on the robot, a little slide)
    coast = ((vl + vr) / 2) * 0.03
    x += coast * math.cos(heading)
    y += coast * math.sin(heading)
    ref = (x + GRIPPER_PX * math.cos(heading), y + GRIPPER_PX * math.sin(heading))
    final_mm = math.dist(ref, target) * MM_PER_PX
    return {"ok": arrived_t is not None, "time": t, "final_mm": final_mm, "flips": turn_flips}


def main() -> int:
    for fps in (30.0, 15.0):
        results = [run(seed, fps=fps) for seed in range(60)]
        ok = [r for r in results if r["ok"]]
        finals = sorted(r["final_mm"] for r in ok)
        print(f"{fps:.0f} fps: arrived {len(ok)}/{len(results)} | "
              f"final error mm median {finals[len(finals)//2]:.0f} worst {finals[-1]:.0f} | "
              f"time median {sorted(r['time'] for r in ok)[len(ok)//2]:.1f}s | "
              f"arc steering flips avg {sum(r['flips'] for r in ok)/len(ok):.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
