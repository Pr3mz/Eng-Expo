"""Testable preflight interlocks for autonomous movement."""


def auto_preflight_blockers(
    *,
    auto_requested: bool,
    motion_confirmed: bool,
    manual_mode: bool = False,
    scale_valid: bool,
    arena_valid: bool,
    zones_ready: bool,
    stable_marker_frames: int,
    robot_online: bool,
    required_marker_frames: int = 15,
) -> list[str]:
    blockers = []
    if not auto_requested:
        blockers.append("start with --enable-auto")
    if not motion_confirmed:
        blockers.append("confirm the manual drive/gripper check with --confirm-motion")
    if manual_mode:
        blockers.append("turn manual mode off with M before starting auto")
    if not scale_valid:
        blockers.append("set a measured --mm-per-pixel value")
    if not arena_valid:
        blockers.append("calibrate all four arena corners")
    if not zones_ready:
        blockers.append("mark every active drop zone")
    if stable_marker_frames < required_marker_frames:
        blockers.append(f"keep robot marker visible for {required_marker_frames} frames")
    if not robot_online:
        blockers.append("wait for a fresh ESP32 UDP reply")
    return blockers
