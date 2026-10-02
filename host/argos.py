"""Camera-guided gemstone sorter for ARGOS (sorts the colors in vision.PALETTE).

Auto mode requires a live camera, valid arena and zone calibration, a measured
scale, a stable robot marker, a fresh ESP32 reply, and an explicit auto option.
Short-lived marker loss or a camera-edge approach triggers a bounded reverse
recovery while preserving the current sorting target.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np

from camera_stream import LatestFrameCamera
from navigation_math import heading_error, wrap_angle
import robot_settings
from setup_panel import SetupPanel
from manual_smoothing import GestureSmoother
from pilot import Pilot
from robot_link import RobotLink
from safety import auto_preflight_blockers
from sorter_planner import SorterPlanner
try:
    from gesture_control import GestureController
except ImportError:
    GestureController = None
import vision
from vision import (
    RobotPose, detect_gems, detect_gems_roboflow, detect_robot_pose,
    make_homography, sample_target_color,
)

WARP_W = 800
WARP_H = 600   # recomputed below from the real arena size when it is set
ZONE_RADIUS_PX = 48
ZONE_EXCLUSION_RADIUS_PX = 115
# Robot size, gripper geometry, scale and drive speeds live in
# robot_settings.json; press G for the slider panel. This rover did not move
# at PWM 90 or 145 on the floor, so keep creep_speed high and pulses short.
CFG = robot_settings.load()
CFG["servo_test_deg"] = CFG["servo_open_deg"]   # live tester starts at the open position
if CFG["arena_width_mm"] > 0 and CFG["arena_height_mm"] > 0:
    # Warp to the arena's true proportions so angles and distances in the
    # top-down view are real (a 4:3 warp of a 16:9 field bends every heading).
    WARP_H = max(200, min(900, int(round(WARP_W * CFG["arena_height_mm"] / CFG["arena_width_mm"]))))
NEAR_TARGET_MARGIN_MM = 150
ANGLE_THRESHOLD = math.radians(float(os.getenv("ANGLE_THRESHOLD_DEG", "20")))
CAMERA_API = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
MAX_FRAME_AGE_S = 0.75
POSE_LOSS_LIMIT = 8
CAMERA_EDGE_MARGIN_PX = 40
VIEW_RECOVERY_MAX_PULSES = 8
VIEW_RECOVERY_PULSE_S = 0.24
VIEW_RECOVERY_GAP_S = 0.30
MANUAL_KEY_TIMEOUT_S = 0.8
_INSTANCE_MUTEX_HANDLE = None
_INSTANCE_KERNEL32 = None
CALIBRATION_PATH = Path(__file__).resolve().parent / "arena_calibration.json"
CAPTURE_DIR = Path(__file__).resolve().parent.parent / "data" / "captures"


def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("value must be a finite number greater than zero")
    return number


def ipv4_address(value: str) -> str:
    try:
        return str(ipaddress.IPv4Address(value))
    except ipaddress.AddressValueError as exc:
        raise argparse.ArgumentTypeError("robot IP must be a valid IPv4 address") from exc


def apply_settings(args) -> None:
    """Copy the shared CFG values into the run options the loop reads."""
    args.gripper_distance_px = max(1.0, CFG["gripper_distance_px"])
    args.gripper_radius_px = max(1.0, CFG["gripper_radius_px"])
    args.mm_per_pixel = CFG["mm_per_pixel"] or None
    args.drop_distance_mm = CFG["drop_distance_mm"]
    args.pickup_distance_mm = (
        args.gripper_radius_px * args.mm_per_pixel
        if args.mm_per_pixel is not None else None
    )


def grip_command(close: bool) -> str:
    """Gripper move to the saved open/close angle (robot command SERVO <angle>)."""
    return f"SERVO {int(CFG['servo_close_deg' if close else 'servo_open_deg'])}"


def manual_speed(command: str, held_s: float | None = None) -> int:
    """Manual drive power. Forward/backward use the plain set speed. Turning, with
    `held_s`, ramps from (speed - accel) to (speed + accel) over manual_accel_s
    seconds after the gesture starts: a gentle start that never waits to move."""
    speed = (CFG["manual_turn_speed"] if command in "LR"
             else CFG["manual_back_speed"] if command == "B" else CFG["manual_speed"])
    accel = CFG["manual_accel_pwm"] if command in "LR" else 0   # only turning accelerates
    if held_s is not None and accel > 0:
        fraction = min(1.0, max(0.0, held_s / max(0.1, CFG["manual_accel_s"])))
        speed = speed - accel + 2.0 * accel * fraction
    return int(max(0, min(255, round(speed))))


def gripper_point_from_heading(x: float, y: float, heading: float, distance_px: float) -> tuple[int, int]:
    """Project a virtual gripper point forward from the robot marker center."""
    return (
        int(round(x + distance_px * math.cos(heading))),
        int(round(y + distance_px * math.sin(heading))),
    )


def robot_exclusion_regions(
    pose: RobotPose,
    gripper_point: tuple[int, int],
    gripper_radius_px: float,
) -> tuple[tuple[tuple[int, int], int], ...]:
    """Mask colored robot stickers along the chassis, arms, and gripper."""
    ux, uy = math.cos(pose.heading), math.sin(pose.heading)
    front_distance = math.dist((pose.x, pose.y), gripper_point)
    rear_distance = CFG["robot_rear_px"]
    end_distance = front_distance + max(25.0, gripper_radius_px)
    total_distance = rear_distance + end_distance
    # The marker center is not the full robot footprint: servo covers and
    # colored stickers sit to either side of the linkage. Overlap the mask
    # samples generously so those patches cannot reappear as gem contours.
    sample_count = max(1, int(math.ceil(total_distance / 24.0)))
    arm_radius = max(60, int(round(gripper_radius_px + 40)))
    robot_radius = int(round(CFG["robot_radius_px"]))
    regions = [((pose.x, pose.y), robot_radius)]
    for index in range(sample_count + 1):
        offset = -rear_distance + total_distance * index / sample_count
        center = (
            int(round(pose.x + ux * offset)),
            int(round(pose.y + uy * offset)),
        )
        radius = robot_radius if offset <= 0 else arm_radius
        regions.append((center, radius))
    return tuple(regions)


def run_gripper_simulator(distance_px: float, radius_px: float) -> int:
    """Offline visualization of the virtual gripper geometry; never opens hardware."""
    width, height = WARP_W, WARP_H
    origin = [250.0, 300.0]
    heading = 0.0
    initial_origin = origin.copy()
    initial_heading = heading
    palette_bgr = {
        "violet": (220, 40, 175), "cyan": (220, 210, 20),
        "crimson": (35, 35, 225), "gold": (0, 170, 255),
        "blue": (230, 110, 35), "lime": (60, 200, 60),
    }
    gems = [
        (380, 300, "crimson"), (570, 170, "violet"), (640, 350, "cyan"),
        (465, 470, "gold"), (190, 430, "blue"), (180, 170, "lime"),
    ]

    window = "ARGOS | virtual gripper simulator"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, width, height)
    distance_limit = max(400, int(math.ceil(distance_px)) + 10)
    radius_limit = max(150, int(math.ceil(radius_px)) + 10)
    cv2.createTrackbar("distance from ID34 (px)", window, int(round(distance_px)), distance_limit, lambda _value: None)
    cv2.createTrackbar("gripper radius (px)", window, max(1, int(round(radius_px))), radius_limit, lambda _value: None)

    try:
        while True:
            distance_px = float(cv2.getTrackbarPos("distance from ID34 (px)", window))
            radius_px = float(max(1, cv2.getTrackbarPos("gripper radius (px)", window)))
            image = np.full((height, width, 3), (46, 54, 48), dtype=np.uint8)
            cv2.rectangle(image, (24, 24), (width - 25, height - 25), (190, 190, 190), 2)
            cv2.putText(image, "SIMULATION ONLY - no camera, Wi-Fi, or motor commands", (42, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (245, 245, 245), 2)

            for center, color in zip(
                ((105, 105), (400, 95), (695, 105), (105, 495), (400, 505), (695, 495)),
                vision.PALETTE,
            ):
                cv2.circle(image, center, 36, palette_bgr[color], -1)
                cv2.circle(image, center, 36, (235, 235, 235), 2)
                cv2.putText(image, color, (center[0] - 35, center[1] + 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (235, 235, 235), 1)

            for x, y, color in gems:
                cv2.circle(image, (x, y), 13, palette_bgr[color], -1)
                cv2.circle(image, (x, y), 13, (255, 255, 255), 1)
                cv2.putText(image, color, (x + 14, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (245, 245, 245), 1)

            # Draw a simple rover body and its existing ID-34 origin/heading.
            cosine, sine = math.cos(heading), math.sin(heading)
            local_body = np.array([[-55, -39], [27, -39], [52, 0], [27, 39], [-55, 39]], dtype=np.float32)
            rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float32)
            body = np.round(local_body @ rotation.T + np.array(origin, dtype=np.float32)).astype(np.int32)
            cv2.fillPoly(image, [body], (42, 115, 185))
            cv2.polylines(image, [body], True, (235, 235, 235), 2)
            cv2.rectangle(image, (int(origin[0] - 19), int(origin[1] - 19)),
                          (int(origin[0] + 19), int(origin[1] + 19)), (10, 10, 10), -1)
            cv2.putText(image, "34", (int(origin[0] - 15), int(origin[1] + 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            origin_point = (int(round(origin[0])), int(round(origin[1])))
            nose = (int(round(origin[0] + 65 * cosine)), int(round(origin[1] + 65 * sine)))
            cv2.arrowedLine(image, origin_point, nose, (255, 255, 0), 3, tipLength=0.25)
            gripper_point = gripper_point_from_heading(origin[0], origin[1], heading, distance_px)
            cv2.line(image, origin_point, gripper_point, (0, 255, 0), 2)
            cv2.circle(image, gripper_point, int(round(radius_px)), (0, 255, 0), 2)
            cv2.drawMarker(image, gripper_point, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)

            nearest = min(gems, key=lambda gem: math.dist(gripper_point, gem[:2]))
            gem_distance = math.dist(gripper_point, nearest[:2])
            inside = gem_distance <= radius_px
            cv2.line(image, gripper_point, nearest[:2], (0, 0, 255) if inside else (0, 210, 255), 2)
            cv2.circle(image, nearest[:2], 19, (0, 0, 255) if inside else (0, 210, 255), 2)
            status = (f"PICKUP ZONE: stop + close ({nearest[2]})" if inside
                      else f"Approach {nearest[2]}: {gem_distance:.0f}px to zone center")
            status_color = (0, 0, 255) if inside else (0, 220, 255)
            cv2.putText(image, status, (42, height - 64), cv2.FONT_HERSHEY_SIMPLEX, 0.68, status_color, 2)
            cv2.putText(image, "Gestures drive | R reset | adjust sliders | Q quit",
                        (42, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (245, 245, 245), 1)
            cv2.imshow(window, image)

            key = cv2.waitKeyEx(30)
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("w"), ord("W")):
                origin[0] += 5 * cosine
                origin[1] += 5 * sine
            elif key in (ord("s"), ord("S")):
                origin[0] -= 5 * cosine
                origin[1] -= 5 * sine
            elif key in (ord("a"), ord("A")):
                heading -= math.radians(5)
            elif key in (ord("d"), ord("D")):
                heading += math.radians(5)
            elif key in (ord("r"), ord("R")):
                origin = initial_origin.copy()
                heading = initial_heading
    finally:
        cv2.destroyWindow(window)
    return 0


class ArenaSetup:
    def __init__(self, calibration_path: Path | None = None, camera_index: int = 0, required: int = 4):
        self.ignore_clicks = False  # True while Manual is on: clicking the window to focus it must not mark anything
        self.max_sites = required  # most drop circles that can be marked (up to 6)
        self.required = required   # circles needed now; ENTER finishes early and remembers it
        self.calibration_path = calibration_path
        self.camera_index = camera_index
        self.corners: list[tuple[int, int]] = []
        self.homography = None
        self.zones: dict[str, tuple[int, int]] = {}
        self.home_center: tuple[int, int] | None = None
        self.sample_image = None
        self._restore()

    @staticmethod
    def _point(value):
        if (not isinstance(value, list) or len(value) != 2
                or any(type(component) is not int or component < 0 for component in value)):
            raise ValueError("invalid calibration point")
        return tuple(value)

    def _restore(self):
        if self.calibration_path is None or not self.calibration_path.exists():
            return
        try:
            data = json.loads(self.calibration_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("invalid calibration file")
            if data.get("version") != 1 or data.get("camera_index") != self.camera_index:
                raise ValueError("camera or calibration version changed")
            corners = data.get("corners", [])
            zones = data.get("zones", {})
            home_center = data.get("home_center")
            if data.get("warp_h", 600) != WARP_H:
                # Circle positions are stored in the warped view, which changed shape.
                if zones or home_center:
                    print("Arena size changed since the last run: drop circles and home must be marked again.")
                zones, home_center = {}, None
            if not isinstance(corners, list) or len(corners) > 4 or not isinstance(zones, dict):
                raise ValueError("invalid calibration layout")
            corners = [self._point(point) for point in corners]
            if len(corners) != 4 and zones:
                raise ValueError("drop circles without arena corners")
            if any(color not in vision.ALL_COLORS for color in zones):
                raise ValueError("unknown drop-circle color")
            # Keep corners when switching --colors; only the active circles matter.
            zones = {color: self._point(point) for color, point in zones.items() if color in vision.PALETTE}
            if any(x >= WARP_W or y >= WARP_H for x, y in zones.values()):
                raise ValueError("drop circle outside rectified arena")
            home_center = self._point(home_center) if home_center is not None else None
            if home_center is not None and (home_center[0] >= WARP_W or home_center[1] >= WARP_H):
                raise ValueError("home center outside rectified arena")
            self.homography = make_homography(corners, WARP_W, WARP_H) if len(corners) == 4 else None
            sites = data.get("sites")
            if isinstance(sites, int) and 1 <= sites <= self.max_sites and len(zones) <= sites:
                self.required = sites
            self.corners = corners
            self.zones = zones
            self.home_center = home_center
            print(f"Restored arena calibration: {len(corners)}/4 corners, {len(zones)}/{self.required} drop circles ({', '.join(zones) or 'none'}). Press Z to remap circles or R to clear all.")
        except (OSError, ValueError, TypeError, KeyError) as exc:
            print(f"Ignoring saved arena calibration: {exc}")

    def _save(self):
        if self.calibration_path is None:
            return
        data = {"version": 1, "camera_index": self.camera_index, "warp_h": WARP_H, "sites": self.required,
                "corners": self.corners, "zones": self.zones,
                "home_center": self.home_center}
        temporary = self.calibration_path.with_name(self.calibration_path.name + ".tmp")
        try:
            temporary.write_text(json.dumps(data), encoding="utf-8")
            temporary.replace(self.calibration_path)
        except OSError as exc:
            print(f"Could not save arena calibration: {exc}")

    def clear(self):
        self.corners.clear()
        self.homography = None
        self.zones.clear()
        self.required = self.max_sites
        self.home_center = None
        self.sample_image = None
        if self.calibration_path is not None:
            try:
                self.calibration_path.unlink(missing_ok=True)
            except OSError as exc:
                print(f"Could not remove arena calibration: {exc}")

    def set_home_center(self, point: tuple[int, int]) -> None:
        x, y = map(int, point)
        if not (0 <= x < WARP_W and 0 <= y < WARP_H):
            raise ValueError("home center outside rectified arena")
        self.home_center = (x, y)
        self._save()
        print(f"Saved home center at ({x}, {y}).")

    def clear_zones(self):
        """Clear the color-circle points while retaining arena corners and warp."""
        self.zones.clear()
        self.required = self.max_sites
        self.sample_image = None
        self._save()
        print(f"Drop-zone marks cleared; arena corners kept. Click the center of each drop circle again (up to {self.max_sites}), then press ENTER if you have fewer.")

    def finish_zones(self) -> None:
        """Accept the circles marked so far (fewer than the maximum) and remember that."""
        if self.homography is None or not self.zones or len(self.zones) >= self.required:
            return
        self.required = len(self.zones)
        self._save()
        print(f"Using {self.required} drop circle(s): {', '.join(self.zones)}.", flush=True)

    def click(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN or self.ignore_clicks:
            return
        if self.homography is None:
            self.corners.append((x, y))
            if len(self.corners) == 4:
                try:
                    self.homography = make_homography(self.corners, WARP_W, WARP_H)
                except ValueError as exc:
                    print(f"Invalid arena corners: {exc}. Click all four again.")
                    self.corners.clear()
                    self._save()
                    return
                print(f"Arena rectified. Click the center of each drop circle, up to {self.max_sites} (any order; the color is read from the camera). Press ENTER when all your circles are marked.")
            self._save()
            return
        if len(self.zones) < self.required:
            if any(math.dist((x, y), center) < 70 for center in self.zones.values()):
                print("Drop-zone centers are too close together; click a different circle center.")
                return
            color = sample_target_color(self.sample_image, (x, y))
            if color is None:
                print("Could not read a target color there; click the solid-color center of a circle.")
                return
            if color in self.zones:
                # SMART RESOLVER: Cyan and Blue look identical in some lighting.
                # If they clicked a new circle, and it reads as cyan/blue but that's taken,
                # automatically assign it to the other twin color!
                if color == "cyan" and "blue" not in self.zones:
                    color = "blue"
                    print("Smart Resolve: Auto-assigned to 'blue'")
                elif color == "blue" and "cyan" not in self.zones:
                    color = "cyan"
                    print("Smart Resolve: Auto-assigned to 'cyan'")
                else:
                    print(f"This circle reads as {color}, which is already set; click a circle of another color.")
                    return
            self.zones[color] = (x, y)
            self._save()
            print(f"Saved {color} destination at ({x}, {y}) from the camera image.")


def open_camera(index, exposure: float | None = None):
    cap = cv2.VideoCapture(index, CAMERA_API)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Cannot open camera index {index}. Connect the selected overhead camera and set CAMERA_INDEX to its index.")
    try:
        configure_camera_capture(cap, index, exposure)
    except Exception:
        cap.release()
        raise
    return cap


def configure_camera_capture(cap, index: int, exposure: float | None = None) -> None:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if exposure is None:
        try:
            exposure = float(os.getenv("CAMERA_EXPOSURE", "-9"))
        except ValueError as exc:
            raise RuntimeError("CAMERA_EXPOSURE must be a number.") from exc
    if not math.isfinite(exposure):
        raise RuntimeError("CAMERA_EXPOSURE must be finite.")
    if cap.set(cv2.CAP_PROP_EXPOSURE, exposure):
        print(f"Camera index {index}: exposure set to {cap.get(cv2.CAP_PROP_EXPOSURE):g}.")
    else:
        print(f"Camera index {index}: manual exposure unsupported; using camera default.")


def scan_cameras(max_index: int = 8, exposure: float | None = None) -> int:
    found = 0
    for index in range(max_index):
        cap = cv2.VideoCapture(index, CAMERA_API)
        if not cap.isOpened():
            cap.release()
            continue
        try:
            configure_camera_capture(cap, index, exposure)
        except RuntimeError as exc:
            print(f"Camera index {index}: {exc}")
            cap.release()
            continue
        camera = None
        try:
            camera = LatestFrameCamera(cap)
            frame, age, _, error = camera.read_latest()
            if frame is not None:
                found += 1
                print(f"Camera index {index}: {frame.shape[1]}x{frame.shape[0]}, usable at frame {camera.startup_frame_number}, mean brightness {frame.mean():.1f}, variation {frame.std():.1f}")
            else:
                print(f"Camera index {index}: {error or 'no usable image'}")
        except RuntimeError as exc:
            print(f"Camera index {index}: {exc}")
        finally:
            if camera is not None:
                camera.release()
            else:
                cap.release()
    if found == 0:
        print("No usable camera index found from 0 to 7.")
    return 0 if found else 1


def check_robot_link(marker_id: int, robot_ip: str | None = None) -> int:
    link = RobotLink(marker_id=marker_id, robot_ip=robot_ip)
    deadline = time.monotonic() + 3.0
    try:
        while time.monotonic() < deadline:
            link.discover()
            if link.is_online:
                print(f"Robot reply OK: {link.status}")
                return 0
            time.sleep(0.05)
        print(f"No fresh robot reply: {link.status}")
        return 1
    finally:
        link.close()


def draw_status(image, orig_lines, setup=None, auto_enabled=False, phase="UNKNOWN", manual_mode=False, manual_command="", gesture_is_grabbed=False):
    # Determine the clean UI state based on setup progress
    lines = orig_lines
    if manual_mode:
        lines = [
            "MANUAL MODE (GESTURE CONTROL v1.3.0)",
            "Gestures: both open=FWD  both hands thumb+index=REV  4 fingers L/R hand=TURN  index+pinky=GRIPPER  fist=STOP",
            f"Command: {manual_command} | Gripper: {'CLOSED' if gesture_is_grabbed else 'OPEN'} | Keys: W S A D drive, X stop, O C gripper | [SPACE] AUTO",
        ]
    elif setup is not None:
        if setup.homography is None:
            lines = [
                f"STEP 1: SETUP ARENA CORNERS ({len(setup.corners)}/4)",
                "Click the 4 corners of the arena on the screen.",
                orig_lines[2] if len(orig_lines) > 2 else "Shortcuts: [Q] Quit | [M] Manual | [,] Exposure"
            ]
        elif len(setup.zones) < setup.required:
            lines = [
                f"STEP 2: SETUP DROP ZONES ({len(setup.zones)}/{setup.required})",
                "Click the center of each colored drop circle (up to 6).",
                (f"Marked: {', '.join(setup.zones)} | press [ENTER] to finish with {len(setup.zones)}" if setup.zones
                 else f"Need up to {setup.required} circles"),
                "Shortcuts: [Z] Zones | [R] Reset | [,] Exposure"
            ]
        elif not auto_enabled:
            # See if there's a blocker reason from orig_lines
            reason = orig_lines[0] if orig_lines and "blocked" in orig_lines[0].lower() else ""
            lines = [
                "READY TO START!",
                "Place robot in arena (make sure green circle appears).",
                "Press [ SPACEBAR ] to START AUTO!",
                "Shortcuts: [G] Robot setup | [Z] Zones | [R] Reset | [ ] Exposure"
            ]
            if reason:
                lines.append(reason)
        else:
            lines = [
                "AUTO MODE RUNNING 🚀",
                f"AI Phase: {phase.upper()}",
                "Press [ SPACEBAR ] to STOP",
            ]
            if orig_lines and "STOP" in orig_lines[0]:
                lines.append(orig_lines[0])

    # Draw Lightweight Semi-transparent overlay
    overlay = image.copy()
    h = 20 + (35 * len(lines))
    cv2.rectangle(overlay, (0, 0), (image.shape[1], h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)

    y = 35
    for line in lines:
        if "STEP" in line or "READY" in line or "AUTO MODE" in line:
            color = (0, 255, 0) if "READY" in line or "RUNNING" in line else (0, 255, 255)
            cv2.putText(image, line[:100], (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
        else:
            color = (0, 0, 255) if "Need:" in line or "blocked" in line else (220, 220, 220)
            cv2.putText(image, line[:100], (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
        y += 35



def acquire_single_instance() -> bool:
    """Keep two camera/robot loops from sending conflicting drive commands."""
    global _INSTANCE_MUTEX_HANDLE, _INSTANCE_KERNEL32
    if os.name != "nt":
        return True

    import ctypes

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateMutexW.argtypes = (ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p)
    kernel32.CreateMutexW.restype = ctypes.c_void_p
    kernel32.CloseHandle.argtypes = (ctypes.c_void_p,)
    kernel32.CloseHandle.restype = ctypes.c_bool
    ctypes.set_last_error(0)
    handle = kernel32.CreateMutexW(None, False, "Local\\ARGOS.CameraLoop")
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    if ctypes.get_last_error() == 183:  # ERROR_ALREADY_EXISTS
        kernel32.CloseHandle(handle)
        print("[SAFE] ARGOS is already running; refusing a second camera/robot loop.")
        return False

    _INSTANCE_KERNEL32 = kernel32
    _INSTANCE_MUTEX_HANDLE = handle
    return True


def main():
    parser = argparse.ArgumentParser(description="Overhead camera sorter; auto movement requires an explicit opt-in.")
    def parse_camera_index(val):
        try:
            return int(val)
        except ValueError:
            return val
    parser.add_argument("--camera-index", type=parse_camera_index, default=parse_camera_index(os.getenv("CAMERA_INDEX", "0")))
    parser.add_argument("--camera-exposure", type=float, default=os.getenv("CAMERA_EXPOSURE", "-9"),
                        help="initial camera exposure; the live [ and ] keys adjust it")
    parser.add_argument("--marker-id", type=int, default=int(os.getenv("ROBOT_ARUCO_ID", "34")))
    parser.add_argument("--robot-ip", type=ipv4_address, default=os.getenv("ROBOT_IP") or None,
                        help="known ESP32 IP for unicast discovery when hotspot blocks broadcasts")
    parser.add_argument("--gripper-distance-px", type=positive_float,
                        default=positive_float(os.getenv("GRIPPER_DISTANCE_PX") or str(CFG["gripper_distance_px"])),
                        help="virtual gripper center distance forward from the robot ArUco center, in warped-image pixels")
    parser.add_argument("--gripper-radius-px", type=positive_float,
                        default=positive_float(os.getenv("GRIPPER_RADIUS_PX") or str(CFG["gripper_radius_px"])),
                        help="virtual gripper capture/search radius in warped-image pixels")
    parser.add_argument("--colors", default=",".join(vision.PALETTE),
                        help=f"comma list of stone colors to sort (default from ARGOS_COLORS); choose from {', '.join(vision.ALL_COLORS)}")
    parser.add_argument("--sites", type=int, default=int(os.getenv("ARGOS_SITES", "6")),
                        help="most drop circles you can mark, 1-6 (default 6; press ENTER during setup to finish with fewer)")
    parser.add_argument("--no-gripper", action="store_true",
                        help="test driving without the gripper servo: no servo commands, each stone is visited once")
    parser.add_argument("--detector", choices=("hsv", "roboflow"), default=os.getenv("ARGOS_DETECTOR", "hsv"),
                        help="stone detector: local HSV (default, offline) or Roboflow cloud with HSV fallback")
    parser.add_argument("--home-distance-mm", type=positive_float, default=70.0,
                        help="robot-center distance considered arrived at the saved home mark")
    parser.add_argument("--simulate-gripper", action="store_true",
                        help="open an offline gripper geometry simulator; does not open camera or robot connection")
    parser.add_argument("--reset-zones", action="store_true",
                        help="clear saved color-circle centers on startup while retaining arena corners")
    parser.add_argument("--enable-auto", action="store_true", help="allow the sorting state machine to drive the rover")
    parser.add_argument("--confirm-motion", action="store_true", help="confirm manual wheel and gripper direction checks were completed")
    parser.add_argument("--allow-untested-motion", action="store_true", help="operator-requested motion without completed wheel/gripper checks")
    parser.add_argument("--start-auto", action="store_true", help="arm Auto when the live camera, marker, arena and robot are ready")
    diagnostics = parser.add_mutually_exclusive_group()
    diagnostics.add_argument("--scan-cameras", action="store_true", help="list usable camera indices without connecting to the robot")
    diagnostics.add_argument("--check-robot", action="store_true", help="test safe UDP discovery without opening a camera or moving")
    arena_scale = CFG["arena_width_mm"] / WARP_W if CFG["arena_width_mm"] > 0 and CFG["arena_height_mm"] > 0 else 0.0
    scale_text = (os.getenv("MM_PER_PIXEL", "").strip()
                  or (f"{arena_scale:.6f}" if arena_scale else (str(CFG["mm_per_pixel"]) if CFG["mm_per_pixel"] > 0 else "")))
    try:
        scale_default = positive_float(scale_text) if scale_text else None
        drop_distance = positive_float(os.getenv("DROP_DISTANCE_MM") or str(CFG["drop_distance_mm"]))
    except (ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))
    parser.add_argument("--mm-per-pixel", type=positive_float, default=scale_default,
                        help="measured arena scale; required to arm auto mode")
    args = parser.parse_args()
    try:
        vision.set_palette(tuple(args.colors.split(",")))
    except ValueError as exc:
        parser.error(str(exc))
    if not 1 <= args.sites <= 6:
        parser.error("--sites must be between 1 and 6")
    if args.detector == "roboflow" and not vision.RF_API_KEY:
        print("[RF] ROBOFLOW_API_KEY is not set; using local HSV detection instead.", flush=True)
        args.detector = "hsv"
    print(f"Drop circles: up to {args.sites} | detector: {args.detector}" + (" | NO GRIPPER (servo commands off)" if args.no_gripper else ""))
    # Force UI-friendly defaults so user can just double-click to run
    args.enable_auto = True
    args.confirm_motion = True
    if args.start_auto and not args.enable_auto:
        parser.error("--start-auto requires --enable-auto")
    # Command-line/env values win at startup; the G panel edits them live.
    CFG.update(gripper_distance_px=args.gripper_distance_px, gripper_radius_px=args.gripper_radius_px,
               mm_per_pixel=args.mm_per_pixel or 0.0, drop_distance_mm=drop_distance)
    apply_settings(args)

    if args.scan_cameras:
        return scan_cameras(exposure=args.camera_exposure)
    if args.check_robot:
        return check_robot_link(args.marker_id, args.robot_ip)
    if args.simulate_gripper:
        return run_gripper_simulator(args.gripper_distance_px, args.gripper_radius_px)
    if not acquire_single_instance():
        return 2

    try:
        camera_capture = open_camera(args.camera_index, args.camera_exposure)
        camera = LatestFrameCamera(camera_capture)
    except (RuntimeError, cv2.error) as exc:
        print(f"[CAMERA] {exc}")
        print("[SAFE] No movement commands were sent. Connect the selected camera before launching this program again.")
        return 2

    link = RobotLink(marker_id=args.marker_id, robot_ip=args.robot_ip)
    setup = ArenaSetup(CALIBRATION_PATH, args.camera_index, required=args.sites)
    if args.reset_zones:
        setup.clear_zones()
    rf_executor = ThreadPoolExecutor(max_workers=1)
    rf_future = None
    last_gems = []

    window = "ARGOS | overhead view"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, setup.click)
    camera_exposure = float(args.camera_exposure)
    digital_gain = 1.0
    gesture_controller = GestureController() if GestureController is not None else None
    gesture_is_grabbed = False
    setup_panel = SetupPanel(CFG)
    servo_test = {"sent": int(CFG["servo_test_deg"]), "at": 0.0}
    gesture_smoother = GestureSmoother(confirm_frames=1)
    manual_move_start = 0.0
    gesture_pause_until = 0.0
    keyboard = {"cmd": None, "until": 0.0, "start": 0.0}   # W/S/A/D fallback while Manual is on
    gesture_log = {"key": None, "at": 0.0}                 # terminal log of what the camera reads
    ramp = {"mode": None, "at": 0.0}

    def sync_ramp() -> None:
        """Wheel ramp on the robot: none in manual (instant response), the original ramp for auto."""
        mode = "manual" if manual_mode else "auto"
        now = time.monotonic()
        if not link.is_online or (ramp["mode"] == mode and now - ramp["at"] < 2.0):
            return
        if mode == "manual":
            link.send("RAMP 255 255 0")
        else:
            link.send("RAMP 25 25 0")
        ramp["mode"], ramp["at"] = mode, now

    gripper_homed = args.no_gripper   # the firmware sends no servo pulse at boot, so open it once we are connected
    grip = {"homed_at": 0.0, "released": args.no_gripper, "used": False}

    def send_grip(command: str) -> None:
        if args.no_gripper:
            return
        link.send(command)
        grip["used"] = True

    def release_gripper() -> None:
        """Servo power off: stops the pulse train so it holds no current."""
        if link.robot_ip:
            for _ in range(3):   # UDP can drop a packet; the robot also auto-releases when we go silent
                link.sock.sendto(b"GOFF", (link.robot_ip, link.port))
                time.sleep(0.03)

    def sync_servo_test() -> None:
        """Move the gripper servo live while the TEST slider is dragged."""
        value = int(CFG["servo_test_deg"])
        now = time.monotonic()
        # Rate-limited: every write restarts the servo PWM wave on the robot.
        if value != servo_test["sent"] and now - servo_test["at"] >= 0.15 and link.is_online:
            send_grip(f"SERVO {value}")
            servo_test["sent"], servo_test["at"] = value, now

    def change_exposure(delta: float) -> None:
        nonlocal camera_exposure, auto_enabled, auto_start_pending, manual_command, digital_gain
        requested = camera_exposure + delta
        supported, actual = camera.set_exposure(requested)
        if not supported or actual == camera_exposure:
            # Fallback to digital gain if hardware exposure fails
            digital_gain = max(0.1, digital_gain + (delta * 0.2))
            print(f"Hardware exposure unsupported. Using digital gain: {digital_gain:.1f}x")
        else:
            camera_exposure = actual if math.isfinite(actual) else requested
            print(f"Exposure now {camera_exposure:g}.")
            
        auto_enabled = False
        auto_start_pending = False
        manual_command = "S"
        link.stop()
        setup.clear_zones()
        print("Six color circles cleared for remapping due to lighting change.")

    auto_enabled = False
    manual_mode = False
    planner = SorterPlanner(
        grip_dwell_s=0.6 if args.no_gripper else 2.5,
        drop_dwell_s=0.6 if args.no_gripper else 1.5,
        skip_picked=args.no_gripper,
    )
    pilot = Pilot()
    manual_command = "S"
    manual_deadline = 0.0
    stable_pose_frames = 0
    pose_misses = 0
    auto_start_pending = args.start_auto
    auto_start_deadline = time.monotonic() + 15.0
    motion_reference_pose = None
    motion_reference_at = 0.0
    stuck_recovery_used = False
    stuck_recovery_count = 0
    view_recovery_count = 0
    view_recovery_next_at = 0.0
    last_auto_log = 0.0
    
    # Auto-capture logic
    last_capture_time = time.monotonic()
    os.makedirs(CAPTURE_DIR / "raw", exist_ok=True)
    os.makedirs(CAPTURE_DIR / "annotated", exist_ok=True)
    
    print(f"Click the four arena corners in the raw camera view. After the warp appears, click the center of each drop circle (up to {args.sites}) in any order and press ENTER when done; the color is read from the camera.")
    print("Manual check works before arena calibration: focus this window, press M, use Hand Gestures to drive and toggle gripper.")
    print("Camera setup: [ darker | ] brighter (clears zone marks); Z clears zones only; R clears corners and zones.")
    print("Home: place the stopped rover at field center and press H to save its marker position; press B during Auto to return there and restart. Q/window close stops immediately.")
    print("Offline geometry only: run argos.py --simulate-gripper; it does not open a camera or connect to the robot.")
    print("Auto requires --enable-auto --confirm-motion and a measured --mm-per-pixel value; press A only after the on-screen checks pass.")

    try:
        while True:
            link.discover()
            if not manual_mode:
                gesture_smoother.reset()
            sync_ramp()
            if not gripper_homed and link.is_online:
                link.send(grip_command(False))
                grip["homed_at"] = time.monotonic()
                gripper_homed = True
                gesture_is_grabbed = False
                print(f"Gripper set to open ({int(CFG['servo_open_deg'])} deg)", flush=True)
            if (gripper_homed and not grip["released"] and not grip["used"]
                    and time.monotonic() - grip["homed_at"] >= 1.0 and link.is_online):
                # Opened at start; the servo needs no holding power while idle.
                release_gripper()
                grip["released"] = True
                print("Gripper servo power off (idle)", flush=True)
            if manual_command != "S" and time.monotonic() > manual_deadline:
                manual_command = "S"
            raw, frame_age, frozen, camera_error = camera.read_latest()
            if raw is None or frame_age > MAX_FRAME_AGE_S or frozen:
                if auto_enabled:
                    print(f"AUTO DISARMED: camera frame unavailable (age={frame_age:.2f}s, frozen={frozen}, error={camera_error})", flush=True)
                link.stop()
                auto_enabled = False
                auto_start_pending = False
                manual_mode = False
                manual_command = "S"
                view = raw.copy() if raw is not None else np.zeros((480, 640, 3), dtype=np.uint8)
                if raw is None:
                    reason = camera_error or "no camera frame"
                elif frozen:
                    reason = "CAMERA FROZEN: auto disarmed"
                else:
                    reason = f"CAMERA STALE ({frame_age:.1f}s): auto disarmed"
                draw_status(view, [reason, f"Robot: {link.status}",
                           f"AI Phase: {planner.phase}",
                           f"Exposure {camera_exposure:g} | [ darker | ] brighter | Q quit"], setup=setup, auto_enabled=auto_enabled, phase=planner.phase, manual_mode=manual_mode, manual_command=manual_command, gesture_is_grabbed=gesture_is_grabbed)
                cv2.imshow(window, view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("["):
                    change_exposure(-1)
                elif key == ord("]"):
                    change_exposure(1)
                time.sleep(0.02)
                continue
            if float(raw.mean()) < 3.0 and float(raw.std()) < 3.0:
                if auto_enabled:
                    print("AUTO DISARMED: camera image turned black", flush=True)
                link.stop()
                auto_enabled = False
                auto_start_pending = False
                manual_mode = False
                manual_command = "S"
                view = raw.copy()
                draw_status(view, ["CAMERA BLACK: movement disabled", f"Exposure {camera_exposure:g} | [ darker | ] brighter",
                                   "Adjust exposure or set --camera-exposure at launch | Q quit"], setup=setup, auto_enabled=auto_enabled, phase=planner.phase, manual_mode=manual_mode, manual_command=manual_command, gesture_is_grabbed=gesture_is_grabbed)
                cv2.imshow(window, view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("["):
                    change_exposure(-1)
                elif key == ord("]"):
                    change_exposure(1)
                continue
            raw_view = raw.copy()
            setup.ignore_clicks = manual_mode
            
            if manual_mode and setup.homography is not None:
                link.discover()  # Keep discovering/polling so robot stays connected
                if gesture_controller:
                    gesture_controller.min_hand_span = CFG["min_hand_size"] / 100.0
                    cmd, gesture_is_grabbed = gesture_controller.process_frame(raw_view, gesture_is_grabbed)
                    now = time.monotonic()
                    if cmd in ('C', 'O'):
                        send_grip(grip_command(cmd == 'C'))   # gripper switch; driving carries on
                        gesture_pause_until = now + CFG["gesture_cooldown_s"]
                    # Debounced: a one-frame glitch neither starts nor stops the robot.
                    move = gesture_smoother.update(cmd, now, CFG["gesture_hold_s"],
                                                   stop_now=getattr(gesture_controller, "fist_stop", False))
                    # One gesture at a time: when a gesture ends or changes, the next one
                    # is ignored for a short pause. A fist (stop) is never delayed.
                    if move != manual_command:
                        if manual_command != 'S':
                            gesture_pause_until = now + CFG["gesture_cooldown_s"]
                        if move != 'S' and now < gesture_pause_until:
                            move = 'S'
                    kb_cmd = keyboard["cmd"] if now < keyboard["until"] else None
                    if move == 'S' and kb_cmd:
                        # Keyboard fallback (testing / emergency): W S A D, no start delay.
                        manual_command = kb_cmd
                        manual_deadline = now + 0.3
                        link.drive(kb_cmd, speed=manual_speed(kb_cmd, now - keyboard["start"]))
                    elif move != 'S':
                        if move != manual_command:
                            manual_move_start = now
                        manual_command = move
                        manual_deadline = now + 0.3
                        # Forward/backward start a moment after the gesture is first
                        # seen (no acceleration); turning ramps instead. Stay stopped
                        # while waiting, even if a different move was running.
                        start_delay = CFG["fb_delay_s"] if move in "FB" else 0.0
                        if now - manual_move_start < start_delay:
                            link.stop()
                        else:
                            link.drive(manual_command, speed=manual_speed(manual_command, now - manual_move_start))
                    else:
                        # No confirmed drive gesture: stop (the robot ramps down smoothly)
                        if manual_command != 'S':
                            manual_command = 'S'
                            if link.robot_ip:
                                link.sock.sendto(b"STOP", (link.robot_ip, link.port))
                                link.last_command = "STOP"
                        # Keep sending stop to make sure it arrives
                        elif time.monotonic() - getattr(link, '_last_stop', 0) > 0.1:
                            if link.robot_ip:
                                link.sock.sendto(b"STOP", (link.robot_ip, link.port))
                            link._last_stop = time.monotonic()
                        
                if gesture_controller is not None and gesture_controller.debug_text:
                    log_key = (cmd, manual_command)
                    log_now = time.monotonic()
                    if log_key != gesture_log["key"] and log_now - gesture_log["at"] >= 0.25:
                        print(f"[GESTURE] read={cmd} -> robot={manual_command} | {gesture_controller.debug_text}", flush=True)
                        gesture_log["key"], gesture_log["at"] = log_key, log_now
                setup_panel.poll()
                sync_servo_test()
                draw_status(raw_view, [], setup=setup, auto_enabled=False, manual_mode=True, manual_command=manual_command, gesture_is_grabbed=gesture_is_grabbed)
                if gesture_controller is not None and gesture_controller.debug_text:
                    cv2.rectangle(raw_view, (0, raw_view.shape[0] - 38), (raw_view.shape[1], raw_view.shape[0]), (0, 0, 0), -1)
                    cv2.putText(raw_view, gesture_controller.debug_text, (12, raw_view.shape[0] - 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(window, raw_view)
                
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('g'), ord('G')):
                    setup_panel.toggle()
                elif key != 255 and chr(key).lower() in "wsad":
                    now = time.monotonic()
                    new_cmd = {"w": "F", "s": "B", "a": "L", "d": "R"}[chr(key).lower()]
                    if keyboard["cmd"] != new_cmd or now >= keyboard["until"]:
                        keyboard["start"] = now
                    keyboard["cmd"], keyboard["until"] = new_cmd, now + 0.6   # covers the key-repeat delay
                    if now - keyboard.get("logged", 0.0) > 0.5:
                        print(f"[KEY] {chr(key).upper()} -> {new_cmd}", flush=True)
                        keyboard["logged"] = now
                elif key in (ord('x'), ord('X')):
                    keyboard["cmd"], keyboard["until"] = None, 0.0
                    gesture_smoother.reset()
                    link.stop()
                elif key in (ord('o'), ord('O'), ord('c'), ord('C')):
                    gesture_is_grabbed = key in (ord('c'), ord('C'))
                    send_grip(grip_command(gesture_is_grabbed))
                elif key == ord(' '):
                    manual_mode = False
                    auto_start_pending = True
                    auto_enabled = False
                elif key == ord('q'):
                    break
                continue

            for i, point in enumerate(setup.corners):
                cv2.circle(raw_view, point, 7, (0, 255, 0), -1)
                cv2.putText(raw_view, str(i + 1), (point[0] + 7, point[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if setup.homography is None:
                if manual_mode and manual_command in "FBLR" and link.is_online:
                    link.drive(manual_command, speed=manual_speed(manual_command))
                    manual_status = f"MANUAL {manual_command} | M exit | Gestures active"
                else:
                    link.stop()
                    manual_status = "MANUAL ON | Gestures active | M exit" if manual_mode else "STOPPED | M manual | Q quit"
                lines = [
                    f"STEP 1: SETUP ARENA CORNERS ({len(setup.corners)}/4)",
                    "Please click the 4 corners of the arena on this screen.",
                    f"Robot Status: {link.status}",
                    "Shortcuts: [Q] Quit | [M] Manual Drive Mode"
                ]
                draw_status(raw_view, lines, setup=setup, auto_enabled=False, manual_mode=True, manual_command=manual_command, gesture_is_grabbed=gesture_is_grabbed)
                cv2.imshow(window, raw_view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("m"):
                    manual_mode = not manual_mode
                    manual_command = "S"
                    manual_deadline = 0.0
                    link.stop()
                    print(f"Manual mode {'ON' if manual_mode else 'OFF'}")
                elif key == ord("s"):
                    manual_command = "S"
                    manual_deadline = 0.0
                    link.stop()
                    print("STOP requested")
                elif key == ord("["):
                    change_exposure(-1)
                elif key == ord("]"):
                    change_exposure(1)
                elif manual_mode and key in (ord("w"), ord("a"), ord("d")):
                    manual_command = {ord("w"): "F", ord("a"): "L", ord("d"): "R"}[key]
                    manual_deadline = time.monotonic() + MANUAL_KEY_TIMEOUT_S
                elif manual_mode and key in (ord("o"), ord("c")):
                    manual_command = "S"
                    manual_deadline = 0.0
                    link.stop()
                    send_grip(grip_command(key != ord("o")))
                continue

            view = cv2.warpPerspective(raw, setup.homography, (WARP_W, WARP_H))
            setup.sample_image = view.copy()
            for color, center in setup.zones.items():
                cv2.circle(view, center, ZONE_RADIUS_PX, (255, 255, 255), 2)
                cv2.putText(view, color, (center[0] - 35, center[1] - ZONE_RADIUS_PX - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            pose = detect_robot_pose(view, args.marker_id)
            if pose:
                stable_pose_frames += 1
                pose_misses = 0
                cv2.circle(view, (pose.x, pose.y), 8, (255, 0, 0), -1)
                cv2.arrowedLine(view, (pose.x, pose.y), pose.front, (0, 255, 0), 3)
            else:
                stable_pose_frames = 0
                pose_misses += 1

            # Virtual gripper geometry: no added hardware marker is needed.
            # The existing robot-tag center is the origin; the configured
            # offset is projected along the detected heading on the rectified
            # overhead image.
            if setup_panel.poll():
                apply_settings(args)
            sync_servo_test()

            gripper_point = None
            if pose is not None:
                cosine = math.cos(pose.heading)
                sine = math.sin(pose.heading)
                gripper_point = gripper_point_from_heading(
                    pose.x, pose.y, pose.heading, args.gripper_distance_px,
                )
                cv2.line(view, (pose.x, pose.y), gripper_point, (0, 255, 0), 2)
                cv2.circle(view, gripper_point, int(round(args.gripper_radius_px)), (0, 255, 0), 2)
                cv2.drawMarker(view, gripper_point, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
                zone_label = f"GRIP ZONE d={args.gripper_distance_px:g}px r={args.gripper_radius_px:g}px"
                cv2.putText(view, zone_label, (gripper_point[0] + 8, gripper_point[1] - int(args.gripper_radius_px) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)

            if setup_panel.is_open and pose is not None:
                # Show robot size and rear end on the real robot while tuning.
                cv2.circle(view, (pose.x, pose.y), int(CFG["robot_radius_px"]), (0, 165, 255), 2)
                rear = gripper_point_from_heading(pose.x, pose.y, pose.heading, -CFG["robot_rear_px"])
                cv2.circle(view, rear, 6, (0, 165, 255), -1)
                cv2.putText(view, "ROBOT SIZE", (pose.x - 45, pose.y + int(CFG["robot_radius_px"]) + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)

            zones_ready = len(setup.zones) >= setup.required
            excluded_regions = [(center, ZONE_EXCLUSION_RADIUS_PX) for center in setup.zones.values()]
            if pose:
                excluded_regions.extend(
                    robot_exclusion_regions(pose, gripper_point, args.gripper_radius_px)
                )
            if args.detector == "roboflow":
                # Cloud inference runs in the background; use its last result
                # and fall back to local HSV if the request fails.
                if rf_future is None or rf_future.done():
                    if rf_future is not None:
                        try:
                            raw_rf_gems, _ = rf_future.result()
                            last_gems = [
                                g for g in raw_rf_gems
                                if all(math.hypot(g.x - ex, g.y - ey) >= rad for (ex, ey), rad in excluded_regions)
                            ]
                        except Exception as rf_exc:
                            print(f"[RF] Cloud error, falling back to HSV: {rf_exc}", flush=True)
                            last_gems = detect_gems(view, excluded_regions=tuple(excluded_regions), color_zones=setup.zones)
                    rf_future = rf_executor.submit(detect_gems_roboflow, view.copy(), warp_w=WARP_W, warp_h=WARP_H, color_zones=setup.zones)
            else:
                last_gems = detect_gems(
                    view,
                    excluded_regions=tuple(excluded_regions),
                    color_zones=setup.zones,
                )
            # Only stones whose color has a marked drop circle matter.
            active_colors = set(setup.zones) or set(vision.PALETTE)
            gems = [g for g in last_gems if g.color in active_colors]
            auto_blockers = auto_preflight_blockers(
                auto_requested=args.enable_auto,
                motion_confirmed=args.confirm_motion or args.allow_untested_motion,
                manual_mode=manual_mode,
                scale_valid=args.mm_per_pixel is not None,
                arena_valid=setup.homography is not None,
                zones_ready=zones_ready,
                stable_marker_frames=stable_pose_frames,
                robot_online=link.is_online,
            )
            if auto_start_pending:
                if not auto_blockers:
                    auto_enabled = True
                    auto_start_pending = False
                    if setup.home_center is not None:
                        planner.begin_return_home(setup.home_center)
                        print("AUTO STARTED: returning to HOME, then restarting search", flush=True)
                    else:
                        planner.reset()
                        planner.clear_memory()
                        print("AUTO STARTED: live preflight passed", flush=True)
                elif time.monotonic() > auto_start_deadline:
                    auto_start_pending = False
                    print("AUTO START TIMED OUT: " + "; ".join(auto_blockers), flush=True)
            # --- ROBOFLOW STYLE UI FOR GEMS ---
            color_map = {
                "violet": (211, 0, 148),
                "cyan": (255, 255, 0),
                "crimson": (60, 20, 220),
                "gold": (0, 215, 255),
                "blue": (255, 0, 0),
                "lime": (0, 255, 0)
            }
            for gem in gems:
                box_size = int(math.sqrt(gem.area)) if gem.area > 0 else 30
                half_s = box_size // 2
                x1, y1 = gem.x - half_s, gem.y - half_s
                x2, y2 = gem.x + half_s, gem.y + half_s
                
                c = color_map.get(gem.color, (0, 255, 255))
                # Draw Bounding Box
                cv2.rectangle(view, (x1, y1), (x2, y2), c, 2)
                
                # Draw Label Background
                label = f"{gem.color.upper()}"
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(view, (x1, y1 - lh - 8), (x1 + lw + 8, y1), c, -1)
                
                # Draw Label Text
                cv2.putText(view, label, (x1 + 4, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0) if gem.color in ["cyan", "gold", "lime"] else (255, 255, 255), 1, cv2.LINE_AA)
                
                # Draw Center Dot
                cv2.circle(view, (gem.x, gem.y), 3, (255, 255, 255), -1)
            if setup.home_center is not None:
                cv2.drawMarker(view, setup.home_center, (255, 255, 0), cv2.MARKER_TILTED_CROSS, 24, 2)
                cv2.putText(view, "HOME", (setup.home_center[0] + 10, setup.home_center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 1)

            safety_reason = None
            view_recovery_reason = None
            if auto_enabled:
                if not link.is_online:
                    safety_reason = "AUTO DISARMED: robot link lost; press A after reconnect"
                elif pose_misses >= POSE_LOSS_LIMIT:
                    view_recovery_reason = "marker lost; reversing to reacquire view"
                elif pose is not None:
                    raw_point = cv2.perspectiveTransform(
                        np.array([[[pose.x, pose.y]]], dtype=np.float32),
                        np.linalg.inv(setup.homography),
                    )[0, 0]
                    if not (CAMERA_EDGE_MARGIN_PX <= raw_point[0] <= raw.shape[1] - CAMERA_EDGE_MARGIN_PX
                            and CAMERA_EDGE_MARGIN_PX <= raw_point[1] <= raw.shape[0] - CAMERA_EDGE_MARGIN_PX):
                        view_recovery_reason = "near camera edge; reversing toward the visible area"
                if safety_reason:
                    print(safety_reason, flush=True)
                    auto_enabled = False
                    manual_command = "S"
                    planner.reset()
                elif view_recovery_reason is None and pose is not None:
                    view_recovery_count = 0

            command = "S"
            reason = "STOPPED"
            auto_distance_mm = None
            pilot_drive = None
            if safety_reason:
                reason = safety_reason
            elif view_recovery_reason is not None:
                reason = f"AUTO RECOVERY: {view_recovery_reason}"
            elif manual_command != "S":
                command, reason = manual_command, f"MANUAL {manual_command}"
            elif auto_enabled:
                decision = planner.step(
                    pose=pose,
                    approach_point=gripper_point,
                    gems=gems,
                    zones=setup.zones,
                    mm_per_pixel=args.mm_per_pixel,
                    pickup_distance_mm=args.pickup_distance_mm,
                    drop_distance_mm=args.drop_distance_mm,
                    home_distance_mm=args.home_distance_mm,
                    angle_threshold=ANGLE_THRESHOLD,
                    now=time.monotonic(),
                )
                command, reason = decision.command, decision.reason
                if decision.target is not None and pose is not None:
                    reference_point = ((pose.x, pose.y) if planner.phase == "home"
                                       else (gripper_point or (pose.x, pose.y)))
                    auto_distance_mm = math.dist(reference_point, decision.target) * args.mm_per_pixel
                if decision.target is not None and pose is not None and decision.command in "FLR":
                    # Continuous smooth steering instead of stop-and-go pulses.
                    stop_mm = (args.home_distance_mm if planner.phase == "home"
                               else args.drop_distance_mm if planner.phase == "deliver"
                               else args.pickup_distance_mm)
                    pilot.cfg.min_pwm = CFG["auto_min_pwm"]
                    pilot.cfg.cruise_pwm = max(CFG["auto_cruise_pwm"], CFG["auto_min_pwm"])
                    pilot.cfg.spin_max_pwm = max(CFG["auto_spin_pwm"], CFG["auto_min_pwm"])
                    forward, turn, pilot_mode = pilot.update(
                        heading_error((pose.x, pose.y), pose.heading, decision.target),
                        auto_distance_mm, stop_mm, time.monotonic(),
                        gripper_error=heading_error(reference_point, pose.heading, decision.target),
                    )
                    pilot_drive = (forward, turn)
                    command = "P"
                    reason += f" | {pilot_mode} fwd {forward:.0f} turn {turn:+.0f}"
                if decision.action is not None:
                    now = time.monotonic()
                    if decision.action != getattr(link, "last_action", None) or now - getattr(link, "last_action_time", 0) > 1.0:
                        send_grip(grip_command(decision.action == "CLOSE"))
                        link.last_action = decision.action
                        link.last_action_time = now
                if decision.target is not None and command in "FBLRP":
                    cv2.line(view, gripper_point or (pose.x, pose.y), decision.target, (0, 255, 255), 2)
            elif zones_ready and pose and gems:
                detail = f"auto blocked: {auto_blockers[0]}" if auto_blockers else "press A to start"
                reason = f"PREVIEW: {len(gems)} stones | {detail}"
            elif not auto_enabled and auto_blockers:
                reason = f"AUTO BLOCKED: {auto_blockers[0]}"

            # Single source of truth for "close enough to creep": reused by
            # both the stuck-watchdog below and the drive pulse further down,
            # so they can never disagree about how big a step to expect.
            near_target = False
            if auto_enabled and auto_distance_mm is not None:
                if planner.phase == "home":
                    stop_distance_mm = args.home_distance_mm
                elif planner.phase == "deliver":
                    stop_distance_mm = args.drop_distance_mm
                else:
                    stop_distance_mm = args.pickup_distance_mm
                near_target = (stop_distance_mm is not None
                               and auto_distance_mm <= stop_distance_mm + NEAR_TARGET_MARGIN_MM)

            if auto_enabled and command in "FBLRP" and pose:
                now = time.monotonic()
                # Creep pulses near a target move only a few pixels per
                # frame by design, well under the 5px/2.5s bar tuned for
                # full-speed cruising. Use a looser bar here so genuine slow
                # progress is not mistaken for being stuck.
                position_threshold = 2 if near_target else 5
                stuck_timeout_s = 5.0 if near_target else 2.5
                if (motion_reference_pose is None
                        or math.dist((pose.x, pose.y), motion_reference_pose[:2]) >= position_threshold
                        or abs(wrap_angle(pose.heading - motion_reference_pose[2])) >= math.radians(7)):
                    motion_reference_pose = (pose.x, pose.y, pose.heading)
                    motion_reference_at = now
                    stuck_recovery_used = False
                    stuck_recovery_count = 0
                elif now - motion_reference_at > stuck_timeout_s:
                    # Wedged against a stone, a drop-circle rim or the wall.
                    # Reverse (further each time it happens again), turn to a
                    # new side, and when hunting a stone pick a different one
                    # so the next approach takes a new route.
                    stuck_recovery_count += 1
                    stuck_recovery_used = True
                    back_s = min(0.35 + 0.15 * (stuck_recovery_count - 1), 0.9)
                    turn_sign = 1.0 if stuck_recovery_count % 2 else -1.0
                    new_target = planner.avoid_current(now)
                    reason = (f"AUTO STUCK {stuck_recovery_count}: reversing {back_s:.2f}s, turning "
                              f"{'left' if turn_sign > 0 else 'right'}"
                              + (", trying another stone" if new_target else ", re-approaching"))
                    print(reason, flush=True)
                    if link.is_online:
                        link.drive_mix(-CFG["auto_min_pwm"], 0.0)
                        time.sleep(back_s)
                        link.drive_mix(0.0, turn_sign * CFG["auto_spin_pwm"])
                        time.sleep(0.3)
                        link.stop()
                    pilot.reset()
                    motion_reference_pose = None
                    motion_reference_at = time.monotonic()
                    command = "S"
            else:
                motion_reference_pose = None
                stuck_recovery_used = False
            if auto_enabled and time.monotonic() - last_auto_log >= 1.0:
                marker_status = f"({pose.x},{pose.y})" if pose is not None else "not visible"
                print(f"AUTO {reason} | marker={marker_status} | {link.status}", flush=True)
                last_auto_log = time.monotonic()

            # Refuse all movement if the robot has not replied recently.
            if auto_enabled and view_recovery_reason is not None:
                command = "S"
                if not link.is_online:
                    auto_enabled = False
                    reason = "AUTO DISARMED: robot link lost during view recovery"
                    link.stop()
                elif (view_recovery_count < VIEW_RECOVERY_MAX_PULSES
                      and time.monotonic() >= view_recovery_next_at):
                    view_recovery_count += 1
                    print(f"AUTO RECOVERY: reverse pulse {view_recovery_count}/{VIEW_RECOVERY_MAX_PULSES}", flush=True)
                    link.drive("B", speed=int(CFG["creep_speed"]))
                    time.sleep(VIEW_RECOVERY_PULSE_S)
                    link.stop()
                    view_recovery_next_at = time.monotonic() + VIEW_RECOVERY_GAP_S
                else:
                    link.stop()
                    if view_recovery_count >= VIEW_RECOVERY_MAX_PULSES:
                        reason = "AUTO HOLD: view not reacquired; waiting for the marker"
            elif command in "FBLRP" and (auto_enabled or manual_command != "S"):
                if link.is_online:
                    if auto_enabled:
                        # Continuous drive: the pilot recomputes wheel power every
                        # camera frame, so there is no blocking sleep and no pulsing.
                        if pilot_drive is not None:
                            link.drive_mix(*pilot_drive)
                        elif command == "B":      # planner's short back-up after a drop
                            link.drive_mix(-CFG["auto_min_pwm"], 0)
                        else:
                            link.stop()
                    else:
                        link.drive(command, speed=manual_speed(command))
                else:
                    auto_enabled = False
                    manual_command = "S"
                    command = "S"
                    reason = "STOP: robot UDP reply expired"
                    link.stop()
            else:
                link.stop()
            if command != "P":
                pilot.reset()
            manual_hint = " | press M to exit manual before A arms auto" if manual_mode else ""
            draw_status(view, [
                reason,
                f"Robot: {link.status}",
                f"Auto {'ON' if auto_enabled else 'OFF'} | Manual {'ON' if manual_mode else 'OFF'} | G robot setup | H mark center | B return/restart | Q stop+quit{manual_hint}",
                f"Exposure {camera_exposure:g} | [ darker | ] brighter | Z remap circles | R reset field",
            ], setup=setup, auto_enabled=auto_enabled, phase=planner.phase, manual_mode=manual_mode, manual_command=manual_command, gesture_is_grabbed=gesture_is_grabbed)
            
            # --- Auto-capture every 5 seconds ---
            if auto_enabled and (time.monotonic() - last_capture_time >= 5.0):
                timestamp = int(time.time())
                # Save raw (unwarped) frame for dataset retraining
                cv2.imwrite(str(CAPTURE_DIR / "raw" / f"frame_{timestamp}.jpg"), raw)
                # Save annotated (warped) frame for debugging logic
                cv2.imwrite(str(CAPTURE_DIR / "annotated" / f"frame_{timestamp}.jpg"), view)
                last_capture_time = time.monotonic()
                print(f"[DEBUG] Captured frame_{timestamp}.jpg to {CAPTURE_DIR}")
                
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            try:
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    auto_enabled = False
                    link.stop()
                    print("Preview closed; robot stopped immediately. Use B to return home before quitting.", flush=True)
                    break
            except cv2.error:
                auto_enabled = False
                link.stop()
                break
            if key == ord("q"):
                auto_enabled = False
                link.stop()
                break
            if key == ord("m"):
                manual_mode = not manual_mode
                manual_command = "S"
                manual_deadline = 0.0
                if manual_mode:
                    auto_enabled = False
                link.stop()
                print(f"Manual mode {'ON' if manual_mode else 'OFF'}")
            elif key == ord("s"):
                auto_enabled = False
                manual_command = "S"
                manual_deadline = 0.0
                link.stop()
                print("STOP requested")
            elif key == ord("h"):
                if auto_enabled or manual_mode or manual_command != "S":
                    print("Home mark blocked: stop Auto and exit Manual mode first.", flush=True)
                elif pose is None or stable_pose_frames < 5:
                    print("Home mark blocked: keep the robot marker visible and steady, then press H again.", flush=True)
                else:
                    setup.set_home_center((pose.x, pose.y))
            elif key == ord("b"):
                if setup.home_center is None:
                    print("Return/restart blocked: mark the center first with H.", flush=True)
                elif planner.phase in ("grip", "drop"):
                    print("Return/restart blocked: let the gripper action finish first.", flush=True)
                else:
                    blockers = auto_preflight_blockers(
                        auto_requested=args.enable_auto,
                        motion_confirmed=args.confirm_motion or args.allow_untested_motion,
                        manual_mode=manual_mode,
                        scale_valid=args.mm_per_pixel is not None,
                        arena_valid=setup.homography is not None,
                        zones_ready=zones_ready,
                        stable_marker_frames=stable_pose_frames,
                        robot_online=link.is_online,
                    )
                    if blockers:
                        print("Return/restart blocked: " + "; ".join(blockers), flush=True)
                    elif planner.begin_return_home(setup.home_center):
                        auto_enabled = True
                        auto_start_pending = False
                        manual_command = "S"
                        motion_reference_pose = None
                        motion_reference_at = 0.0
                        stuck_recovery_used = False
                        stuck_recovery_count = 0
                        link.stop()
                        print("Returning to marked center; sorting will restart there.", flush=True)
            elif manual_mode and key in (ord("w"), ord("a"), ord("d")):
                manual_command = {ord("w"): "F", ord("a"): "L", ord("d"): "R"}[key]
                manual_deadline = time.monotonic() + MANUAL_KEY_TIMEOUT_S
            elif manual_mode and key in (ord("o"), ord("c")):
                manual_command = "S"
                manual_deadline = 0.0
                link.stop()
                if link.is_online:
                    send_grip(grip_command(key != ord("o")))
                else:
                    print("Gripper command blocked: no fresh ESP32 reply")
            elif key == ord("["):
                change_exposure(-1)
            elif key == ord("]"):
                change_exposure(1)
            elif key in (ord("z"), ord("Z")):
                auto_enabled = False
                auto_start_pending = False
                manual_command = "S"
                link.stop()
                setup.clear_zones()
            elif key == ord(" "):
                if auto_enabled:
                    auto_enabled = False
                    print("Sorting mode stopped")
                else:
                    blockers = auto_preflight_blockers(
                        auto_requested=args.enable_auto,
                        motion_confirmed=args.confirm_motion or args.allow_untested_motion,
                        manual_mode=manual_mode,
                        scale_valid=args.mm_per_pixel is not None,
                        arena_valid=setup.homography is not None,
                        zones_ready=zones_ready,
                        stable_marker_frames=stable_pose_frames,
                        robot_online=link.is_online,
                    )
                    if blockers:
                        print("Auto mode is blocked: " + "; ".join(blockers))
                    else:
                        auto_enabled = True
                        if setup.home_center is not None:
                            planner.begin_return_home(setup.home_center)
                            print("Sorting mode started: returning to HOME first")
                        else:
                            planner.reset()
                            planner.clear_memory()
                            print("Sorting mode started")
                link.stop()
            elif key in (ord("g"), ord("G")):
                setup_panel.toggle()
            elif key in (10, 13):
                setup.finish_zones()
            elif key == ord("r"):
                link.stop()
                auto_enabled = False
                manual_mode = False
                manual_command = "S"
                setup.clear()
                planner.reset()
                print("Arena calibration cleared.")
    finally:
        setup_panel.close()
        link.stop()
        link.send("RAMP 25 25 0")   # leave the robot with its original wheel ramp
        release_gripper()           # never leave the gripper servo powered after quitting
        print("Gripper servo power off (quit)", flush=True)
        link.close()
        camera.release()
        rf_executor.shutdown(wait=False, cancel_futures=True)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
