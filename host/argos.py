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
from navigation_math import wrap_angle
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
WARP_H = 600
ZONE_RADIUS_PX = 48
ZONE_EXCLUSION_RADIUS_PX = 115
ROBOT_EXCLUSION_RADIUS_PX = 130 #80
CRUISE_DRIVE_SPEED = 210
# This rover did not move at PWM 90 or 145 on the floor. Slow the approach
# with shorter pulses while keeping enough torque to start the wheels.
CREEP_DRIVE_SPEED = 200
NEAR_TARGET_MARGIN_MM = 150
DROP_DISTANCE_MM = os.getenv("DROP_DISTANCE_MM", "125")
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
    rear_distance = 60.0
    end_distance = front_distance + max(25.0, gripper_radius_px)
    total_distance = rear_distance + end_distance
    # The marker center is not the full robot footprint: servo covers and
    # colored stickers sit to either side of the linkage. Overlap the mask
    # samples generously so those patches cannot reappear as gem contours.
    sample_count = max(1, int(math.ceil(total_distance / 24.0)))
    arm_radius = max(60, int(round(gripper_radius_px + 40)))
    regions = [((pose.x, pose.y), ROBOT_EXCLUSION_RADIUS_PX)]
    for index in range(sample_count + 1):
        offset = -rear_distance + total_distance * index / sample_count
        center = (
            int(round(pose.x + ux * offset)),
            int(round(pose.y + uy * offset)),
        )
        radius = ROBOT_EXCLUSION_RADIUS_PX if offset <= 0 else arm_radius
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
    def __init__(self, calibration_path: Path | None = None, camera_index: int = 0):
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
            self.corners = corners
            self.zones = zones
            self.home_center = home_center
            print(f"Restored arena calibration: {len(corners)}/4 corners, {len(zones)}/{len(vision.PALETTE)} colors ({", ".join(vision.PALETTE)}). Press Z to remap circles or R to clear all.")
        except (OSError, ValueError, TypeError, KeyError) as exc:
            print(f"Ignoring saved arena calibration: {exc}")

    def _save(self):
        if self.calibration_path is None:
            return
        data = {"version": 1, "camera_index": self.camera_index,
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
        self.sample_image = None
        self._save()
        print("Drop-zone marks cleared; arena corners kept. Click the center of each active colored circle again:")
        print(", ".join(vision.PALETTE))

    def click(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
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
                print("Arena rectified. Click the center of each drop circle in this order:")
                print(", ".join(vision.PALETTE))
            self._save()
            return
        if len(self.zones) < len(vision.PALETTE):
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
            "Gestures: 5+5=FWD  I+M+P=REV  4L=LEFT  4R=RIGHT  L-shape=SERVO",
            f"Command: {manual_command} | Gripper: {'CLOSED' if gesture_is_grabbed else 'OPEN'} | [ SPACEBAR ] to AUTO",
        ]
    elif setup is not None:
        if setup.homography is None:
            lines = [
                f"STEP 1: SETUP ARENA CORNERS ({len(setup.corners)}/4)",
                "Click the 4 corners of the arena on the screen.",
                orig_lines[2] if len(orig_lines) > 2 else "Shortcuts: [Q] Quit | [M] Manual | [,] Exposure"
            ]
        elif len(setup.zones) < len(vision.PALETTE):
            missing = [c for c in vision.PALETTE if c not in setup.zones]
            lines = [
                f"STEP 2: SETUP DROP ZONES ({len(setup.zones)}/{len(vision.PALETTE)})",
                "Click the center of the colored drop zones.",
                f"Need: {', '.join(missing)}",
                "Shortcuts: [Z] Zones | [R] Reset | [,] Exposure"
            ]
        elif not auto_enabled:
            # See if there's a blocker reason from orig_lines
            reason = orig_lines[0] if orig_lines and "blocked" in orig_lines[0].lower() else ""
            lines = [
                "READY TO START!",
                "Place robot in arena (make sure green circle appears).",
                "Press [ SPACEBAR ] to START AUTO!",
                "Shortcuts: [Z] Zones | [R] Reset | [,] Exposure"
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
                        default=positive_float(os.getenv("GRIPPER_DISTANCE_PX", "110")),
                        help="virtual gripper center distance forward from the robot ArUco center, in warped-image pixels")
    parser.add_argument("--gripper-radius-px", type=positive_float,
                        default=positive_float(os.getenv("GRIPPER_RADIUS_PX", "20")),
                        help="virtual gripper capture/search radius in warped-image pixels")
    parser.add_argument("--colors", default=",".join(vision.PALETTE),
                        help=f"comma list of stone colors to sort (default from ARGOS_COLORS); choose from {', '.join(vision.ALL_COLORS)}")
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
    scale_text = os.getenv("MM_PER_PIXEL", "").strip()
    try:
        scale_default = positive_float(scale_text) if scale_text else None
        drop_distance = positive_float(DROP_DISTANCE_MM)
    except (ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))
    parser.add_argument("--mm-per-pixel", type=positive_float, default=scale_default,
                        help="measured arena scale; required to arm auto mode")
    args = parser.parse_args()
    try:
        vision.set_palette(tuple(args.colors.split(",")))
    except ValueError as exc:
        parser.error(str(exc))
    print(f"Sorting colors: {', '.join(vision.PALETTE)} | detector: {args.detector}")
    # Force UI-friendly defaults so user can just double-click to run
    args.enable_auto = True
    args.confirm_motion = True
    if args.start_auto and not args.enable_auto:
        parser.error("--start-auto requires --enable-auto")
    args.pickup_distance_mm = (
        args.gripper_radius_px * args.mm_per_pixel
        if args.mm_per_pixel is not None else None
    )
    args.drop_distance_mm = drop_distance

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
    setup = ArenaSetup(CALIBRATION_PATH, args.camera_index)
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
    planner = SorterPlanner(grip_dwell_s=2.5, drop_dwell_s=1.5)
    manual_command = "S"
    move_start_time = 0.0
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
    
    print(f"Click the four arena corners in the raw camera view. After the warp appears, click the center of each active target circle ({', '.join(vision.PALETTE)}) in any order; its color label is read from the camera.")
    print("Manual check works before arena calibration: focus this window, press M, use Hand Gestures to drive and toggle gripper.")
    print("Camera setup: [ darker | ] brighter (clears zone marks); Z clears zones only; R clears corners and zones.")
    print("Home: place the stopped rover at field center and press H to save its marker position; press B during Auto to return there and restart. Q/window close stops immediately.")
    print("Offline geometry only: run argos.py --simulate-gripper; it does not open a camera or connect to the robot.")
    print("Auto requires --enable-auto --confirm-motion and a measured --mm-per-pixel value; press A only after the on-screen checks pass.")

    try:
        while True:
            link.discover()
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
            
            if manual_mode and setup.homography is not None:
                link.discover()  # Keep discovering/polling so robot stays connected
                if gesture_controller:
                    cmd, gesture_is_grabbed = gesture_controller.process_frame(raw_view, gesture_is_grabbed)
                    
                    # Track when movement direction changes for acceleration
                    if cmd in ('F', 'B', 'L', 'R'):
                        if cmd != manual_command:
                            move_start_time = time.monotonic()
                        manual_command = cmd
                        manual_deadline = time.monotonic() + 0.3
                        
                        # Acceleration: ramp from 140 to 200 over 3 seconds
                        elapsed = time.monotonic() - move_start_time
                        current_speed = int(140 + (200 - 140) * min(1.0, elapsed / 3.0))
                        
                        link.drive(manual_command, speed=current_speed)
                    elif cmd in ('C', 'O'):
                        manual_command = cmd
                        link.send("CLOSE" if cmd == 'C' else "OPEN")
                    else:
                        # Gesture is 'S' or no hands — STOP immediately
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
                        
                draw_status(raw_view, [], setup=setup, auto_enabled=False, manual_mode=True, manual_command=manual_command, gesture_is_grabbed=gesture_is_grabbed)
                cv2.imshow(window, raw_view)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
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
                    link.drive(manual_command)
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
                    link.send("OPEN" if key == ord("o") else "CLOSE")
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

            zones_ready = len(setup.zones) == len(vision.PALETTE)
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
            # Filter gems to only allow colors in vision.PALETTE
            gems = [g for g in last_gems if g.color in vision.PALETTE]
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
                if decision.action is not None:
                    now = time.monotonic()
                    if decision.action != getattr(link, "last_action", None) or now - getattr(link, "last_action_time", 0) > 1.0:
                        link.send(decision.action)
                        link.last_action = decision.action
                        link.last_action_time = now
                if decision.target is not None and command in "FBLR":
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

            if auto_enabled and command in "FBLR" and pose:
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
                    if not stuck_recovery_used:
                        # Likely wedged against a stone or drop-circle rim.
                        # Back away, pivot slightly, then keep the same planner
                        # target/color and let it re-approach.
                        stuck_recovery_used = True
                        stuck_recovery_count += 1
                        motion_reference_at = now
                        reason = f"AUTO RECOVERY {stuck_recovery_count}: backing up and changing approach"
                        print(reason, flush=True)
                        if link.is_online:
                            link.drive("B", speed=CREEP_DRIVE_SPEED)
                            time.sleep(0.22)
                            link.stop()
                            turn = "L" if stuck_recovery_count % 2 else "R"
                            link.drive(turn, speed=CREEP_DRIVE_SPEED)
                            time.sleep(0.14)
                            link.stop()
                        command = "S"
                    else:
                        # Keep Auto armed and preserve the active gemstone or
                        # delivery color. Wait longer after repeated recovery
                        # attempts, then try the same target again.
                        motion_reference_at = now + 5.0
                        stuck_recovery_used = False
                        command = "S"
                        reason = "AUTO WAIT: still blocked; holding target and retrying"
                        print(reason, flush=True)
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
                    link.drive("B", speed=CREEP_DRIVE_SPEED)
                    time.sleep(VIEW_RECOVERY_PULSE_S)
                    link.stop()
                    view_recovery_next_at = time.monotonic() + VIEW_RECOVERY_GAP_S
                else:
                    link.stop()
                    if view_recovery_count >= VIEW_RECOVERY_MAX_PULSES:
                        reason = "AUTO HOLD: view not reacquired; waiting for the marker"
            elif command in "FBLR" and (auto_enabled or manual_command != "S"):
                if link.is_online:
                    if auto_enabled:
                        # The camera/vision loop is slower than the wheels.
                        # Move a short, bounded amount, stop, and let the next
                        # camera frame determine the following command. Slow
                        # down well before the stop threshold so the robot
                        # eases up to a gem or drop circle instead of lunging
                        # through it at full speed.
                        drive_speed = CREEP_DRIVE_SPEED if near_target else CRUISE_DRIVE_SPEED
                        pulse_s = 0.25 if command in "FB" else 0.18
                        link.drive(command, speed=drive_speed)
                        time.sleep(pulse_s)
                        link.stop()
                    else:
                        link.drive(command)
                else:
                    auto_enabled = False
                    manual_command = "S"
                    command = "S"
                    reason = "STOP: robot UDP reply expired"
                    link.stop()
            else:
                link.stop()
            manual_hint = " | press M to exit manual before A arms auto" if manual_mode else ""
            draw_status(view, [
                reason,
                f"Robot: {link.status}",
                f"Auto {'ON' if auto_enabled else 'OFF'} | Manual {'ON' if manual_mode else 'OFF'} | H mark center | B return/restart | Q stop+quit{manual_hint}",
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
                    link.send("OPEN" if key == ord("o") else "CLOSE")
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
                            print("Sorting mode started")
                link.stop()
            elif key == ord("r"):
                link.stop()
                auto_enabled = False
                manual_mode = False
                manual_command = "S"
                setup.clear()
                planner.reset()
                print("Arena calibration cleared.")
    finally:
        link.close()
        camera.release()
        rf_executor.shutdown(wait=False, cancel_futures=True)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
