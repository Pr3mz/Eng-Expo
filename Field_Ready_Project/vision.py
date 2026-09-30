"""Offline arena, ArUco and six-color gemstone vision helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np


PALETTE = ("violet", "cyan", "crimson", "gold", "blue", "lime")
COLOR_HUE_RANGES = {
    "crimson": ((0, 10), (170, 179)),
    "gold": ((11, 28),),
    "lime": ((29, 75),),
    "cyan": ((76, 98),),
    "blue": ((99, 128),),
    "violet": ((129, 169),),
}
# Fallback only. With calibrated cyan and blue destination circles, the live
# detector derives a better split from their current camera saturation.
BLUE_CYAN_SATURATION_SPLIT = 215
_ARUCO_DETECTOR = None
_ROBOT_TEMPLATE = None
_ROBOT_TEMPLATE_PATH = Path(__file__).resolve().parent / "assets" / "marker_34_id34.png"


@dataclass(frozen=True)
class Gem:
    x: int
    y: int
    color: str
    area: float


@dataclass(frozen=True)
class RobotPose:
    x: int
    y: int
    heading: float
    front: tuple[int, int]


def classify_hue(hue: int, saturation: int = 255, value: int = 255, blue_cyan_split: int = BLUE_CYAN_SATURATION_SPLIT) -> str | None:
    if saturation < 45 or value < 45:
        return None
    # Use saturation split for cyan/blue if they share hue
    if 76 <= hue <= 128:
        return "cyan" if saturation >= blue_cyan_split else "blue"
    for color, ranges in COLOR_HUE_RANGES.items():
        if any(low <= hue <= high for low, high in ranges):
            return color
    return None
    for color, ranges in COLOR_HUE_RANGES.items():
        if any(low <= hue <= high for low, high in ranges):
            # If the hue matches Cyan but it's very pale, it might be Blue in this lighting
            if color == "cyan" and saturation < 150:
                return "blue"
            # If the hue matches Blue but it's super vivid, it might be Cyan
            if color == "blue" and saturation > 230:
                return "cyan"
            return color
    return None


def sample_target_color(image: np.ndarray, center: tuple[int, int], radius: int = 18, split: int = BLUE_CYAN_SATURATION_SPLIT) -> str | None:
    """Classify a clicked target circle by its dominant inner-disk color."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        return None
    x, y = (int(center[0]), int(center[1]))
    height, width = image.shape[:2]
    if not (0 <= x < width and 0 <= y < height):
        return None

    x0, x1 = max(0, x - radius), min(width, x + radius + 1)
    y0, y1 = max(0, y - radius), min(height, y + radius + 1)
    yy, xx = np.ogrid[y0:y1, x0:x1]
    disk = (xx - x) ** 2 + (yy - y) ** 2 <= radius**2
    hsv = cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    pixels = hsv[disk]
    labels = [classify_hue(int(h), int(s), int(v), split) for h, s, v in pixels]
    labels = [label for label in labels if label is not None]
    if len(labels) < 30:
        return None

    counts = {color: labels.count(color) for color in PALETTE}
    best = max(PALETTE, key=counts.get)
    if counts[best] / len(labels) < 0.35:
        return None
    return best


def estimate_blue_cyan_saturation_split(
    hsv_image: np.ndarray,
    color_zones: dict[str, tuple[int, int]] | None,
    *,
    radius: int = 14,
    fallback: int = BLUE_CYAN_SATURATION_SPLIT,
) -> int:
    """Adapt the blue/cyan saturation boundary to the two marked field targets.

    The camera and lighting can shift saturation enough that a fixed threshold
    rejects cyan pixels on small stones. A robust median from each target's
    inner disk tracks those changes. Fall back when either sample is missing,
    contaminated, or not clearly separated.
    """
    if hsv_image is None or hsv_image.ndim != 3 or hsv_image.shape[2] != 3 or not color_zones:
        return fallback

    medians: dict[str, float] = {}
    height, width = hsv_image.shape[:2]
    for color in ("cyan", "blue"):
        center = color_zones.get(color)
        if center is None:
            return fallback
        x, y = map(int, center)
        x0, x1 = max(0, x - radius), min(width, x + radius + 1)
        y0, y1 = max(0, y - radius), min(height, y + radius + 1)
        if x0 >= x1 or y0 >= y1:
            return fallback
        patch = hsv_image[y0:y1, x0:x1]
        yy, xx = np.ogrid[y0:y1, x0:x1]
        disk = (xx - x) ** 2 + (yy - y) ** 2 <= radius**2
        hues, saturations, values = patch[:, :, 0], patch[:, :, 1], patch[:, :, 2]
        valid = disk & (hues >= 76) & (hues <= 128) & (saturations >= 65) & (values >= 55)
        samples = saturations[valid]
        if samples.size < 30:
            return fallback
        medians[color] = float(np.median(samples))

    # This setup labels vivid cyan above pale blue. If current references do
    # not preserve that distinction, the old conservative split is safer.
    if medians["cyan"] - medians["blue"] < 35:
        return fallback
    split = int(round((medians["cyan"] + medians["blue"]) / 2.0))
    return max(80, min(250, split))


def order_corners(points) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if points.shape != (4, 2):
        raise ValueError("Exactly four 2D arena corners are required")
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).reshape(-1)
    return np.array([
        points[np.argmin(sums)], points[np.argmin(diffs)],
        points[np.argmax(sums)], points[np.argmax(diffs)],
    ], dtype=np.float32)


def make_homography(points, width: int, height: int) -> np.ndarray:
    src = order_corners(points)
    pairwise = [np.linalg.norm(src[i] - src[j]) for i in range(4) for j in range(i + 1, 4)]
    contour = src.reshape((-1, 1, 2))
    if min(pairwise) < 25 or abs(cv2.contourArea(contour)) < 5000 or not cv2.isContourConvex(contour):
        raise ValueError("Selected corners are too close together or do not form a valid convex arena")
    dst = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-12:
        raise ValueError("Arena corner selection does not form a usable quadrilateral")
    return matrix


def detect_robot_pose(image: np.ndarray, marker_id: int = 34) -> RobotPose | None:
    """Find the project's printed robot symbol, then fall back to ArUco tags.

    The supplied ID-34 sheet is a custom black square with a white U glyph,
    rather than an ArUco code. On this rover it is mounted backward relative
    to the gripper, so the detected marker direction is reversed to get the
    actual robot front.
    """
    global _ROBOT_TEMPLATE
    if marker_id == 34:
        if _ROBOT_TEMPLATE is None and _ROBOT_TEMPLATE_PATH.is_file():
            marker = cv2.imread(str(_ROBOT_TEMPLATE_PATH), cv2.IMREAD_GRAYSCALE)
            if marker is not None:
                _, _ROBOT_TEMPLATE = cv2.threshold(marker, 127, 255, cv2.THRESH_BINARY)
        if _ROBOT_TEMPLATE is not None:
            pose = _detect_custom_robot_marker(image, _ROBOT_TEMPLATE)
            if pose is not None:
                return _reverse_pose_direction(pose)

    global _ARUCO_DETECTOR
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module is missing; install opencv-contrib-python")
    if _ARUCO_DETECTOR is None:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _ARUCO_DETECTOR = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = _ARUCO_DETECTOR.detectMarkers(image)
    if ids is None:
        return None
    found = np.flatnonzero(ids.flatten() == marker_id)
    if not len(found):
        return None
    quad = corners[int(found[0])][0]
    center = quad.mean(axis=0)
    front = (quad[0] + quad[1]) / 2.0
    pose = RobotPose(
        int(round(center[0])), int(round(center[1])),
        math.atan2(float(front[1] - center[1]), float(front[0] - center[0])),
        (int(round(front[0])), int(round(front[1]))),
    )
    return _reverse_pose_direction(pose)


def _reverse_pose_direction(pose: RobotPose) -> RobotPose:
    """Use the gripper-facing side as forward for the mounted ID-34 plate."""
    front = (2 * pose.x - pose.front[0], 2 * pose.y - pose.front[1])
    heading = math.atan2(front[1] - pose.y, front[0] - pose.x)
    return RobotPose(pose.x, pose.y, heading, front)


def _detect_custom_robot_marker(image: np.ndarray, template: np.ndarray) -> RobotPose | None:
    """Perspective-normalize dark square candidates and match the U-glyph."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    # The marker's black square is surrounded by the lighter floor/paper.
    _, dark = cv2.threshold(gray, 72, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    template = cv2.resize(template, (128, 128), interpolation=cv2.INTER_NEAREST)
    template_rotations = [np.rot90(template, k).copy() for k in range(4)]
    height, width = gray.shape[:2]
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if not max(2400, height * width * 0.002) <= area <= height * width * 0.45:
            continue
        perimeter = cv2.arcLength(contour, True)
        quad = cv2.approxPolyDP(contour, 0.035 * perimeter, True).reshape(-1, 2)
        if quad.shape != (4, 2) or not cv2.isContourConvex(quad.reshape((-1, 1, 2))):
            continue

        edges = np.roll(quad, -1, axis=0) - quad
        lengths = np.linalg.norm(edges, axis=1)
        if lengths.min() < 45 or lengths.max() / lengths.min() > 1.55:
            continue
        center = quad.astype(np.float32).mean(axis=0)
        angles = np.arctan2(quad[:, 1] - center[1], quad[:, 0] - center[0])
        quad = quad[np.argsort(angles)].astype(np.float32)  # clockwise in image coordinates
        start = int(np.argmin(quad[:, 0] + quad[:, 1]))
        quad = np.roll(quad, -start, axis=0)

        side = 128
        destination = np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32)
        transform = cv2.getPerspectiveTransform(quad, destination)
        square = cv2.warpPerspective(gray, transform, (side, side), flags=cv2.INTER_LINEAR)
        _, square = cv2.threshold(square, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        scores = [float(cv2.matchTemplate(square, rotated, cv2.TM_CCOEFF_NORMED)[0, 0]) for rotated in template_rotations]
        rotation = int(np.argmax(scores))
        score = scores[rotation]
        if score < 0.50:
            continue
        candidates.append((score, quad, rotation))

    if not candidates:
        return None
    score, quad, rotation = max(candidates, key=lambda candidate: candidate[0])
    del score  # Candidate ranking is complete; geometry determines the returned pose.
    center = quad.mean(axis=0)
    front_edge_indices = ((0, 1), (3, 0), (2, 3), (1, 2))
    edge_start, edge_end = front_edge_indices[rotation]
    front = (quad[edge_start] + quad[edge_end]) / 2.0
    return RobotPose(
        int(round(center[0])), int(round(center[1])),
        math.atan2(float(front[1] - center[1]), float(front[0] - center[0])),
        (int(round(front[0])), int(round(front[1]))),
    )


def detect_gems(
    image: np.ndarray,
    *,
    min_area: float = 90.0,
    max_area: float = 4500.0,
    excluded_centers: tuple[tuple[int, int], ...] = (),
    excluded_radius: int = 48,
    excluded_regions: tuple[tuple[tuple[int, int], int], ...] = (),
    color_zones: dict[str, tuple[int, int]] | None = None,
) -> list[Gem]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue_cyan_split = estimate_blue_cyan_saturation_split(hsv, color_zones)
    excluded = np.zeros(image.shape[:2], dtype=np.uint8)
    for center in excluded_centers:
        cv2.circle(excluded, center, excluded_radius, 255, -1)
    for center, radius in excluded_regions:
        cv2.circle(excluded, center, radius, 255, -1)

    gems: list[Gem] = []
    kernel = np.ones((3, 3), np.uint8)
    for color in PALETTE:
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        if color in ("cyan", "blue"):
            # Use the standard hue ranges!
            ranges = COLOR_HUE_RANGES[color]
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            for low, high in ranges:
                sat_low = 150 if color == "cyan" else 45
                sat_high = 255 if color == "cyan" else 230
                part = cv2.inRange(hsv, np.array([low, sat_low, 45]), np.array([high, sat_high, 255]))
                mask = cv2.bitwise_or(mask, part)
        else:
            for low, high in COLOR_HUE_RANGES[color]:
                part = cv2.inRange(hsv, np.array([low, 65, 55]), np.array([high, 255, 255]))
                mask = cv2.bitwise_or(mask, part)
        mask[excluded > 0] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not min_area <= area <= max_area:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
            gems.append(Gem(x, y, color, area))
    return gems


# ========================== ROBOFLOW CLOUD API ==========================

_RF_CLIENT = None
RF_API_KEY   = "X5wKaATp2EknpnzoIAqF"
RF_WORKSPACE = "premsupthaksina1-gmail-com"
RF_WORKFLOW  = "arena-gemstone-color-detector"
RF_QUERY_W   = 640
RF_QUERY_H   = 480


def _get_rf_client():
    """สร้าง InferenceHTTPClient ครั้งเดียว แล้ว cache ไว้ใช้ซ้ำ"""
    global _RF_CLIENT
    if _RF_CLIENT is None:
        from inference_sdk import InferenceHTTPClient, InferenceConfiguration
        _RF_CLIENT = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key=RF_API_KEY,
        ).configure(InferenceConfiguration(api_key_transport="header"))
    return _RF_CLIENT


def _is_drop_zone(cls: str) -> bool:
    n = cls.lower()
    return "drop" in n or "zone" in n or "area" in n or "base" in n


def _extract_predictions(data) -> list:
    """ดึง bounding box จาก Roboflow response ที่มีโครงสร้างหลากหลาย"""
    preds = []
    if hasattr(data, "predictions"):
        for p in data.predictions:
            preds.append({
                "x":      getattr(p, "x", 0),
                "y":      getattr(p, "y", 0),
                "width":  getattr(p, "width", 30),
                "height": getattr(p, "height", 30),
                "class":  getattr(p, "class_name", getattr(p, "class", "Object")),
            })
    elif isinstance(data, dict):
        if "x" in data and "y" in data:
            preds.append(data)
        elif "center_x" in data:
            d = dict(data)
            d["x"] = data["center_x"]
            d["y"] = data["center_y"]
            preds.append(d)
        for v in data.values():
            preds.extend(_extract_predictions(v))
    elif isinstance(data, list):
        for item in data:
            preds.extend(_extract_predictions(item))
    return preds


# Mapping จาก Roboflow class name → PALETTE color name
_RF_CLASS_TO_COLOR: dict[str, str] = {
    "violet": "violet",
    "purple": "violet",
    "cyan":   "cyan",
    "teal":   "cyan",
    "crimson": "crimson",
    "red":    "crimson",
    "gold":   "gold",
    "yellow": "gold",
    "blue":   "blue",
    "lime":   "lime",
    "green":  "lime",
}


def detect_gems_roboflow(
    image: np.ndarray,
    *,
    warp_w: int = 800,
    warp_h: int = 600,
    color_zones: dict[str, tuple[int, int]] | None = None,
) -> tuple[list[Gem], list[tuple[int, int, str]]]:
    """ส่งภาพไป Roboflow Cloud แล้วคืนมาเป็น Gem objects + Drop zone list

    Returns:
        (gems, drops)
        gems  = [Gem(x, y, color, area), ...]
        drops = [(cx, cy, class_name), ...]
    """
    client = _get_rf_client()
    small = cv2.resize(image, (RF_QUERY_W, RF_QUERY_H), interpolation=cv2.INTER_AREA)

    res = client.run_workflow(
        workspace_name=RF_WORKSPACE,
        workflow_id=RF_WORKFLOW,
        images={"image": small},
        use_cache=True,
    )

    sx = warp_w / RF_QUERY_W
    sy = warp_h / RF_QUERY_H
    raw_preds = _extract_predictions(res)

    gems: list[Gem] = []
    drops: list[tuple[int, int, str]] = []

    for p in raw_preds:
        cls = p.get("class", "Object")
        cx  = int(p["x"] * sx)
        cy  = int(p["y"] * sy)
        w   = p.get("width", 30) * sx
        h   = p.get("height", 30) * sy
        area = w * h

        if _is_drop_zone(cls):
            drops.append((cx, cy, cls))
        else:
            color = _RF_CLASS_TO_COLOR.get(cls.lower().strip())
            if not color:
                # The model predicted a generic 'Gem' class without a color.
                # Use our robust local HSV logic to determine the color of this bounding box.
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                split = estimate_blue_cyan_saturation_split(hsv, color_zones) if color_zones else BLUE_CYAN_SATURATION_SPLIT
                sampled = sample_target_color(image, (cx, cy), radius=8, split=split)
                color = sampled if sampled else "crimson"
            gems.append(Gem(cx, cy, color, area))

    return gems, drops
