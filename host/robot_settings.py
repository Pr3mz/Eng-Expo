"""Robot geometry and tuning values, saved in robot_settings.json.

The file sits next to this module and can be edited by hand or with the
Robot Setup panel in argos.py (press G). Pixel values are in the warped
800x600 overhead image, so they match what the camera view shows.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

SETTINGS_PATH = Path(__file__).resolve().parent / "robot_settings.json"


@dataclass(frozen=True)
class Field:
    key: str
    label: str        # shown on the setup panel slider
    default: float
    low: int
    high: int
    scale: int = 1    # slider stores value * scale (for decimals)
    persist: bool = True  # False = live-only slider, never written to the file


FIELDS = (
    Field("gripper_distance_px", "Gripper length px", 110, 10, 300),
    Field("gripper_radius_px", "Gripper size px", 20, 5, 120),
    Field("robot_radius_px", "Robot size px", 130, 30, 250),
    Field("robot_rear_px", "Robot rear px", 60, 0, 200),
    Field("mm_per_pixel", "mm per px x100", 0, 0, 1000, scale=100),
    Field("drop_distance_mm", "Drop distance mm", 125, 20, 400),
    Field("cruise_speed", "Cruise speed", 210, 120, 255),
    Field("creep_speed", "Creep speed", 200, 120, 255),
    Field("manual_speed", "Manual speed", 255, 120, 255),
    Field("manual_turn_speed", "Manual turn speed", 255, 120, 255),
    Field("turn_start_speed", "Turn start speed", 140, 100, 255),
    Field("turn_ramp_s", "Turn ramp sec x10", 1.5, 0, 50, scale=10),
    Field("servo_open_us", "Servo OPEN us", 1100, 500, 2500),
    Field("servo_close_us", "Servo CLOSE us", 1950, 500, 2500),
    Field("servo_test_us", "Servo TEST us", 1100, 500, 2500, persist=False),
)
FIELD_BY_KEY = {field.key: field for field in FIELDS}


def load() -> dict[str, float]:
    """Return every setting, using defaults for missing or invalid values."""
    values = {field.key: float(field.default) for field in FIELDS}
    if not SETTINGS_PATH.exists():
        return values
    try:
        data = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print(f"Ignoring {SETTINGS_PATH.name}: {exc}")
        return values
    for key, value in data.items() if isinstance(data, dict) else ():
        field = FIELD_BY_KEY.get(key)
        if field is not None and isinstance(value, (int, float)) and value >= 0:
            values[key] = float(value)
    return values


def save(values: dict[str, float]) -> None:
    data = {}
    for field in FIELDS:
        if not field.persist:
            continue
        value = values[field.key]
        data[field.key] = round(value, 3) if field.scale > 1 else int(round(value))
    temporary = SETTINGS_PATH.with_name(SETTINGS_PATH.name + ".tmp")
    try:
        temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        temporary.replace(SETTINGS_PATH)
    except OSError as exc:
        print(f"Could not save robot settings: {exc}")
