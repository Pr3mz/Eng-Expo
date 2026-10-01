"""Small, testable geometry helpers for the overhead-camera controller."""

from math import atan2, pi


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def heading_error(robot_xy, heading: float, target_xy) -> float:
    return wrap_angle(atan2(target_xy[1] - robot_xy[1], target_xy[0] - robot_xy[0]) - heading)


def steering_command(error: float, threshold: float = 0.30) -> str:
    # When the target is almost exactly behind, tiny camera noise can flip the
    # wrapped error between +pi and -pi and make the rover alternate L/R.
    # Pick one direction within this narrow dead zone to keep the turn stable.
    if abs(abs(error) - pi) <= 0.2094395102:  # 12 degrees
        error = pi
    if error > threshold:
        return "R"
    if error < -threshold:
        return "L"
    return "F"
