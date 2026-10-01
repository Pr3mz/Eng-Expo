"""Fail-safe UDP link to the ExpoRedCrObot ESP32 firmware."""

from __future__ import annotations

import socket
import time

DRIVE_SIGN = {
    # Measured on this rover with the InEngMotor library in the firmware:
    # positive PWM drives the gripper side forward. (The earlier hand-written
    # LEDC firmware drove the opposite pins and needed the negated table.)
    "F": (1, 1),
    "B": (-1, -1),
    "L": (1, -1),
    "R": (-1, 1),
}
DEFAULT_DRIVE_SPEED = 200


class RobotLink:
    def __init__(self, port: int = 4217, marker_id: int = 34, robot_ip: str | None = None):
        self.port = port
        self.marker_id = marker_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setblocking(False)
        # Some phone hotspots filter broadcast packets. An optional known IP
        # lets discovery use unicast PING while keeping reply/ACK checks intact.
        self.robot_ip: str | None = robot_ip
        self.last_discovery = 0.0
        self.last_command = ""
        self.last_send = 0.0
        self.last_robot_seen = 0.0
        self.status = "searching for robot"

    @property
    def is_online(self) -> bool:
        """A recent READY/ACK is required before any motion command is sent."""
        return time.monotonic() - self.last_robot_seen < 1.5

    def discover(self) -> None:
        now = time.monotonic()
        if now - self.last_discovery >= 0.5:
            try:
                destination = self.robot_ip or "255.255.255.255"
                self.sock.sendto(b"PING", (destination, self.port))
            except OSError as exc:
                self.status = f"UDP discovery error: {exc}"
            self.last_discovery = now
        self.poll()

    def poll(self) -> None:
        while True:
            try:
                data, sender = self.sock.recvfrom(256)
            except BlockingIOError:
                return
            except OSError as exc:
                self.status = f"UDP receive error: {exc}"
                return
            reply = data.decode("ascii", errors="replace").strip()
            if reply.startswith(f"READY {self.marker_id} EXPORED"):
                self.robot_ip = sender[0]
                self.status = f"robot ready at {self.robot_ip}"
                self.last_robot_seen = time.monotonic()
            elif sender[0] == self.robot_ip:
                self.status = reply
                self.last_robot_seen = time.monotonic()

    def send(self, command: str, *, keepalive: bool = False) -> None:
        self.poll()
        now = time.monotonic()
        if not self.robot_ip:
            self.discover()
            return
        if command not in ("STOP", "S") and not self.is_online:
            self.discover()
            return
        minimum_interval = 0.10 if keepalive else 0.20
        if command == self.last_command and now - self.last_send < minimum_interval:
            return
        try:
            self.sock.sendto(command.encode("ascii"), (self.robot_ip, self.port))
            self.last_command = command
            self.last_send = now
        except OSError as exc:
            self.status = f"UDP send error: {exc}"
            self.robot_ip = None

    def drive(self, command: str, speed: int = DEFAULT_DRIVE_SPEED) -> None:
        try:
            sign_left, sign_right = DRIVE_SIGN[command]
        except KeyError as exc:
            raise ValueError(f"unsupported drive command: {command!r}") from exc
        speed = max(0, min(255, speed))
        # Match the working GemBot controller packet while keeping this
        # project's own discovery, acknowledgement, and watchdog safeguards.
        self.send(f"M {sign_left * speed} {sign_right * speed}", keepalive=True)

    def stop(self) -> None:
        if self.robot_ip:
            self.send("STOP", keepalive=True)
        self.last_command = "STOP"

    def close(self) -> None:
        self.stop()
        self.sock.close()
