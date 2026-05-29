"""Telemetry — owns the shared telemetry dict and its lock, parses incoming
MAVLink into it, and exposes typed getters.

The receiver thread (in MAVLinkConnection) calls ``handle_message`` for every
frame. COMMAND_ACK frames are forwarded to ``route_ack`` (wired by the
controller to the command sender). STATUSTEXT frames are mirrored to ``viz``.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Optional

from pymavlink import mavutil

from .constants import ACK_RESULT_NAMES, PX4_CUSTOM_MAIN_MODE_OFFBOARD
from .types import Position
from .utils import normalize_yaw_deg

logger = logging.getLogger("drone")


class Telemetry:
    def __init__(self, connection) -> None:
        self._connection = connection
        self._lock = threading.Lock()
        self._tel: dict = {
            "armed": False,
            "custom_mode": 0,
            "main_mode": 0,
            "sub_mode": 0,
            "local_x": 0.0,
            "local_y": 0.0,
            "local_z": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "yaw": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "battery_remaining": 1.0,
            "landed_state": 0,
            "ekf_healthy": True,
            "local_position_ok": False,
            "last_local_pos_ts": 0.0,
            "last_ack": None,
        }
        # Wired by the controller after construction.
        self.viz = None
        self.route_ack: Callable[[int, int], None] = lambda command, result: None

    @property
    def target_system(self) -> int:
        return self._connection.target_system if self._connection is not None else 1

    def handle_message(self, msg) -> None:
        t = msg.get_type()
        try:
            if msg.get_srcSystem() != self.target_system and self.target_system != 0:
                return
        except Exception:
            pass
        now = time.time()
        with self._lock:
            if t == "HEARTBEAT":
                self._tel["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                cm = msg.custom_mode
                self._tel["custom_mode"] = cm
                self._tel["main_mode"] = (cm >> 16) & 0xFF
                self._tel["sub_mode"] = (cm >> 24) & 0xFF
            elif t == "LOCAL_POSITION_NED":
                self._tel["local_x"] = msg.x
                self._tel["local_y"] = msg.y
                self._tel["local_z"] = msg.z
                self._tel["vx"] = msg.vx
                self._tel["vy"] = msg.vy
                self._tel["vz"] = msg.vz
                self._tel["last_local_pos_ts"] = now
                self._tel["local_position_ok"] = True
            elif t == "ATTITUDE":
                self._tel["roll"] = msg.roll
                self._tel["pitch"] = msg.pitch
                self._tel["yaw"] = msg.yaw
            elif t == "EXTENDED_SYS_STATE":
                self._tel["landed_state"] = msg.landed_state
            elif t == "BATTERY_STATUS":
                if msg.battery_remaining >= 0:
                    self._tel["battery_remaining"] = msg.battery_remaining / 100.0
            elif t == "SYS_STATUS":
                # Bit MAV_SYS_STATUS_AHRS = 1<<5 = 32. Health bit set if EKF OK.
                health = getattr(msg, "onboard_control_sensors_health", 0)
                self._tel["ekf_healthy"] = bool(health & 32)
            elif t == "STATUSTEXT":
                text = msg.text.rstrip("\0\t ") if isinstance(msg.text, str) else msg.text
                sev = msg.severity
                if sev <= 3:
                    logger.error(f"PX4: {text}")
                elif sev <= 5:
                    logger.warning(f"PX4: {text}")
                else:
                    logger.info(f"PX4: {text}")
                if self.viz is not None:
                    self.viz.publish({
                        "type": "log",
                        "severity": sev,
                        "text": text,
                        "ts": now,
                    })
            elif t == "COMMAND_ACK":
                r = ACK_RESULT_NAMES.get(msg.result, str(msg.result))
                self._tel["last_ack"] = (msg.command, msg.result)
                level = logging.INFO if msg.result == 0 else logging.WARNING
                logger.log(level, f"ACK cmd={msg.command} result={r}")
                self.route_ack(msg.command, msg.result)

    # ---- typed getters ----------------------------------------------------

    def get_local_position(self) -> Position:
        with self._lock:
            return Position(self._tel["local_x"], self._tel["local_y"], self._tel["local_z"])

    def get_yaw_rad(self) -> float:
        with self._lock:
            return self._tel["yaw"]

    def get_yaw_deg(self) -> float:
        return normalize_yaw_deg(math.degrees(self.get_yaw_rad()))

    def is_armed(self) -> bool:
        with self._lock:
            return self._tel["armed"]

    def get_main_mode(self) -> int:
        with self._lock:
            return self._tel["main_mode"]

    def get_sub_mode(self) -> int:
        with self._lock:
            return self._tel["sub_mode"]

    def is_offboard(self) -> bool:
        return self.get_main_mode() == PX4_CUSTOM_MAIN_MODE_OFFBOARD

    def landed_state(self) -> int:
        with self._lock:
            return self._tel["landed_state"]
