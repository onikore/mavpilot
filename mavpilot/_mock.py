"""Mock connection + in-process flight simulator.

MockMavConnection mirrors the MAVLinkConnection interface (send/recv/close/
start_heartbeat/start_receiver) with no real I/O, so the controller can treat
mock and real links uniformly. MockSimulator runs a background thread that
integrates a simple kinematic model into the telemetry dict, driven by the
current setpoint and flight mode.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Optional

from .constants import (
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
)

logger = logging.getLogger("drone")


class MockMavConnection:
    """No-I/O stand-in for MAVLinkConnection used in mock mode."""

    def __init__(self) -> None:
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self._lock = threading.Lock()

    async def connect(self, timeout_s: float = 30.0, baud: int = 57600) -> None:
        return None

    def send(self, method_name: str, *args, **kwargs) -> None:
        return None

    def recv(self, blocking: bool = True, timeout: float = 0.05) -> Optional[Any]:
        return None

    def start_heartbeat(self) -> None:
        return None

    def start_receiver(self, handle_message: Callable[[Any], None]) -> None:
        return None

    def close(self) -> None:
        return None


class MockSimulator:
    """Background kinematic simulator that writes into the telemetry dict."""

    def __init__(self, ctx) -> None:
        self._ctx = ctx
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        c = self._ctx
        max_speed_xy = 3.0
        max_speed_z = 1.5
        max_yaw_rate = c.yaw_slew_rate_rad

        def loop():
            last = time.time()
            while not c._stop_event.is_set():
                if c._mock_sim_paused:
                    # Simulated telemetry loss: don't touch _tel at all.
                    time.sleep(0.01)
                    last = time.time()
                    continue
                now = time.time()
                dt = max(0.001, min(0.05, now - last))
                last = now

                with c._setpoint_lock:
                    sp = dict(c._setpoint)

                with c._tel_lock:
                    armed = c._tel["armed"]
                    main_mode = c._tel["main_mode"]
                    sub_mode = c._tel["sub_mode"]
                    cx = c._tel["local_x"]
                    cy = c._tel["local_y"]
                    cz = c._tel["local_z"]
                    cyaw = c._tel["yaw"]

                target_x, target_y, target_z, target_yaw = cx, cy, cz, cyaw
                tracking = False

                if armed and main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                    target_x, target_y, target_z = sp["x"], sp["y"], sp["z"]
                    if not math.isnan(sp["yaw"]):
                        target_yaw = sp["yaw"]
                    tracking = True

                elif armed and main_mode == PX4_CUSTOM_MAIN_MODE_AUTO:
                    if sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                        target_x, target_y = cx, cy
                        target_z = 0.0
                        tracking = True
                    elif sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                        target_x, target_y = 0.0, 0.0
                        if math.hypot(cx, cy) < 0.5:
                            target_z = 0.0
                        else:
                            target_z = cz
                        tracking = True
                    elif sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                        target_z = sp["z"] if sp["z"] < cz else cz - 5.0
                        tracking = True
                    elif sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                        target_x, target_y, target_z = cx, cy, cz
                        tracking = True

                new_x, new_y, new_z, new_yaw = cx, cy, cz, cyaw
                vx, vy, vz = 0.0, 0.0, 0.0

                if tracking:
                    dx = target_x - cx
                    dy = target_y - cy
                    dz = target_z - cz
                    dist_xy = math.hypot(dx, dy)
                    max_step_xy = max_speed_xy * dt
                    if dist_xy > max_step_xy:
                        ratio = max_step_xy / dist_xy
                        new_x = cx + dx * ratio
                        new_y = cy + dy * ratio
                        vx = dx * ratio / dt
                        vy = dy * ratio / dt
                    else:
                        new_x = target_x
                        new_y = target_y
                        vx = dx / dt if dt > 0 else 0.0
                        vy = dy / dt if dt > 0 else 0.0

                    max_step_z = max_speed_z * dt
                    if abs(dz) > max_step_z:
                        new_z = cz + math.copysign(max_step_z, dz)
                        vz = math.copysign(max_speed_z, dz)
                    else:
                        new_z = target_z
                        vz = dz / dt if dt > 0 else 0.0

                    yaw_err = math.atan2(
                        math.sin(target_yaw - cyaw), math.cos(target_yaw - cyaw)
                    )
                    max_yaw_step = max_yaw_rate * dt
                    if abs(yaw_err) > max_yaw_step:
                        new_yaw = cyaw + math.copysign(max_yaw_step, yaw_err)
                    else:
                        new_yaw = target_yaw

                if new_z > 0:
                    new_z = 0.0

                with c._tel_lock:
                    c._tel["local_x"] = new_x
                    c._tel["local_y"] = new_y
                    c._tel["local_z"] = new_z
                    c._tel["yaw"] = new_yaw
                    c._tel["vx"] = vx
                    c._tel["vy"] = vy
                    c._tel["vz"] = vz
                    c._tel["last_local_pos_ts"] = now

                    if new_z >= -0.05:
                        c._tel["landed_state"] = 1
                        if (
                            c._tel["armed"]
                            and c._tel["main_mode"] == PX4_CUSTOM_MAIN_MODE_AUTO
                            and c._tel["sub_mode"]
                            in (
                                PX4_CUSTOM_SUB_MODE_AUTO_LAND,
                                PX4_CUSTOM_SUB_MODE_AUTO_RTL,
                            )
                        ):
                            c._tel["armed"] = False
                            logger.info("[MOCK] Auto-disarmed after land")
                    elif c._tel["armed"]:
                        c._tel["landed_state"] = 2

                time.sleep(0.01)

        self._thread = threading.Thread(target=loop, daemon=True, name="mock-sim")
        self._thread.start()

    def stop(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
