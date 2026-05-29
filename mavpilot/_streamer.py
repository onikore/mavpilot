"""OffboardStreamer — publishes SET_POSITION_TARGET_LOCAL_NED at loop_hz with
yaw slew-rate limiting, and runs the telemetry watchdog.

Owns the setpoint dict and its lock. The mission layer calls ``set_position``
to update the target; ``start`` spins up the publisher thread (no-op send in
mock mode); ``watchdog_tripped`` latches True after telemetry_watchdog_s of
LOCAL_POSITION_NED silence.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from pymavlink import mavutil

from .constants import DEFAULT_POS_TYPE_MASK
from .errors import DroneError

logger = logging.getLogger("drone")


class OffboardStreamer:
    def __init__(
        self,
        connection,
        telemetry,
        mock: bool,
        loop_hz: float,
        yaw_slew_rate_rad: float,
        telemetry_watchdog_s: float,
        stop_event: threading.Event,
        proc_start_monotonic: float,
    ) -> None:
        self._connection = connection
        self._telemetry = telemetry
        self._mock = mock
        self.loop_period = 1.0 / loop_hz
        self.yaw_slew_rate_rad = yaw_slew_rate_rad
        self.telemetry_watchdog_s = telemetry_watchdog_s
        self._stop_event = stop_event
        self._proc_start_monotonic = proc_start_monotonic

        self._setpoint_lock = threading.Lock()
        self._setpoint = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "yaw": float("nan"),
            "yaw_target": float("nan"),
            "type_mask": DEFAULT_POS_TYPE_MASK,
        }
        self._streaming = False
        self._streamer_thread: threading.Thread | None = None
        self.watchdog_tripped = False

    @property
    def streaming(self) -> bool:
        return self._streaming

    def snapshot(self) -> dict:
        with self._setpoint_lock:
            return dict(self._setpoint)

    def set_position(
        self,
        x: float,
        y: float,
        z: float,
        yaw_rad: float | None = None,
    ) -> None:
        with self._setpoint_lock:
            self._setpoint["x"] = x
            self._setpoint["y"] = y
            self._setpoint["z"] = z
            self._setpoint["vx"] = 0.0
            self._setpoint["vy"] = 0.0
            self._setpoint["vz"] = 0.0
            if yaw_rad is None:
                self._setpoint["yaw_target"] = float("nan")
                self._setpoint["yaw"] = float("nan")
            else:
                self._setpoint["yaw_target"] = yaw_rad
                if math.isnan(self._setpoint.get("yaw", float("nan"))):
                    self._setpoint["yaw"] = yaw_rad
            self._setpoint["type_mask"] = DEFAULT_POS_TYPE_MASK

    def start(self) -> None:
        if self._streaming:
            return
        tel = self._telemetry
        with tel._lock:
            if not tel._tel["local_position_ok"]:
                raise DroneError(
                    "Cannot start offboard streamer before EKF gives LOCAL_POSITION_NED. "
                    "Call wait_until_ready() first."
                )
        pos = tel.get_local_position()
        self.set_position(pos.x, pos.y, pos.z, tel.get_yaw_rad())
        self._streaming = True

        def loop():
            last = time.time()
            while self._streaming and not self._stop_event.is_set():
                now = time.time()
                dt = max(1e-6, min(0.2, now - last))
                last = now
                try:
                    with self._setpoint_lock:
                        prev_yaw = self._setpoint.get("yaw", float("nan"))
                        target_yaw = self._setpoint.get("yaw_target", float("nan"))
                        if not math.isnan(target_yaw):
                            if math.isnan(prev_yaw):
                                self._setpoint["yaw"] = target_yaw
                            else:
                                yaw_err = math.atan2(
                                    math.sin(target_yaw - prev_yaw),
                                    math.cos(target_yaw - prev_yaw),
                                )
                                max_step = self.yaw_slew_rate_rad * dt
                                if abs(yaw_err) <= max_step:
                                    self._setpoint["yaw"] = target_yaw
                                else:
                                    self._setpoint["yaw"] = prev_yaw + math.copysign(
                                        max_step, yaw_err
                                    )
                        sp = dict(self._setpoint)
                    if not self._mock:
                        tb_ms = (
                            int((time.monotonic() - self._proc_start_monotonic) * 1e3) & 0xFFFFFFFF
                        )
                        self._connection.send(
                            "set_position_target_local_ned_send",
                            tb_ms,
                            self._connection.target_system,
                            self._connection.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            sp["type_mask"],
                            sp["x"],
                            sp["y"],
                            sp["z"],
                            sp["vx"],
                            sp["vy"],
                            sp["vz"],
                            0.0,
                            0.0,
                            0.0,
                            sp["yaw"],
                            0.0,
                        )
                except Exception as e:
                    logger.warning(f"streamer error: {e}")
                time.sleep(self.loop_period)
                with tel._lock:
                    last_ts = tel._tel["last_local_pos_ts"]
                stale = last_ts > 0 and (time.time() - last_ts) > self.telemetry_watchdog_s
                if stale and not self.watchdog_tripped:
                    logger.error(
                        f"telemetry watchdog tripped: no LOCAL_POSITION_NED "
                        f"for {time.time() - last_ts:.1f}s "
                        f"(threshold {self.telemetry_watchdog_s}s)"
                    )
                    self.watchdog_tripped = True

        self._streamer_thread = threading.Thread(target=loop, daemon=True, name="streamer")
        self._streamer_thread.start()

    def stop(self) -> None:
        self._streaming = False
        if self._streamer_thread is not None:
            self._streamer_thread.join(timeout=2.0)
            self._streamer_thread = None
