"""DroneController class and DroneError exception.

Async wrapper around pymavlink. Background threads handle the heartbeat,
incoming MAVLink messages, and offboard setpoint streaming. The asyncio
event loop runs user mission code on top.
"""
import asyncio
import logging
import math
import threading
import time
from typing import Callable, Optional

from pymavlink import mavutil

from .constants import (
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
)
from ._commands import CommandSender
from ._connection import MAVLinkConnection
from ._mission import MissionOps
from ._streamer import OffboardStreamer
from ._telemetry import Telemetry
from .errors import DroneError
from .types import Position, MarkerObservation
from .utils import body_to_ned
from .viz import VizServer

logger = logging.getLogger("drone")


class DroneController:
    """Async wrapper around pymavlink for sequential autonomous control.

    Architecture:
      - heartbeat_thread (1 Hz)
      - receiver_thread — parses incoming MAVLink into self._tel
      - streamer_thread — publishes offboard setpoints at loop_hz
      - viz_server — optional HTTP+SSE browser UI
      - asyncio main loop — user mission code
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14540",
        source_system: int = 255,
        source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        loop_hz: float = 50.0,
        enable_viz: bool = True,
        viz_port: int = 8765,
        viz_host: str = "127.0.0.1",
        mock: bool = False,
        yaw_slew_rate_deg: float = 15.0,
        telemetry_watchdog_s: float = 2.0,
    ):
        self.connection_string = connection_string
        self.source_system = source_system
        self.source_component = source_component
        self.loop_hz = loop_hz
        self.loop_period = 1.0 / loop_hz
        self._mock = mock

        # MAVLink I/O is owned by MAVLinkConnection (non-mock). In mock mode
        # there is no real connection; mav/target_* are served from a tiny
        # in-process holder so the property shims below keep working.
        if not mock:
            self._connection: Optional[MAVLinkConnection] = MAVLinkConnection(
                connection_string=connection_string,
                source_system=source_system,
                source_component=source_component,
            )
        else:
            self._connection = None
        self._mock_target_system = 0
        self._mock_target_component = 0
        self._mock_sim_thread: Optional[threading.Thread] = None

        self._stop_event = threading.Event()

        # Telemetry state + parsing live in the Telemetry collaborator. The
        # controller keeps self._tel / self._tel_lock property shims (below)
        # for the many call sites that still read them directly.
        self._telemetry = Telemetry(self._connection)
        # The MAVLink I/O lock now lives inside MAVLinkConnection. This
        # fallback lock is only used in mock mode (no real connection) so the
        # `with self._mav_lock:` blocks still work; nothing real is guarded.
        self._mock_mav_lock = threading.Lock()
        # Command emission + COMMAND_ACK Future routing live in CommandSender.
        self._commands = CommandSender(
            self._connection,
            self._telemetry,
            mock,
            get_target=lambda: (self.target_system, self.target_component),
        )
        self._proc_start_monotonic = time.monotonic()
        self.telemetry_watchdog_s = telemetry_watchdog_s
        # Offboard setpoint streaming + telemetry watchdog live in
        # OffboardStreamer (constructed after yaw_slew_rate_rad below).
        # Mock-only fault injection: when True, the simulator stops emitting
        # telemetry (freezes last_local_pos_ts), simulating an autopilot link
        # going silent. Used to exercise the telemetry watchdog in tests.
        self._mock_sim_paused = False

        self._viz: Optional[VizServer] = (
            VizServer(port=viz_port, host=viz_host) if enable_viz else None
        )
        # Wire telemetry's outbound hooks now that viz exists.
        self._telemetry.viz = self._viz
        self._telemetry.route_ack = self._commands.route_command_ack
        self._viz_publisher_task_handle: Optional[asyncio.Task] = None

        self._shutdown_requested = False
        self.yaw_slew_rate_rad = math.radians(yaw_slew_rate_deg)

        self._streamer = OffboardStreamer(
            connection=self._connection,
            telemetry=self._telemetry,
            mock=mock,
            loop_hz=loop_hz,
            yaw_slew_rate_rad=self.yaw_slew_rate_rad,
            telemetry_watchdog_s=telemetry_watchdog_s,
            stop_event=self._stop_event,
            proc_start_monotonic=self._proc_start_monotonic,
        )
        self._mission = MissionOps(self)

    # ---- MAVLink connection shims (Phase 3) -------------------------------
    # The pymavlink connection, the I/O lock, and target sysid live inside
    # self._connection (non-mock). These properties keep the historical
    # attribute surface (self.mav, self._mav_lock, self.target_system, …)
    # working for call sites and tests during the decomposition.

    @property
    def _ack_loop(self):
        return self._commands._ack_loop

    @_ack_loop.setter
    def _ack_loop(self, value) -> None:
        self._commands._ack_loop = value

    @property
    def _pending_acks(self) -> dict:
        return self._commands._pending_acks

    @property
    def _pending_acks_lock(self) -> threading.Lock:
        return self._commands._pending_acks_lock

    @property
    def _tel(self) -> dict:
        return self._telemetry._tel

    @property
    def _tel_lock(self) -> threading.Lock:
        return self._telemetry._lock

    @property
    def _setpoint(self) -> dict:
        return self._streamer._setpoint

    @property
    def _setpoint_lock(self) -> threading.Lock:
        return self._streamer._setpoint_lock

    @property
    def _streaming(self) -> bool:
        return self._streamer._streaming

    @_streaming.setter
    def _streaming(self, value: bool) -> None:
        self._streamer._streaming = value

    @property
    def _watchdog_tripped(self) -> bool:
        return self._streamer.watchdog_tripped

    @_watchdog_tripped.setter
    def _watchdog_tripped(self, value: bool) -> None:
        self._streamer.watchdog_tripped = value

    @property
    def _mav_lock(self) -> threading.Lock:
        if self._connection is None:
            return self._mock_mav_lock
        return self._connection._lock

    @property
    def mav(self):
        return None if self._connection is None else self._connection.mav

    @mav.setter
    def mav(self, value):
        # Used by tests that stub mav directly. Forward to the connection;
        # in mock mode there is no connection so the value is ignored.
        if self._connection is not None:
            self._connection.mav = value

    @property
    def target_system(self) -> int:
        if self._connection is None:
            return self._mock_target_system
        return self._connection.target_system

    @target_system.setter
    def target_system(self, v: int) -> None:
        if self._connection is None:
            self._mock_target_system = v
        else:
            self._connection.target_system = v

    @property
    def target_component(self) -> int:
        if self._connection is None:
            return self._mock_target_component
        return self._connection.target_component

    @target_component.setter
    def target_component(self, v: int) -> None:
        if self._connection is None:
            self._mock_target_component = v
        else:
            self._connection.target_component = v

    async def connect(self, timeout_s: float = 30.0, baud: int = 57600):
        self._ack_loop = asyncio.get_running_loop()
        if self._mock:
            logger.info("[MOCK MODE] no MAVLink connection, starting simulator")
            self.target_system = 1
            self.target_component = 1
            with self._tel_lock:
                self._tel["local_position_ok"] = True
                self._tel["last_local_pos_ts"] = time.time()
                self._tel["landed_state"] = 1
            if self._viz is not None:
                try:
                    self._viz.start()
                    self._viz_publisher_task_handle = asyncio.create_task(
                        self._viz_publisher_loop()
                    )
                except OSError as e:
                    logger.warning(f"Viz server failed to start: {e}")
                    self._viz = None
                    self._telemetry.viz = None
            self._start_mock_sim_thread()
            return

        try:
            await self._connection.connect(timeout_s=timeout_s, baud=baud)
        except RuntimeError as e:
            raise DroneError(str(e)) from e
        logger.info(
            f"Heartbeat from sys={self.target_system} comp={self.target_component} "
            f"src_sys={self.source_system} src_comp={self.source_component}"
        )

        self._connection.start_heartbeat()
        self._connection.start_receiver(self._handle_message)
        await self._request_data_streams()

        if self._viz is not None:
            try:
                self._viz.start()
                self._viz_publisher_task_handle = asyncio.create_task(
                    self._viz_publisher_loop()
                )
            except OSError as e:
                logger.warning(f"Viz server failed to start (port busy?): {e}")
                self._viz = None
                self._telemetry.viz = None

    def close(self):
        self._shutdown_requested = True
        self._stop_event.set()
        self._streamer.stop()
        if self._viz_publisher_task_handle is not None:
            self._viz_publisher_task_handle.cancel()
        if self._viz is not None:
            self._viz.stop()
        if self._mock_sim_thread is not None and self._mock_sim_thread.is_alive():
            self._mock_sim_thread.join(timeout=2.0)
        # Heartbeat/receiver threads and the pymavlink socket are owned by the
        # connection (non-mock only).
        if self._connection is not None:
            self._connection.close()

    def _start_mock_sim_thread(self):
        max_speed_xy = 3.0
        max_speed_z = 1.5
        max_yaw_rate = self.yaw_slew_rate_rad

        def loop():
            last = time.time()
            while not self._stop_event.is_set():
                if self._mock_sim_paused:
                    # Simulated telemetry loss: don't touch _tel at all.
                    time.sleep(0.01)
                    last = time.time()
                    continue
                now = time.time()
                dt = max(0.001, min(0.05, now - last))
                last = now

                with self._setpoint_lock:
                    sp = dict(self._setpoint)

                with self._tel_lock:
                    armed = self._tel["armed"]
                    main_mode = self._tel["main_mode"]
                    sub_mode = self._tel["sub_mode"]
                    cx = self._tel["local_x"]
                    cy = self._tel["local_y"]
                    cz = self._tel["local_z"]
                    cyaw = self._tel["yaw"]

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

                with self._tel_lock:
                    self._tel["local_x"] = new_x
                    self._tel["local_y"] = new_y
                    self._tel["local_z"] = new_z
                    self._tel["yaw"] = new_yaw
                    self._tel["vx"] = vx
                    self._tel["vy"] = vy
                    self._tel["vz"] = vz
                    self._tel["last_local_pos_ts"] = now

                    if new_z >= -0.05:
                        self._tel["landed_state"] = 1
                        if (
                            self._tel["armed"]
                            and self._tel["main_mode"] == PX4_CUSTOM_MAIN_MODE_AUTO
                            and self._tel["sub_mode"]
                            in (
                                PX4_CUSTOM_SUB_MODE_AUTO_LAND,
                                PX4_CUSTOM_SUB_MODE_AUTO_RTL,
                            )
                        ):
                            self._tel["armed"] = False
                            logger.info("[MOCK] Auto-disarmed after land")
                    elif self._tel["armed"]:
                        self._tel["landed_state"] = 2

                time.sleep(0.01)

        self._mock_sim_thread = threading.Thread(target=loop, daemon=True, name="mock-sim")
        self._mock_sim_thread.start()

    def _handle_message(self, msg):
        # Parsing lives in Telemetry; this delegate preserves the historical
        # entry point (the connection's receiver thread and some tests call it).
        self._telemetry.handle_message(msg)

    async def _request_data_streams(self):
        return await self._commands.request_data_streams()

    async def apply_safe_params(
        self,
        com_rcl_except: int = 7,
        com_obl_rc_act: int = 4,
        com_of_loss_t: float = 2.0,
        com_rc_in_mode: int = 1,
    ):
        """Write recommended PX4 safety parameters for offboard missions."""
        return await self._commands.apply_safe_params(
            com_rcl_except=com_rcl_except,
            com_obl_rc_act=com_obl_rc_act,
            com_of_loss_t=com_of_loss_t,
            com_rc_in_mode=com_rc_in_mode,
        )

    async def wait_until_ready(self, timeout_s: float = 60.0):
        """Block until EKF reports a fresh LOCAL_POSITION_NED."""
        if self._mock:
            logger.info("[MOCK] EKF ready (instant)")
            return
        logger.info("Waiting for EKF (LOCAL_POSITION_NED)...")
        start = time.time()
        pos_ok = ekf_ok = False
        while time.time() - start < timeout_s:
            with self._tel_lock:
                pos_ok = self._tel["local_position_ok"] and (
                    time.time() - self._tel["last_local_pos_ts"] < 2.0
                )
                ekf_ok = self._tel["ekf_healthy"]
            if pos_ok and ekf_ok:
                logger.info("EKF ready")
                return
            await asyncio.sleep(0.5)
        raise DroneError(
            f"EKF readiness timeout (pos_ok={pos_ok}, ekf_healthy={ekf_ok})"
        )

    def get_local_position(self) -> Position:
        return self._telemetry.get_local_position()

    def get_yaw_rad(self) -> float:
        return self._telemetry.get_yaw_rad()

    def get_yaw_deg(self) -> float:
        return self._telemetry.get_yaw_deg()

    def is_armed(self) -> bool:
        return self._telemetry.is_armed()

    def get_main_mode(self) -> int:
        return self._telemetry.get_main_mode()

    def get_sub_mode(self) -> int:
        return self._telemetry.get_sub_mode()

    def is_offboard(self) -> bool:
        return self._telemetry.is_offboard()

    def landed_state(self) -> int:
        return self._telemetry.landed_state()

    def _set_setpoint_position(self, x, y, z, yaw_rad: Optional[float] = None):
        self._streamer.set_position(x, y, z, yaw_rad)

    def _ensure_streamer_started(self):
        self._streamer.start()

    def _stop_streamer(self):
        self._streamer.stop()

    async def send_command_long(self, *args, **kwargs) -> bool:
        """Send MAV_CMD_<cmd_id> via COMMAND_LONG and await the terminal ACK.

        Delegates to CommandSender. IN_PROGRESS extends the deadline; terminal
        non-ACCEPTED results raise DroneError; a duplicate in-flight command
        raises immediately.
        """
        return await self._commands.send_command_long(*args, **kwargs)

    async def _set_mode(self, *args, **kwargs) -> bool:
        return await self._commands.set_mode(*args, **kwargs)

    async def _send_arm(self, arm: bool, force: bool = False, timeout_s: float = 5.0) -> bool:
        return await self._commands.send_arm(arm=arm, force=force, timeout_s=timeout_s)

    async def arm(self, timeout_s: float = 10.0) -> bool:
        return await self._mission.arm(timeout_s=timeout_s)

    async def disarm(self, force: bool = False, timeout_s: float = 5.0) -> bool:
        return await self._mission.disarm(force=force, timeout_s=timeout_s)

    def _check_watchdog(self) -> None:
        self._mission.check_watchdog()

    async def takeoff(self, altitude_m: float, timeout_s: float = 30.0) -> bool:
        """Arm the vehicle, enter OFFBOARD mode, and climb to altitude_m."""
        return await self._mission.takeoff(altitude_m, timeout_s=timeout_s)

    async def goto(self, *args, **kwargs) -> bool:
        """Fly to an absolute NED position. Requires OFFBOARD + armed."""
        return await self._mission.goto(*args, **kwargs)

    async def goto_relative(self, dx: float, dy: float, dz: float, yaw_deg: Optional[float] = None, **kwargs) -> bool:
        """Fly to a position offset from the current NED position."""
        return await self._mission.goto_relative(dx, dy, dz, yaw_deg=yaw_deg, **kwargs)

    async def goto_body_relative(self, forward_m: float, right_m: float, down_m: float, yaw_deg: Optional[float] = None, **kwargs) -> bool:
        """Fly to a position offset in body FRD frame (no heading math required)."""
        return await self._mission.goto_body_relative(forward_m, right_m, down_m, yaw_deg=yaw_deg, **kwargs)

    async def hover(self, duration_s: float):
        """Hold current position for duration_s seconds."""
        return await self._mission.hover(duration_s)

    async def set_yaw(self, yaw_deg: float, timeout_s: float = 10.0) -> bool:
        """Rotate in-place to yaw_deg (degrees, NED convention)."""
        return await self._mission.set_yaw(yaw_deg, timeout_s=timeout_s)

    async def land(self, timeout_s: float = 60.0) -> bool:
        """Switch to AUTO_LAND and wait until on-ground."""
        return await self._mission.land(timeout_s=timeout_s)

    async def precision_land(
        self,
        get_marker_offset: Callable[[], Optional[MarkerObservation]],
        descent_rate_mps: float = 0.3,
        final_altitude_m: float = 0.5,
        horizontal_tolerance_m: float = 0.15,
        timeout_s: float = 60.0,
        lateral_p_gain: float = 0.7,
        max_horizontal_step_m: float = 1.0,
        marker_lost_timeout_s: float = 3.0,
        min_altitude_floor_m: float = 0.3,
    ) -> "PrecisionLandResult":
        """Vision-guided descent onto a marker.

        Returns a PrecisionLandResult describing the terminal outcome.
        Safety rules in v0.2.0:
          * Descent below ``-min_altitude_floor_m`` (z in NED) is permitted
            ONLY when the marker is currently visible AND horizontally
            centered within ``horizontal_tolerance_m``.
          * AUTO_LAND handoff (when altitude crosses ``final_altitude_m``)
            also requires the marker to be visible AND centered. Otherwise
            the call returns ABORTED_AT_FLOOR — drone holds floor altitude.
        """
        from .types import PrecisionLandResult, PrecisionLandStatus

        self._check_watchdog()
        if not self.is_offboard():
            raise DroneError("precision_land() requires OFFBOARD mode")
        if not self.is_armed():
            raise DroneError("precision_land() requires armed")

        logger.info("Precision landing starting")
        self._viz_publish_command(
            "precision_land",
            timeout_s=timeout_s,
            descent_rate_mps=descent_rate_mps,
            final_altitude_m=final_altitude_m,
            min_altitude_floor_m=min_altitude_floor_m,
        )

        # Latch floor in NED z (negative = above ground). This is the deepest
        # the setpoint is ever commanded; only marker-locked + centered
        # descent below floor is allowed (used at handoff).
        floor_z = -min_altitude_floor_m

        start = time.time()
        last_seen = time.time()
        iterations = 0
        last_centered = False
        reached_floor = False

        while time.time() - start < timeout_s:
            iterations += 1
            pos = self.get_local_position()
            yaw = self.get_yaw_rad()
            altitude = -pos.z

            obs: Optional[MarkerObservation] = None
            try:
                obs = get_marker_offset()
            except Exception as e:
                logger.error(f"marker callback error: {e}")

            if obs is not None:
                last_seen = time.time()
                ned_dx, ned_dy = body_to_ned(obs.dx, obs.dy, yaw)
                horizontal_err = math.hypot(ned_dx, ned_dy)
                last_centered = horizontal_err < horizontal_tolerance_m

                step_x = max(
                    -max_horizontal_step_m,
                    min(max_horizontal_step_m, ned_dx * lateral_p_gain),
                )
                step_y = max(
                    -max_horizontal_step_m,
                    min(max_horizontal_step_m, ned_dy * lateral_p_gain),
                )
                target_x = pos.x + step_x
                target_y = pos.y + step_y

                if altitude <= final_altitude_m:
                    reached_floor = True
                    if last_centered:
                        logger.info(
                            f"Reached final altitude {altitude:.2f} m with marker "
                            f"centered (err={horizontal_err:.2f} m); handing off to AUTO_LAND"
                        )
                        landed = await self.land(
                            timeout_s=max(10.0, timeout_s - (time.time() - start))
                        )
                        return PrecisionLandResult(
                            status=(
                                PrecisionLandStatus.LANDED if landed
                                else PrecisionLandStatus.HANDED_OFF
                            ),
                            final_position=self.get_local_position(),
                            iterations=iterations,
                        )
                    else:
                        # At floor but off-center — hold floor altitude, keep trying.
                        target_z = floor_z
                else:
                    if last_centered:
                        descent = descent_rate_mps * self.loop_period
                        target_z = min(pos.z + descent, floor_z)  # never below floor
                    else:
                        target_z = pos.z  # hold current z while off-center

                self._set_setpoint_position(target_x, target_y, target_z, yaw)

                if self._viz is not None:
                    self._viz.publish({
                        "type": "marker",
                        "marker_ned": {"x": pos.x + ned_dx, "y": pos.y + ned_dy},
                        "horizontal_err": horizontal_err,
                        "centered": last_centered,
                        "ts": time.time(),
                    })
            else:
                if reached_floor:
                    logger.warning(
                        "Marker lost at floor altitude — aborting precision_land "
                        "(holding floor altitude, NOT handing off to AUTO_LAND)"
                    )
                    return PrecisionLandResult(
                        status=PrecisionLandStatus.ABORTED_AT_FLOOR,
                        final_position=self.get_local_position(),
                        iterations=iterations,
                    )
                if time.time() - last_seen > marker_lost_timeout_s:
                    logger.warning(
                        f"Marker lost for {marker_lost_timeout_s}s above floor "
                        f"(altitude {altitude:.2f} m) — fallback to AUTO_LAND"
                    )
                    landed = await self.land(
                        timeout_s=max(10.0, timeout_s - (time.time() - start))
                    )
                    status = (
                        PrecisionLandStatus.LANDED if landed
                        else PrecisionLandStatus.MARKER_LOST
                    )
                    return PrecisionLandResult(
                        status=status,
                        final_position=self.get_local_position(),
                        iterations=iterations,
                    )

            await asyncio.sleep(self.loop_period)

        logger.warning("Precision land timeout — returning TIMEOUT (no AUTO_LAND fallback)")
        return PrecisionLandResult(
            status=PrecisionLandStatus.TIMEOUT,
            final_position=self.get_local_position(),
            iterations=iterations,
        )

    async def return_to_launch(self, timeout_s: float = 120.0) -> bool:
        """Switch to AUTO_RTL and wait until landed."""
        return await self._mission.return_to_launch(timeout_s=timeout_s)

    async def emergency_land(self) -> None:
        """Best-effort land: AUTO_LAND → MAV_CMD_NAV_LAND → FLIGHT_TERMINATION.

        Lands NOW, where the drone is; does NOT attempt RTL. Intentionally
        ignores the telemetry watchdog flag — it is the recovery path the
        watchdog is meant to trigger.
        """
        return await self._mission.emergency_land()

    def _viz_publish_command(self, command: str, **payload):
        if self._viz is None:
            return
        evt = {"type": "command", "command": command, "ts": time.time()}
        evt.update(payload)
        self._viz.publish(evt)

    async def _viz_publisher_loop(self):
        try:
            while not self._shutdown_requested:
                if self._viz is not None:
                    with self._tel_lock:
                        t = dict(self._tel)
                    with self._setpoint_lock:
                        sp = dict(self._setpoint)
                    self._viz.publish({
                        "type": "telemetry",
                        "x": t["local_x"],
                        "y": t["local_y"],
                        "z": t["local_z"],
                        "vx": t["vx"],
                        "vy": t["vy"],
                        "vz": t["vz"],
                        "yaw": t["yaw"],
                        "roll": t["roll"],
                        "pitch": t["pitch"],
                        "armed": t["armed"],
                        "main_mode": t["main_mode"],
                        "sub_mode": t["sub_mode"],
                        "battery": t["battery_remaining"],
                        "landed": t["landed_state"],
                        "streaming": self._streaming,
                        "setpoint": {
                            "x": sp["x"], "y": sp["y"], "z": sp["z"],
                            "yaw": (
                                None if math.isnan(sp["yaw"]) else sp["yaw"]
                            ),
                        },
                        "ts": time.time(),
                    })
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

    async def _wait_position_reached(self, x: float, y: float, z: float, timeout_s: float, xy_tol: float = 0.5, z_tol: float = 0.5) -> bool:
        return await self._mission.wait_position_reached(x, y, z, timeout_s, xy_tol, z_tol)
