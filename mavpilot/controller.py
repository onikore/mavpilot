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
from typing import Optional

from pymavlink import mavutil

from ._commands import CommandSender
from ._connection import MAVLinkConnection
from ._mock import MockMavConnection, MockSimulator
from ._mission import MissionOps
from ._precision_land import PrecisionLand
from ._safety import SafetyOps
from ._streamer import OffboardStreamer
from ._telemetry import Telemetry
from .errors import DroneError
from .types import Position
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

        # MAVLink I/O is owned by the connection object. Real links use
        # MAVLinkConnection; mock mode uses MockMavConnection (same interface,
        # no real I/O) so the rest of the controller is link-agnostic.
        self._connection = (
            MAVLinkConnection(
                connection_string=connection_string,
                source_system=source_system,
                source_component=source_component,
            )
            if not mock
            else MockMavConnection()
        )

        self._stop_event = threading.Event()

        # Telemetry state + parsing live in the Telemetry collaborator. The
        # controller keeps self._tel / self._tel_lock property shims (below)
        # for the many call sites that still read them directly.
        self._telemetry = Telemetry(self._connection)
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
        self._precision = PrecisionLand(self)
        self._safety = SafetyOps(self)
        self._mock_sim = MockSimulator(self) if mock else None

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
        return self._connection._lock

    @property
    def mav(self):
        return self._connection.mav

    @mav.setter
    def mav(self, value):
        # Used by tests that stub mav directly. Forward to the connection.
        self._connection.mav = value

    @property
    def target_system(self) -> int:
        return self._connection.target_system

    @target_system.setter
    def target_system(self, v: int) -> None:
        self._connection.target_system = v

    @property
    def target_component(self) -> int:
        return self._connection.target_component

    @target_component.setter
    def target_component(self, v: int) -> None:
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
            self._mock_sim.start()
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
        """Synchronous shutdown. Prefer ``await aclose()`` or
        ``async with DroneController(...)``; this remains for back-compat."""
        self._shutdown_requested = True
        self._stop_event.set()
        self._streamer.stop()
        if self._viz_publisher_task_handle is not None:
            self._viz_publisher_task_handle.cancel()
        if self._viz is not None:
            self._viz.stop()
        if self._mock_sim is not None:
            self._mock_sim.stop()
        # Heartbeat/receiver threads and the pymavlink socket are owned by the
        # connection object.
        self._connection.close()

    async def aclose(self) -> None:
        """Async shutdown: cancel the viz publisher task (awaiting it) then
        stop all threads and close the connection."""
        self._shutdown_requested = True
        if self._viz_publisher_task_handle is not None:
            self._viz_publisher_task_handle.cancel()
            try:
                await self._viz_publisher_task_handle
            except (asyncio.CancelledError, Exception):
                pass
            self._viz_publisher_task_handle = None
        self.close()

    async def __aenter__(self) -> "DroneController":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        # If we exit via an exception while still armed mid-air, attempt an
        # emergency land before tearing down the connection.
        if exc is not None and self.is_armed():
            try:
                await self.emergency_land()
            except Exception as e:
                logger.error(f"emergency_land during __aexit__ failed: {e}")
        await self.aclose()

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
        """Block until EKF reports a fresh LOCAL_POSITION_NED + AHRS health."""
        return await self._safety.wait_until_ready(timeout_s=timeout_s)

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

    async def precision_land(self, *args, **kwargs) -> "PrecisionLandResult":
        """Vision-guided descent onto a marker; returns a PrecisionLandResult."""
        return await self._precision.precision_land(*args, **kwargs)

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
