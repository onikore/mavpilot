"""The :class:`DroneController` facade — the package's main entry point.

Async wrapper around pymavlink that composes the ``mavpilot.core`` collaborators.
Background threads handle the heartbeat, incoming MAVLink messages, and the
OFFBOARD setpoint stream; user mission code runs on the asyncio event loop.
(``DroneError`` lives in :mod:`mavpilot.errors`.)
"""

import asyncio
import contextlib
import logging
import math
import threading
import time
from collections.abc import Callable

from pymavlink import mavutil

from .core.commands import CommandSender
from .core.connection import MAVLinkConnection
from .core.mission import MissionOps
from .core.mock import MockMavConnection, MockSimulator
from .core.precision_land import PrecisionLand
from .core.safety import SafetyOps
from .core.streamer import OffboardStreamer
from .core.telemetry import Telemetry
from .errors import DroneError
from .types import MarkerObservation, Position, PrecisionLandResult
from .viz import VizServer

logger = logging.getLogger("drone")


class DroneController:
    """Async facade for sequential autonomous PX4 control over MAVLink.

    Write missions as straight-line ``async``/``await`` code — ``takeoff``,
    ``goto``, ``land`` — and the controller handles the MAVLink plumbing:
    connection, the 1 Hz heartbeat, telemetry parsing, the 50 Hz OFFBOARD
    setpoint stream, command ACK tracking, and an optional browser visualizer.

    Construct it, then ``await connect()`` (or use it as an async context
    manager, which connects on entry and tears down on exit)::

        async with DroneController(mock=True) as drone:
            await drone.wait_until_ready()
            await drone.takeoff(altitude_m=5.0)
            await drone.goto(x=10, y=0, z=-5)
            await drone.land()

    Internally it composes a set of collaborators (in ``mavpilot.core``): a
    MAVLink connection, telemetry cache, command sender, OFFBOARD streamer,
    mission/precision-land logic, and a mock simulator. Background threads run
    the heartbeat, the receiver, and the setpoint streamer; your mission code
    runs on the asyncio loop and never touches threads directly.

    Coordinates are PX4 **NED** (see :class:`mavpilot.Position`). Set ``mock=True``
    to fly against the built-in simulator with no hardware or SITL.
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
        """Configure a controller. No I/O happens until :meth:`connect`.

        Args:
            connection_string: pymavlink endpoint, e.g. ``"udp:127.0.0.1:14540"``
                (SITL), ``"/dev/ttyAMA0"`` (serial), or ``"tcp:host:5760"``.
                Ignored when ``mock=True``.
            source_system: MAVLink source system id this GCS announces (1-255).
            source_component: MAVLink source component id (defaults to the
                mission-planner component).
            loop_hz: OFFBOARD setpoint publish rate in Hz. PX4 requires a
                steady stream (>2 Hz) to stay in OFFBOARD; 50 Hz is typical.
            enable_viz: Start the browser visualization server on ``connect``.
            viz_port: TCP port for the visualization server.
            viz_host: Interface the visualization server binds to. Defaults to
                loopback; use ``"0.0.0.0"`` to expose it on the LAN (telemetry
                becomes visible to the whole network).
            mock: Use the in-process physics simulator instead of a real
                MAVLink link — no hardware or SITL needed.
            yaw_slew_rate_deg: Maximum yaw rate in deg/s used to smooth heading
                changes while streaming setpoints.
            telemetry_watchdog_s: If no fresh ``LOCAL_POSITION_NED`` arrives for
                this many seconds, the streamer trips a watchdog and the next
                mission call raises :class:`mavpilot.DroneError`
                (``emergency_land`` is exempt).
        """
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

        self._viz: VizServer | None = (
            VizServer(port=viz_port, host=viz_host) if enable_viz else None
        )
        # Wire telemetry's outbound hooks now that viz exists.
        self._telemetry.viz = self._viz
        self._telemetry.route_ack = self._commands.route_command_ack
        self._viz_publisher_task_handle: asyncio.Task | None = None

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
        """Open the MAVLink link and start the background threads.

        Waits for the first autopilot heartbeat (discovering the target
        system/component), then starts the heartbeat, receiver, and — if
        ``enable_viz`` — the visualization server. In ``mock`` mode no real link
        is opened; the in-process simulator is started instead. Must be awaited
        before any flight method (``__aenter__`` calls it for you).

        Args:
            timeout_s: Seconds to wait for the initial heartbeat.
            baud: Baud rate for serial connection strings (ignored for udp/tcp).

        Raises:
            DroneError: If no heartbeat arrives within ``timeout_s`` or the
                autopilot system id cannot be determined.
        """
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
            assert self._mock_sim is not None  # mock mode always has a simulator
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
                self._viz_publisher_task_handle = asyncio.create_task(self._viz_publisher_loop())
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
        """Cancel the viz publisher task and tear everything down (preferred).

        The async counterpart of :meth:`close`: it awaits cancellation of the
        visualization publisher task before stopping the streamer/heartbeat/
        receiver threads and closing the connection. Called automatically when
        used as an async context manager. Safe to call more than once.
        """
        self._shutdown_requested = True
        if self._viz_publisher_task_handle is not None:
            self._viz_publisher_task_handle.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._viz_publisher_task_handle
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
        """Write the recommended PX4 safety parameters for OFFBOARD missions.

        Sets sensible failsafe behavior so a brief RC or link glitch doesn't
        abort an autonomous flight. No-op in mock mode.

        Args:
            com_rcl_except: ``COM_RCL_EXCEPT`` bitmask — RC-loss exceptions
                (default 7 = allow OFFBOARD/mission/hold without RC).
            com_obl_rc_act: ``COM_OBL_RC_ACT`` — action on RC loss
                (default 4 = hold, not RTL).
            com_of_loss_t: ``COM_OF_LOSS_T`` — OFFBOARD-loss timeout in seconds.
            com_rc_in_mode: ``COM_RC_IN_MODE`` (default 1 = RC stick input not
                required).
        """
        return await self._commands.apply_safe_params(
            com_rcl_except=com_rcl_except,
            com_obl_rc_act=com_obl_rc_act,
            com_of_loss_t=com_of_loss_t,
            com_rc_in_mode=com_rc_in_mode,
        )

    async def wait_until_ready(self, timeout_s: float = 60.0):
        """Block until the EKF is healthy and local position is flowing.

        Polls until a fresh ``LOCAL_POSITION_NED`` is being received AND
        ``SYS_STATUS`` reports AHRS health, which together mean the estimator
        has converged enough to arm and fly. Returns immediately in mock mode.

        Args:
            timeout_s: Maximum seconds to wait for readiness.

        Raises:
            DroneError: If readiness is not reached within ``timeout_s``.
        """
        return await self._safety.wait_until_ready(timeout_s=timeout_s)

    def get_local_position(self) -> Position:
        """Return the latest local position as a NED :class:`mavpilot.Position`
        (meters). Reads the cached telemetry; never blocks."""
        return self._telemetry.get_local_position()

    def get_yaw_rad(self) -> float:
        """Return the current heading in radians (NED, 0 = North)."""
        return self._telemetry.get_yaw_rad()

    def get_yaw_deg(self) -> float:
        """Return the current heading in degrees, normalized to ``[-180, 180)``."""
        return self._telemetry.get_yaw_deg()

    def is_armed(self) -> bool:
        """Return ``True`` if the vehicle is currently armed."""
        return self._telemetry.is_armed()

    def get_main_mode(self) -> int:
        """Return the PX4 custom *main* mode id from the latest heartbeat."""
        return self._telemetry.get_main_mode()

    def get_sub_mode(self) -> int:
        """Return the PX4 custom *sub* mode id from the latest heartbeat."""
        return self._telemetry.get_sub_mode()

    def is_offboard(self) -> bool:
        """Return ``True`` if the vehicle is currently in OFFBOARD mode."""
        return self._telemetry.is_offboard()

    def landed_state(self) -> int:
        """Return the MAVLink ``landed_state`` (1 = on ground, 2 = in air,
        3 = taking off, 4 = landing; 0 = unknown)."""
        return self._telemetry.landed_state()

    def _set_setpoint_position(self, x, y, z, yaw_rad: float | None = None):
        self._streamer.set_position(x, y, z, yaw_rad)

    def _ensure_streamer_started(self):
        self._streamer.start()

    def _stop_streamer(self):
        self._streamer.stop()

    async def send_command_long(
        self,
        cmd_id: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        timeout_s: float = 2.0,
        confirmation: int = 0,
    ) -> bool:
        """Send a ``COMMAND_LONG`` and await its terminal ``COMMAND_ACK``.

        Low-level escape hatch for issuing any ``MAV_CMD_*`` the higher-level
        methods don't wrap. The ACK is matched by ``(cmd_id, target_system,
        target_component)``; an ``IN_PROGRESS`` ack extends the deadline.

        Args:
            cmd_id: The ``MAV_CMD_*`` command id to send.
            param1: Command-specific parameter 1 (through ``param7``).
            param2: Command-specific parameter 2.
            param3: Command-specific parameter 3.
            param4: Command-specific parameter 4.
            param5: Command-specific parameter 5.
            param6: Command-specific parameter 6.
            param7: Command-specific parameter 7.
            timeout_s: Seconds to wait for a terminal ACK (reset by IN_PROGRESS).
            confirmation: MAVLink confirmation counter (0 for first transmission).

        Returns:
            ``True`` when the autopilot ACKs with ``ACCEPTED``.

        Raises:
            DroneError: On a non-``ACCEPTED`` terminal result, a timeout, or a
                duplicate command with the same key already in flight.
        """
        return await self._commands.send_command_long(
            cmd_id,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
            timeout_s=timeout_s,
            confirmation=confirmation,
        )

    async def _set_mode(self, *args, **kwargs) -> bool:
        return await self._commands.set_mode(*args, **kwargs)

    async def _send_arm(self, arm: bool, force: bool = False, timeout_s: float = 5.0) -> bool:
        return await self._commands.send_arm(arm=arm, force=force, timeout_s=timeout_s)

    async def arm(self, timeout_s: float = 10.0) -> bool:
        """Arm the vehicle and wait for confirmation.

        Args:
            timeout_s: Seconds to wait for the armed state to be confirmed.

        Returns:
            ``True`` if armed (or already armed) within ``timeout_s``.
        """
        return await self._mission.arm(timeout_s=timeout_s)

    async def disarm(self, force: bool = False, timeout_s: float = 5.0) -> bool:
        """Disarm the vehicle and wait for confirmation.

        Args:
            force: Force-disarm even in air (kill). Dangerous — only for ground
                aborts or emergencies.
            timeout_s: Seconds to wait for the disarmed state to be confirmed.

        Returns:
            ``True`` if disarmed (or already disarmed) within ``timeout_s``.
        """
        return await self._mission.disarm(force=force, timeout_s=timeout_s)

    def _check_watchdog(self) -> None:
        self._mission.check_watchdog()

    async def takeoff(self, altitude_m: float, timeout_s: float = 30.0) -> bool:
        """Arm, enter OFFBOARD, and climb to ``altitude_m`` above the takeoff point.

        Starts streaming the current position as the setpoint, arms, switches to
        OFFBOARD (in that order — PX4 ≥1.13 can refuse arm-in-OFFBOARD), then
        commands the climb and waits until the target altitude is reached.

        Args:
            altitude_m: Target height above the current position, in meters
                (positive = up).
            timeout_s: Seconds to wait for the climb to complete.

        Returns:
            ``True`` if the target altitude was reached within ``timeout_s``.

        Raises:
            DroneError: If arming or the OFFBOARD switch fails, or if the
                telemetry watchdog has tripped.
        """
        return await self._mission.takeoff(altitude_m, timeout_s=timeout_s)

    async def goto(
        self,
        x: float,
        y: float,
        z: float,
        yaw_deg: float | None = None,
        timeout_s: float = 30.0,
        hover_time_s: float = 2.0,
        xy_tol_m: float = 0.5,
        z_tol_m: float = 0.5,
    ) -> bool:
        """Fly to an absolute NED position. Requires OFFBOARD + armed.

        Args:
            x: Target North coordinate, in meters (NED).
            y: Target East coordinate, in meters (NED).
            z: Target Down coordinate, in meters (NED) — negative is up, e.g.
                ``z=-5`` is 5 m altitude.
            yaw_deg: Target heading in degrees. ``None`` (default) faces the
                direction of travel.
            timeout_s: Seconds to wait for arrival before giving up.
            hover_time_s: Seconds to hold position after arriving.
            xy_tol_m: Horizontal arrival tolerance, in meters.
            z_tol_m: Vertical arrival tolerance, in meters.

        Returns:
            ``True`` if the target was reached within tolerance before timeout.

        Raises:
            DroneError: If not in OFFBOARD/armed, or the telemetry watchdog has
                tripped.
        """
        return await self._mission.goto(
            x,
            y,
            z,
            yaw_deg=yaw_deg,
            timeout_s=timeout_s,
            hover_time_s=hover_time_s,
            xy_tol_m=xy_tol_m,
            z_tol_m=z_tol_m,
        )

    async def goto_relative(
        self, dx: float, dy: float, dz: float, yaw_deg: float | None = None, **kwargs
    ) -> bool:
        """Fly to an offset from the current position, in the NED frame.

        Convenience wrapper around :meth:`goto` that adds ``(dx, dy, dz)`` to the
        current position. Extra keyword arguments are forwarded to :meth:`goto`.

        Args:
            dx: North offset from current position, in meters.
            dy: East offset, in meters.
            dz: Down offset, in meters (negative = climb).
            yaw_deg: Target heading; ``None`` faces the direction of travel.

        Returns:
            ``True`` if the target was reached within tolerance before timeout.
        """
        return await self._mission.goto_relative(dx, dy, dz, yaw_deg=yaw_deg, **kwargs)

    async def goto_body_relative(
        self,
        forward_m: float,
        right_m: float,
        down_m: float,
        yaw_deg: float | None = None,
        **kwargs,
    ) -> bool:
        """Fly to an offset in the body FRD frame (no manual heading math).

        Converts a forward/right/down offset relative to the vehicle's current
        heading into an NED target and flies there via :meth:`goto`. Extra
        keyword arguments are forwarded to :meth:`goto`.

        Args:
            forward_m: Offset along the nose direction, in meters.
            right_m: Offset to the right, in meters.
            down_m: Offset downward, in meters (negative = climb).
            yaw_deg: Target heading; ``None`` faces the direction of travel.

        Returns:
            ``True`` if the target was reached within tolerance before timeout.
        """
        return await self._mission.goto_body_relative(
            forward_m, right_m, down_m, yaw_deg=yaw_deg, **kwargs
        )

    async def hover(self, duration_s: float):
        """Hold the current setpoint for ``duration_s`` seconds.

        Args:
            duration_s: How long to hover, in seconds.
        """
        return await self._mission.hover(duration_s)

    async def set_yaw(self, yaw_deg: float, timeout_s: float = 10.0) -> bool:
        """Rotate in place to an absolute heading (NED, slew-rate limited).

        Args:
            yaw_deg: Target heading in degrees (0 = North, 90 = East).
            timeout_s: Seconds to wait for the heading to be reached.

        Returns:
            ``True`` if the heading was reached within ~5° before timeout.

        Raises:
            DroneError: If the telemetry watchdog has tripped.
        """
        return await self._mission.set_yaw(yaw_deg, timeout_s=timeout_s)

    async def land(self, timeout_s: float = 60.0) -> bool:
        """Switch to PX4 AUTO_LAND and wait until on the ground.

        Args:
            timeout_s: Seconds to wait for touchdown (or auto-disarm).

        Returns:
            ``True`` if the vehicle reached the ground / disarmed within
            ``timeout_s``.

        Raises:
            DroneError: If the telemetry watchdog has tripped.
        """
        return await self._mission.land(timeout_s=timeout_s)

    async def precision_land(
        self,
        get_marker_offset: Callable[[], MarkerObservation | None],
        descent_rate_mps: float = 0.3,
        final_altitude_m: float = 0.5,
        horizontal_tolerance_m: float = 0.15,
        timeout_s: float = 60.0,
        lateral_p_gain: float = 0.7,
        max_horizontal_step_m: float = 1.0,
        marker_lost_timeout_s: float = 3.0,
        min_altitude_floor_m: float = 0.3,
    ) -> PrecisionLandResult:
        """Vision-guided descent onto a marker, with a hard altitude floor.

        Repeatedly calls ``get_marker_offset`` to track the pad and steer toward
        it while descending. Descending below ``min_altitude_floor_m`` and the
        final AUTO_LAND handoff are allowed **only** when the marker is visible
        and centered — otherwise the call returns
        :attr:`mavpilot.PrecisionLandStatus.ABORTED_AT_FLOOR` rather than
        landing blind. Requires OFFBOARD + armed.

        Args:
            get_marker_offset: Callback returning a
                :class:`mavpilot.MarkerObservation` (the marker's offset in the
                body FRD frame) when the pad is seen, or ``None`` when it isn't.
                See :func:`mavpilot.utils.pixel_to_body_offset`.
            descent_rate_mps: Descent speed while centered, in m/s.
            final_altitude_m: Altitude at which to hand off to AUTO_LAND, in m.
            horizontal_tolerance_m: Max horizontal error to count as "centered".
            timeout_s: Overall budget for the whole maneuver, in seconds.
            lateral_p_gain: Proportional gain mapping marker offset to lateral
                setpoint steps.
            max_horizontal_step_m: Clamp on per-iteration lateral step, in m.
            marker_lost_timeout_s: Seconds the marker may stay lost (above the
                floor) before falling back to plain AUTO_LAND.
            min_altitude_floor_m: Hard floor (meters above ground) below which
                descent is only allowed with a centered marker.

        Returns:
            A :class:`mavpilot.PrecisionLandResult` whose ``status`` describes
            the outcome; truthy only for ``LANDED`` / ``HANDED_OFF``.

        Raises:
            DroneError: If not in OFFBOARD/armed, or the telemetry watchdog has
                tripped.
        """
        return await self._precision.precision_land(
            get_marker_offset,
            descent_rate_mps=descent_rate_mps,
            final_altitude_m=final_altitude_m,
            horizontal_tolerance_m=horizontal_tolerance_m,
            timeout_s=timeout_s,
            lateral_p_gain=lateral_p_gain,
            max_horizontal_step_m=max_horizontal_step_m,
            marker_lost_timeout_s=marker_lost_timeout_s,
            min_altitude_floor_m=min_altitude_floor_m,
        )

    async def return_to_launch(self, timeout_s: float = 120.0) -> bool:
        """Switch to PX4 AUTO_RTL and wait until landed at the launch point.

        Reports success only when the vehicle is both ON_GROUND **and** disarmed
        (a mid-air disarm is treated as a failsafe, not an RTL completion).

        Args:
            timeout_s: Seconds to wait for the RTL to complete.

        Returns:
            ``True`` if landed and disarmed at launch within ``timeout_s``.

        Raises:
            DroneError: If the telemetry watchdog has tripped.
        """
        return await self._mission.return_to_launch(timeout_s=timeout_s)

    async def emergency_land(self) -> None:
        """Best-effort immediate descent: AUTO_LAND → NAV_LAND → FLIGHT_TERMINATION.

        Lands *where the drone currently is* — it does **not** attempt RTL. The
        chain escalates: try AUTO_LAND (wait ~10 s), then a direct
        ``MAV_CMD_NAV_LAND`` (wait ~5 s), and finally
        ``MAV_CMD_DO_FLIGHTTERMINATION`` (motor cut — the vehicle falls) only if
        both stall. Intentionally ignores the telemetry watchdog flag, since
        this is the recovery path the watchdog exists to trigger; it is also
        what Ctrl-C / SIGTERM invoke from the CLI.
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
                    self._viz.publish(
                        {
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
                                "x": sp["x"],
                                "y": sp["y"],
                                "z": sp["z"],
                                "yaw": (None if math.isnan(sp["yaw"]) else sp["yaw"]),
                            },
                            "ts": time.time(),
                        }
                    )
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

    async def _wait_position_reached(
        self,
        x: float,
        y: float,
        z: float,
        timeout_s: float,
        xy_tol: float = 0.5,
        z_tol: float = 0.5,
    ) -> bool:
        return await self._mission.wait_position_reached(x, y, z, timeout_s, xy_tol, z_tol)
