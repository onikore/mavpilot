"""MissionOps — the sequential flight API: arm/disarm, takeoff, goto family,
set_yaw, hover, land, return_to_launch, emergency_land, and the position-reach
wait helper.

These methods orchestrate the other collaborators (telemetry, commands,
streamer, connection, viz) through the controller facade passed as ``ctx``.
Keeping a facade reference makes the inter-method calls (e.g. takeoff → arm,
goto → set_yaw) read naturally while the heavy lifting lives in the
specialised collaborators.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from pymavlink import mavutil

from ..constants import (
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
)
from ..errors import DroneError
from ..utils import body_to_ned

logger = logging.getLogger("drone")


class MissionOps:
    def __init__(self, ctx) -> None:
        self._ctx = ctx

    def check_watchdog(self) -> None:
        if self._ctx._watchdog_tripped:
            raise DroneError("telemetry lost: streamer watchdog tripped — call emergency_land()")

    async def arm(self, timeout_s: float = 10.0) -> bool:
        c = self._ctx
        if c.is_armed():
            logger.info("Already armed")
            return True
        c._viz_publish_command("arm")
        logger.info("Arming...")
        return await c._send_arm(arm=True, timeout_s=timeout_s)  # type: ignore[no-any-return]

    async def disarm(self, force: bool = False, timeout_s: float = 5.0) -> bool:
        c = self._ctx
        if not c.is_armed():
            logger.info("Already disarmed")
            return True
        c._viz_publish_command("disarm", force=force)
        logger.info(f"Disarming{' (FORCED)' if force else ''}...")
        return await c._send_arm(  # type: ignore[no-any-return]
            arm=False, force=force, timeout_s=timeout_s
        )

    async def takeoff(self, altitude_m: float, timeout_s: float = 30.0) -> bool:
        """Arm the vehicle, enter OFFBOARD mode, and climb to altitude_m.

        Order: start setpoint stream → arm → set OFFBOARD → wait position.
        PX4 firmware ≥1.13 can refuse arm-in-OFFBOARD; arming first is the
        canonical sequence.
        """
        c = self._ctx
        self.check_watchdog()
        logger.info(f"Takeoff to {altitude_m} m")
        pos = c.get_local_position()
        yaw = c.get_yaw_rad()
        c._viz_publish_command(
            "takeoff",
            altitude_m=altitude_m,
            from_pos={"x": pos.x, "y": pos.y, "z": pos.z},
            target={"x": pos.x, "y": pos.y, "z": pos.z - altitude_m},
            timeout_s=timeout_s,
        )

        # 1. Start streaming current position as the setpoint, so PX4 sees
        #    a fresh setpoint stream before any mode/arm transition.
        c._set_setpoint_position(pos.x, pos.y, pos.z, yaw)
        c._ensure_streamer_started()
        await asyncio.sleep(1.5)  # let PX4 see ~75 setpoints before any state change

        # 2. Arm first — must precede OFFBOARD on PX4 ≥1.13.
        if not c.is_armed() and not await self.arm():
            raise DroneError("Arm failed")

        # 3. Enter OFFBOARD now that we're armed and streaming.
        if not await c._set_mode(PX4_CUSTOM_MAIN_MODE_OFFBOARD):
            raise DroneError("Failed to enter OFFBOARD")

        # 4. Command the climb setpoint and wait for it.
        pos2 = c.get_local_position()
        target_z = pos2.z - altitude_m
        c._set_setpoint_position(pos2.x, pos2.y, target_z, c.get_yaw_rad())

        return await self.wait_position_reached(
            pos2.x,
            pos2.y,
            target_z,
            timeout_s=timeout_s,
            xy_tol=2.0,
            z_tol=0.5,
        )

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
        """Fly to an absolute NED position. Requires OFFBOARD + armed."""
        c = self._ctx
        self.check_watchdog()
        if not c.is_offboard():
            raise DroneError(
                f"goto() requires OFFBOARD mode, current main_mode={c.get_main_mode()}"
            )
        if not c.is_armed():
            raise DroneError("goto() requires armed")

        yaw_rad = math.radians(yaw_deg) if yaw_deg is not None else None
        from_pos = c.get_local_position()

        c._viz_publish_command(
            "goto",
            from_pos={"x": from_pos.x, "y": from_pos.y, "z": from_pos.z},
            target={"x": x, "y": y, "z": z},
            yaw_deg=yaw_deg,
            timeout_s=timeout_s,
            hover_time_s=hover_time_s,
        )
        logger.info(
            f"goto NED=({x:.2f}, {y:.2f}, {z:.2f}) "
            f"yaw={yaw_deg if yaw_deg is not None else 'face-travel-dir'}"
        )

        if yaw_rad is None:
            dx = x - from_pos.x
            dy = y - from_pos.y
            dist_xy = math.hypot(dx, dy)
            if dist_xy > 0.05:
                heading: float | None = math.atan2(dy, dx)
                try:
                    c._ensure_streamer_started()
                except DroneError:
                    logger.warning("Could not start streamer for pre-yaw; proceeding to move")
                    heading = None

                if heading is not None:
                    current_yaw = c.get_yaw_rad()
                    yaw_err = math.degrees(
                        abs(
                            math.atan2(
                                math.sin(heading - current_yaw), math.cos(heading - current_yaw)
                            )
                        )
                    )
                    deg_per_sec = (
                        math.degrees(c.yaw_slew_rate_rad) if c.yaw_slew_rate_rad > 0 else 30.0
                    )
                    yaw_timeout = min(60.0, max(2.0, yaw_err / max(1e-3, deg_per_sec) + 2.0))
                    ok = await self.set_yaw(math.degrees(heading), timeout_s=yaw_timeout)
                    if not ok:
                        logger.warning("Pre-yaw timed out; proceeding to move toward target")

        try:
            c._ensure_streamer_started()
        except DroneError as e:
            raise DroneError("Cannot start offboard streamer before goto()") from e

        c._set_setpoint_position(x, y, z, yaw_rad)

        reached = await self.wait_position_reached(x, y, z, timeout_s, xy_tol_m, z_tol_m)

        if hover_time_s > 0:
            logger.info(f"Hovering for {hover_time_s}s")
            c._viz_publish_command("hover", duration_s=hover_time_s)
            await asyncio.sleep(hover_time_s)

        return reached

    async def goto_relative(
        self, dx: float, dy: float, dz: float, yaw_deg: float | None = None, **kwargs
    ) -> bool:
        """Fly to a position offset from the current NED position."""
        pos = self._ctx.get_local_position()
        return await self.goto(pos.x + dx, pos.y + dy, pos.z + dz, yaw_deg=yaw_deg, **kwargs)

    async def goto_body_relative(
        self,
        forward_m: float,
        right_m: float,
        down_m: float,
        yaw_deg: float | None = None,
        **kwargs,
    ) -> bool:
        """Fly to a position offset in body FRD frame (no heading math required)."""
        c = self._ctx
        pos = c.get_local_position()
        yaw = c.get_yaw_rad()
        ned_dx, ned_dy = body_to_ned(forward_m, right_m, yaw)
        return await self.goto(
            pos.x + ned_dx,
            pos.y + ned_dy,
            pos.z + down_m,
            yaw_deg=yaw_deg,
            **kwargs,
        )

    async def hover(self, duration_s: float):
        """Hold current position for duration_s seconds."""
        logger.info(f"Hover {duration_s}s")
        self._ctx._viz_publish_command("hover", duration_s=duration_s)
        await asyncio.sleep(duration_s)

    async def set_yaw(self, yaw_deg: float, timeout_s: float = 10.0) -> bool:
        """Rotate in-place to yaw_deg (degrees, NED convention)."""
        c = self._ctx
        self.check_watchdog()
        pos = c.get_local_position()
        logger.info(f"Yaw → {yaw_deg}°")
        c._viz_publish_command("set_yaw", yaw_deg=yaw_deg)
        target_yaw_rad = math.radians(yaw_deg)
        c._set_setpoint_position(pos.x, pos.y, pos.z, target_yaw_rad)

        start = time.time()
        err = math.pi
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.1)
            current = c.get_yaw_rad()
            err = math.atan2(math.sin(target_yaw_rad - current), math.cos(target_yaw_rad - current))
            if abs(err) < math.radians(5.0):
                logger.info(f"Yaw reached (err {math.degrees(err):.1f}°)")
                return True
        logger.warning(f"Yaw timeout (err {math.degrees(err):.1f}°)")
        return False

    async def land(self, timeout_s: float = 60.0) -> bool:
        """Switch to AUTO_LAND and wait until on-ground."""
        c = self._ctx
        self.check_watchdog()
        logger.info("Auto LAND")
        pos = c.get_local_position()
        c._viz_publish_command(
            "land",
            from_pos={"x": pos.x, "y": pos.y, "z": pos.z},
        )

        if not await c._set_mode(
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_LAND,
        ):
            logger.warning("Land mode change rejected — sending MAV_CMD_NAV_LAND directly")
            self._raw_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)

        c._stop_streamer()

        start = time.time()
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.5)
            if c.landed_state() == 1:
                logger.info("Landed")
                return True
            if not c.is_armed():
                logger.info("Auto-disarmed after land")
                return True
        logger.warning("Land timeout")
        return False

    async def return_to_launch(self, timeout_s: float = 120.0) -> bool:
        """Switch to AUTO_RTL and wait until landed."""
        c = self._ctx
        self.check_watchdog()
        logger.info("Return to Launch")
        pos = c.get_local_position()
        c._viz_publish_command(
            "rtl",
            from_pos={"x": pos.x, "y": pos.y, "z": pos.z},
            target={"x": 0.0, "y": 0.0, "z": 0.0},
        )
        if not await c._set_mode(
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_RTL,
        ):
            return False

        c._stop_streamer()

        start = time.time()
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.5)
            # Must be ON_GROUND AND disarmed. Disarm-in-air without landing is
            # a kill-switch / failsafe; the vehicle is NOT at the launch site.
            if c.landed_state() == 1 and not c.is_armed():
                logger.info("RTL complete")
                return True
        logger.warning("RTL timeout")
        return False

    async def emergency_land(self) -> None:
        """Best-effort land. Chains AUTO_LAND → MAV_CMD_NAV_LAND → FLIGHT_TERMINATION.

        Semantics: "land NOW, where the drone currently is". Intentionally
        does NOT attempt RTL — that's a separate concern handled by
        return_to_launch(). A failed AUTO_LAND signals a serious problem
        (loss of comm, EKF fail) under which RTL is even less likely to
        succeed.

        This method intentionally IGNORES the _watchdog_tripped flag. The flag
        is set when telemetry is lost; the watchdog's job is precisely to
        surface an error that callers handle by invoking emergency_land. If
        emergency_land also raised on the flag, the safety path would
        self-cancel.
        """
        c = self._ctx
        logger.error("EMERGENCY LAND")
        c._viz_publish_command("emergency_land")

        # Step 1: try AUTO_LAND mode change + wait up to 10s for touchdown.
        try:
            landed = await self.land(timeout_s=10.0)
        except Exception as e:
            logger.error(f"land() raised during emergency_land: {e}")
            landed = False

        if landed or not c.is_armed():
            return

        # Step 2: AUTO_LAND timed out. Try MAV_CMD_NAV_LAND directly
        # (sometimes accepted when the mode-switch is stuck).
        if not c._mock and c._connection is not None and c._connection.mav is not None:
            logger.warning("AUTO_LAND timed out — sending MAV_CMD_NAV_LAND command")
            try:
                self._raw_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
            except Exception as e:
                logger.error(f"NAV_LAND command send failed: {e}")

        # Wait 5 s for landing to happen via the command.
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if c.landed_state() == 1 or not c.is_armed():
                return
            await asyncio.sleep(0.2)

        # Step 3: still in the air. Last resort — flight termination.
        if not c._mock and c._connection is not None and c._connection.mav is not None:
            logger.error("Land and NAV_LAND both timed out — sending DO_FLIGHTTERMINATION")
            try:
                self._raw_command_long(
                    mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 1, 0, 0, 0, 0, 0, 0
                )
            except Exception as e:
                logger.error(f"DO_FLIGHTTERMINATION send failed: {e}")

    def _raw_command_long(self, cmd_id: int, *params: float) -> None:
        """Fire-and-forget COMMAND_LONG (no ACK wait) for emergency paths."""
        c = self._ctx
        if c._connection is None or c._connection.mav is None:
            return
        c._connection.send(
            "command_long_send",
            c.target_system,
            c.target_component,
            cmd_id,
            0,
            *params,
        )

    async def wait_position_reached(
        self,
        x: float,
        y: float,
        z: float,
        timeout_s: float,
        xy_tol: float = 0.5,
        z_tol: float = 0.5,
    ) -> bool:
        c = self._ctx
        start = time.time()
        while time.time() - start < timeout_s:
            if c._shutdown_requested:
                raise DroneError("shutdown requested")
            await asyncio.sleep(0.2)
            pos = c.get_local_position()
            dxy = math.hypot(pos.x - x, pos.y - y)
            dz = abs(pos.z - z)
            if dxy < xy_tol and dz < z_tol:
                logger.info(
                    f"Reached ({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}), err xy={dxy:.2f} z={dz:.2f}"
                )
                return True
        pos = c.get_local_position()
        logger.warning(
            f"Position timeout: target=({x:.2f},{y:.2f},{z:.2f}) "
            f"current=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f})"
        )
        return False
