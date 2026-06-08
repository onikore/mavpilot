"""PrecisionLand — vision-guided descent onto a marker.

Drives the offboard setpoint from a marker-offset callback, enforcing the
v0.2.0 safety rules: descent below the floor and AUTO_LAND handoff are allowed
only with a visible, centered marker. Orchestrates the other collaborators via
the controller facade (``ctx``).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable

from ..errors import DroneError
from ..types import MarkerObservation, PrecisionLandResult, PrecisionLandStatus
from ..utils import body_to_ned
from .controllers import LateralController, PController

logger = logging.getLogger("drone")


class PrecisionLand:
    def __init__(self, ctx) -> None:
        self._ctx = ctx

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
        lateral_controller: LateralController | None = None,
    ) -> PrecisionLandResult:
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
        c = self._ctx
        c._check_watchdog()
        if not c.is_offboard():
            raise DroneError("precision_land() requires OFFBOARD mode")
        if not c.is_armed():
            raise DroneError("precision_land() requires armed")

        logger.info("Precision landing starting")
        c._viz_publish_command(
            "precision_land",
            timeout_s=timeout_s,
            descent_rate_mps=descent_rate_mps,
            final_altitude_m=final_altitude_m,
            min_altitude_floor_m=min_altitude_floor_m,
        )

        _ctrl = lateral_controller if lateral_controller is not None else PController(kp=lateral_p_gain)
        _ctrl.reset()
        dt = c.loop_period

        # Latch floor in NED z (negative = above ground). This is the deepest
        # the setpoint is ever commanded; only marker-locked + centered
        # descent below floor is allowed (used at handoff). The handoff fires
        # when altitude crosses final_altitude_m, so the floor must sit STRICTLY
        # BELOW it: otherwise the ramp stalls exactly at the handoff altitude
        # and — because the drone hovers a few cm above its z-setpoint from
        # position-tracking lag — altitude never actually crosses the threshold,
        # so the drone just hovers at the floor and never hands off.
        effective_floor_m = min(min_altitude_floor_m, final_altitude_m)
        if effective_floor_m >= final_altitude_m:
            effective_floor_m = max(0.05, final_altitude_m * 0.5)
            logger.info(
                f"final_altitude_m ({final_altitude_m:.2f} m) <= safety floor; "
                f"lowering descent floor to {effective_floor_m:.2f} m so the "
                f"handoff altitude stays reachable"
            )
        floor_z = -effective_floor_m

        # Independent descent ramp. Anchoring the z-setpoint to the *live*
        # position each tick (pos.z + rate*dt) makes the commanded error only
        # ~rate*dt (e.g. 1 cm at 0.5 m/s & 50 Hz), so PX4's position controller
        # crawls at a few cm/s regardless of descent_rate_mps. Instead ramp an
        # absolute commanded altitude that LEADS the drone by up to
        # descent_lead_m, giving PX4 a real tracking error so it descends at the
        # requested rate. The lead is clamped to stop the setpoint running away
        # (and lunging on recovery) if the drone ever lags; cmd_z re-inits to
        # the live position whenever descent is interrupted.
        descent_lead_m = max(0.4, descent_rate_mps)
        cmd_z: float | None = None

        start = time.time()
        last_seen = time.time()
        iterations = 0
        last_centered = False
        reached_floor = False

        while time.time() - start < timeout_s:
            iterations += 1
            pos = c.get_local_position()
            yaw = c.get_yaw_rad()
            altitude = -pos.z

            obs: MarkerObservation | None = None
            try:
                obs = get_marker_offset()
            except Exception as e:
                logger.error(f"marker callback error: {e}")

            if obs is not None:
                last_seen = time.time()
                ned_dx, ned_dy = body_to_ned(obs.dx, obs.dy, yaw)
                horizontal_err = math.hypot(ned_dx, ned_dy)
                last_centered = horizontal_err < horizontal_tolerance_m

                raw_x, raw_y = _ctrl.update(ned_dx, ned_dy, dt)
                step_x = max(-max_horizontal_step_m, min(max_horizontal_step_m, raw_x))
                step_y = max(-max_horizontal_step_m, min(max_horizontal_step_m, raw_y))
                target_x = pos.x + step_x
                target_y = pos.y + step_y

                if altitude <= final_altitude_m:
                    reached_floor = True
                    if last_centered:
                        logger.info(
                            f"Reached final altitude {altitude:.2f} m with marker "
                            f"centered (err={horizontal_err:.2f} m); handing off to AUTO_LAND"
                        )
                        landed = await c.land(
                            timeout_s=max(10.0, timeout_s - (time.time() - start))
                        )
                        return PrecisionLandResult(
                            status=(
                                PrecisionLandStatus.LANDED
                                if landed
                                else PrecisionLandStatus.HANDED_OFF
                            ),
                            final_position=c.get_local_position(),
                            iterations=iterations,
                        )
                    else:
                        # At floor but off-center — hold floor altitude, keep trying.
                        target_z = floor_z
                        cmd_z = None  # interrupt ramp; re-init on next descent
                else:
                    if last_centered:
                        if cmd_z is None:
                            cmd_z = pos.z
                        descent = descent_rate_mps * c.loop_period
                        # Ramp the commanded altitude down at descent_rate, but
                        # never lead the drone by more than descent_lead_m and
                        # never below the floor.
                        cmd_z = min(cmd_z + descent, pos.z + descent_lead_m, floor_z)
                        target_z = cmd_z
                    else:
                        target_z = pos.z  # hold current z while off-center
                        cmd_z = None  # interrupt ramp; re-init on next descent

                c._set_setpoint_position(target_x, target_y, target_z, yaw)

                if c._viz is not None:
                    c._viz.publish(
                        {
                            "type": "marker",
                            "marker_ned": {"x": pos.x + ned_dx, "y": pos.y + ned_dy},
                            "horizontal_err": horizontal_err,
                            "centered": last_centered,
                            "ts": time.time(),
                        }
                    )
            else:
                if reached_floor:
                    logger.warning(
                        "Marker lost at floor altitude — aborting precision_land "
                        "(holding floor altitude, NOT handing off to AUTO_LAND)"
                    )
                    return PrecisionLandResult(
                        status=PrecisionLandStatus.ABORTED_AT_FLOOR,
                        final_position=c.get_local_position(),
                        iterations=iterations,
                    )
                if time.time() - last_seen > marker_lost_timeout_s:
                    logger.warning(
                        f"Marker lost for {marker_lost_timeout_s}s above floor "
                        f"(altitude {altitude:.2f} m) — fallback to AUTO_LAND"
                    )
                    landed = await c.land(timeout_s=max(10.0, timeout_s - (time.time() - start)))
                    status = (
                        PrecisionLandStatus.LANDED if landed else PrecisionLandStatus.MARKER_LOST
                    )
                    return PrecisionLandResult(
                        status=status,
                        final_position=c.get_local_position(),
                        iterations=iterations,
                    )

            await asyncio.sleep(c.loop_period)

        logger.warning("Precision land timeout — returning TIMEOUT (no AUTO_LAND fallback)")
        return PrecisionLandResult(
            status=PrecisionLandStatus.TIMEOUT,
            final_position=c.get_local_position(),
            iterations=iterations,
        )
