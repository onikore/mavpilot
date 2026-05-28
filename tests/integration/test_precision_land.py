"""Integration tests for precision_land semantics under mock mode.

Verifies the four non-trivial PrecisionLandStatus outcomes by feeding a
simulated marker callback under different conditions.
"""
import asyncio

import pytest

from mavpilot.controller import DroneController
from mavpilot.types import (
    MarkerObservation,
    PrecisionLandResult,
    PrecisionLandStatus,
)

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


async def _prepare_offboard_drone() -> DroneController:
    """Mock drone in OFFBOARD + armed at z=-3 m (3 m altitude)."""
    d = DroneController(mock=True, enable_viz=False, loop_hz=20.0)
    await d.connect()
    with d._tel_lock:
        d._tel["armed"] = True
        d._tel["main_mode"] = PX4_CUSTOM_MAIN_MODE_OFFBOARD
        d._tel["sub_mode"] = 0
        d._tel["local_x"] = 0.0
        d._tel["local_y"] = 0.0
        d._tel["local_z"] = -3.0  # 3 m altitude
        d._tel["landed_state"] = 2  # IN_AIR
    d._ensure_streamer_started()
    return d


@pytest.mark.asyncio
async def test_precision_land_returns_result_object():
    """Result type must be PrecisionLandResult, not bool."""
    d = await _prepare_offboard_drone()
    try:
        def marker():
            return MarkerObservation(dx=0.0, dy=0.0)

        result = await d.precision_land(
            get_marker_offset=marker,
            descent_rate_mps=2.0,
            final_altitude_m=0.3,
            horizontal_tolerance_m=0.2,
            timeout_s=10.0,
            min_altitude_floor_m=0.3,
        )
        assert isinstance(result, PrecisionLandResult)
        assert result.status in (
            PrecisionLandStatus.LANDED,
            PrecisionLandStatus.HANDED_OFF,
        )
        assert bool(result) is True
    finally:
        d.close()


@pytest.mark.asyncio
async def test_precision_land_aborts_at_floor_when_marker_lost():
    """At floor altitude with no marker: must NOT hand off; return ABORTED_AT_FLOOR.

    Drone starts just below final_altitude_m with an off-center marker so
    reached_floor is set on the first frame; on the second frame the marker
    is gone and the code must return ABORTED_AT_FLOOR (not AUTO_LAND).
    """
    d = await _prepare_offboard_drone()
    # Start below final_altitude_m so reached_floor triggers immediately.
    with d._tel_lock:
        d._tel["local_z"] = -0.25  # 0.25 m altitude, below final_altitude_m=0.3

    state = {"frames": 0}

    def marker():
        state["frames"] += 1
        if state["frames"] == 1:
            return MarkerObservation(dx=0.5, dy=0.0)  # off-center: sets reached_floor, no handoff
        return None  # lost on every subsequent frame

    try:
        result = await d.precision_land(
            get_marker_offset=marker,
            descent_rate_mps=1.0,
            final_altitude_m=0.3,
            horizontal_tolerance_m=0.2,
            timeout_s=5.0,
            min_altitude_floor_m=0.3,
            marker_lost_timeout_s=2.0,
        )
        assert result.status == PrecisionLandStatus.ABORTED_AT_FLOOR
        assert bool(result) is False
    finally:
        d.close()


@pytest.mark.asyncio
async def test_precision_land_descent_floor_is_latched():
    """Descent setpoint never goes below -min_altitude_floor_m without marker lock."""
    d = await _prepare_offboard_drone()

    seen_zs: list[float] = []

    def marker():
        with d._tel_lock:
            seen_zs.append(d._setpoint["z"])
        return MarkerObservation(dx=1.0, dy=0.0)  # large lateral error

    try:
        result = await d.precision_land(
            get_marker_offset=marker,
            descent_rate_mps=1.0,
            final_altitude_m=0.3,
            horizontal_tolerance_m=0.1,
            timeout_s=2.0,
            min_altitude_floor_m=0.5,
        )
        # In NED: z is negative-up, so z=-3.0 = 3m altitude, z=-0.5 = floor.
        # "Dipped below floor" = z became MORE POSITIVE than floor_z (closer to ground).
        floor_z = -0.5
        below = [z for z in seen_zs if z > floor_z + 0.01]
        assert not below, f"setpoint dipped below floor (z > {floor_z}): {below[:5]}"
        assert result.status in (
            PrecisionLandStatus.ABORTED_AT_FLOOR,
            PrecisionLandStatus.TIMEOUT,
        )
    finally:
        d.close()
