"""Telemetry watchdog: trip flag after silence; next mission method raises."""

import asyncio

import pytest

from mavpilot.controller import DroneController
from mavpilot.errors import DroneError


@pytest.mark.asyncio
async def test_watchdog_trips_after_silence_and_next_call_raises():
    d = DroneController(mock=True, enable_viz=False, telemetry_watchdog_s=0.2)
    await d.connect()
    try:
        with d._tel_lock:
            d._tel["armed"] = True
            d._tel["main_mode"] = 6
            d._tel["local_position_ok"] = True
            d._tel["last_local_pos_ts"] = __import__("time").time()
        d._ensure_streamer_started()

        # Simulate the autopilot link going silent: the mock sim stops
        # emitting telemetry, so last_local_pos_ts ages past the threshold.
        d._mock_sim_paused = True
        # Wait for streamer loop to notice (it runs at loop_hz=50 by default).
        await asyncio.sleep(0.5)
        assert d._watchdog_tripped, "watchdog should have tripped"

        # Any high-level mission method must raise DroneError now.
        with pytest.raises(DroneError, match="telemetry"):
            await d.goto(x=0, y=0, z=-1, timeout_s=0.2)
    finally:
        d.close()


@pytest.mark.asyncio
async def test_watchdog_does_not_block_emergency_land():
    d = DroneController(mock=True, enable_viz=False, telemetry_watchdog_s=0.2)
    await d.connect()
    try:
        with d._tel_lock:
            d._tel["armed"] = True
            d._tel["main_mode"] = 6
            d._tel["local_position_ok"] = True
            d._tel["last_local_pos_ts"] = __import__("time").time()
        d._ensure_streamer_started()

        # Trip the watchdog by simulating telemetry loss.
        d._mock_sim_paused = True
        await asyncio.sleep(0.5)
        assert d._watchdog_tripped

        # Restore telemetry so AUTO_LAND can complete quickly; emergency_land
        # must run despite the tripped flag (it does NOT call _check_watchdog).
        d._mock_sim_paused = False
        await d.emergency_land()
    finally:
        d.close()
