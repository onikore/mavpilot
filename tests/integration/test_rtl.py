"""Integration tests for return_to_launch correctness."""
import asyncio

import pytest

from mavpilot.controller import DroneController


@pytest.mark.asyncio
async def test_rtl_reports_complete_only_when_landed_and_disarmed():
    """RTL must require BOTH landed_state==ON_GROUND AND not armed.

    Manually setting armed=False mid-air must NOT report 'RTL complete'.
    """
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    try:
        # Simulate mid-air state: armed, 10 m above ground.
        # local_z is negative-down (NED), so -10.0 = 10 m altitude.
        # Setting local_z keeps the mock sim from snapping landed_state back to 1.
        with d._tel_lock:
            d._tel["armed"] = True
            d._tel["local_z"] = -10.0
            d._tel["landed_state"] = 2  # MAV_LANDED_STATE_IN_AIR

        # Spawn the RTL call, then force armed=False while still mid-air.
        async def disarm_mid_air():
            await asyncio.sleep(0.5)
            with d._tel_lock:
                d._tel["armed"] = False
                # landed_state stays = 2 (IN_AIR) — pilot pulled the kill switch.

        # RTL with short timeout to bound the test.
        task = asyncio.create_task(disarm_mid_air())
        result = await d.return_to_launch(timeout_s=2.0)
        await task

        # With the bug, this asserts True (test fails before fix).
        # After fix, result is False because landed_state != ON_GROUND.
        assert result is False, (
            "RTL must NOT report complete when vehicle is disarmed mid-air "
            "without touching ground"
        )
    finally:
        d.close()


@pytest.mark.asyncio
async def test_rtl_reports_complete_when_landed_and_disarmed():
    """The happy path: both conditions met → True."""
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    try:
        async def land_and_disarm():
            await asyncio.sleep(0.5)
            with d._tel_lock:
                d._tel["landed_state"] = 1  # ON_GROUND
                d._tel["armed"] = False

        task = asyncio.create_task(land_and_disarm())
        result = await d.return_to_launch(timeout_s=2.0)
        await task
        assert result is True
    finally:
        d.close()
