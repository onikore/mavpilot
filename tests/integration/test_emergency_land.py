"""Integration tests for emergency_land() chain semantics."""

from unittest.mock import patch

import pytest

from mavpilot.controller import DroneController


@pytest.mark.asyncio
async def test_emergency_land_succeeds_via_auto_land():
    """Happy path: AUTO_LAND mode change → drone touches ground → exit."""
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    try:
        with d._tel_lock:
            d._tel["armed"] = True
            d._tel["local_z"] = -3.0
            d._tel["landed_state"] = 2  # IN_AIR
        d._ensure_streamer_started()

        await d.emergency_land()

        assert d.landed_state() == 1 or not d.is_armed()
    finally:
        d.close()


@pytest.mark.asyncio
async def test_emergency_land_sends_termination_on_land_timeout():
    """If AUTO_LAND doesn't reach ground within timeout and NAV_LAND
    fallback also fails, DO_FLIGHTTERMINATION must be sent."""
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    sent_commands: list[int] = []

    class _FakeMavInner:
        def command_long_send(self, sys_, comp, cmd_id, *_args, **_kw):
            sent_commands.append(cmd_id)

        def __getattr__(self, _name):
            def _no(*_a, **_kw):
                return None

            return _no

    class _FakeMav:
        def __init__(self) -> None:
            self.mav = _FakeMavInner()

        def close(self):
            pass

    d.mav = _FakeMav()
    # Drone must appear armed; otherwise emergency_land early-exits on "already disarmed".
    with d._tel_lock:
        d._tel["armed"] = True
        d._tel["landed_state"] = 2  # IN_AIR

    async def fake_land(timeout_s: float = 60.0) -> bool:
        return False

    with patch.object(d, "land", side_effect=fake_land):
        await d.emergency_land()

    from pymavlink import mavutil

    assert (
        mavutil.mavlink.MAV_CMD_NAV_LAND in sent_commands
    ), f"expected NAV_LAND command in sent={sent_commands}"
    assert (
        mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION in sent_commands
    ), f"expected FLIGHTTERMINATION in sent={sent_commands}"

    d.close()
