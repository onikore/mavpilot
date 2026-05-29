"""Unit tests for the CLI shutdown signal handler.

The mission loop must trigger emergency_land on:
  * Any uncaught Exception (already worked in v0.1.0)
  * KeyboardInterrupt (Ctrl-C — was broken in v0.1.0)
  * Receiving SIGINT/SIGTERM during a long-running await
"""

import asyncio

import pytest


class _RecordingDrone:
    """Stand-in for DroneController used to verify the CLI's shutdown path."""

    def __init__(self) -> None:
        self.emergency_land_called = False
        self.close_called = False

    async def connect(self, timeout_s: float = 30.0) -> None:
        pass

    async def apply_safe_params(self) -> None:
        pass

    async def wait_until_ready(self, timeout_s: float = 60.0) -> None:
        pass

    async def takeoff(self, altitude_m: float, timeout_s: float = 30.0) -> bool:
        await asyncio.sleep(60.0)
        return True

    async def goto(self, *args, **kwargs) -> bool:
        await asyncio.sleep(60.0)
        return True

    async def disarm(self) -> bool:
        return True

    async def emergency_land(self) -> None:
        self.emergency_land_called = True

    def close(self) -> None:
        self.close_called = True

    def get_local_position(self):
        from mavpilot.types import Position

        return Position(0.0, 0.0, 0.0)


@pytest.mark.asyncio
async def test_shutdown_handler_calls_emergency_land():
    """handle_shutdown_signal(drone, ...) must await drone.emergency_land()."""
    from mavpilot.cli import handle_shutdown_signal

    drone = _RecordingDrone()
    await handle_shutdown_signal(drone, reason="SIGINT")
    assert drone.emergency_land_called


@pytest.mark.asyncio
async def test_run_mission_with_keyboard_interrupt_triggers_emergency_land():
    """If the mission body raises KeyboardInterrupt, run_mission must
    still call emergency_land() (not let it propagate uncaught)."""
    from mavpilot.cli import run_mission

    drone = _RecordingDrone()

    async def boom():
        raise KeyboardInterrupt()

    await run_mission(drone, mission_body=boom)
    assert drone.emergency_land_called
    assert drone.close_called


@pytest.mark.asyncio
async def test_run_mission_with_exception_triggers_emergency_land():
    from mavpilot.cli import run_mission

    drone = _RecordingDrone()

    async def boom():
        raise RuntimeError("simulated mission failure")

    await run_mission(drone, mission_body=boom)
    assert drone.emergency_land_called
    assert drone.close_called


@pytest.mark.asyncio
async def test_run_mission_normal_completion_no_emergency_land():
    from mavpilot.cli import run_mission

    drone = _RecordingDrone()

    async def normal():
        pass

    await run_mission(drone, mission_body=normal)
    assert not drone.emergency_land_called
    assert drone.close_called
