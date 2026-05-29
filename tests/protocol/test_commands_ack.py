"""Protocol tests for COMMAND_ACK routing via asyncio.Future."""

import asyncio
import contextlib
from types import SimpleNamespace

import pytest

from mavpilot.controller import DroneController
from mavpilot.errors import DroneError


class _AckMsg:
    """Stand-in for a parsed MAVLink COMMAND_ACK message."""

    def __init__(
        self, command: int, result: int, target_system: int = 255, target_component: int = 1
    ) -> None:
        self.command = command
        self.result = result
        self.target_system = target_system
        self.target_component = target_component

    def get_type(self) -> str:
        return "COMMAND_ACK"

    def get_srcSystem(self) -> int:
        return 1

    def get_srcComponent(self) -> int:
        return 1


@pytest.mark.asyncio
async def test_send_command_resolves_on_accepted_ack():
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    d.mav = SimpleNamespace(
        mav=SimpleNamespace(command_long_send=lambda *a, **k: None),
        recv_match=lambda **_: None,
        close=lambda: None,
    )

    task = asyncio.create_task(
        d.send_command_long(cmd_id=176, timeout_s=2.0)  # MAV_CMD_DO_SET_MODE
    )
    await asyncio.sleep(0.05)
    # Inject ACCEPTED (result=0).
    d._handle_message(_AckMsg(command=176, result=0))
    result = await asyncio.wait_for(task, timeout=1.0)
    assert result is True


@pytest.mark.asyncio
async def test_send_command_raises_on_denied_ack():
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    d.mav = SimpleNamespace(
        mav=SimpleNamespace(command_long_send=lambda *a, **k: None),
        recv_match=lambda **_: None,
        close=lambda: None,
    )

    task = asyncio.create_task(d.send_command_long(cmd_id=176, timeout_s=2.0))
    await asyncio.sleep(0.05)
    d._handle_message(_AckMsg(command=176, result=2))  # DENIED

    with pytest.raises(DroneError):
        await asyncio.wait_for(task, timeout=1.0)


@pytest.mark.asyncio
async def test_send_command_in_progress_extends_deadline():
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    d.mav = SimpleNamespace(
        mav=SimpleNamespace(command_long_send=lambda *a, **k: None),
        recv_match=lambda **_: None,
        close=lambda: None,
    )

    # Short timeout (0.3s); send IN_PROGRESS once, then ACCEPTED at 0.5s.
    task = asyncio.create_task(d.send_command_long(cmd_id=21, timeout_s=0.3))  # NAV_LAND
    await asyncio.sleep(0.05)
    d._handle_message(_AckMsg(command=21, result=5))  # IN_PROGRESS
    await asyncio.sleep(0.4)
    # Without IN_PROGRESS extension this would have timed out at 0.3s.
    d._handle_message(_AckMsg(command=21, result=0))  # ACCEPTED
    result = await asyncio.wait_for(task, timeout=1.0)
    assert result is True


@pytest.mark.asyncio
async def test_send_command_duplicate_in_flight_raises():
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    d.mav = SimpleNamespace(
        mav=SimpleNamespace(command_long_send=lambda *a, **k: None),
        recv_match=lambda **_: None,
        close=lambda: None,
    )

    first = asyncio.create_task(d.send_command_long(cmd_id=400, timeout_s=2.0))
    await asyncio.sleep(0.05)
    with pytest.raises(DroneError, match="duplicate"):
        await d.send_command_long(cmd_id=400, timeout_s=2.0)
    first.cancel()
    with contextlib.suppress(asyncio.CancelledError):
        await first


@pytest.mark.asyncio
async def test_send_command_timeout_raises():
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    d.mav = SimpleNamespace(
        mav=SimpleNamespace(command_long_send=lambda *a, **k: None),
        recv_match=lambda **_: None,
        close=lambda: None,
    )
    with pytest.raises(DroneError, match="timeout"):
        await d.send_command_long(cmd_id=176, timeout_s=0.2)
