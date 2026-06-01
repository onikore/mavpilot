"""Explicit reconnect: reopens the link, restarts threads, clears latches."""

from __future__ import annotations

import pytest

from mavpilot.core import connection as conn_mod
from mavpilot.core.connection import MAVLinkConnection
from tests.conftest import FakeMavConnection


@pytest.mark.asyncio
async def test_reconnect_clears_flags_and_keeps_link_alive(mock_drone):
    d = mock_drone
    d._watchdog_tripped = True
    d._send_fault_tripped = True
    assert not d.link_alive()

    await d.reconnect()

    assert d.link_alive()
    assert not d._watchdog_tripped
    assert not d._send_fault_tripped
    # The drone is still usable afterwards.
    assert await d.arm()


@pytest.mark.asyncio
async def test_connection_reconnect_opens_a_fresh_link(monkeypatch):
    opened: list[FakeMavConnection] = []

    def fake_factory(*_args, **_kwargs) -> FakeMavConnection:
        f = FakeMavConnection()
        opened.append(f)
        return f

    monkeypatch.setattr(conn_mod.mavutil, "mavlink_connection", fake_factory)

    c = MAVLinkConnection("udp:127.0.0.1:14540")
    await c.connect(timeout_s=1.0)
    first = c.mav
    assert first is opened[0]

    await c.reconnect(timeout_s=1.0)
    assert c.mav is opened[1], "reconnect should open a new mavlink connection"
    assert c.mav is not first
    assert c.target_system == 1
    c.close()
