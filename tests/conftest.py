"""Shared pytest fixtures for mavpilot tests."""
from __future__ import annotations

import threading
from collections import deque
from typing import Any, Optional

import pytest


class FakeMavConnection:
    """Test double that mimics pymavlink's mavfile interface enough for our use.

    Records every outgoing message in `sent` (a list of tuples
    `(method_name, args, kwargs)`). Lets tests inject incoming messages via
    `inject(msg)`; the receiver's `recv_match(timeout=...)` returns them
    FIFO and then None when the queue is drained.

    The `.mav` attribute provides the message-builder API used by the
    controller (e.g. `mav.command_long_send(...)`, `mav.heartbeat_send(...)`).
    Each builder appends to `sent` and never actually transmits bytes.
    """

    def __init__(self) -> None:
        self.sent: list[tuple[str, tuple, dict]] = []
        self._incoming: deque = deque()
        self._lock = threading.Lock()
        self.target_system = 1
        self.target_component = 1
        self.mav = _FakeMavBuilder(self)

    def inject(self, msg: Any) -> None:
        with self._lock:
            self._incoming.append(msg)

    def recv_match(self, blocking: bool = True, timeout: float = 0.05, **_: Any) -> Optional[Any]:
        with self._lock:
            if self._incoming:
                return self._incoming.popleft()
        return None

    def wait_heartbeat(self, timeout: float = 5.0) -> Optional[Any]:
        return _FakeHeartbeat(srcSystem=self.target_system, autopilot=12)

    def close(self) -> None:
        pass

    def mavlink20(self) -> None:
        pass


class _FakeMavBuilder:
    def __init__(self, parent: FakeMavConnection) -> None:
        self._parent = parent

    def __getattr__(self, name: str):
        def _record(*args, **kwargs):
            self._parent.sent.append((name, args, kwargs))
        return _record


class _FakeHeartbeat:
    """Minimal HEARTBEAT stand-in returned by FakeMavConnection.wait_heartbeat."""
    def __init__(self, srcSystem: int, autopilot: int) -> None:
        self._sys = srcSystem
        self.autopilot = autopilot
        self.base_mode = 0
        self.custom_mode = 0

    def get_type(self) -> str:
        return "HEARTBEAT"

    def get_srcSystem(self) -> int:
        return self._sys

    def get_srcComponent(self) -> int:
        return 1


@pytest.fixture
def fake_mav() -> FakeMavConnection:
    return FakeMavConnection()


@pytest.fixture
async def mock_drone():
    """A connected DroneController in mock mode, with viz disabled.

    Yields the drone; tears down via aclose() after the test.
    aclose() does not exist until Phase 3; before that, falls back to close().
    """
    from mavpilot.controller import DroneController
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    try:
        yield d
    finally:
        aclose = getattr(d, "aclose", None)
        if aclose is not None:
            await aclose()
        else:
            d.close()
