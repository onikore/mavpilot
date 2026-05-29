"""apply_safe_params verifies each write via PARAM_VALUE read-back.

PARAM_SET is unacknowledged; set_param_checked must confirm the value and raise
DroneError when it cannot, rather than silently leaving a safety param unset.
"""

from __future__ import annotations

import asyncio

import pytest

from mavpilot.core.commands import CommandSender
from mavpilot.core.telemetry import Telemetry
from mavpilot.errors import DroneError
from mavpilot.utils import int_to_float_bits


class ReplyConn:
    """Fake connection that replies to a PARAM_REQUEST_READ with a value.

    When ``reply_value`` is None it never replies (forces a timeout). Otherwise
    it schedules ``route_param_value(name, reply_value)`` on the running loop,
    simulating the autopilot's PARAM_VALUE response.
    """

    def __init__(self, reply_value: float | None, *, name: str = "COM_RCL_EXCEPT") -> None:
        self.mav = object()  # not None → send() proceeds
        self.target_system = 1
        self.target_component = 1
        self.sent: list[tuple] = []
        self._reply_value = reply_value
        self._name = name
        self.cs: CommandSender | None = None

    def send(self, method: str, *args, **kwargs) -> None:
        self.sent.append((method, args, kwargs))
        if method == "param_request_read_send" and self._reply_value is not None:
            assert self.cs is not None
            asyncio.get_event_loop().call_soon(
                self.cs.route_param_value, self._name, self._reply_value
            )

    def methods_sent(self) -> list[str]:
        return [m for (m, _a, _k) in self.sent]


def _make_sender(conn: ReplyConn) -> CommandSender:
    cs = CommandSender(conn, telemetry=object(), mock=False, get_target=lambda: (1, 1))
    conn.cs = cs
    return cs


@pytest.mark.asyncio
async def test_set_param_checked_success():
    conn = ReplyConn(reply_value=int_to_float_bits(7))
    cs = _make_sender(conn)

    await cs.set_param_checked("COM_RCL_EXCEPT", int_value=7, retries=3, timeout_s=0.5)

    methods = conn.methods_sent()
    assert "param_set_send" in methods
    assert "param_request_read_send" in methods


@pytest.mark.asyncio
async def test_set_param_checked_mismatch_raises():
    # Autopilot reports a different value than requested → never verifies.
    conn = ReplyConn(reply_value=int_to_float_bits(9))
    cs = _make_sender(conn)

    with pytest.raises(DroneError, match="COM_RCL_EXCEPT"):
        await cs.set_param_checked("COM_RCL_EXCEPT", int_value=7, retries=3, timeout_s=0.5)

    # Attempted the full retry budget (3 set + 3 request).
    assert conn.methods_sent().count("param_set_send") == 3


@pytest.mark.asyncio
async def test_set_param_checked_timeout_raises():
    conn = ReplyConn(reply_value=None)  # never replies
    cs = _make_sender(conn)

    with pytest.raises(DroneError, match="timeout"):
        await cs.set_param_checked("COM_RCL_EXCEPT", int_value=7, retries=2, timeout_s=0.05)


@pytest.mark.asyncio
async def test_set_param_checked_float_within_tolerance():
    conn = ReplyConn(reply_value=2.0, name="COM_OF_LOSS_T")
    cs = _make_sender(conn)
    await cs.set_param_checked("COM_OF_LOSS_T", float_value=2.0, retries=2, timeout_s=0.5)


def test_telemetry_routes_param_value():
    tel = Telemetry(connection=None)
    seen: list[tuple] = []
    tel.route_param = lambda pid, val: seen.append((pid, val))

    class _Msg:
        param_id = "COM_OF_LOSS_T"
        param_value = 2.0

        def get_type(self) -> str:
            return "PARAM_VALUE"

        def get_srcSystem(self) -> int:
            return 1

    tel.handle_message(_Msg())
    assert seen == [("COM_OF_LOSS_T", 2.0)]
