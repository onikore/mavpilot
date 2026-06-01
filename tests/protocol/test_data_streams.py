"""request_data_streams sends per-message intervals plus a legacy fallback."""

from __future__ import annotations

import pytest

from mavpilot.core.commands import CommandSender


class _RecConn:
    target_system = 1
    target_component = 1

    def __init__(self) -> None:
        self.mav = object()
        self.sent: list[str] = []

    def send(self, method: str, *_args, **_kwargs) -> None:
        self.sent.append(method)


@pytest.mark.asyncio
async def test_request_data_streams_includes_legacy_fallback():
    conn = _RecConn()
    cs = CommandSender(conn, telemetry=object(), mock=False, get_target=lambda: (1, 1))

    await cs.request_data_streams()

    # One SET_MESSAGE_INTERVAL (command_long) per requested stream...
    assert conn.sent.count("command_long_send") == 7
    # ...plus the legacy REQUEST_DATA_STREAM fallback for older stacks.
    assert "request_data_stream_send" in conn.sent


@pytest.mark.asyncio
async def test_request_data_streams_is_noop_in_mock():
    conn = _RecConn()
    cs = CommandSender(conn, telemetry=object(), mock=True, get_target=lambda: (1, 1))
    await cs.request_data_streams()
    assert conn.sent == []
