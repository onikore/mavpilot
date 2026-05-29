"""Smoke test inherited from v0.1.0: just verify mock connect/close lifecycle."""
import pytest

from mavpilot import DroneController


@pytest.mark.asyncio
async def test_mock_connect_close():
    async with DroneController(mock=True, enable_viz=False) as d:
        pass
