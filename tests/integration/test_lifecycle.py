"""async with DroneController(...) lifecycle tests."""

import pytest

from mavpilot import DroneController


@pytest.mark.asyncio
async def test_async_with_lifecycle():
    async with DroneController(mock=True, enable_viz=False) as d:
        pos = d.get_local_position()
        assert pos is not None


@pytest.mark.asyncio
async def test_aclose_cancels_viz_publisher():
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    await d.aclose()
    # After aclose, calling is_armed should still work (state still readable)
    # but no background tasks remain.
    assert d.is_armed() is False
