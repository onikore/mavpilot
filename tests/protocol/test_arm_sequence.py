"""Verify takeoff() emits the correct ordering: stream -> arm -> OFFBOARD."""

import pytest

from mavpilot.controller import DroneController


@pytest.mark.asyncio
async def test_takeoff_orders_arm_before_offboard_mode():
    """In mock mode, record the order of mode-change vs arm operations."""
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    try:
        events: list[str] = []

        original_set_mode = d._set_mode

        async def watched_set_mode(*a, **kw):
            events.append("set_mode")
            return await original_set_mode(*a, **kw)

        d._set_mode = watched_set_mode

        original_send_arm = d._send_arm

        async def watched_arm(*a, **kw):
            events.append("arm")
            return await original_send_arm(*a, **kw)

        d._send_arm = watched_arm

        original_ensure = d._ensure_streamer_started

        def watched_stream():
            events.append("stream")
            return original_ensure()

        d._ensure_streamer_started = watched_stream

        # Pre-condition: position OK and not in OFFBOARD.
        with d._tel_lock:
            d._tel["local_position_ok"] = True
            d._tel["last_local_pos_ts"] = __import__("time").time()
        await d.takeoff(altitude_m=1.0, timeout_s=5.0)

        # Expected order: stream -> arm -> set_mode (OFFBOARD).
        # (set_mode may also be called later for other reasons; we only check
        # that the first stream/arm/mode triplet is in the right order.)
        first_stream = events.index("stream")
        first_arm = events.index("arm")
        first_mode = events.index("set_mode")
        assert (
            first_stream < first_arm < first_mode
        ), f"expected stream<arm<set_mode; got order: {events}"
    finally:
        d.close()
