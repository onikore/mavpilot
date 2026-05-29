"""__aexit__ runs emergency_land on error using the sticky ever_armed flag.

A frozen-telemetry is_armed()==False must not skip the safety path once the
vehicle has been armed.
"""

from __future__ import annotations

import pytest

from mavpilot.controller import DroneController


@pytest.mark.asyncio
async def test_aexit_emergency_land_runs_when_ever_armed_even_if_not_currently_armed():
    called: list[bool] = []
    d = DroneController(mock=True, enable_viz=False)

    with pytest.raises(RuntimeError, match="boom"):
        async with d:
            await d.arm()
            assert d.ever_armed()

            # Simulate stale telemetry: current armed reads False, but the
            # sticky flag remembers we armed.
            with d._tel_lock:
                d._tel["armed"] = False
            assert not d.is_armed()

            orig = d.emergency_land

            async def spy():
                called.append(True)
                return await orig()

            d.emergency_land = spy  # type: ignore[method-assign]
            raise RuntimeError("boom")

    assert called == [True], "emergency_land should have run via ever_armed"


@pytest.mark.asyncio
async def test_aexit_skips_emergency_land_when_never_armed():
    called: list[bool] = []
    d = DroneController(mock=True, enable_viz=False)

    with pytest.raises(RuntimeError, match="nope"):
        async with d:
            assert not d.ever_armed()

            async def spy():
                called.append(True)

            d.emergency_land = spy  # type: ignore[method-assign]
            raise RuntimeError("nope")

    assert called == [], "emergency_land must not run if never armed"
