"""Public telemetry subscription API: subscribe() and telemetry_stream()."""

from __future__ import annotations

import asyncio

import pytest

from mavpilot.controller import DroneController


@pytest.mark.asyncio
async def test_subscribe_receives_snapshots_with_viz_disabled():
    received: list[dict] = []
    got = asyncio.Event()

    async with DroneController(mock=True, enable_viz=False) as d:

        def cb(snap: dict) -> None:
            received.append(snap)
            got.set()

        unsubscribe = d.subscribe(cb)
        await asyncio.wait_for(got.wait(), timeout=2.0)
        unsubscribe()

    assert received, "subscriber should have received at least one snapshot"
    snap = received[0]
    assert snap["type"] == "telemetry"
    for key in ("x", "y", "z", "yaw", "armed", "setpoint", "ts"):
        assert key in snap


@pytest.mark.asyncio
async def test_unsubscribe_stops_delivery():
    received: list[dict] = []

    async with DroneController(mock=True, enable_viz=False) as d:
        unsubscribe = d.subscribe(received.append)
        await asyncio.sleep(0.25)
        unsubscribe()
        count_after_unsub = len(received)
        await asyncio.sleep(0.3)

    assert len(received) == count_after_unsub, "no snapshots should arrive after unsubscribe"


@pytest.mark.asyncio
async def test_telemetry_stream_yields_snapshots():
    snaps: list[dict] = []

    async with DroneController(mock=True, enable_viz=False) as d:
        async for snap in d.telemetry_stream():
            snaps.append(snap)
            if len(snaps) >= 3:
                break

    assert len(snaps) == 3
    assert all(s["type"] == "telemetry" for s in snaps)
