"""Telemetry watchdog: trip flag after silence; next mission method raises.

Also covers the outbound send-fault latch (streamer can't send for ~0.5 s).
"""

import asyncio
import threading
import time

import pytest

from mavpilot.controller import DroneController
from mavpilot.core.mission import MissionOps
from mavpilot.core.streamer import OffboardStreamer
from mavpilot.core.telemetry import Telemetry
from mavpilot.errors import DroneError


@pytest.mark.asyncio
async def test_watchdog_trips_after_silence_and_next_call_raises():
    d = DroneController(mock=True, enable_viz=False, telemetry_watchdog_s=0.2)
    await d.connect()
    try:
        with d._tel_lock:
            d._tel["armed"] = True
            d._tel["main_mode"] = 6
            d._tel["local_position_ok"] = True
            d._tel["last_local_pos_ts"] = __import__("time").time()
        d._ensure_streamer_started()

        # Simulate the autopilot link going silent: the mock sim stops
        # emitting telemetry, so last_local_pos_ts ages past the threshold.
        d._mock_sim_paused = True
        # Wait for streamer loop to notice (it runs at loop_hz=50 by default).
        await asyncio.sleep(0.5)
        assert d._watchdog_tripped, "watchdog should have tripped"

        # Any high-level mission method must raise DroneError now.
        with pytest.raises(DroneError, match="telemetry"):
            await d.goto(x=0, y=0, z=-1, timeout_s=0.2)
    finally:
        d.close()


@pytest.mark.asyncio
async def test_watchdog_does_not_block_emergency_land():
    d = DroneController(mock=True, enable_viz=False, telemetry_watchdog_s=0.2)
    await d.connect()
    try:
        with d._tel_lock:
            d._tel["armed"] = True
            d._tel["main_mode"] = 6
            d._tel["local_position_ok"] = True
            d._tel["last_local_pos_ts"] = __import__("time").time()
        d._ensure_streamer_started()

        # Trip the watchdog by simulating telemetry loss.
        d._mock_sim_paused = True
        await asyncio.sleep(0.5)
        assert d._watchdog_tripped

        # Restore telemetry so AUTO_LAND can complete quickly; emergency_land
        # must run despite the tripped flag (it does NOT call _check_watchdog).
        d._mock_sim_paused = False
        await d.emergency_land()
    finally:
        d.close()


class _RaisingConn:
    """Connection whose every send raises — simulates a dead outbound link."""

    target_system = 1
    target_component = 1

    def send(self, *_args, **_kwargs):
        raise OSError("link down")


def test_streamer_latches_send_fault_after_persistent_failures():
    conn = _RaisingConn()
    tel = Telemetry(conn)
    with tel._lock:
        tel._tel["local_position_ok"] = True
        tel._tel["last_local_pos_ts"] = time.time()
    stop = threading.Event()
    streamer = OffboardStreamer(
        connection=conn,
        telemetry=tel,
        mock=False,
        loop_hz=50.0,
        yaw_slew_rate_rad=0.26,
        telemetry_watchdog_s=100.0,  # keep the *incoming* watchdog out of the way
        stop_event=stop,
        proc_start_monotonic=time.monotonic(),
    )
    streamer.start()
    try:
        deadline = time.time() + 3.0
        while time.time() < deadline and not streamer.send_fault_tripped:
            time.sleep(0.02)
        assert streamer.send_fault_tripped, "send fault should latch after ~0.5 s of failures"
        assert not streamer.watchdog_tripped, "incoming watchdog must not have tripped"
    finally:
        streamer.stop()
        stop.set()


def test_check_watchdog_raises_on_send_fault():
    class _Ctx:
        _watchdog_tripped = False
        _send_fault_tripped = True

    with pytest.raises(DroneError, match="link"):
        MissionOps(_Ctx()).check_watchdog()
