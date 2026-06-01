import time

import pytest

from mavpilot.controller import DroneController
from mavpilot.errors import DroneError


@pytest.mark.asyncio
async def test_wait_until_ready_passes_when_ekf_healthy_in_mock():
    """Mock mode reports ekf_healthy=True instantly."""
    d = DroneController(mock=True, enable_viz=False)
    await d.connect()
    try:
        await d.wait_until_ready(timeout_s=1.0)
    finally:
        d.close()


@pytest.mark.asyncio
async def test_wait_until_ready_raises_on_unhealthy_ekf():
    """Non-mock: if ekf_healthy stays False, raises after timeout."""
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    with d._tel_lock:
        d._tel["local_position_ok"] = True
        d._tel["last_local_pos_ts"] = time.time()
        d._tel["ekf_healthy"] = False  # explicit unhealthy
    with pytest.raises(DroneError, match="AHRS"):
        await d.wait_until_ready(timeout_s=0.3)
    d.close()


@pytest.mark.asyncio
async def test_wait_until_ready_error_names_missing_position_stream():
    """When position is stale, the timeout error names that gate specifically."""
    d = DroneController(mock=False, enable_viz=False, connection_string="udp:127.0.0.1:0")
    d.target_system = 1
    d.target_component = 1
    with d._tel_lock:
        d._tel["local_position_ok"] = False  # no position fix
        d._tel["ekf_healthy"] = True
    with pytest.raises(DroneError, match="LOCAL_POSITION_NED"):
        await d.wait_until_ready(timeout_s=0.3)
    d.close()
