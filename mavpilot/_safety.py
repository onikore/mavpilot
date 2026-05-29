"""SafetyOps — pre-flight readiness gating.

wait_until_ready blocks until the EKF reports a fresh LOCAL_POSITION_NED AND
AHRS health. Reads telemetry through the controller facade (``ctx``);
apply_safe_params lives in CommandSender and is exposed by the controller.
"""
from __future__ import annotations

import asyncio
import logging
import time

from .errors import DroneError

logger = logging.getLogger("drone")


class SafetyOps:
    def __init__(self, ctx) -> None:
        self._ctx = ctx

    async def wait_until_ready(self, timeout_s: float = 60.0) -> None:
        """Block until EKF reports a fresh LOCAL_POSITION_NED."""
        c = self._ctx
        if c._mock:
            logger.info("[MOCK] EKF ready (instant)")
            return
        logger.info("Waiting for EKF (LOCAL_POSITION_NED)...")
        tel = c._telemetry
        start = time.time()
        pos_ok = ekf_ok = False
        while time.time() - start < timeout_s:
            with tel._lock:
                pos_ok = tel._tel["local_position_ok"] and (
                    time.time() - tel._tel["last_local_pos_ts"] < 2.0
                )
                ekf_ok = tel._tel["ekf_healthy"]
            if pos_ok and ekf_ok:
                logger.info("EKF ready")
                return
            await asyncio.sleep(0.5)
        raise DroneError(
            f"EKF readiness timeout (pos_ok={pos_ok}, ekf_healthy={ekf_ok})"
        )
