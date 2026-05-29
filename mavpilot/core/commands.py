"""CommandSender — COMMAND_LONG / PARAM_SET emission, mode & arm control,
and COMMAND_ACK Future routing.

Holds the pending-ACK table keyed by (cmd_id, target_sys, target_comp). The
telemetry layer forwards COMMAND_ACK frames to ``route_command_ack`` from the
receiver thread; ``send_command_long`` awaits the resulting Future with an
IN_PROGRESS-extendable deadline.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import struct
import threading
import time
from collections.abc import Callable

from pymavlink import mavutil

from ..constants import ACK_RESULT_NAMES
from ..errors import DroneError
from ..utils import int_to_float_bits

logger = logging.getLogger("drone")


class CommandSender:
    def __init__(
        self,
        connection,
        telemetry,
        mock: bool,
        get_target: Callable[[], tuple[int, int]],
    ) -> None:
        self._connection = connection
        self._telemetry = telemetry
        self._mock = mock
        self._get_target = get_target
        self._pending_acks: dict[tuple[int, int, int], dict] = {}
        self._pending_acks_lock = threading.Lock()
        self._ack_loop: asyncio.AbstractEventLoop | None = None
        # PARAM_VALUE read-back: futures keyed by normalized param name.
        self._pending_params: dict[str, asyncio.Future] = {}
        self._pending_params_lock = threading.Lock()

    @property
    def target_system(self) -> int:
        return self._get_target()[0]

    @property
    def target_component(self) -> int:
        return self._get_target()[1]

    def route_command_ack(self, command: int, result: int) -> None:
        """Resolve the pending Future for (command, target_sys, target_comp).

        Called from the receiver thread (or _handle_message in tests). Uses
        loop.call_soon_threadsafe to flip the Future on the asyncio loop.
        """
        IN_PROGRESS = 5
        ACCEPTED = 0
        key = (command, self.target_system, self.target_component)
        with self._pending_acks_lock:
            entry = self._pending_acks.get(key)
            if entry is None:
                return
            if result == IN_PROGRESS:
                entry["deadline"] += entry["base_timeout"]
                logger.debug(f"IN_PROGRESS for cmd={command}; deadline extended")
                return
            fut = entry["future"]

        if self._ack_loop is None or fut.done():
            return

        def _set() -> None:
            if fut.done():
                return
            if result == ACCEPTED:
                fut.set_result(True)
            else:
                name = ACK_RESULT_NAMES.get(result, str(result))
                fut.set_exception(DroneError(f"cmd_id={command} ACK={name}"))

        # Loop may be closed during shutdown — drop silently.
        with contextlib.suppress(RuntimeError):
            self._ack_loop.call_soon_threadsafe(_set)

    async def send_command_long(
        self,
        cmd_id: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        timeout_s: float = 2.0,
        confirmation: int = 0,
    ) -> bool:
        """Send MAV_CMD_<cmd_id> via COMMAND_LONG and await the terminal ACK.

        IN_PROGRESS resets the deadline by ``timeout_s``; terminal non-ACCEPTED
        results raise ``DroneError``. A duplicate in-flight command with the
        same (cmd_id, target_sys, target_comp) raises immediately.
        """
        if self._ack_loop is None:
            self._ack_loop = asyncio.get_running_loop()

        tgt_sys, tgt_comp = self._get_target()
        key = (cmd_id, tgt_sys, tgt_comp)
        with self._pending_acks_lock:
            if key in self._pending_acks:
                raise DroneError(f"duplicate in-flight command: cmd_id={cmd_id}")
            fut: asyncio.Future = self._ack_loop.create_future()
            self._pending_acks[key] = {
                "future": fut,
                "base_timeout": timeout_s,
                "deadline": time.monotonic() + timeout_s,
            }

        try:
            if self._mock:
                if not fut.done():
                    fut.set_result(True)
            elif self._connection is not None and self._connection.mav is not None:
                self._connection.send(
                    "command_long_send",
                    tgt_sys,
                    tgt_comp,
                    cmd_id,
                    confirmation,
                    param1,
                    param2,
                    param3,
                    param4,
                    param5,
                    param6,
                    param7,
                )

            while True:
                with self._pending_acks_lock:
                    entry = self._pending_acks.get(key)
                    if entry is None:
                        break
                    remaining = entry["deadline"] - time.monotonic()
                if remaining <= 0:
                    raise DroneError(f"COMMAND_ACK timeout for cmd_id={cmd_id}")
                try:
                    return await asyncio.wait_for(asyncio.shield(fut), timeout=remaining)
                except asyncio.TimeoutError:
                    continue
            return fut.result()  # type: ignore[no-any-return]
        except DroneError:
            raise
        except Exception as e:
            raise DroneError(f"send_command_long failed: {e}") from e
        finally:
            with self._pending_acks_lock:
                self._pending_acks.pop(key, None)

    @staticmethod
    def _normalize_param_id(param_id: object) -> str:
        if isinstance(param_id, bytes):
            param_id = param_id.decode("ascii", "ignore")
        return str(param_id).strip("\x00 \t").upper()

    def route_param_value(self, param_id: object, value: float) -> None:
        """Resolve a pending ``set_param_checked`` future for ``param_id``.

        Called from the receiver thread when a ``PARAM_VALUE`` arrives. Flips
        the matching future on the asyncio loop via call_soon_threadsafe.
        """
        key = self._normalize_param_id(param_id)
        with self._pending_params_lock:
            fut = self._pending_params.get(key)
        if fut is None or self._ack_loop is None or fut.done():
            return

        def _set() -> None:
            if not fut.done():
                fut.set_result(float(value))

        # Loop may be closed during shutdown — drop silently.
        with contextlib.suppress(RuntimeError):
            self._ack_loop.call_soon_threadsafe(_set)

    async def set_param_checked(
        self,
        name: str,
        *,
        int_value: int | None = None,
        float_value: float | None = None,
        retries: int = 3,
        timeout_s: float = 1.0,
    ) -> None:
        """Write a parameter and verify it via ``PARAM_VALUE`` read-back.

        ``PARAM_SET`` is unacknowledged in MAVLink, so a dropped frame would
        silently leave a parameter unset — unacceptable for safety params. This
        sends ``PARAM_SET`` + ``PARAM_REQUEST_READ``, awaits the resulting
        ``PARAM_VALUE``, compares it (int via the bytewise bit-cast PX4 uses;
        float within tolerance), and retries up to ``retries`` times.

        Args:
            name: PX4 parameter name (e.g. ``"COM_RCL_EXCEPT"``).
            int_value: Integer value for an ``INT32`` param. Mutually exclusive
                with ``float_value``.
            float_value: Value for a ``REAL32`` param.
            retries: Set+verify attempts before giving up.
            timeout_s: Per-attempt wait for the ``PARAM_VALUE`` reply.

        Raises:
            DroneError: If the value cannot be confirmed within ``retries``.
        """
        if (int_value is None) == (float_value is None):
            raise ValueError("set_param_checked needs exactly one of int_value/float_value")

        if int_value is not None:
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32
            wire = int_to_float_bits(int_value)
        else:
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            wire = float(float_value)  # type: ignore[arg-type]

        if self._ack_loop is None:
            self._ack_loop = asyncio.get_running_loop()

        key = self._normalize_param_id(name)
        tgt_sys, tgt_comp = self._get_target()
        last_err = "no PARAM_VALUE received"
        for attempt in range(1, retries + 1):
            fut: asyncio.Future = self._ack_loop.create_future()
            with self._pending_params_lock:
                self._pending_params[key] = fut
            try:
                self._connection.send(
                    "param_set_send", tgt_sys, tgt_comp, name.encode(), wire, ptype
                )
                self._connection.send(
                    "param_request_read_send", tgt_sys, tgt_comp, name.encode(), -1
                )
                try:
                    got = await asyncio.wait_for(fut, timeout=timeout_s)
                except asyncio.TimeoutError:
                    last_err = f"timeout after {timeout_s}s"
                    logger.warning(
                        f"param {name} verify attempt {attempt}/{retries} failed: {last_err}"
                    )
                    continue
            finally:
                with self._pending_params_lock:
                    self._pending_params.pop(key, None)

            if int_value is not None:
                got_int = struct.unpack("<i", struct.pack("<f", got))[0]
                if got_int == int_value:
                    logger.info(f"param {name} = {int_value} (verified)")
                    return
                last_err = f"read-back {got_int} != {int_value}"
            else:
                if math.isclose(got, float_value, rel_tol=1e-4, abs_tol=1e-4):  # type: ignore[arg-type]
                    logger.info(f"param {name} = {float_value} (verified)")
                    return
                last_err = f"read-back {got} != {float_value}"
            logger.warning(f"param {name} verify attempt {attempt}/{retries} failed: {last_err}")

        raise DroneError(f"failed to set/verify param {name}: {last_err}")

    async def set_mode(
        self,
        custom_main_mode: int,
        custom_sub_mode: int = 0,
        wait_for_confirm_s: float = 3.0,
    ) -> bool:
        tel = self._telemetry
        if self._mock:
            with tel._lock:
                tel._tel["main_mode"] = custom_main_mode
                tel._tel["sub_mode"] = custom_sub_mode
            logger.info(f"[MOCK] Mode → main={custom_main_mode} sub={custom_sub_mode}")
            await asyncio.sleep(0.05)
            return True
        tgt_sys, tgt_comp = self._get_target()
        self._connection.send(
            "command_long_send",
            tgt_sys,
            tgt_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(custom_main_mode),
            float(custom_sub_mode),
            0,
            0,
            0,
            0,
        )
        start = time.time()
        while time.time() - start < wait_for_confirm_s:
            await asyncio.sleep(0.1)
            if tel.get_main_mode() == custom_main_mode and (
                custom_sub_mode == 0 or tel.get_sub_mode() == custom_sub_mode
            ):
                logger.info(f"Mode → main={custom_main_mode} sub={custom_sub_mode}")
                return True
        logger.warning(
            f"Mode change timeout: requested main={custom_main_mode} sub={custom_sub_mode}, "
            f"actual main={tel.get_main_mode()} sub={tel.get_sub_mode()}"
        )
        return False

    async def send_arm(self, arm: bool, force: bool = False, timeout_s: float = 5.0) -> bool:
        tel = self._telemetry
        if self._mock:
            with tel._lock:
                tel._tel["armed"] = arm
                if arm:
                    tel._tel["ever_armed"] = True
            logger.info(f"[MOCK] {'Armed' if arm else 'Disarmed'}")
            await asyncio.sleep(0.05)
            return True
        param1 = 1.0 if arm else 0.0
        param2 = 21196.0 if force else 0.0
        tgt_sys, tgt_comp = self._get_target()
        self._connection.send(
            "command_long_send",
            tgt_sys,
            tgt_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            param1,
            param2,
            0,
            0,
            0,
            0,
            0,
        )
        start = time.time()
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.1)
            if tel.is_armed() == arm:
                logger.info(f"{'Armed' if arm else 'Disarmed'} (after {time.time() - start:.1f}s)")
                return True
        logger.error(f"{'Arm' if arm else 'Disarm'} timeout")
        return False

    async def apply_safe_params(
        self,
        com_rcl_except: int = 7,
        com_obl_rc_act: int = 4,
        com_of_loss_t: float = 2.0,
        com_rc_in_mode: int = 1,
    ) -> None:
        """Write recommended PX4 safety parameters, verifying each via read-back.

        Each parameter is set with :meth:`set_param_checked`, so a dropped
        ``PARAM_SET`` no longer leaves a safety parameter silently unset:
        a value that cannot be confirmed raises :class:`DroneError`.
        """
        if self._mock:
            logger.info(
                f"[MOCK] apply_safe_params: rcl_except={com_rcl_except}, "
                f"obl_rc_act={com_obl_rc_act}, of_loss_t={com_of_loss_t}, "
                f"rc_in_mode={com_rc_in_mode} (no-op)"
            )
            return
        await self.set_param_checked("COM_RCL_EXCEPT", int_value=com_rcl_except)
        await self.set_param_checked("COM_OBL_RC_ACT", int_value=com_obl_rc_act)
        await self.set_param_checked("COM_OF_LOSS_T", float_value=com_of_loss_t)
        await self.set_param_checked("COM_RC_IN_MODE", int_value=com_rc_in_mode)

    async def request_data_streams(self) -> None:
        if self._mock:
            return
        tgt_sys, tgt_comp = self._get_target()
        wanted = [
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1),
        ]
        for msg_id, hz in wanted:
            interval_us = int(1e6 / hz)
            self._connection.send(
                "command_long_send",
                tgt_sys,
                tgt_comp,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                float(interval_us),
                0,
                0,
                0,
                0,
                0,
            )
            await asyncio.sleep(0.05)
