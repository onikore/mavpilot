"""DroneController class and DroneError exception.

Async wrapper around pymavlink. Background threads handle the heartbeat,
incoming MAVLink messages, and offboard setpoint streaming. The asyncio
event loop runs user mission code on top.
"""
import asyncio
import logging
import math
import struct
import threading
import time
from typing import Callable, Optional

from pymavlink import mavutil

from .constants import (
    ACK_RESULT_NAMES,
    DEFAULT_POS_TYPE_MASK,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
)
from .types import Position, MarkerObservation
from .utils import int_to_float_bits, body_to_ned
from .viz import VizServer

logger = logging.getLogger("drone")


class DroneError(RuntimeError):
    """Any non-standard situation: command failure, timeout, loss-of-comm."""


class DroneController:
    """Async wrapper around pymavlink for sequential autonomous control.

    Architecture:
      - heartbeat_thread (1 Hz)
      - receiver_thread — parses incoming MAVLink into self._tel
      - streamer_thread — publishes offboard setpoints at loop_hz
      - viz_server — optional HTTP+SSE browser UI
      - asyncio main loop — user mission code
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14540",
        source_system: int = 255,
        source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        loop_hz: float = 50.0,
        enable_viz: bool = True,
        viz_port: int = 8765,
        mock: bool = False,
        yaw_slew_rate_deg: float = 15.0,
    ):
        self.connection_string = connection_string
        self.source_system = source_system
        self.source_component = source_component
        self.loop_hz = loop_hz
        self.loop_period = 1.0 / loop_hz
        self._mock = mock

        self.mav: Optional[mavutil.mavfile] = None
        self.target_system = 0
        self.target_component = 0
        self._mock_sim_thread: Optional[threading.Thread] = None

        self._setpoint_lock = threading.Lock()
        self._setpoint = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "yaw": float("nan"),
            "yaw_target": float("nan"),
            "type_mask": DEFAULT_POS_TYPE_MASK,
        }
        self._streaming = False
        self._streamer_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._receiver_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._tel_lock = threading.Lock()
        self._tel: dict = {
            "armed": False,
            "custom_mode": 0,
            "main_mode": 0,
            "sub_mode": 0,
            "local_x": 0.0,
            "local_y": 0.0,
            "local_z": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "yaw": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "battery_remaining": 1.0,
            "landed_state": 0,
            "local_position_ok": False,
            "last_local_pos_ts": 0.0,
            "last_ack": None,
        }

        self._viz: Optional[VizServer] = VizServer(viz_port) if enable_viz else None
        self._viz_publisher_task_handle: Optional[asyncio.Task] = None

        self._shutdown_requested = False
        self.yaw_slew_rate_rad = math.radians(yaw_slew_rate_deg)

    async def connect(self, timeout_s: float = 30.0, baud: int = 57600):
        if self._mock:
            logger.info("[MOCK MODE] no MAVLink connection, starting simulator")
            self.target_system = 1
            self.target_component = 1
            with self._tel_lock:
                self._tel["local_position_ok"] = True
                self._tel["last_local_pos_ts"] = time.time()
                self._tel["landed_state"] = 1
            if self._viz is not None:
                try:
                    self._viz.start()
                    self._viz_publisher_task_handle = asyncio.create_task(
                        self._viz_publisher_loop()
                    )
                except OSError as e:
                    logger.warning(f"Viz server failed to start: {e}")
                    self._viz = None
            self._start_mock_sim_thread()
            return

        logger.info(f"Connecting to {self.connection_string}")
        kwargs = {
            "source_system": self.source_system,
            "source_component": self.source_component,
        }
        if not (
            self.connection_string.startswith("udp")
            or self.connection_string.startswith("tcp")
        ):
            kwargs["baud"] = baud
        self.mav = mavutil.mavlink_connection(self.connection_string, **kwargs)
        self.mav.mavlink20()

        loop = asyncio.get_event_loop()
        hb = await asyncio.wait_for(
            loop.run_in_executor(None, lambda: self.mav.wait_heartbeat(timeout=timeout_s)),
            timeout=timeout_s + 5,
        )
        if hb is None:
            raise DroneError("No heartbeat received")

        self.target_system = self.mav.target_system
        self.target_component = self.mav.target_component
        if self.target_system == 0:
            try:
                src_sys = hb.get_srcSystem()
                src_comp = hb.get_srcComponent()
            except Exception:
                src_sys = None
                src_comp = None

            try:
                is_ap = getattr(hb, "autopilot", None) is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            except Exception:
                is_ap = False

            if src_sys and is_ap:
                self.target_system = src_sys
                self.target_component = src_comp
                logger.warning(
                    "Inferred target from heartbeat: sys=%s comp=%s",
                    self.target_system,
                    self.target_component,
                )
            else:
                found = False
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    try:
                        msg = await loop.run_in_executor(None, lambda: self.mav.recv_match(blocking=True, timeout=1.0))
                    except Exception:
                        msg = None
                    if msg is None:
                        continue
                    if msg.get_type() == "HEARTBEAT":
                        src = msg.get_srcSystem()
                        comp = msg.get_srcComponent()
                        try:
                            ap = getattr(msg, "autopilot", None)
                        except Exception:
                            ap = None
                        if ap is not None and ap != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                            self.target_system = src
                            self.target_component = comp
                            found = True
                            logger.warning("Detected autopilot heartbeat from sys=%s comp=%s", src, comp)
                            break
                if not found and self.target_system == 0:
                    if src_sys:
                        self.target_system = src_sys
                        self.target_component = src_comp
                        logger.warning(
                            "Falling back to initial heartbeat source sys=%s comp=%s",
                            src_sys,
                            src_comp,
                        )
                    else:
                        raise DroneError(
                            "target_system == 0 after heartbeat. Could not determine autopilot sysid. "
                            "Ensure SITL/autopilot is running and the connection string is correct."
                        )
        logger.info(
            f"Heartbeat from sys={self.target_system} comp={self.target_component} "
            f"src_sys={self.source_system} src_comp={self.source_component}"
        )

        self._start_heartbeat_thread()
        self._start_receiver_thread()
        await self._request_data_streams()

        if self._viz is not None:
            try:
                self._viz.start()
                self._viz_publisher_task_handle = asyncio.create_task(
                    self._viz_publisher_loop()
                )
            except OSError as e:
                logger.warning(f"Viz server failed to start (port busy?): {e}")
                self._viz = None

    def close(self):
        self._shutdown_requested = True
        self._streaming = False
        self._stop_event.set()
        if self._viz_publisher_task_handle is not None:
            self._viz_publisher_task_handle.cancel()
        if self._viz is not None:
            self._viz.stop()
        for thr in (
            self._streamer_thread,
            self._heartbeat_thread,
            self._receiver_thread,
            self._mock_sim_thread,
        ):
            if thr is not None and thr.is_alive():
                thr.join(timeout=2.0)
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass

    def _start_heartbeat_thread(self):
        if self._mock:
            return

        def loop():
            while not self._stop_event.is_set():
                try:
                    self.mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                except Exception as e:
                    logger.warning(f"heartbeat send error: {e}")
                if self._stop_event.wait(1.0):
                    break

        self._heartbeat_thread = threading.Thread(target=loop, daemon=True, name="hb")
        self._heartbeat_thread.start()

    def _start_mock_sim_thread(self):
        max_speed_xy = 3.0
        max_speed_z = 1.5
        max_yaw_rate = self.yaw_slew_rate_rad

        def loop():
            last = time.time()
            while not self._stop_event.is_set():
                now = time.time()
                dt = max(0.001, min(0.05, now - last))
                last = now

                with self._setpoint_lock:
                    sp = dict(self._setpoint)

                with self._tel_lock:
                    armed = self._tel["armed"]
                    main_mode = self._tel["main_mode"]
                    sub_mode = self._tel["sub_mode"]
                    cx = self._tel["local_x"]
                    cy = self._tel["local_y"]
                    cz = self._tel["local_z"]
                    cyaw = self._tel["yaw"]

                target_x, target_y, target_z, target_yaw = cx, cy, cz, cyaw
                tracking = False

                if armed and main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                    target_x, target_y, target_z = sp["x"], sp["y"], sp["z"]
                    if not math.isnan(sp["yaw"]):
                        target_yaw = sp["yaw"]
                    tracking = True

                elif armed and main_mode == PX4_CUSTOM_MAIN_MODE_AUTO:
                    if sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                        target_x, target_y = cx, cy
                        target_z = 0.0
                        tracking = True
                    elif sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                        target_x, target_y = 0.0, 0.0
                        if math.hypot(cx, cy) < 0.5:
                            target_z = 0.0
                        else:
                            target_z = cz
                        tracking = True
                    elif sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                        target_z = sp["z"] if sp["z"] < cz else cz - 5.0
                        tracking = True
                    elif sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                        target_x, target_y, target_z = cx, cy, cz
                        tracking = True

                new_x, new_y, new_z, new_yaw = cx, cy, cz, cyaw
                vx, vy, vz = 0.0, 0.0, 0.0

                if tracking:
                    dx = target_x - cx
                    dy = target_y - cy
                    dz = target_z - cz
                    dist_xy = math.hypot(dx, dy)
                    max_step_xy = max_speed_xy * dt
                    if dist_xy > max_step_xy:
                        ratio = max_step_xy / dist_xy
                        new_x = cx + dx * ratio
                        new_y = cy + dy * ratio
                        vx = dx * ratio / dt
                        vy = dy * ratio / dt
                    else:
                        new_x = target_x
                        new_y = target_y
                        vx = dx / dt if dt > 0 else 0.0
                        vy = dy / dt if dt > 0 else 0.0

                    max_step_z = max_speed_z * dt
                    if abs(dz) > max_step_z:
                        new_z = cz + math.copysign(max_step_z, dz)
                        vz = math.copysign(max_speed_z, dz)
                    else:
                        new_z = target_z
                        vz = dz / dt if dt > 0 else 0.0

                    yaw_err = math.atan2(
                        math.sin(target_yaw - cyaw), math.cos(target_yaw - cyaw)
                    )
                    max_yaw_step = max_yaw_rate * dt
                    if abs(yaw_err) > max_yaw_step:
                        new_yaw = cyaw + math.copysign(max_yaw_step, yaw_err)
                    else:
                        new_yaw = target_yaw

                if new_z > 0:
                    new_z = 0.0

                with self._tel_lock:
                    self._tel["local_x"] = new_x
                    self._tel["local_y"] = new_y
                    self._tel["local_z"] = new_z
                    self._tel["yaw"] = new_yaw
                    self._tel["vx"] = vx
                    self._tel["vy"] = vy
                    self._tel["vz"] = vz
                    self._tel["last_local_pos_ts"] = now

                    if new_z >= -0.05:
                        self._tel["landed_state"] = 1
                        if (
                            self._tel["armed"]
                            and self._tel["main_mode"] == PX4_CUSTOM_MAIN_MODE_AUTO
                            and self._tel["sub_mode"]
                            in (
                                PX4_CUSTOM_SUB_MODE_AUTO_LAND,
                                PX4_CUSTOM_SUB_MODE_AUTO_RTL,
                            )
                        ):
                            self._tel["armed"] = False
                            logger.info("[MOCK] Auto-disarmed after land")
                    elif self._tel["armed"]:
                        self._tel["landed_state"] = 2

                time.sleep(0.01)

        self._mock_sim_thread = threading.Thread(target=loop, daemon=True, name="mock-sim")
        self._mock_sim_thread.start()

    def _start_receiver_thread(self):
        def loop():
            while not self._stop_event.is_set():
                try:
                    msg = self.mav.recv_match(blocking=True, timeout=0.5)
                    if msg is None:
                        continue
                    self._handle_message(msg)
                except Exception as e:
                    logger.error(f"receiver error: {e}")
                    time.sleep(0.1)

        self._receiver_thread = threading.Thread(target=loop, daemon=True, name="recv")
        self._receiver_thread.start()

    def _handle_message(self, msg):
        t = msg.get_type()
        now = time.time()
        with self._tel_lock:
            if t == "HEARTBEAT" and msg.get_srcSystem() == self.target_system:
                self._tel["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                cm = msg.custom_mode
                self._tel["custom_mode"] = cm
                self._tel["main_mode"] = (cm >> 16) & 0xFF
                self._tel["sub_mode"] = (cm >> 24) & 0xFF
            elif t == "LOCAL_POSITION_NED":
                self._tel["local_x"] = msg.x
                self._tel["local_y"] = msg.y
                self._tel["local_z"] = msg.z
                self._tel["vx"] = msg.vx
                self._tel["vy"] = msg.vy
                self._tel["vz"] = msg.vz
                self._tel["last_local_pos_ts"] = now
                self._tel["local_position_ok"] = True
            elif t == "ATTITUDE":
                self._tel["roll"] = msg.roll
                self._tel["pitch"] = msg.pitch
                self._tel["yaw"] = msg.yaw
            elif t == "EXTENDED_SYS_STATE":
                self._tel["landed_state"] = msg.landed_state
            elif t == "BATTERY_STATUS":
                if msg.battery_remaining >= 0:
                    self._tel["battery_remaining"] = msg.battery_remaining / 100.0
            elif t == "STATUSTEXT":
                text = msg.text.rstrip("\0\t ") if isinstance(msg.text, str) else msg.text
                sev = msg.severity
                if sev <= 3:
                    logger.error(f"PX4: {text}")
                elif sev <= 5:
                    logger.warning(f"PX4: {text}")
                else:
                    logger.info(f"PX4: {text}")
                if self._viz is not None:
                    self._viz.publish({
                        "type": "log",
                        "severity": sev,
                        "text": text,
                        "ts": now,
                    })
            elif t == "COMMAND_ACK":
                r = ACK_RESULT_NAMES.get(msg.result, str(msg.result))
                self._tel["last_ack"] = (msg.command, msg.result)
                level = logging.INFO if msg.result == 0 else logging.WARNING
                logger.log(level, f"ACK cmd={msg.command} result={r}")

    async def _request_data_streams(self):
        if self._mock:
            return
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
            self.mav.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id), float(interval_us),
                0, 0, 0, 0, 0,
            )
            await asyncio.sleep(0.05)

    async def apply_safe_params(
        self,
        com_rcl_except: int = 7,
        com_obl_rc_act: int = 4,
        com_of_loss_t: float = 2.0,
        com_rc_in_mode: int = 1,
    ):
        """Write recommended PX4 safety parameters for offboard missions."""
        if self._mock:
            logger.info(f"[MOCK] apply_safe_params: rcl_except={com_rcl_except}, "
                        f"obl_rc_act={com_obl_rc_act}, of_loss_t={com_of_loss_t}, "
                        f"rc_in_mode={com_rc_in_mode} (no-op)")
            return
        params = [
            ("COM_RCL_EXCEPT", int_to_float_bits(com_rcl_except), mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("COM_OBL_RC_ACT", int_to_float_bits(com_obl_rc_act), mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            ("COM_OF_LOSS_T", float(com_of_loss_t), mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ("COM_RC_IN_MODE", int_to_float_bits(com_rc_in_mode), mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        ]
        for name, value, ptype in params:
            self.mav.mav.param_set_send(
                self.target_system, self.target_component,
                name.encode(), value, ptype,
            )
            human = (
                struct.unpack("<i", struct.pack("<f", value))[0]
                if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32
                else value
            )
            logger.info(f"param {name} = {human}")
            await asyncio.sleep(0.1)

    async def wait_until_ready(self, timeout_s: float = 60.0):
        """Block until EKF reports a fresh LOCAL_POSITION_NED."""
        if self._mock:
            logger.info("[MOCK] EKF ready (instant)")
            return
        logger.info("Waiting for EKF (LOCAL_POSITION_NED)...")
        start = time.time()
        while time.time() - start < timeout_s:
            with self._tel_lock:
                ok = self._tel["local_position_ok"] and (
                    time.time() - self._tel["last_local_pos_ts"] < 2.0
                )
            if ok:
                logger.info("EKF ready")
                return
            await asyncio.sleep(0.5)
        raise DroneError("EKF readiness timeout")

    def get_local_position(self) -> Position:
        with self._tel_lock:
            return Position(self._tel["local_x"], self._tel["local_y"], self._tel["local_z"])

    def get_yaw_rad(self) -> float:
        with self._tel_lock:
            return self._tel["yaw"]

    def get_yaw_deg(self) -> float:
        return math.degrees(self.get_yaw_rad())

    def is_armed(self) -> bool:
        with self._tel_lock:
            return self._tel["armed"]

    def get_main_mode(self) -> int:
        with self._tel_lock:
            return self._tel["main_mode"]

    def get_sub_mode(self) -> int:
        with self._tel_lock:
            return self._tel["sub_mode"]

    def is_offboard(self) -> bool:
        return self.get_main_mode() == PX4_CUSTOM_MAIN_MODE_OFFBOARD

    def landed_state(self) -> int:
        with self._tel_lock:
            return self._tel["landed_state"]

    def _set_setpoint_position(
        self,
        x: float,
        y: float,
        z: float,
        yaw_rad: Optional[float] = None,
    ):
        with self._setpoint_lock:
            self._setpoint["x"] = x
            self._setpoint["y"] = y
            self._setpoint["z"] = z
            self._setpoint["vx"] = 0.0
            self._setpoint["vy"] = 0.0
            self._setpoint["vz"] = 0.0
            if yaw_rad is None:
                self._setpoint["yaw_target"] = float("nan")
                self._setpoint["yaw"] = float("nan")
            else:
                self._setpoint["yaw_target"] = yaw_rad
                if math.isnan(self._setpoint.get("yaw", float("nan"))):
                    self._setpoint["yaw"] = yaw_rad
            self._setpoint["type_mask"] = DEFAULT_POS_TYPE_MASK

    def _ensure_streamer_started(self):
        if self._streaming:
            return
        with self._tel_lock:
            if not self._tel["local_position_ok"]:
                raise DroneError(
                    "Cannot start offboard streamer before EKF gives LOCAL_POSITION_NED. "
                    "Call wait_until_ready() first."
                )
        pos = self.get_local_position()
        self._set_setpoint_position(pos.x, pos.y, pos.z, self.get_yaw_rad())
        self._streaming = True

        def loop():
            last = time.time()
            while self._streaming and not self._stop_event.is_set():
                now = time.time()
                dt = max(1e-6, min(0.2, now - last))
                last = now
                try:
                    with self._setpoint_lock:
                        prev_yaw = self._setpoint.get("yaw", float("nan"))
                        target_yaw = self._setpoint.get("yaw_target", float("nan"))
                        if not math.isnan(target_yaw):
                            if math.isnan(prev_yaw):
                                self._setpoint["yaw"] = target_yaw
                            else:
                                yaw_err = math.atan2(
                                    math.sin(target_yaw - prev_yaw),
                                    math.cos(target_yaw - prev_yaw),
                                )
                                max_step = self.yaw_slew_rate_rad * dt
                                if abs(yaw_err) <= max_step:
                                    self._setpoint["yaw"] = target_yaw
                                else:
                                    self._setpoint["yaw"] = prev_yaw + math.copysign(max_step, yaw_err)
                        sp = dict(self._setpoint)
                    if not self._mock:
                        tb_ms = int(time.time() * 1e3) & 0xFFFFFFFF
                        self.mav.mav.set_position_target_local_ned_send(
                            tb_ms,
                            self.target_system, self.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            sp["type_mask"],
                            sp["x"], sp["y"], sp["z"],
                            sp["vx"], sp["vy"], sp["vz"],
                            0.0, 0.0, 0.0,
                            sp["yaw"], 0.0,
                        )
                except Exception as e:
                    logger.warning(f"streamer error: {e}")
                time.sleep(self.loop_period)

        self._streamer_thread = threading.Thread(target=loop, daemon=True, name="streamer")
        self._streamer_thread.start()

    def _stop_streamer(self):
        self._streaming = False
        if self._streamer_thread is not None:
            self._streamer_thread.join(timeout=2.0)
            self._streamer_thread = None

    async def _set_mode(
        self,
        custom_main_mode: int,
        custom_sub_mode: int = 0,
        wait_for_confirm_s: float = 3.0,
    ) -> bool:
        if self._mock:
            with self._tel_lock:
                self._tel["main_mode"] = custom_main_mode
                self._tel["sub_mode"] = custom_sub_mode
            logger.info(f"[MOCK] Mode → main={custom_main_mode} sub={custom_sub_mode}")
            await asyncio.sleep(0.05)
            return True
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(custom_main_mode),
            float(custom_sub_mode),
            0, 0, 0, 0,
        )
        start = time.time()
        while time.time() - start < wait_for_confirm_s:
            await asyncio.sleep(0.1)
            if self.get_main_mode() == custom_main_mode and (
                custom_sub_mode == 0 or self.get_sub_mode() == custom_sub_mode
            ):
                logger.info(f"Mode → main={custom_main_mode} sub={custom_sub_mode}")
                return True
        logger.warning(
            f"Mode change timeout: requested main={custom_main_mode} sub={custom_sub_mode}, "
            f"actual main={self.get_main_mode()} sub={self.get_sub_mode()}"
        )
        return False

    async def _send_arm(self, arm: bool, force: bool = False, timeout_s: float = 5.0) -> bool:
        if self._mock:
            with self._tel_lock:
                self._tel["armed"] = arm
            logger.info(f"[MOCK] {'Armed' if arm else 'Disarmed'}")
            await asyncio.sleep(0.05)
            return True
        param1 = 1.0 if arm else 0.0
        param2 = 21196.0 if force else 0.0
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            param1, param2,
            0, 0, 0, 0, 0,
        )
        start = time.time()
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.1)
            if self.is_armed() == arm:
                logger.info(f"{'Armed' if arm else 'Disarmed'} (after {time.time() - start:.1f}s)")
                return True
        logger.error(f"{'Arm' if arm else 'Disarm'} timeout")
        return False

    async def arm(self, timeout_s: float = 10.0) -> bool:
        if self.is_armed():
            logger.info("Already armed")
            return True
        self._viz_publish_command("arm")
        logger.info("Arming...")
        return await self._send_arm(arm=True, timeout_s=timeout_s)

    async def disarm(self, force: bool = False, timeout_s: float = 5.0) -> bool:
        if not self.is_armed():
            logger.info("Already disarmed")
            return True
        self._viz_publish_command("disarm", force=force)
        logger.info(f"Disarming{' (FORCED)' if force else ''}...")
        return await self._send_arm(arm=False, force=force, timeout_s=timeout_s)

    async def takeoff(self, altitude_m: float, timeout_s: float = 30.0) -> bool:
        """Arm the vehicle, enter OFFBOARD mode, and climb to altitude_m."""
        logger.info(f"Takeoff to {altitude_m} m")
        pos = self.get_local_position()
        yaw = self.get_yaw_rad()
        self._viz_publish_command(
            "takeoff",
            altitude_m=altitude_m,
            from_pos={"x": pos.x, "y": pos.y, "z": pos.z},
            target={"x": pos.x, "y": pos.y, "z": pos.z - altitude_m},
            timeout_s=timeout_s,
        )

        self._set_setpoint_position(pos.x, pos.y, pos.z, yaw)
        self._ensure_streamer_started()
        await asyncio.sleep(1.5)

        if not await self._set_mode(PX4_CUSTOM_MAIN_MODE_OFFBOARD):
            raise DroneError("Failed to enter OFFBOARD")

        if not self.is_armed():
            if not await self.arm():
                raise DroneError("Arm failed")

        pos2 = self.get_local_position()
        target_z = pos2.z - altitude_m
        self._set_setpoint_position(pos2.x, pos2.y, target_z, self.get_yaw_rad())

        return await self._wait_position_reached(
            pos2.x, pos2.y, target_z,
            timeout_s=timeout_s,
            xy_tol=2.0,
            z_tol=0.5,
        )

    async def goto(
        self,
        x: float,
        y: float,
        z: float,
        yaw_deg: Optional[float] = None,
        timeout_s: float = 30.0,
        hover_time_s: float = 2.0,
        xy_tol_m: float = 0.5,
        z_tol_m: float = 0.5,
    ) -> bool:
        """Fly to an absolute NED position. Requires OFFBOARD + armed."""
        if not self.is_offboard():
            raise DroneError(
                f"goto() requires OFFBOARD mode, current main_mode={self.get_main_mode()}"
            )
        if not self.is_armed():
            raise DroneError("goto() requires armed")

        yaw_rad = math.radians(yaw_deg) if yaw_deg is not None else None
        from_pos = self.get_local_position()

        self._viz_publish_command(
            "goto",
            from_pos={"x": from_pos.x, "y": from_pos.y, "z": from_pos.z},
            target={"x": x, "y": y, "z": z},
            yaw_deg=yaw_deg,
            timeout_s=timeout_s,
            hover_time_s=hover_time_s,
        )
        logger.info(
            f"goto NED=({x:.2f}, {y:.2f}, {z:.2f}) "
            f"yaw={yaw_deg if yaw_deg is not None else 'face-travel-dir'}"
        )

        if yaw_rad is None:
            dx = x - from_pos.x
            dy = y - from_pos.y
            dist_xy = math.hypot(dx, dy)
            if dist_xy > 0.05:
                heading = math.atan2(dy, dx)
                try:
                    self._ensure_streamer_started()
                except DroneError:
                    logger.warning("Could not start streamer for pre-yaw; proceeding to move")
                    heading = None

                if heading is not None:
                    current_yaw = self.get_yaw_rad()
                    yaw_err = math.degrees(abs(math.atan2(math.sin(heading - current_yaw), math.cos(heading - current_yaw))))
                    deg_per_sec = math.degrees(self.yaw_slew_rate_rad) if self.yaw_slew_rate_rad > 0 else 30.0
                    yaw_timeout = min(60.0, max(2.0, yaw_err / max(1e-3, deg_per_sec) + 2.0))
                    ok = await self.set_yaw(math.degrees(heading), timeout_s=yaw_timeout)
                    if not ok:
                        logger.warning("Pre-yaw timed out; proceeding to move toward target")

        try:
            self._ensure_streamer_started()
        except DroneError:
            raise DroneError("Cannot start offboard streamer before goto()")

        self._set_setpoint_position(x, y, z, yaw_rad)

        reached = await self._wait_position_reached(x, y, z, timeout_s, xy_tol_m, z_tol_m)

        if hover_time_s > 0:
            logger.info(f"Hovering for {hover_time_s}s")
            self._viz_publish_command("hover", duration_s=hover_time_s)
            await asyncio.sleep(hover_time_s)

        return reached

    async def goto_relative(self, dx: float, dy: float, dz: float, yaw_deg: Optional[float] = None, **kwargs) -> bool:
        """Fly to a position offset from the current NED position."""
        pos = self.get_local_position()
        return await self.goto(pos.x + dx, pos.y + dy, pos.z + dz, yaw_deg=yaw_deg, **kwargs)

    async def goto_body_relative(self, forward_m: float, right_m: float, down_m: float, yaw_deg: Optional[float] = None, **kwargs) -> bool:
        """Fly to a position offset in body FRD frame (no heading math required)."""
        pos = self.get_local_position()
        yaw = self.get_yaw_rad()
        ned_dx, ned_dy = body_to_ned(forward_m, right_m, yaw)
        return await self.goto(
            pos.x + ned_dx, pos.y + ned_dy, pos.z + down_m,
            yaw_deg=yaw_deg, **kwargs,
        )

    async def hover(self, duration_s: float):
        """Hold current position for duration_s seconds."""
        logger.info(f"Hover {duration_s}s")
        self._viz_publish_command("hover", duration_s=duration_s)
        await asyncio.sleep(duration_s)

    async def set_yaw(self, yaw_deg: float, timeout_s: float = 10.0) -> bool:
        """Rotate in-place to yaw_deg (degrees, NED convention)."""
        pos = self.get_local_position()
        logger.info(f"Yaw → {yaw_deg}°")
        self._viz_publish_command("set_yaw", yaw_deg=yaw_deg)
        target_yaw_rad = math.radians(yaw_deg)
        self._set_setpoint_position(pos.x, pos.y, pos.z, target_yaw_rad)

        start = time.time()
        err = math.pi
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.1)
            current = self.get_yaw_rad()
            err = math.atan2(
                math.sin(target_yaw_rad - current), math.cos(target_yaw_rad - current)
            )
            if abs(err) < math.radians(5.0):
                logger.info(f"Yaw reached (err {math.degrees(err):.1f}°)")
                return True
        logger.warning(f"Yaw timeout (err {math.degrees(err):.1f}°)")
        return False

    async def land(self, timeout_s: float = 60.0) -> bool:
        """Switch to AUTO_LAND and wait until on-ground."""
        logger.info("Auto LAND")
        pos = self.get_local_position()
        self._viz_publish_command(
            "land",
            from_pos={"x": pos.x, "y": pos.y, "z": pos.z},
        )

        if not await self._set_mode(
            PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND,
        ):
            logger.warning("Land mode change rejected — sending MAV_CMD_NAV_LAND directly")
            self.mav.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0,
            )

        self._stop_streamer()

        start = time.time()
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.5)
            if self.landed_state() == 1:
                logger.info("Landed")
                return True
            if not self.is_armed():
                logger.info("Auto-disarmed after land")
                return True
        logger.warning("Land timeout")
        return False

    async def precision_land(
        self,
        get_marker_offset: Callable[[], Optional[MarkerObservation]],
        descent_rate_mps: float = 0.3,
        final_altitude_m: float = 0.5,
        horizontal_tolerance_m: float = 0.15,
        timeout_s: float = 60.0,
        lateral_p_gain: float = 0.7,
        max_horizontal_step_m: float = 1.0,
        marker_lost_timeout_s: float = 3.0,
    ) -> bool:
        """Vision-guided descent onto a marker.

        get_marker_offset: callable returning MarkerObservation in body FRD,
            or None when the marker is not visible.
        Falls back to AUTO_LAND if the marker is lost for marker_lost_timeout_s.
        """
        if not self.is_offboard():
            raise DroneError("precision_land() requires OFFBOARD mode")
        if not self.is_armed():
            raise DroneError("precision_land() requires armed")

        logger.info("Precision landing starting")
        self._viz_publish_command(
            "precision_land",
            timeout_s=timeout_s,
            descent_rate_mps=descent_rate_mps,
            final_altitude_m=final_altitude_m,
        )
        start = time.time()
        last_seen = time.time()

        while time.time() - start < timeout_s:
            pos = self.get_local_position()
            yaw = self.get_yaw_rad()
            altitude = -pos.z

            obs: Optional[MarkerObservation] = None
            try:
                obs = get_marker_offset()
            except Exception as e:
                logger.error(f"marker callback error: {e}")

            if obs is not None:
                last_seen = time.time()
                ned_dx, ned_dy = body_to_ned(obs.dx, obs.dy, yaw)

                step_x = max(
                    -max_horizontal_step_m,
                    min(max_horizontal_step_m, ned_dx * lateral_p_gain),
                )
                step_y = max(
                    -max_horizontal_step_m,
                    min(max_horizontal_step_m, ned_dy * lateral_p_gain),
                )

                target_x = pos.x + step_x
                target_y = pos.y + step_y
                horizontal_err = math.hypot(ned_dx, ned_dy)

                if altitude <= final_altitude_m:
                    logger.info(
                        f"Reached final altitude {altitude:.2f} m, hand-off to AUTO_LAND"
                    )
                    return await self.land(
                        timeout_s=max(10.0, timeout_s - (time.time() - start))
                    )

                if horizontal_err < horizontal_tolerance_m:
                    descent = descent_rate_mps * self.loop_period
                    target_z = pos.z + descent
                else:
                    target_z = pos.z

                self._set_setpoint_position(target_x, target_y, target_z, yaw)

                if self._viz is not None:
                    self._viz.publish({
                        "type": "marker",
                        "marker_ned": {"x": pos.x + ned_dx, "y": pos.y + ned_dy},
                        "horizontal_err": horizontal_err,
                        "ts": time.time(),
                    })
            else:
                if time.time() - last_seen > marker_lost_timeout_s:
                    logger.warning(
                        f"Marker lost for {marker_lost_timeout_s}s — fallback to AUTO_LAND"
                    )
                    return await self.land(
                        timeout_s=max(10.0, timeout_s - (time.time() - start))
                    )

            await asyncio.sleep(self.loop_period)

        logger.warning("Precision land timeout — fallback to AUTO_LAND")
        return await self.land(timeout_s=30.0)

    async def return_to_launch(self, timeout_s: float = 120.0) -> bool:
        """Switch to AUTO_RTL and wait until landed."""
        logger.info("Return to Launch")
        pos = self.get_local_position()
        self._viz_publish_command(
            "rtl",
            from_pos={"x": pos.x, "y": pos.y, "z": pos.z},
            target={"x": 0.0, "y": 0.0, "z": 0.0},
        )
        if not await self._set_mode(
            PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL,
        ):
            return False

        self._stop_streamer()

        start = time.time()
        while time.time() - start < timeout_s:
            await asyncio.sleep(0.5)
            if self.landed_state() == 1 or not self.is_armed():
                logger.info("RTL complete")
                return True
        logger.warning("RTL timeout")
        return False

    async def emergency_land(self):
        """Best-effort land. Sends flight termination if land fails."""
        logger.error("EMERGENCY LAND")
        self._viz_publish_command("emergency_land")
        try:
            await self.land(timeout_s=60.0)
        except Exception as e:
            logger.error(f"Emergency land failed: {e}")
            if not self._mock and self.mav is not None:
                self.mav.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                    0, 1, 0, 0, 0, 0, 0, 0,
                )

    def _viz_publish_command(self, command: str, **payload):
        if self._viz is None:
            return
        evt = {"type": "command", "command": command, "ts": time.time()}
        evt.update(payload)
        self._viz.publish(evt)

    async def _viz_publisher_loop(self):
        try:
            while not self._shutdown_requested:
                if self._viz is not None:
                    with self._tel_lock:
                        t = dict(self._tel)
                    with self._setpoint_lock:
                        sp = dict(self._setpoint)
                    self._viz.publish({
                        "type": "telemetry",
                        "x": t["local_x"],
                        "y": t["local_y"],
                        "z": t["local_z"],
                        "vx": t["vx"],
                        "vy": t["vy"],
                        "vz": t["vz"],
                        "yaw": t["yaw"],
                        "roll": t["roll"],
                        "pitch": t["pitch"],
                        "armed": t["armed"],
                        "main_mode": t["main_mode"],
                        "sub_mode": t["sub_mode"],
                        "battery": t["battery_remaining"],
                        "landed": t["landed_state"],
                        "streaming": self._streaming,
                        "setpoint": {
                            "x": sp["x"], "y": sp["y"], "z": sp["z"],
                            "yaw": (
                                None if math.isnan(sp["yaw"]) else sp["yaw"]
                            ),
                        },
                        "ts": time.time(),
                    })
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

    async def _wait_position_reached(self, x: float, y: float, z: float, timeout_s: float, xy_tol: float = 0.5, z_tol: float = 0.5) -> bool:
        start = time.time()
        while time.time() - start < timeout_s:
            if self._shutdown_requested:
                raise DroneError("shutdown requested")
            await asyncio.sleep(0.2)
            pos = self.get_local_position()
            dxy = math.hypot(pos.x - x, pos.y - y)
            dz = abs(pos.z - z)
            if dxy < xy_tol and dz < z_tol:
                logger.info(
                    f"Reached ({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}), err xy={dxy:.2f} z={dz:.2f}"
                )
                return True
        pos = self.get_local_position()
        logger.warning(
            f"Position timeout: target=({x:.2f},{y:.2f},{z:.2f}) current=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f})"
        )
        return False
