"""MAVLinkConnection — owns the pymavlink connection, the I/O lock, and the
heartbeat / receiver threads.

Public methods:
  * connect(timeout_s, baud) → discovers target_system/target_component
  * send(method_name, *args, **kwargs) — invokes self.mav.mav.<method_name>(...) under lock
  * recv(blocking, timeout) — calls self.mav.recv_match(...) under lock
  * start_heartbeat()/start_receiver(handle_message)
  * close()
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from typing import Any, Callable, Optional

from pymavlink import mavutil

logger = logging.getLogger("drone")


class MAVLinkConnection:
    """Wraps a pymavlink mavfile with thread-safe send/recv and lifecycle.

    All MAVLink I/O — sends from any thread, receives from the receiver thread,
    sends from asyncio handlers — funnels through `self._lock`. Receiver uses
    a 50 ms blocking recv so streamer starvation is bounded.
    """

    RECV_TIMEOUT_S = 0.05

    def __init__(
        self,
        connection_string: str,
        source_system: int = 255,
        source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    ) -> None:
        self.connection_string = connection_string
        self.source_system = source_system
        self.source_component = source_component
        self.mav: Optional[mavutil.mavfile] = None
        self.target_system: int = 0
        self.target_component: int = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._receiver_thread: Optional[threading.Thread] = None

    async def connect(self, timeout_s: float = 30.0, baud: int = 57600) -> None:
        logger.info(f"Connecting to {self.connection_string}")
        kwargs: dict[str, Any] = {
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

        loop = asyncio.get_running_loop()
        hb = await asyncio.wait_for(
            loop.run_in_executor(None, lambda: self.mav.wait_heartbeat(timeout=timeout_s)),
            timeout=timeout_s + 5,
        )
        if hb is None:
            raise RuntimeError("No heartbeat received")

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
                        raise RuntimeError(
                            "target_system == 0 after heartbeat. Could not determine autopilot sysid. "
                            "Ensure SITL/autopilot is running and the connection string is correct."
                        )
        logger.info(
            f"Heartbeat from sys={self.target_system} comp={self.target_component} "
            f"src_sys={self.source_system} src_comp={self.source_component}"
        )

    def send(self, method_name: str, *args, **kwargs) -> None:
        """Invoke self.mav.mav.<method_name>(*args, **kwargs) under lock.

        Raises RuntimeError if not connected.
        """
        if self.mav is None:
            raise RuntimeError("MAVLinkConnection.send before connect()")
        with self._lock:
            method = getattr(self.mav.mav, method_name)
            method(*args, **kwargs)

    def recv(self, blocking: bool = True, timeout: float = RECV_TIMEOUT_S) -> Optional[Any]:
        if self.mav is None:
            return None
        with self._lock:
            return self.mav.recv_match(blocking=blocking, timeout=timeout)

    def start_heartbeat(self) -> None:
        def loop() -> None:
            while not self._stop_event.is_set():
                try:
                    self.send(
                        "heartbeat_send",
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

    def start_receiver(self, handle_message: Callable[[Any], None]) -> None:
        def loop() -> None:
            while not self._stop_event.is_set():
                try:
                    msg = self.recv(blocking=True, timeout=self.RECV_TIMEOUT_S)
                    if msg is None:
                        continue
                    handle_message(msg)
                except Exception as e:
                    logger.error(f"receiver error: {e}")
                    time.sleep(0.1)
        self._receiver_thread = threading.Thread(target=loop, daemon=True, name="recv")
        self._receiver_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        for thr in (self._heartbeat_thread, self._receiver_thread):
            if thr is not None and thr.is_alive():
                thr.join(timeout=2.0)
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass
