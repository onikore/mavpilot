"""Precision landing через Gazebo ROS2 камеру + ArUco Fractal.

Сценарий:
  1. Запустить: bash examples/run_gazebo.sh 06
  2. Лететь вручную к площадке (скрипт шлёт LANDING_TARGET пассивно)
  3. Переключить FC в OFFBOARD → скрипт берёт управление:
     центрируется над маркером, снижается, садится с фиксированным явом

Просмотр детекции:
  ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image
"""
from __future__ import annotations

import asyncio
import argparse
import logging
import math
import subprocess
import threading
import time

import cv2
import numpy as np
from pymavlink import mavutil

from mavpilot.types import MarkerObservation

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("landing_target")

IMAGE_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
CAMERA_INFO_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"

# PX4 custom_mode: bits[16:24]=main_mode, bits[24:32]=sub_mode
_MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 6: "OFFBOARD", 7: "STABILIZED"}
_AUTO_SUBS  = {2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND", 9: "PRECLAND"}

def _px4_mode(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    name = _MAIN_MODES.get(main, f"MAIN_{main}")
    return f"AUTO/{_AUTO_SUBS.get(sub, f'SUB_{sub}')}" if main == 4 and sub else name


# ---------------------------------------------------------------------------
# ROS2 image stream  (совместим с arucofractal.DetectionThread)
# ---------------------------------------------------------------------------

class ROS2ImageStream:
    def __init__(self, node, topic: str) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        node.create_subscription(Image, topic, self._on_image, 1)

    def _on_image(self, msg) -> None:
        try:
            ch = 3 if msg.encoding in ("rgb8", "bgr8") else 1
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self._lock:
                self._frame = frame.copy()
        except Exception as e:
            log.error(f"frame decode: {e}")

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    def stop(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Visualization publisher  (/mavpilot/detection_image)
# ---------------------------------------------------------------------------

class VisualizationPublisher:
    def __init__(self, node, stream: ROS2ImageStream, detector, fps: float = 10.0) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
        self._node, self._stream, self._detector = node, stream, detector
        self._running = True
        self._pub = node.create_publisher(Image, "/mavpilot/detection_image", 1)
        threading.Thread(target=self._loop, daemon=True).start()
        log.info("viz: /mavpilot/detection_image")

    def _loop(self) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
        interval = 1.0 / 10.0
        while self._running:
            t0 = time.time()
            frame = self._stream.read()
            if frame is not None:
                vis = frame.copy()
                self._detector.draw_overlays(vis)
                st = self._detector.state
                if st and st.detected and st.has_pose:
                    cv2.putText(vis, f"dz={st.tvec.flatten()[2]:.2f}m  {self._detector.det_fps:.0f}fps",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(vis, "no marker", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                msg = Image()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.height, msg.width = vis.shape[:2]
                msg.encoding = "bgr8"
                msg.step = msg.width * 3
                msg.data = vis.tobytes()
                self._pub.publish(msg)
            rem = interval - (time.time() - t0)
            if rem > 0:
                time.sleep(rem)

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# Gazebo ArUco source
# ---------------------------------------------------------------------------

class GazeboArucoSource:
    def __init__(
        self,
        image_topic: str = IMAGE_TOPIC,
        camera_info_topic: str = CAMERA_INFO_TOPIC,
        marker_size: float = 0.17,
        camera_yaw_deg: float = 0.0,
        camera_info_timeout_s: float = 10.0,
    ) -> None:
        self._image_topic = image_topic
        self._camera_info_topic = camera_info_topic
        self._marker_size = marker_size
        self._camera_yaw_deg = camera_yaw_deg
        self._camera_info_timeout_s = camera_info_timeout_s
        self._stream: ROS2ImageStream | None = None
        self._detector = None
        self._viz: VisualizationPublisher | None = None
        self._node = None
        self._bridge: subprocess.Popen | None = None  # type: ignore[type-arg]

    async def __aenter__(self) -> GazeboArucoSource:
        import rclpy  # type: ignore[import]
        from arucofractal import Config, DetectionThread  # type: ignore[import]

        await asyncio.to_thread(self._start_bridge)
        rclpy.init()
        self._node = rclpy.create_node("mavpilot_landing_target")
        cam = await asyncio.to_thread(self._wait_camera_info, rclpy, self._camera_info_timeout_s)
        log.info(f"camera: fx={cam['fx']:.1f} fy={cam['fy']:.1f} {cam['width']}x{cam['height']}")

        cfg = Config(
            marker_size=self._marker_size,
            camera_fx=cam["fx"], camera_fy=cam["fy"],
            camera_cx=cam["cx"], camera_cy=cam["cy"],
            frame_width=cam["width"], frame_height=cam["height"],
        )
        self._stream = ROS2ImageStream(self._node, self._image_topic)
        self._detector = DetectionThread(self._stream, cfg)
        self._viz = VisualizationPublisher(self._node, self._stream, self._detector)
        threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()
        return self

    async def __aexit__(self, *_: object) -> None:
        if self._viz:      self._viz.stop()
        if self._detector: self._detector.stop()
        if self._node:     self._node.destroy_node()
        try:
            import rclpy  # type: ignore[import]
            rclpy.shutdown()
        except Exception:
            pass
        if self._bridge:
            self._bridge.terminate()
            self._bridge.wait(timeout=5)

    def marker_callback(self) -> MarkerObservation | None:
        if self._detector is None:
            return None
        st = self._detector.state
        if st is None or not st.detected or not st.has_pose:
            return None
        tvec = st.tvec.flatten()
        dx, dy, dz = -float(tvec[1]), float(tvec[0]), float(tvec[2])
        if self._camera_yaw_deg:
            t = math.radians(self._camera_yaw_deg)
            dx, dy = dx * math.cos(t) - dy * math.sin(t), dx * math.sin(t) + dy * math.cos(t)
        return MarkerObservation(dx=dx, dy=dy, dz=dz)

    def _start_bridge(self) -> None:
        rules = [
            f"{self._image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{self._camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
        log.info("starting gz bridge")
        self._bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + rules,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(2.0)

    def _wait_camera_info(self, rclpy, timeout_s: float) -> dict:
        from sensor_msgs.msg import CameraInfo  # type: ignore[import]
        result: dict = {}
        event = threading.Event()

        def _cb(msg) -> None:
            k = msg.k
            result.update(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]),
                          width=int(msg.width), height=int(msg.height))
            event.set()

        sub = self._node.create_subscription(CameraInfo, self._camera_info_topic, _cb, 1)
        deadline = time.time() + timeout_s
        while not event.is_set() and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_subscription(sub)
        if not event.is_set():
            raise TimeoutError(f"camera_info timeout {timeout_s}s")
        return result


# ---------------------------------------------------------------------------
# MAVLink publisher + OFFBOARD descent
# ---------------------------------------------------------------------------

class LandingTargetPublisher:
    # SET_POSITION_TARGET_LOCAL_NED type_mask: position + yaw only
    _POS_YAW_MASK = 0b100_111_111_000  # ignore vel(3-5), acc(6-8), yaw_rate(11)

    def __init__(
        self,
        connection_string: str,
        rate_hz: float = 15.0,
        landing_yaw_deg: float | None = None,
        descent_rate_mps: float = 0.2,
        land_distance_m: float = 0.5,
        lateral_p_gain: float = 0.6,
    ) -> None:
        self._conn_str = connection_string
        self._rate_hz = rate_hz
        self._landing_yaw_rad = math.radians(landing_yaw_deg) if landing_yaw_deg is not None else None
        self._descent_rate = descent_rate_mps
        self._land_dist = land_distance_m
        self._lat_p = lateral_p_gain

        # q для LANDING_TARGET (желаемая ориентация площадки)
        yaw_r = self._landing_yaw_rad or 0.0
        self._landing_q = [math.cos(yaw_r / 2), 0.0, 0.0, math.sin(yaw_r / 2)]

        self._mav: mavutil.mavfile | None = None  # type: ignore[type-arg]
        self._running = False
        self._mode = ""
        self._yaw = 0.0
        self._ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._lock = threading.Lock()

    async def __aenter__(self) -> LandingTargetPublisher:
        log.info(f"connecting to PX4: {self._conn_str}")
        self._mav = await asyncio.to_thread(self._connect)
        self._running = True
        threading.Thread(target=self._tele_loop, daemon=True).start()
        return self

    async def __aexit__(self, *_: object) -> None:
        self._running = False
        if self._mav:
            self._mav.close()

    def _connect(self) -> mavutil.mavfile:  # type: ignore[type-arg]
        mav = mavutil.mavlink_connection(self._conn_str, source_system=1)
        mav.wait_heartbeat(timeout=30)
        log.info(f"PX4 connected sys={mav.target_system}")
        return mav

    def _tele_loop(self) -> None:
        last_mode = None
        while self._running and self._mav:
            try:
                msg = self._mav.recv_match(
                    type=["HEARTBEAT", "ATTITUDE", "LOCAL_POSITION_NED"],
                    blocking=True, timeout=1.0,
                )
            except OSError:
                break
            if msg is None:
                continue
            t = msg.get_type()
            if t == "ATTITUDE":
                with self._lock: self._yaw = float(msg.yaw)
            elif t == "LOCAL_POSITION_NED":
                with self._lock: self._ned = (float(msg.x), float(msg.y), float(msg.z))
            elif t == "HEARTBEAT":
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                mode = _px4_mode(msg.custom_mode)
                self._mode = mode
                if mode != last_mode:
                    log.info(f"FC mode: {mode}")
                    last_mode = mode

    async def run(self, get_marker_offset) -> None:
        """Единый цикл: hold-setpoints (для разрешения OFFBOARD) + снижение."""
        interval = 1.0 / self._rate_hz
        last_seen = time.time()
        last_log = time.time()
        marker_lost_s = 3.0
        in_offboard = False
        target_yaw = 0.0

        # Ждём первую позицию (до 5с)
        for _ in range(50):
            with self._lock:
                if self._ned != (0.0, 0.0, 0.0):
                    break
            await asyncio.sleep(0.1)

        with self._lock:
            hx, hy, hz = self._ned
            hyaw = self._yaw

        log.info("ready — switch FC to OFFBOARD when over the pad")

        while True:
            t0 = time.time()
            with self._lock:
                px, py, pz = self._ned
                cyaw = self._yaw
            obs = get_marker_offset()

            if self._mode == "OFFBOARD":
                if not in_offboard:
                    in_offboard = True
                    target_yaw = self._landing_yaw_rad if self._landing_yaw_rad is not None else cyaw
                    log.info(f"OFFBOARD descent, yaw={math.degrees(target_yaw):.1f}°")

                if obs is not None and obs.dz is not None:
                    last_seen = time.time()
                    # body FRD → NED (использует текущий яв для трансформа)
                    cos_y, sin_y = math.cos(cyaw), math.sin(cyaw)
                    hx = px + (obs.dx * cos_y - obs.dy * sin_y) * self._lat_p
                    hy = py + (obs.dx * sin_y + obs.dy * cos_y) * self._lat_p
                    hz = pz + interval * self._descent_rate
                    if obs.dz < self._land_dist:
                        log.info(f"dz={obs.dz:.2f}m — landing")
                        self._land(target_yaw)
                        return
                elif time.time() - last_seen > marker_lost_s:
                    log.warning("marker lost — emergency land")
                    self._land(target_yaw)
                    return

                self._pos_setpoint(hx, hy, hz, target_yaw)

            else:
                in_offboard = False
                hx, hy, hz, hyaw = px, py, pz, cyaw
                # Hold-setpoints: поток нужен до переключения в OFFBOARD
                self._pos_setpoint(hx, hy, hz, hyaw)
                if obs is not None and obs.dz is not None:
                    self._landing_target(obs)

            if time.time() - last_log >= 5.0:
                info = f"dz={obs.dz:.2f}m" if obs and obs.dz else "no marker"
                log.info(f"[{self._mode}] {info}")
                last_log = time.time()

            rem = interval - (time.time() - t0)
            if rem > 0:
                await asyncio.sleep(rem)

    def _pos_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        if not self._mav:
            return
        self._mav.mav.set_position_target_local_ned_send(
            0, self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, self._POS_YAW_MASK,
            x, y, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            float(yaw), 0.0,
        )

    def _land(self, yaw_rad: float) -> None:
        if not self._mav:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0.0, 0.0, 0.0, math.degrees(yaw_rad), 0.0, 0.0, 0.0,
        )

    def _landing_target(self, obs: MarkerObservation) -> None:
        if not self._mav or not obs.dz or obs.dz <= 0:
            return
        with self._lock:
            yaw = self._yaw
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        x_ned = obs.dx * cos_y - obs.dy * sin_y
        y_ned = obs.dx * sin_y + obs.dy * cos_y
        try:
            self._mav.mav.landing_target_send(
                int(time.time() * 1e6), 0,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0.0, 0.0, float(obs.dz), 0.0, 0.0,
                float(x_ned), float(y_ned), float(obs.dz),
                self._landing_q,
                mavutil.mavlink.LANDING_TARGET_TYPE_LIGHT_BEACON, 1,
            )
        except Exception as e:
            log.debug(f"landing_target_send: {e}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def main_async(args) -> None:
    async with GazeboArucoSource(
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        marker_size=args.marker_size,
        camera_yaw_deg=args.camera_yaw,
    ) as src:
        async with LandingTargetPublisher(
            args.connection,
            rate_hz=args.rate,
            landing_yaw_deg=args.landing_yaw,
            descent_rate_mps=args.descent_rate,
            land_distance_m=args.land_distance,
        ) as lt:
            await lt.run(src.marker_callback)


def main() -> None:
    p = argparse.ArgumentParser(description="Precision landing: manual fly → OFFBOARD → land on marker")
    p.add_argument("--connection",        default="udp:127.0.0.1:14540")
    p.add_argument("--image-topic",       default=IMAGE_TOPIC)
    p.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC)
    p.add_argument("--marker-size",       type=float, default=0.17,  help="marker size, m")
    p.add_argument("--camera-yaw",        type=float, default=0.0,   help="camera rotation, deg (0=top toward nose)")
    p.add_argument("--rate",              type=float, default=15.0,  help="setpoint rate, Hz")
    p.add_argument("--landing-yaw",       type=float, default=None,  help="desired yaw at touchdown, deg NED")
    p.add_argument("--descent-rate",      type=float, default=0.2,   help="descent speed in OFFBOARD, m/s")
    p.add_argument("--land-distance",     type=float, default=0.5,   help="dz threshold to trigger land, m")
    args = p.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        log.info("stopped")


if __name__ == "__main__":
    main()
