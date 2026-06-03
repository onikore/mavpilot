"""Precision landing через Gazebo ROS2 камеру + ArUco Fractal.

Сценарий:
  1. Запустить: bash examples/run_gazebo.sh 06
  2. Лететь вручную к площадке
  3. Переключить FC в OFFBOARD → скрипт берёт управление:
       rotate → precision_land (центровка + снижение)

Просмотр детекции:
  ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image
"""
from __future__ import annotations

import asyncio
import argparse
import logging
import subprocess
import threading
import time

import cv2
import numpy as np

from mavpilot import DroneController
from mavpilot.types import MarkerObservation
import math

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("landing_target")

IMAGE_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
CAMERA_INFO_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"


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
    def __init__(self, node, stream: ROS2ImageStream, detector) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
        self._node, self._stream, self._detector = node, stream, detector
        self._running = True
        self._pub = node.create_publisher(Image, "/mavpilot/detection_image", 1)
        threading.Thread(target=self._loop, daemon=True).start()
        log.info("viz: /mavpilot/detection_image")

    def _loop(self) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
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
            rem = 0.1 - (time.time() - t0)
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
        self._timeout = camera_info_timeout_s
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
        self._node = rclpy.create_node("mavpilot_precland")
        cam = await asyncio.to_thread(self._wait_camera_info, rclpy)
        log.info(f"camera {cam['width']}x{cam['height']}  fx={cam['fx']:.1f}")

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

    def _wait_camera_info(self, rclpy) -> dict:
        from sensor_msgs.msg import CameraInfo  # type: ignore[import]
        result: dict = {}
        event = threading.Event()

        def _cb(msg) -> None:
            k = msg.k
            result.update(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]),
                          width=int(msg.width), height=int(msg.height))
            event.set()

        sub = self._node.create_subscription(CameraInfo, self._camera_info_topic, _cb, 1)
        deadline = time.time() + self._timeout
        while not event.is_set() and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_subscription(sub)
        if not event.is_set():
            raise TimeoutError(f"camera_info timeout {self._timeout}s")
        return result


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

async def run_mission(src: GazeboArucoSource, drone: DroneController, args) -> None:
    log.info("waiting for OFFBOARD — switch FC to OFFBOARD when over the pad")

    # Стримим hold-setpoints пока не активируется OFFBOARD.
    # PX4 требует активный поток ДО переключения, иначе отклоняет.
    while not drone.is_offboard():
        pos = drone.get_local_position()
        drone._set_setpoint_position(pos.x, pos.y, pos.z, drone.get_yaw_rad())
        await asyncio.sleep(0.05)

    log.info("OFFBOARD active")

    # Поворот к нужному явам — полностью завершается до снижения
    if args.landing_yaw is not None:
        log.info(f"rotating to {args.landing_yaw}°...")
        await drone.set_yaw(args.landing_yaw, timeout_s=30.0)
        log.info("yaw aligned")

    # Центровка + снижение через precision_land
    log.info("starting precision landing")
    result = await drone.precision_land(
        get_marker_offset=src.marker_callback,
        descent_rate_mps=args.descent_rate,
        final_altitude_m=args.land_distance,
        horizontal_tolerance_m=0.15,
        timeout_s=120.0,
    )
    log.info(f"result: {result.status.value}  pos={result.final_position}")
    if not result:
        log.warning("precision_land failed — fallback land")
        await drone.land()


async def main_async(args) -> None:
    async with GazeboArucoSource(
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        marker_size=args.marker_size,
        camera_yaw_deg=args.camera_yaw,
    ) as src:
        async with DroneController(connection_string=args.connection) as drone:
            await drone.connect(timeout_s=30.0)
            await run_mission(src, drone, args)


def main() -> None:
    p = argparse.ArgumentParser(description="Manual fly → OFFBOARD → rotate → precision land")
    p.add_argument("--connection",        default="udp:127.0.0.1:14540")
    p.add_argument("--image-topic",       default=IMAGE_TOPIC)
    p.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC)
    p.add_argument("--marker-size",       type=float, default=0.17)
    p.add_argument("--camera-yaw",        type=float, default=0.0,   help="camera rotation, deg")
    p.add_argument("--landing-yaw",       type=float, default=None,  help="desired yaw at touchdown, deg NED")
    p.add_argument("--descent-rate",      type=float, default=0.2,   help="descent speed, m/s")
    p.add_argument("--land-distance",     type=float, default=0.5,   help="final altitude before AUTO_LAND, m")
    args = p.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        log.info("stopped")


if __name__ == "__main__":
    main()
