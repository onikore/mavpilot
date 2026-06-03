"""Gazebo / ROS2 camera integration for precision landing.

Gazebo publishes camera topics over gz-transport, which ROS2 cannot see
directly. This module starts a ``ros_gz_bridge`` to forward them into ROS2,
reads frames and camera intrinsics, and feeds them to arucofractal for marker
detection — exposing :meth:`GazeboArucoSource.marker_callback` for
:meth:`mavpilot.DroneController.precision_land`.

``rclpy``, ``cv2`` and ``arucofractal`` are soft dependencies, imported lazily
so the rest of mavpilot is unaffected when they're absent. The deps come from a
ROS2 install (``source /opt/ros/<distro>/setup.bash``), not pip.
"""

from __future__ import annotations

import logging
import subprocess
import threading
import time

from mavpilot.types import MarkerObservation

from .arucofractal import observation_from_detection

log = logging.getLogger("drone")


def start_ros_gz_bridge(*topic_type_rules: str) -> subprocess.Popen:
    """Launch ``ros_gz_bridge parameter_bridge`` to forward Gazebo → ROS2.

    Each rule has the form ``TOPIC@ROS_TYPE[GZ_TYPE`` (the ``[`` makes it
    Gazebo→ROS2, one-directional). Returns the subprocess handle; call
    ``terminate()`` to stop it. Sleeps ~2 s so the bridge can register topics.
    """
    cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *topic_type_rules]
    log.info(f"starting gz bridge: {len(topic_type_rules)} topic(s)")
    proc = subprocess.Popen(  # noqa: S603 — fixed ros2 CLI, topics are caller-supplied config
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(2.0)
    return proc


def wait_for_camera_info(node, topic: str, timeout_s: float = 10.0) -> dict:
    """Block until one ``CameraInfo`` message arrives; return its intrinsics.

    Returns a dict with ``fx, fy, cx, cy, width, height``. Spins ``node`` itself,
    so call this before starting a background spin thread.
    """
    import rclpy  # type: ignore[import]
    from sensor_msgs.msg import CameraInfo  # type: ignore[import]

    result: dict = {}
    event = threading.Event()

    def _cb(msg) -> None:
        k = msg.k  # flat row-major 3x3 camera matrix
        result.update(
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            width=int(msg.width),
            height=int(msg.height),
        )
        event.set()

    sub = node.create_subscription(CameraInfo, topic, _cb, 1)
    deadline = time.time() + timeout_s
    while not event.is_set() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)

    if not event.is_set():
        raise TimeoutError(f"camera_info not received in {timeout_s}s on {topic}")
    return result


class ROS2ImageStream:
    """Subscribes to ``sensor_msgs/Image`` and exposes frames via ``read()``.

    Compatible with ``arucofractal.DetectionThread`` (it expects an object with
    ``read() -> np.ndarray | None``). Decodes manually with numpy — cv_bridge
    is avoided because it segfaults when built against a different NumPy ABI.
    """

    def __init__(self, node, topic: str) -> None:
        import numpy as np  # type: ignore[import]
        from sensor_msgs.msg import Image  # type: ignore[import]

        self._np = np
        self._frame = None
        self._lock = threading.Lock()
        node.create_subscription(Image, topic, self._on_image, 1)
        log.info(f"image subscription: {topic}")

    def _on_image(self, msg) -> None:
        import cv2  # type: ignore[import]

        try:
            ch = 3 if msg.encoding in ("rgb8", "bgr8") else 1
            frame = self._np.frombuffer(msg.data, dtype=self._np.uint8).reshape(
                msg.height, msg.width, ch
            )
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self._lock:
                self._frame = frame.copy()
        except Exception as e:
            log.error(f"frame decode: {e}")

    def read(self):
        with self._lock:
            return self._frame

    def stop(self) -> None:
        pass  # subscription dies with the node


class DetectionImagePublisher:
    """Publishes the detection overlay to ``/mavpilot/detection_image`` (bgr8).

    Runs a background thread that draws arucofractal overlays + a dz/fps banner
    onto the latest frame. View with::

        ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image
    """

    def __init__(
        self,
        node,
        stream: ROS2ImageStream,
        detector,
        fps: float = 10.0,
        topic: str = "/mavpilot/detection_image",
    ) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]

        self._node, self._stream, self._detector = node, stream, detector
        self._interval = 1.0 / fps
        self._running = True
        self._pub = node.create_publisher(Image, topic, 1)
        threading.Thread(target=self._loop, daemon=True).start()
        log.info(f"viz: {topic}")

    def _loop(self) -> None:
        import cv2  # type: ignore[import]
        from sensor_msgs.msg import Image  # type: ignore[import]

        while self._running:
            t0 = time.time()
            frame = self._stream.read()
            if frame is not None:
                vis = frame.copy()
                self._detector.draw_overlays(vis)
                st = self._detector.state
                if st and st.detected and st.has_pose:
                    cv2.putText(
                        vis,
                        f"dz={st.tvec.flatten()[2]:.2f}m  {self._detector.det_fps:.0f}fps",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                    )
                else:
                    cv2.putText(
                        vis, "no marker", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
                    )
                msg = Image()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.height, msg.width = vis.shape[:2]
                msg.encoding = "bgr8"
                msg.step = msg.width * 3
                msg.data = vis.tobytes()
                self._pub.publish(msg)
            rem = self._interval - (time.time() - t0)
            if rem > 0:
                time.sleep(rem)

    def stop(self) -> None:
        self._running = False


# Default Gazebo x500 downward camera topics (PX4 gz_x500_mono_cam_down model).
DEFAULT_IMAGE_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
DEFAULT_CAMERA_INFO_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"
)


class GazeboArucoSource:
    """Async context manager: Gazebo camera → arucofractal marker detection.

    On ``__aenter__`` it starts the gz bridge, reads camera intrinsics from the
    ``camera_info`` topic, builds an arucofractal detector on the image stream,
    and (optionally) publishes a detection-overlay topic. ``marker_callback``
    plugs straight into :meth:`mavpilot.DroneController.precision_land`.

    Example::

        async with GazeboArucoSource(marker_size=0.17) as src:
            async with DroneController(connection_string=conn) as drone:
                await drone.connect()
                await drone.wait_until_ready()
                await drone.wait_for_offboard()
                await drone.precision_land(src.marker_callback)
    """

    def __init__(
        self,
        image_topic: str = DEFAULT_IMAGE_TOPIC,
        camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC,
        marker_size: float = 0.17,
        camera_yaw_deg: float = 0.0,
        publish_viz: bool = True,
        camera_info_timeout_s: float = 10.0,
    ) -> None:
        self._image_topic = image_topic
        self._camera_info_topic = camera_info_topic
        self._marker_size = marker_size
        self._camera_yaw_deg = camera_yaw_deg
        self._publish_viz = publish_viz
        self._camera_info_timeout_s = camera_info_timeout_s

        self._stream: ROS2ImageStream | None = None
        self._detector = None
        self._viz: DetectionImagePublisher | None = None
        self._node = None
        self._bridge: subprocess.Popen | None = None

    async def __aenter__(self) -> GazeboArucoSource:
        import asyncio

        import rclpy  # type: ignore[import]

        from arucofractal import Config, DetectionThread  # type: ignore[import]

        self._bridge = await asyncio.to_thread(
            start_ros_gz_bridge,
            f"{self._image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{self._camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        )

        rclpy.init()
        self._node = rclpy.create_node("mavpilot_gazebo_aruco")
        cam = await asyncio.to_thread(
            wait_for_camera_info,
            self._node,
            self._camera_info_topic,
            self._camera_info_timeout_s,
        )
        log.info(f"camera {cam['width']}x{cam['height']} fx={cam['fx']:.1f}")

        cfg = Config(
            marker_size=self._marker_size,
            camera_fx=cam["fx"],
            camera_fy=cam["fy"],
            camera_cx=cam["cx"],
            camera_cy=cam["cy"],
            frame_width=cam["width"],
            frame_height=cam["height"],
        )
        self._stream = ROS2ImageStream(self._node, self._image_topic)
        self._detector = DetectionThread(self._stream, cfg)
        if self._publish_viz:
            self._viz = DetectionImagePublisher(self._node, self._stream, self._detector)

        threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()
        return self

    async def __aexit__(self, *_: object) -> None:
        if self._viz:
            self._viz.stop()
        if self._detector:
            self._detector.stop()
        if self._node:
            self._node.destroy_node()
        try:
            import rclpy  # type: ignore[import]

            rclpy.shutdown()
        except Exception as e:
            log.debug(f"rclpy shutdown: {e}")
        if self._bridge:
            self._bridge.terminate()
            self._bridge.wait(timeout=5)

    def marker_callback(self) -> MarkerObservation | None:
        """Return current marker offset in body FRD, or None if not visible."""
        if self._detector is None:
            return None
        return observation_from_detection(self._detector.state, self._camera_yaw_deg)
