"""Точная посадка через Gazebo ROS2 топики с ArUco Fractal детектором.

Читает кадры и параметры камеры из ROS2:
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info

Требует:
    pip install mavpilot[aruco]
    ROS2 (humble / jazzy) + source /opt/ros/<distro>/setup.bash
    Опционально: pip install opencv-contrib-python (для cv_bridge fallback)

Запуск:
    source /opt/ros/humble/setup.bash
    python examples/05_precision_land_gazebo.py

С реальным соединением к PX4:
    python examples/05_precision_land_gazebo.py --connection udp:127.0.0.1:14540
"""

from __future__ import annotations

import asyncio
import argparse
import logging
import math
import threading
import time

import cv2
import numpy as np

from mavpilot import DroneController
from mavpilot.types import MarkerObservation

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("drone")

IMAGE_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0"
    "/link/camera_link/sensor/camera/image"
)
CAMERA_INFO_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0"
    "/link/camera_link/sensor/camera/camera_info"
)


# ---------------------------------------------------------------------------
# ROS2 image stream — интерфейс совместим с arucofractal.StreamReader
# ---------------------------------------------------------------------------

class ROS2ImageStream:
    """Подписывается на sensor_msgs/Image и предоставляет кадры через read().

    Совместим с arucofractal.DetectionThread: тот ожидает объект
    с методом read() -> np.ndarray | None.
    """

    def __init__(self, node, topic: str) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]

        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()

        # cv_bridge — предпочтительный способ конвертации
        try:
            from cv_bridge import CvBridge  # type: ignore[import]
            self._bridge = CvBridge()
        except ImportError:
            self._bridge = None
            log.warning("cv_bridge не найден, используется ручная конвертация")

        self._sub = node.create_subscription(Image, topic, self._on_image, 1)
        log.info(f"Подписка на изображение: {topic}")

    def _on_image(self, msg) -> None:
        try:
            if self._bridge is not None:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                frame = self._decode_manual(msg)
            with self._lock:
                self._frame = frame
        except Exception as e:
            log.error(f"Ошибка декодирования кадра: {e}")

    @staticmethod
    def _decode_manual(msg) -> np.ndarray:
        channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels
        )
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    def stop(self) -> None:
        pass  # подписка уничтожается вместе с нодой


# ---------------------------------------------------------------------------
# GazeboArucoSource — async context manager, реализует MarkerSource Protocol
# ---------------------------------------------------------------------------

class GazeboArucoSource:
    """Async context manager для точной посадки с Gazebo-камерой.

    Автоматически:
    - инициализирует rclpy и создаёт ноду
    - читает camera_info и передаёт параметры в arucofractal Config
    - запускает DetectionThread на кадрах из ROS2
    - после выхода останавливает всё и завершает rclpy

    Использование::

        async with GazeboArucoSource() as src:
            result = await drone.precision_land(src.marker_callback)
    """

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
        self._node = None
        self._spin_thread: threading.Thread | None = None

    async def __aenter__(self) -> GazeboArucoSource:
        import rclpy  # type: ignore[import]
        from arucofractal import Config, DetectionThread  # type: ignore[import]

        rclpy.init()
        self._node = rclpy.create_node("mavpilot_gazebo_aruco")

        # Читаем camera_info один раз для получения параметров камеры
        cam_info = await asyncio.to_thread(
            self._wait_for_camera_info,
            rclpy,
            timeout_s=self._camera_info_timeout_s,
        )

        log.info(
            f"Параметры камеры: fx={cam_info['fx']:.1f} fy={cam_info['fy']:.1f} "
            f"cx={cam_info['cx']:.1f} cy={cam_info['cy']:.1f} "
            f"w={cam_info['width']} h={cam_info['height']}"
        )

        cfg = Config(
            marker_size=self._marker_size,
            camera_fx=cam_info["fx"],
            camera_fy=cam_info["fy"],
            camera_cx=cam_info["cx"],
            camera_cy=cam_info["cy"],
            frame_width=cam_info["width"],
            frame_height=cam_info["height"],
        )

        self._stream = ROS2ImageStream(self._node, self._image_topic)
        self._detector = DetectionThread(self._stream, cfg)

        # Запускаем spin в фоне — он обрабатывает все ROS2 колбэки
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()
        log.info("ROS2 нода запущена, ждём первые кадры...")

        return self

    async def __aexit__(self, *_: object) -> None:
        if self._detector is not None:
            self._detector.stop()
        if self._node is not None:
            self._node.destroy_node()
        try:
            import rclpy  # type: ignore[import]
            rclpy.shutdown()
        except Exception:
            pass

    def marker_callback(self) -> MarkerObservation | None:
        """Возвращает смещение маркера в теле дрона (FRD) или None."""
        if self._detector is None:
            return None
        state = self._detector.state
        if state is None or not state.detected or not state.has_pose:
            return None

        tvec = state.tvec.flatten()
        # Камера смотрит строго вниз, верх кадра к носу дрона:
        #   tvec[0] (вправо в кадре)  → body Right  (+Y FRD) → dy
        #   tvec[1] (вниз в кадре)    → body Back   (−X FRD) → dx = −tvec[1]
        #   tvec[2] (глубина ≈ высота) → dz
        dx = -float(tvec[1])
        dy = float(tvec[0])
        dz = float(tvec[2])

        if self._camera_yaw_deg != 0.0:
            theta = math.radians(self._camera_yaw_deg)
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            dx, dy = dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t

        return MarkerObservation(dx=dx, dy=dy, dz=dz)

    def _wait_for_camera_info(self, rclpy, timeout_s: float) -> dict:
        """Блокирующий вызов: ждёт первого CameraInfo-сообщения."""
        from sensor_msgs.msg import CameraInfo  # type: ignore[import]

        result: dict = {}
        event = threading.Event()

        def _cb(msg) -> None:
            k = msg.k  # flat 3x3 camera matrix
            result.update(
                fx=float(k[0]),
                fy=float(k[4]),
                cx=float(k[2]),
                cy=float(k[5]),
                width=int(msg.width),
                height=int(msg.height),
            )
            event.set()

        sub = self._node.create_subscription(CameraInfo, self._camera_info_topic, _cb, 1)
        log.info(f"Ожидание camera_info: {self._camera_info_topic}")

        # Спиним ноду пока не придёт camera_info
        deadline = time.time() + timeout_s
        while not event.is_set() and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_subscription(sub)

        if not event.is_set():
            raise TimeoutError(
                f"camera_info не получен за {timeout_s}s. "
                f"Топик: {self._camera_info_topic}"
            )
        return result


# ---------------------------------------------------------------------------
# Миссия
# ---------------------------------------------------------------------------

async def mission(args) -> None:
    async with GazeboArucoSource(
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        marker_size=args.marker_size,
        camera_yaw_deg=args.camera_yaw,
    ) as src:
        async with DroneController(connection_string=args.connection) as drone:
            await drone.connect(timeout_s=30.0)
            await drone.apply_safe_params()
            await drone.wait_until_ready(timeout_s=60.0)

            await drone.takeoff(altitude_m=5.0, timeout_s=30.0)

            result = await drone.precision_land(
                get_marker_offset=src.marker_callback,
                descent_rate_mps=0.3,
                final_altitude_m=0.5,
                horizontal_tolerance_m=0.15,
                timeout_s=120.0,
            )

            log.info(f"Результат посадки: {result.status.value}")
            log.info(f"Финальная позиция: {result.final_position}")

            if not result:
                log.warning("Точная посадка не удалась, выполняется обычная посадка")
                await drone.land(timeout_s=30.0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Точная посадка через Gazebo ROS2 камеру + ArUco Fractal"
    )
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--image-topic", default=IMAGE_TOPIC)
    parser.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC)
    parser.add_argument("--marker-size", type=float, default=0.17)
    parser.add_argument(
        "--camera-yaw", type=float, default=0.0,
        help="Поворот камеры относительно носа дрона, градусы (0 = верх кадра к носу)"
    )
    args = parser.parse_args()
    asyncio.run(mission(args))


if __name__ == "__main__":
    main()
