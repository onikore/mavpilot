"""Точная посадка через Gazebo ROS2 топики с ArUco Fractal детектором.

Gazebo публикует топики в своём транспорте (gz-transport), а не в ROS2 напрямую.
GazeboArucoSource автоматически запускает ros_gz_bridge, который пробрасывает
gz-топики в ROS2, а затем подписывается на них.

Читает кадры и параметры камеры из:
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info

Требует:
    pip install mavpilot[aruco]
    ROS2 (humble / jazzy) + source /opt/ros/<distro>/setup.bash
    ros-<distro>-ros-gz-bridge  (apt install ros-humble-ros-gz-bridge)
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
import subprocess
import threading
import time

import cv2
import numpy as np

from mavpilot import DroneController
from mavpilot.types import MarkerObservation

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("drone")

IMAGE_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
)
CAMERA_INFO_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"
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

        # cv_bridge скомпилирован под NumPy 1.x и падает с NumPy 2.x (сегфолт).
        # Используем только ручную конвертацию через numpy — надёжно и без зависимостей.
        self._sub = node.create_subscription(Image, topic, self._on_image, 1)
        log.info(f"Подписка на изображение: {topic}")

    def _on_image(self, msg) -> None:
        try:
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
# Visualization publisher — публикует кадр с наложением детекции в ROS2
# ---------------------------------------------------------------------------

class VisualizationPublisher:
    """Публикует /mavpilot/detection_image — кадр с нарисованным маркером.

    Запускается в фоновом треде, берёт свежий кадр из stream,
    накладывает оверлей через DetectionThread.draw_overlays() и публикует
    sensor_msgs/Image. Смотреть в RViz2 или:
        ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image
    """

    def __init__(self, node, stream: ROS2ImageStream, detector, fps: float = 10.0) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]

        self._node = node
        self._stream = stream
        self._detector = detector
        self._fps = fps
        self._running = True
        self._pub = node.create_publisher(Image, "/mavpilot/detection_image", 1)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("Визуализация: /mavpilot/detection_image")

    def _loop(self) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]

        interval = 1.0 / self._fps
        while self._running:
            t0 = time.time()
            frame = self._stream.read()
            if frame is not None:
                vis = frame.copy()
                self._detector.draw_overlays(vis)
                # Добавляем текст: дистанция до маркера и FPS детектора
                state = self._detector.state
                if state is not None and state.detected and state.has_pose:
                    dz = float(state.tvec.flatten()[2])
                    cv2.putText(
                        vis, f"dz={dz:.2f}m  det={self._detector.det_fps:.0f}fps",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                    )
                else:
                    cv2.putText(
                        vis, "no marker", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                    )

                msg = Image()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.height, msg.width = vis.shape[:2]
                msg.encoding = "bgr8"
                msg.step = msg.width * 3
                msg.data = vis.tobytes()
                self._pub.publish(msg)

            elapsed = time.time() - t0
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def stop(self) -> None:
        self._running = False


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
        self._viz_pub: VisualizationPublisher | None = None
        self._node = None
        self._spin_thread: threading.Thread | None = None
        self._bridge_proc: subprocess.Popen | None = None  # type: ignore[type-arg]

    def _start_gz_bridge(self) -> None:
        """Запускает ros_gz_bridge для проброса Gazebo-топиков в ROS2.

        Gazebo публикует через gz-transport, ROS2 их не видит без бриджа.
        parameter_bridge форматирует правила как:
            TOPIC@ROS_TYPE[GZ_TYPE   (Gazebo → ROS2, однонаправленно)
        """
        rules = [
            f"{self._image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{self._camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + rules
        log.info(f"Запуск gz bridge: {' '.join(cmd)}")
        self._bridge_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Даём бриджу время подняться и зарегистрировать топики
        time.sleep(2.0)
        log.info("gz bridge запущен")

    async def __aenter__(self) -> GazeboArucoSource:
        import rclpy  # type: ignore[import]
        from arucofractal import Config, DetectionThread  # type: ignore[import]

        # Сначала поднимаем мост Gazebo → ROS2
        await asyncio.to_thread(self._start_gz_bridge)

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
        self._viz_pub = VisualizationPublisher(self._node, self._stream, self._detector)

        # Запускаем spin в фоне — он обрабатывает все ROS2 колбэки
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()
        log.info("ROS2 нода запущена, ждём первые кадры...")

        return self

    async def __aexit__(self, *_: object) -> None:
        if self._viz_pub is not None:
            self._viz_pub.stop()
        if self._detector is not None:
            self._detector.stop()
        if self._node is not None:
            self._node.destroy_node()
        try:
            import rclpy  # type: ignore[import]
            rclpy.shutdown()
        except Exception:
            pass
        if self._bridge_proc is not None:
            self._bridge_proc.terminate()
            self._bridge_proc.wait(timeout=5)
            log.info("gz bridge остановлен")

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
# Снижение по маркеру (без использования высоты дрона)
# ---------------------------------------------------------------------------

async def marker_guided_descent(
    drone: DroneController,
    get_marker_offset,
    descent_rate_mps: float = 0.2,
    lateral_p_gain: float = 0.7,
    max_horizontal_step_m: float = 0.8,
    land_distance_m: float = 0.5,
    marker_lost_timeout_s: float = 3.0,
    timeout_s: float = 120.0,
) -> None:
    """Снижение, управляемое только маркером.

    Логика:
    - Маркер виден → снижаемся непрерывно + корректируем горизонталь
    - dz (расстояние до маркера) < land_distance_m → посадка
    - Маркер потерян на > marker_lost_timeout_s → аварийная посадка
    - Высота дрона не используется в качестве условия
    """
    from mavpilot.utils import body_to_ned

    start = time.time()
    last_seen = time.time()

    while time.time() - start < timeout_s:
        pos = drone.get_local_position()
        yaw = drone.get_yaw_rad()
        obs = get_marker_offset()

        if obs is not None:
            last_seen = time.time()
            ned_dx, ned_dy = body_to_ned(obs.dx, obs.dy, yaw)

            step_x = max(-max_horizontal_step_m,
                         min(max_horizontal_step_m, ned_dx * lateral_p_gain))
            step_y = max(-max_horizontal_step_m,
                         min(max_horizontal_step_m, ned_dy * lateral_p_gain))

            target_x = pos.x + step_x
            target_y = pos.y + step_y
            # NED: z увеличивается вниз, добавляем смещение вниз
            target_z = pos.z + descent_rate_mps * drone.loop_period

            if obs.dz is not None and obs.dz < land_distance_m:
                log.info(f"Маркер близко dz={obs.dz:.2f}m < {land_distance_m}m — посадка")
                await drone.land(timeout_s=30.0)
                return

            drone._set_setpoint_position(target_x, target_y, target_z, yaw)

        else:
            # Маркер не виден — удерживаем позицию
            drone._set_setpoint_position(pos.x, pos.y, pos.z, yaw)
            lost_for = time.time() - last_seen
            if lost_for > marker_lost_timeout_s:
                log.warning(f"Маркер потерян {lost_for:.1f}s — аварийная посадка")
                await drone.land(timeout_s=30.0)
                return

        await asyncio.sleep(drone.loop_period)

    log.warning("Таймаут marker_guided_descent — аварийная посадка")
    await drone.land(timeout_s=30.0)


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

            await drone.takeoff(altitude_m=args.takeoff_alt, timeout_s=30.0)

            log.info("Начинаю снижение по маркеру...")
            await marker_guided_descent(
                drone=drone,
                get_marker_offset=src.marker_callback,
                descent_rate_mps=args.descent_rate,
                land_distance_m=args.land_distance,
                timeout_s=120.0,
            )
            log.info("Миссия завершена")


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
        help="Поворот камеры, градусы (0 = верх кадра к носу)"
    )
    parser.add_argument("--takeoff-alt", type=float, default=3.0,
                        help="Высота взлёта в метрах (default: 3.0)")
    parser.add_argument("--descent-rate", type=float, default=0.2,
                        help="Скорость снижения м/с (default: 0.2)")
    parser.add_argument("--land-distance", type=float, default=0.5,
                        help="Расстояние до маркера для посадки, м (default: 0.5)")
    args = parser.parse_args()
    asyncio.run(mission(args))


if __name__ == "__main__":
    main()
