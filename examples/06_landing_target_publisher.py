"""Постоянный издатель LANDING_TARGET для встроенного precision landing PX4.

Сценарий:
  1. Запустить этот скрипт (работает всегда, не берёт управление дроном)
  2. Лететь вручную к площадке
  3. Включить режим Precision Land на FC (QGC / RC-переключатель / companion)
  4. PX4 сам садится по LANDING_TARGET сообщениям

Скрипт НЕ использует OFFBOARD-режим и не отправляет setpoints.
Он только слушает камеру и шлёт MAVLink LANDING_TARGET.

Требует:
    bash examples/run_gazebo.sh 06   (устанавливает LD_LIBRARY_PATH / PYTHONPATH)
    ros-<distro>-ros-gz-bridge

Параметры PX4 для включения:
    EKF2_AID_MASK: включить бит "vision position" или "landing target"
    PLD_BTOUT: таймаут маяка, с (default 5s)
    PLD_HACC_RAD: точность горизонтального попадания (default 0.2m)
    PLD_FAPPR_ALT: высота начала точного снижения (default 0.1m)

Просмотр визуализации:
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

IMAGE_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
)
CAMERA_INFO_TOPIC = (
    "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"
)

# PX4 custom_mode упакован: bits[16:24] = main_mode, bits[24:32] = sub_mode
_PX4_MAIN_MODES = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO",   6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
}
# Sub-mode только для AUTO (main_mode == 4)
_PX4_AUTO_SUBMODES = {
    2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL",
    6: "LAND",    9: "PRECLAND",
}


def _decode_px4_mode(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    name = _PX4_MAIN_MODES.get(main, f"MAIN_{main}")
    if main == 4 and sub:  # AUTO + sub-mode
        name = f"AUTO/{_PX4_AUTO_SUBMODES.get(sub, f'SUB_{sub}')}"
    return name


# ---------------------------------------------------------------------------
# ROS2 image stream
# ---------------------------------------------------------------------------

class ROS2ImageStream:
    def __init__(self, node, topic: str) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._sub = node.create_subscription(Image, topic, self._on_image, 1)

    def _on_image(self, msg) -> None:
        try:
            channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, channels
            )
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self._lock:
                self._frame = frame.copy()
        except Exception as e:
            log.error(f"Ошибка декодирования кадра: {e}")

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    def stop(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Visualization publisher
# ---------------------------------------------------------------------------

class VisualizationPublisher:
    def __init__(self, node, stream: ROS2ImageStream, detector, fps: float = 10.0) -> None:
        from sensor_msgs.msg import Image  # type: ignore[import]
        self._node = node
        self._stream = stream
        self._detector = detector
        self._fps = fps
        self._running = True
        self._pub = node.create_publisher(Image, "/mavpilot/detection_image", 1)
        threading.Thread(target=self._loop, daemon=True).start()
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
                state = self._detector.state
                if state is not None and state.detected and state.has_pose:
                    dz = float(state.tvec.flatten()[2])
                    cv2.putText(vis, f"dz={dz:.2f}m  {self._detector.det_fps:.0f}fps",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(vis, "no marker", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                msg = Image()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.height, msg.width = vis.shape[:2]
                msg.encoding = "bgr8"
                msg.step = msg.width * 3
                msg.data = vis.tobytes()
                self._pub.publish(msg)
            remaining = interval - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# Aruco source (Gazebo ROS2)
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
        self._viz_pub: VisualizationPublisher | None = None
        self._node = None
        self._spin_thread: threading.Thread | None = None
        self._bridge_proc: subprocess.Popen | None = None  # type: ignore[type-arg]

    async def __aenter__(self) -> GazeboArucoSource:
        import rclpy  # type: ignore[import]
        from arucofractal import Config, DetectionThread  # type: ignore[import]

        await asyncio.to_thread(self._start_gz_bridge)

        rclpy.init()
        self._node = rclpy.create_node("mavpilot_landing_target")

        cam_info = await asyncio.to_thread(
            self._wait_for_camera_info, rclpy, self._camera_info_timeout_s
        )
        log.info(
            f"Камера: fx={cam_info['fx']:.1f} fy={cam_info['fy']:.1f} "
            f"w={cam_info['width']} h={cam_info['height']}"
        )

        cfg = Config(
            marker_size=self._marker_size,
            camera_fx=cam_info["fx"], camera_fy=cam_info["fy"],
            camera_cx=cam_info["cx"], camera_cy=cam_info["cy"],
            frame_width=cam_info["width"], frame_height=cam_info["height"],
        )
        self._stream = ROS2ImageStream(self._node, self._image_topic)
        self._detector = DetectionThread(self._stream, cfg)
        self._viz_pub = VisualizationPublisher(self._node, self._stream, self._detector)

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()
        return self

    async def __aexit__(self, *_: object) -> None:
        if self._viz_pub:
            self._viz_pub.stop()
        if self._detector:
            self._detector.stop()
        if self._node:
            self._node.destroy_node()
        try:
            import rclpy  # type: ignore[import]
            rclpy.shutdown()
        except Exception:
            pass
        if self._bridge_proc:
            self._bridge_proc.terminate()
            self._bridge_proc.wait(timeout=5)

    def marker_callback(self) -> MarkerObservation | None:
        if self._detector is None:
            return None
        state = self._detector.state
        if state is None or not state.detected or not state.has_pose:
            return None
        tvec = state.tvec.flatten()
        dx = -float(tvec[1])
        dy = float(tvec[0])
        dz = float(tvec[2])
        if self._camera_yaw_deg != 0.0:
            theta = math.radians(self._camera_yaw_deg)
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            dx, dy = dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t
        return MarkerObservation(dx=dx, dy=dy, dz=dz)

    def _start_gz_bridge(self) -> None:
        rules = [
            f"{self._image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{self._camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + rules
        log.info(f"gz bridge: {' '.join(cmd)}")
        self._bridge_proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(2.0)

    def _wait_for_camera_info(self, rclpy, timeout_s: float) -> dict:
        from sensor_msgs.msg import CameraInfo  # type: ignore[import]
        result: dict = {}
        event = threading.Event()

        def _cb(msg) -> None:
            k = msg.k
            result.update(fx=float(k[0]), fy=float(k[4]),
                          cx=float(k[2]), cy=float(k[5]),
                          width=int(msg.width), height=int(msg.height))
            event.set()

        sub = self._node.create_subscription(CameraInfo, self._camera_info_topic, _cb, 1)
        deadline = time.time() + timeout_s
        while not event.is_set() and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_subscription(sub)
        if not event.is_set():
            raise TimeoutError(f"camera_info не получен за {timeout_s}s")
        return result


# ---------------------------------------------------------------------------
# LANDING_TARGET publisher (pymavlink, без OFFBOARD)
# ---------------------------------------------------------------------------

class LandingTargetPublisher:
    """Подключается к PX4.

    Фаза 1 (ручной полёт): шлёт LANDING_TARGET непрерывно.
    Фаза 2 (OFFBOARD активирован пользователем): берёт управление —
      центрируется над маркером, снижается, садится с фиксированным явом.

    Переход происходит автоматически при смене режима на OFFBOARD.
    """

    # type_mask для SET_POSITION_TARGET_LOCAL_NED:
    # позиция + яв, игнорировать скорость/ускорение/яв-rate
    _POS_YAW_MASK = (
        0b0000_1111_1000  # ignore vel(3-5) + acc(6-8)
        | 0b1000_0000_0000  # ignore yaw_rate(11)
    )  # = 0x9F8 = 2552

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
        self._landing_yaw_deg = landing_yaw_deg
        self._landing_yaw_rad = math.radians(landing_yaw_deg) if landing_yaw_deg is not None else None
        self._descent_rate = descent_rate_mps
        self._land_distance = land_distance_m
        self._lateral_p = lateral_p_gain

        self._mav: mavutil.mavfile | None = None  # type: ignore[type-arg]
        self._running = False
        self._current_mode: str = ""
        self._yaw_rad: float = 0.0
        self._ned: tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, z
        self._tele_lock = threading.Lock()

        if landing_yaw_deg is not None:
            log.info(f"Целевой яв при посадке: {landing_yaw_deg}°")

    async def __aenter__(self) -> LandingTargetPublisher:
        log.info(f"Подключение к PX4: {self._conn_str}")
        self._mav = await asyncio.to_thread(self._connect)
        self._running = True
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        return self

    async def __aexit__(self, *_: object) -> None:
        self._running = False
        if self._mav:
            self._mav.close()

    def _connect(self) -> mavutil.mavfile:  # type: ignore[type-arg]
        mav = mavutil.mavlink_connection(self._conn_str, source_system=1)
        mav.wait_heartbeat(timeout=30)
        log.info(f"PX4 подключён. sys={mav.target_system} comp={mav.target_component}")
        return mav

    def _telemetry_loop(self) -> None:
        """Читает HEARTBEAT + ATTITUDE + LOCAL_POSITION_NED."""
        last_mode = None
        while self._running and self._mav:
            msg = self._mav.recv_match(
                type=["HEARTBEAT", "ATTITUDE", "LOCAL_POSITION_NED"],
                blocking=True, timeout=1.0,
            )
            if msg is None:
                continue
            t = msg.get_type()

            if t == "ATTITUDE":
                with self._tele_lock:
                    self._yaw_rad = float(msg.yaw)

            elif t == "LOCAL_POSITION_NED":
                with self._tele_lock:
                    self._ned = (float(msg.x), float(msg.y), float(msg.z))

            elif t == "HEARTBEAT":
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                label = _decode_px4_mode(msg.custom_mode)
                self._current_mode = label
                if label != last_mode:
                    log.info(f"FC режим: {label}")
                    last_mode = label

    # ------------------------------------------------------------------
    # Фаза 1: пассивный LANDING_TARGET
    # ------------------------------------------------------------------

    async def run(self, get_marker_offset) -> None:
        """Ждёт OFFBOARD, затем переходит к активному снижению."""
        interval = 1.0 / self._rate_hz
        sent = 0
        last_log = time.time()

        log.info(
            "Скрипт запущен.\n"
            "  Фаза 1: ручной полёт — шлю LANDING_TARGET.\n"
            "  Переключите FC в OFFBOARD → начнётся активное снижение.\n"
            "  Ctrl+C для выхода."
        )

        while True:
            t0 = time.time()

            if self._current_mode == "OFFBOARD":
                log.info("OFFBOARD обнаружен — начинаю активное снижение")
                await self._offboard_descent(get_marker_offset)
                return

            obs = get_marker_offset()
            if obs is not None and obs.dz is not None:
                self._send_landing_target(obs)
                sent += 1

            now = time.time()
            if now - last_log >= 5.0:
                if obs is not None and obs.dz is not None:
                    log.info(
                        f"Маркер: dx={obs.dx:+.2f} dy={obs.dy:+.2f} "
                        f"dz={obs.dz:.2f}m  lt_sent={sent}"
                    )
                else:
                    log.info("Маркер не виден")
                last_log = now
                sent = 0

            rem = interval - (time.time() - t0)
            if rem > 0:
                await asyncio.sleep(rem)

    # ------------------------------------------------------------------
    # Фаза 2: OFFBOARD снижение с фиксированным явом
    # ------------------------------------------------------------------

    async def _offboard_descent(self, get_marker_offset) -> None:
        interval = 1.0 / self._rate_hz
        last_seen = time.time()
        marker_lost_timeout = 3.0

        with self._tele_lock:
            hold_x, hold_y, hold_z = self._ned
            current_yaw = self._yaw_rad

        # Если landing_yaw задан — используем его, иначе держим текущий яв
        target_yaw = self._landing_yaw_rad if self._landing_yaw_rad is not None else current_yaw
        log.info(f"OFFBOARD снижение. Целевой яв: {math.degrees(target_yaw):.1f}°")

        while True:
            t0 = time.time()

            with self._tele_lock:
                pos_x, pos_y, pos_z = self._ned
                cur_yaw = self._yaw_rad

            obs = get_marker_offset()

            if obs is not None and obs.dz is not None:
                last_seen = time.time()

                # Поворот смещения из body FRD в NED (используем cur_yaw для трансформа)
                cos_y = math.cos(cur_yaw)
                sin_y = math.sin(cur_yaw)
                dx_ned = obs.dx * cos_y - obs.dy * sin_y
                dy_ned = obs.dx * sin_y + obs.dy * cos_y

                step = interval * self._descent_rate
                hold_x = pos_x + dx_ned * self._lateral_p
                hold_y = pos_y + dy_ned * self._lateral_p
                hold_z = pos_z + step           # NED: z↑ = вниз

                if obs.dz < self._land_distance:
                    log.info(f"Маркер близко dz={obs.dz:.2f}m — посадка")
                    self._send_land_command(target_yaw)
                    return

            else:
                if time.time() - last_seen > marker_lost_timeout:
                    log.warning("Маркер потерян — аварийная посадка")
                    self._send_land_command(target_yaw)
                    return

            self._send_position_setpoint(hold_x, hold_y, hold_z, target_yaw)

            rem = interval - (time.time() - t0)
            if rem > 0:
                await asyncio.sleep(rem)

    def _send_position_setpoint(
        self, x: float, y: float, z: float, yaw: float
    ) -> None:
        """SET_POSITION_TARGET_LOCAL_NED — позиция + яв."""
        if self._mav is None:
            return
        self._mav.mav.set_position_target_local_ned_send(
            0,                                        # time_boot_ms
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            self._POS_YAW_MASK,
            x, y, z,                                  # позиция (NED, метры)
            0.0, 0.0, 0.0,                            # скорость (игнор)
            0.0, 0.0, 0.0,                            # ускорение (игнор)
            float(yaw),                               # яв (рад)
            0.0,                                      # яв-rate (игнор)
        )

    def _send_land_command(self, yaw_rad: float) -> None:
        """Переключает PX4 в режим AUTO/LAND."""
        if self._mav is None:
            return
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0.0, 0.0, 0.0,
            math.degrees(yaw_rad),   # param4: яв
            0.0, 0.0, 0.0,
        )

    def _send_landing_target(self, obs: MarkerObservation) -> None:
        if self._mav is None:
            return
        dz = obs.dz if obs.dz is not None else 0.0
        if dz <= 0:
            return

        # По документации PX4 precision landing:
        #   frame = MAV_FRAME_LOCAL_NED
        #   x, y, z = позиция маркера относительно дрона в NED-фрейме (метры)
        #   position_valid = 1
        #
        # Наблюдение из камеры в теле дрона (FRD):
        #   obs.dx = вперёд, obs.dy = вправо, dz = вниз/расстояние
        #
        # Поворот body FRD → NED с учётом яв дрона:
        #   x_ned =  dx * cos(yaw) - dy * sin(yaw)   (North)
        #   y_ned =  dx * sin(yaw) + dy * cos(yaw)   (East)
        #   z_ned =  dz                               (Down, положительный)
        with self._yaw_lock:
            yaw = self._yaw_rad

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        x_ned = obs.dx * cos_y - obs.dy * sin_y
        y_ned = obs.dx * sin_y + obs.dy * cos_y
        z_ned = dz

        try:
            self._mav.mav.landing_target_send(
                int(time.time() * 1e6),              # time_usec
                0,                                    # target_num
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame — требуется по доке
                0.0,                                  # angle_x (не используется)
                0.0,                                  # angle_y (не используется)
                float(dz),                            # distance
                0.0,                                  # size_x
                0.0,                                  # size_y
                float(x_ned),                         # x: смещение на север (м)
                float(y_ned),                         # y: смещение на восток (м)
                float(z_ned),                         # z: смещение вниз (м)
                self._landing_q,                      # q: желаемый яв при посадке
                mavutil.mavlink.LANDING_TARGET_TYPE_LIGHT_BEACON,
                1,                                    # position_valid
            )
        except Exception as e:
            log.debug(f"landing_target_send: {e}")


# ---------------------------------------------------------------------------
# Main
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
    parser = argparse.ArgumentParser(
        description=(
            "Непрерывный издатель LANDING_TARGET для PX4 precision landing.\n"
            "Не берёт управление дроном. Летите вручную, "
            "затем включите Precision Land на FC."
        )
    )
    parser.add_argument("--connection", default="udp:127.0.0.1:14540",
                        help="MAVLink endpoint (default: udp:127.0.0.1:14540)")
    parser.add_argument("--image-topic", default=IMAGE_TOPIC)
    parser.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC)
    parser.add_argument("--marker-size", type=float, default=0.17,
                        help="Размер маркера в метрах")
    parser.add_argument("--camera-yaw", type=float, default=0.0,
                        help="Поворот камеры, градусы (0 = верх кадра к носу)")
    parser.add_argument("--rate", type=float, default=15.0,
                        help="Частота отправки LANDING_TARGET, Гц (default: 15)")
    parser.add_argument(
        "--landing-yaw", type=float, default=None,
        help="Желаемый яв при посадке, градусы NED (0=север, 90=восток).",
    )
    parser.add_argument("--descent-rate", type=float, default=0.2,
                        help="Скорость снижения в OFFBOARD, м/с (default: 0.2)")
    parser.add_argument("--land-distance", type=float, default=0.5,
                        help="Расстояние до маркера для посадки, м (default: 0.5)")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        log.info("Остановлено.")


if __name__ == "__main__":
    main()
