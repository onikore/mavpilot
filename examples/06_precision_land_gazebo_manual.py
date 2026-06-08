"""Точная посадка: ручной полёт → OFFBOARD → precision landing (Gazebo).

Сценарий:
  1. Запустить: python examples/06_precision_land_gazebo_manual.py
     (нужен ROS2: source /opt/ros/<distro>/setup.bash, и pip install nanofractal)
  2. Лететь вручную к площадке (любая высота)
  3. Переключить FC в OFFBOARD → скрипт берёт управление:
       • поворот к landing-yaw (опционально)
       • слепое снижение до --approach-alt (высота надёжной детекции)
       • центровка + снижение на маркер (precision_land)

Вся ROS2/Gazebo-обвязка живёт в mavpilot.integrations.gazebo (детекция —
mavpilot.integrations.nanofractal).

Просмотр детекции:
  ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image

Типичный запуск:
  python examples/06_precision_land_gazebo_manual.py \\
      --approach-alt 2.5 --descent-rate 0.3 --marker-size 0.17
"""

from __future__ import annotations

import argparse
import asyncio
import logging

from mavpilot import DroneController
from mavpilot.integrations.gazebo import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    GazeboFractalSource,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("drone")


async def main_async(args) -> None:
    source = GazeboFractalSource(
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        marker_size=args.marker_size,
        camera_yaw_deg=args.camera_yaw,
    )
    drone = DroneController(connection_string=args.connection)
    async with source as src, drone:
        await drone.connect(timeout_s=30.0)
        await drone.wait_until_ready(timeout_s=60.0)

        await drone.wait_for_offboard()  # ручной полёт → ждём переключения
        log.info("OFFBOARD active — script has control")

        if args.landing_yaw is not None:
            log.info(f"Rotating to landing yaw {args.landing_yaw}°")
            await drone.set_yaw(args.landing_yaw, timeout_s=30.0)

        # Слепое снижение до высоты надёжной детекции маркера.
        # Без этого шага дрон зависает: камера не видит маркер с большой высоты,
        # precision_land ждёт детекцию и через marker_lost_timeout делает
        # обычный AUTO_LAND вместо точного.
        pos = drone.get_local_position()
        current_alt = -pos.z  # z NED отрицательный вверх
        if current_alt > args.approach_alt + 0.1:
            log.info(
                f"Blind descent: {current_alt:.1f} m → {args.approach_alt:.1f} m "
                f"(approach altitude for marker detection)"
            )
            await drone.goto(
                x=pos.x,
                y=pos.y,
                z=-args.approach_alt,
                timeout_s=60.0,
            )
        else:
            log.info(f"Already at {current_alt:.1f} m — skipping blind descent")

        log.info("Starting precision landing")
        result = await drone.precision_land(
            get_marker_offset=src.marker_callback,
            descent_rate_mps=args.descent_rate,
            final_altitude_m=args.land_distance,
            horizontal_tolerance_m=args.h_tolerance,
            marker_lost_timeout_s=args.marker_timeout,
            timeout_s=120.0,
        )
        log.info(f"result: {result.status.value}  pos={result.final_position}  iters={result.iterations}")
        if not result:
            log.warning("precision_land failed — fallback land")
            await drone.land()


def main() -> None:
    p = argparse.ArgumentParser(description="Manual fly → OFFBOARD → precision land (Gazebo)")
    p.add_argument("--connection", default="udp:127.0.0.1:14540")
    p.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    p.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    p.add_argument("--marker-size", type=float, default=0.17)
    p.add_argument("--camera-yaw", type=float, default=0.0, help="camera rotation offset, deg")
    p.add_argument("--landing-yaw", type=float, default=None, help="yaw at touchdown, deg NED")
    # Approach altitude: drone descends here blindly before precision_land starts.
    # Set to the highest altitude at which the camera reliably detects the marker.
    p.add_argument(
        "--approach-alt", type=float, default=2.5,
        help="blind-descent target altitude (m) before precision_land; "
             "must be within camera detection range of the marker (default: 2.5)",
    )
    p.add_argument("--descent-rate", type=float, default=0.3, help="descent speed during precision_land, m/s")
    p.add_argument("--land-distance", type=float, default=0.5, help="final alt before AUTO_LAND, m")
    p.add_argument("--h-tolerance", type=float, default=0.15, help="horizontal centering tolerance, m")
    p.add_argument(
        "--marker-timeout", type=float, default=5.0,
        help="seconds without marker detection before fallback AUTO_LAND (default: 5)",
    )
    args = p.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        log.info("stopped")


if __name__ == "__main__":
    main()
