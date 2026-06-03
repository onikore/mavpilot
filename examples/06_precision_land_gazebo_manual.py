"""Точная посадка: ручной полёт → OFFBOARD → precision landing (Gazebo).

Сценарий:
  1. Запустить: bash examples/run_gazebo.sh 06
  2. Лететь вручную к площадке
  3. Переключить FC в OFFBOARD → скрипт берёт управление:
       поворот к landing-yaw → центровка + снижение на маркер

Вся ROS2/Gazebo-обвязка живёт в mavpilot.integrations.gazebo.

Просмотр детекции:
  ros2 run rqt_image_view rqt_image_view /mavpilot/detection_image
"""
from __future__ import annotations

import argparse
import asyncio
import logging

from mavpilot import DroneController
from mavpilot.integrations.gazebo import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    GazeboArucoSource,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("drone")


async def main_async(args) -> None:
    source = GazeboArucoSource(
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

        if args.landing_yaw is not None:
            await drone.set_yaw(args.landing_yaw, timeout_s=30.0)

        result = await drone.precision_land(
            get_marker_offset=src.marker_callback,
            descent_rate_mps=args.descent_rate,
            final_altitude_m=args.land_distance,
            timeout_s=120.0,
        )
        log.info(f"result: {result.status.value}  pos={result.final_position}")
        if not result:
            log.warning("precision_land failed — fallback land")
            await drone.land()


def main() -> None:
    p = argparse.ArgumentParser(description="Manual fly → OFFBOARD → precision land (Gazebo)")
    p.add_argument("--connection", default="udp:127.0.0.1:14540")
    p.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    p.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    p.add_argument("--marker-size", type=float, default=0.17)
    p.add_argument("--camera-yaw", type=float, default=0.0, help="camera rotation, deg")
    p.add_argument("--landing-yaw", type=float, default=None, help="yaw at touchdown, deg NED")
    p.add_argument("--descent-rate", type=float, default=0.2, help="descent speed, m/s")
    p.add_argument("--land-distance", type=float, default=0.5, help="final alt before AUTO_LAND, m")
    args = p.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        log.info("stopped")


if __name__ == "__main__":
    main()
