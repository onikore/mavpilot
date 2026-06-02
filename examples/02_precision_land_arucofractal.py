"""Точная посадка через ArUco Fractal маркер (реальная камера).

Требует: pip install mavpilot[aruco]

Запуск:
    python examples/02_precision_land_arucofractal.py --connection udp:127.0.0.1:14540

Параметры камеры:
    --source usb               USB-камера (индекс 0 по умолчанию)
    --source rtsp              RTSP-поток (см. --rtsp-url)
    --rtsp-url rtsp://...      URL потока
    --camera-yaw 0             Поворот камеры относительно носа дрона, градусы.
                               0   = верх кадра смотрит на нос дрона (стандарт)
                               180 = верх кадра смотрит на хвост
                               90  = верх кадра смотрит вправо
"""

import asyncio
import argparse
import logging

from mavpilot import DroneController
from mavpilot.integrations.arucofractal import ArucoFractalSource

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


async def mission(args) -> None:
    from arucofractal import Config as ArucoConfig

    aruco_cfg = ArucoConfig(
        source=args.source,
        rtsp_url=args.rtsp_url,
        usb_camera_index=args.usb_index,
        marker_size=args.marker_size,
    )

    async with ArucoFractalSource(aruco_cfg, camera_yaw_deg=args.camera_yaw) as src:
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

            logging.getLogger("drone").info(f"Результат посадки: {result.status.value}")

            if not result:
                logging.getLogger("drone").warning(
                    f"Точная посадка не удалась ({result.status.value}), "
                    "выполняется обычная посадка"
                )
                await drone.land(timeout_s=30.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Точная посадка через ArUco Fractal")
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--source", choices=["usb", "rtsp"], default="usb")
    parser.add_argument("--rtsp-url", default="rtsp://192.168.0.3:8080/h264_ulaw.sdp")
    parser.add_argument("--usb-index", type=int, default=0)
    parser.add_argument("--marker-size", type=float, default=0.17,
                        help="Размер маркера в метрах")
    parser.add_argument("--camera-yaw", type=float, default=0.0,
                        help="Угол поворота камеры в градусах (0 = верх кадра к носу)")
    args = parser.parse_args()
    asyncio.run(mission(args))


if __name__ == "__main__":
    main()
