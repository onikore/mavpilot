"""Базовый пример: взлёт → маршрут → посадка.

Запуск в mock-режиме (без дрона):
    python examples/01_basic_takeoff_land.py --mock

Запуск с реальным дроном (PX4 SITL или железо):
    python examples/01_basic_takeoff_land.py --connection udp:127.0.0.1:14540
"""

import asyncio
import argparse
import logging

from mavpilot import DroneController

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


async def mission(connection: str, mock: bool) -> None:
    async with DroneController(connection_string=connection, mock=mock) as drone:
        await drone.connect(timeout_s=30.0)
        await drone.apply_safe_params()
        await drone.wait_until_ready(timeout_s=60.0)

        await drone.takeoff(altitude_m=3.0, timeout_s=30.0)

        await drone.goto(x=5, y=0, z=-3, hover_time_s=2, timeout_s=20)
        await drone.goto(x=5, y=5, z=-3, hover_time_s=2, timeout_s=20)
        await drone.goto(x=0, y=5, z=-3, hover_time_s=2, timeout_s=20)
        await drone.goto(x=0, y=0, z=-3, hover_time_s=2, timeout_s=20)

        await drone.land(timeout_s=30.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Базовый взлёт → маршрут → посадка")
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    asyncio.run(mission(args.connection, args.mock))


if __name__ == "__main__":
    main()
