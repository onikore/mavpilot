"""Точная посадка с симулированным маркером (без камеры).

Полезно для проверки логики precision_land без реального железа.
Маркер "стоит" в фиксированной NED-позиции; дрон должен над ним зависнуть и сесть.

Запуск в mock-режиме:
    python examples/03_precision_land_simulated.py --mock

Запуск с PX4 SITL:
    python examples/03_precision_land_simulated.py --connection udp:127.0.0.1:14540
"""

import asyncio
import argparse
import logging

from mavpilot import DroneController
from mavpilot.cli import make_simulated_marker

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# NED-координаты симулированного маркера (метры от точки взлёта)
MARKER_NED = (2.0, 1.0)


async def mission(connection: str, mock: bool) -> None:
    async with DroneController(connection_string=connection, mock=mock) as drone:
        await drone.connect(timeout_s=30.0)
        await drone.apply_safe_params()
        await drone.wait_until_ready(timeout_s=60.0)

        await drone.takeoff(altitude_m=5.0, timeout_s=30.0)

        # Летим к позиции над маркером
        await drone.goto(x=MARKER_NED[0], y=MARKER_NED[1], z=-5, timeout_s=20)

        marker_callback = make_simulated_marker(drone, marker_ned=MARKER_NED)

        result = await drone.precision_land(
            get_marker_offset=marker_callback,
            descent_rate_mps=0.3,
            final_altitude_m=0.5,
            horizontal_tolerance_m=0.15,
            timeout_s=60.0,
        )

        log = logging.getLogger("drone")
        log.info(f"Результат: {result.status.value}")
        log.info(f"Финальная позиция: {result.final_position}")
        log.info(f"Итераций: {result.iterations}")

        if not result:
            log.warning("Точная посадка не удалась, выполняется обычная")
            await drone.land(timeout_s=30.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Точная посадка (симулированный маркер)")
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    asyncio.run(mission(args.connection, args.mock))


if __name__ == "__main__":
    main()
