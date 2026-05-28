"""Command-line entrypoint and demo helper callbacks."""
import argparse
import asyncio
import logging
import math
import signal
from typing import Optional

from .controller import DroneController
from .types import MarkerObservation
from .utils import ned_to_body
from pymavlink import mavutil


def make_simulated_marker(drone: DroneController, marker_ned: tuple[float, float]):
    """Return a callback that simulates a marker at a fixed NED position."""
    mx, my = marker_ned

    def callback() -> Optional[MarkerObservation]:
        pos = drone.get_local_position()
        yaw = drone.get_yaw_rad()
        ned_dx = mx - pos.x
        ned_dy = my - pos.y
        body_dx, body_dy = ned_to_body(ned_dx, ned_dy, yaw)
        return MarkerObservation(dx=body_dx, dy=body_dy, dz=-pos.z)

    return callback


async def main():
    parser = argparse.ArgumentParser(
        description="mavpilot — PX4 autonomous drone controller with browser visualization."
    )
    parser.add_argument(
        "--connection", default="udp:127.0.0.1:14540",
        help="MAVLink endpoint (default: udp:127.0.0.1:14540 for SITL)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Mock mode: no MAVLink connection, uses built-in physics simulator.",
    )
    parser.add_argument("--viz-port", type=int, default=8765)
    parser.add_argument("--no-viz", action="store_true", help="Disable browser visualization")
    parser.add_argument(
        "--viz-host", default="127.0.0.1",
        help=(
            "Interface VizServer binds to (default: 127.0.0.1 — localhost-only). "
            "Use 0.0.0.0 to expose on LAN (telemetry will be visible to anyone on the network)."
        ),
    )
    parser.add_argument(
        "--precision-land", action="store_true",
        help="Use precision landing with simulated marker instead of regular land.",
    )
    parser.add_argument(
        "--pattern",
        choices=["square", "star"],
        default="square",
        help="Demo flight pattern: square or star (default: square)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    drone = DroneController(
        connection_string=args.connection,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        loop_hz=50.0,
        enable_viz=not args.no_viz,
        viz_port=args.viz_port,
        viz_host=args.viz_host,
        mock=args.mock,
    )

    shutdown = asyncio.Event()

    def on_signal(*_):
        logging.getLogger("drone").warning("Signal received — initiating emergency land")
        shutdown.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        await drone.connect(timeout_s=30.0)
        await drone.apply_safe_params()
        await drone.wait_until_ready(timeout_s=60.0)

        logging.getLogger("drone").info(
            f"Open http://localhost:{args.viz_port} in browser to see trajectory"
        )
        await asyncio.sleep(2.0)

        await drone.takeoff(altitude_m=2.0, timeout_s=30.0)

        if args.pattern == "square":
            await drone.goto(x=2, y=2, z=-1, hover_time_s=2, timeout_s=20)
            await drone.goto(x=-2, y=2, z=-2, hover_time_s=2, timeout_s=20)
            await drone.goto(x=-2, y=-2, z=-3, hover_time_s=2, timeout_s=20)
            await drone.goto(x=2, y=-2, z=-4, hover_time_s=2, timeout_s=20)
        else:
            pos = drone.get_local_position()
            cx, cy, cz = pos.x, pos.y, pos.z
            radius = 5.0
            points = 5
            pts = []
            for k in range(points):
                angle = 2 * math.pi * k / points - math.pi / 2
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                pts.append((x, y))
            order = []
            step = 2
            i = 0
            for _ in range(points):
                order.append(i)
                i = (i + step) % points
            waypoints = [pts[idx] for idx in order] + [pts[order[0]]]
            for x, y in waypoints:
                await drone.goto(x=x, y=y, z=cz, hover_time_s=1.5, timeout_s=20)

        if args.precision_land:
            marker_callback = make_simulated_marker(drone, marker_ned=(1.0, -1.0))
            result = await drone.precision_land(
                get_marker_offset=marker_callback,
                descent_rate_mps=0.5,
                final_altitude_m=0.5,
                horizontal_tolerance_m=0.2,
                timeout_s=60.0,
            )
            logging.getLogger("drone").info(f"precision_land result: {result.status.value}")
            if not result:
                logging.getLogger("drone").warning(
                    f"precision_land did not land cleanly: {result.status.value}; "
                    f"final position={result.final_position}"
                )
        else:
            await drone.land(timeout_s=30.0)

        await drone.disarm()
        logging.getLogger("drone").info("Mission complete")

    except Exception as e:
        logging.getLogger("drone").exception(f"Error in mission: {e}")
        try:
            await drone.emergency_land()
        except Exception:
            pass

    finally:
        drone.close()


def main_sync():
    """Synchronous wrapper for use as a console_scripts entry point."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
