"""Command-line entrypoint and demo helper callbacks."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import math
import signal
from collections.abc import Awaitable, Callable

from pymavlink import mavutil

from .controller import DroneController
from .types import MarkerObservation
from .utils import ned_to_body

logger = logging.getLogger("drone")


def make_simulated_marker(drone: DroneController, marker_ned: tuple[float, float]):
    """Return a callback that simulates a marker at a fixed NED position."""
    mx, my = marker_ned

    def callback() -> MarkerObservation | None:
        pos = drone.get_local_position()
        yaw = drone.get_yaw_rad()
        ned_dx = mx - pos.x
        ned_dy = my - pos.y
        body_dx, body_dy = ned_to_body(ned_dx, ned_dy, yaw)
        return MarkerObservation(dx=body_dx, dy=body_dy, dz=-pos.z)

    return callback


async def handle_shutdown_signal(drone, reason: str = "shutdown") -> None:
    """Best-effort emergency land in response to a shutdown signal.

    Safe to call from outside the asyncio loop via loop.call_soon_threadsafe.
    Swallows all exceptions — this is the last-resort path and must not raise.
    """
    logger.warning(f"{reason} — initiating emergency land")
    try:
        await drone.emergency_land()
    except BaseException as e:
        logger.error(f"emergency_land raised during shutdown: {e}")


async def run_mission(drone, mission_body: Callable[[], Awaitable[None]]) -> None:
    """Run the user's mission body with emergency_land on ANY failure.

    Catches BaseException (NOT just Exception) so KeyboardInterrupt and
    SystemExit also trigger the safety path. Always calls drone.close().
    """
    try:
        await mission_body()
    except BaseException as e:  # noqa: BLE001 — intentional last-resort catch
        logger.exception(f"Mission terminated by {type(e).__name__}: {e}")
        try:
            await drone.emergency_land()
        except BaseException as inner:
            logger.error(f"emergency_land failed inside shutdown path: {inner}")
    finally:
        try:
            drone.close()
        except BaseException as e:
            logger.error(f"drone.close() raised: {e}")


def _build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="mavpilot — PX4 autonomous drone controller with browser visualization."
    )
    parser.add_argument(
        "--connection",
        default="udp:127.0.0.1:14540",
        help="MAVLink endpoint (default: udp:127.0.0.1:14540 for SITL)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Mock mode: no MAVLink connection, uses built-in physics simulator.",
    )
    parser.add_argument("--viz-port", type=int, default=8765)
    parser.add_argument("--no-viz", action="store_true", help="Disable browser visualization")
    parser.add_argument(
        "--viz-host",
        default="127.0.0.1",
        help=(
            "Interface VizServer binds to (default: 127.0.0.1 — localhost-only). "
            "Use 0.0.0.0 to expose on LAN (telemetry will be visible to anyone on the network)."
        ),
    )
    parser.add_argument(
        "--precision-land",
        action="store_true",
        help="Use precision landing with simulated marker instead of regular land.",
    )
    parser.add_argument(
        "--pattern",
        choices=["square", "star"],
        default="square",
        help="Demo flight pattern: square or star (default: square)",
    )
    return parser


async def _demo_mission(drone: DroneController, args) -> None:
    """The legacy demo flight: takeoff → pattern → (precision_)land → disarm."""
    await drone.connect(timeout_s=30.0)
    await drone.apply_safe_params()
    await drone.wait_until_ready(timeout_s=60.0)

    logger.info(f"Open http://{args.viz_host}:{args.viz_port} in browser to see trajectory")
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
        radius, points = 5.0, 5
        pts = []
        for k in range(points):
            angle = 2 * math.pi * k / points - math.pi / 2
            pts.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
        order = []
        step, i = 2, 0
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
        logger.info(f"precision_land result: {result.status.value}")
        if not result:
            logger.warning(
                f"precision_land did not land cleanly: {result.status.value}; "
                f"final position={result.final_position}"
            )
    else:
        await drone.land(timeout_s=30.0)

    await drone.disarm()
    logger.info("Mission complete")


def _install_signal_handlers(loop: asyncio.AbstractEventLoop, drone: DroneController) -> None:
    """Wire SIGINT/SIGTERM to schedule emergency_land on the asyncio loop."""

    def _scheduler(signame: str) -> Callable[..., None]:
        def _h(*_: object) -> None:
            with contextlib.suppress(RuntimeError):
                loop.call_soon_threadsafe(
                    lambda: asyncio.ensure_future(handle_shutdown_signal(drone, reason=signame))
                )

        return _h

    signal.signal(signal.SIGINT, _scheduler("SIGINT"))
    signal.signal(signal.SIGTERM, _scheduler("SIGTERM"))


async def main() -> None:
    args = _build_argparser().parse_args()

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

    _install_signal_handlers(asyncio.get_running_loop(), drone)

    await run_mission(drone, mission_body=lambda: _demo_mission(drone, args))


def main_sync() -> None:
    """Synchronous wrapper for use as a console_scripts entry point."""
    # run_mission already caught and called emergency_land; this just
    # suppresses the noisy traceback on the way out.
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main())
