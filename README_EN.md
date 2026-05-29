# mavpilot

> 🇷🇺 [Русская версия](README.md)

**Async PX4 drone controller for Python** — sequential autonomous flight via MAVLink, with built-in live 3D visualization and a hardware-free mock mode.

[![CI](https://github.com/Onikore/mavpilot/actions/workflows/ci.yml/badge.svg)](https://github.com/Onikore/mavpilot/actions)
[![PyPI](https://img.shields.io/pypi/v/mavpilot)](https://pypi.org/project/mavpilot/)
[![Python](https://img.shields.io/pypi/pyversions/mavpilot)](https://pypi.org/project/mavpilot/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## Features

| | |
|---|---|
| **Pure asyncio API** | Write sequential mission logic with `await` — no callbacks, no state machines |
| **PX4 OFFBOARD mode** | Streams `SET_POSITION_TARGET_LOCAL_NED` at 50 Hz |
| **Precision landing** | Vision-guided descent via a simple callback API |
| **Body-relative movement** | `goto_body_relative()` without manual NED/heading math |
| **Yaw slew-rate limiting** | Smooth heading transitions (15 °/s default, configurable) |
| **Browser visualization** | Live 3D trajectory + telemetry over HTTP+SSE — no npm, no CDN required |
| **Mock mode** | Built-in physics simulator — test your full mission without SITL or hardware |
| **Thread-safe** | Heartbeat, receiver, and setpoint-streamer threads run in the background |

---

## Installation

```bash
pip install mavpilot
```

Or from source:

```bash
git clone https://github.com/Onikore/mavpilot
cd mavpilot
pip install -e ".[dev]"
```

**Runtime dependency:** [pymavlink](https://pypi.org/project/pymavlink/) (installed automatically).

---

## Quick start — mock mode

No drone or SITL needed:

```bash
# Square flight pattern
python -m mavpilot --mock

# Star/pentagram flight pattern
python -m mavpilot --mock --pattern star

# Precision landing demo
python -m mavpilot --mock --precision-land
```

Open **http://localhost:8765** in a browser to watch the live 3D visualization.

---

## Library usage

```python
import asyncio
from mavpilot import DroneController

async def mission():
    drone = DroneController(
        connection_string="udp:127.0.0.1:14540",  # SITL default
        enable_viz=True,   # browser viz on :8765
    )

    await drone.connect()
    await drone.apply_safe_params()  # recommended PX4 safety params
    await drone.wait_until_ready()   # wait for EKF / LOCAL_POSITION_NED

    await drone.takeoff(altitude_m=5.0)

    # NED coordinates (x=North, y=East, z=Down)
    await drone.goto(x=10, y=0, z=-5)
    await drone.goto(x=10, y=10, z=-5, yaw_deg=90)
    await drone.goto_body_relative(forward_m=5, right_m=0, down_m=0)
    await drone.hover(duration_s=3.0)

    await drone.land()
    drone.close()

asyncio.run(mission())
```

### Precision landing

Supply a callback that returns the landing marker offset in **body FRD frame**:

```python
from mavpilot import DroneController, MarkerObservation

def get_marker() -> MarkerObservation | None:
    # plug in your vision pipeline here
    # dx = forward offset (m), dy = right offset (m)
    return MarkerObservation(dx=0.3, dy=-0.1)

async def mission():
    drone = DroneController(mock=True, enable_viz=False)
    await drone.connect()
    await drone.takeoff(altitude_m=10.0)
    result = await drone.precision_land(
        get_marker_offset=get_marker,
        descent_rate_mps=0.3,
        final_altitude_m=0.5,
        horizontal_tolerance_m=0.15,
        min_altitude_floor_m=0.3,   # new in v0.2.0
    )
    if not result:
        # status ∈ {ABORTED_AT_FLOOR, MARKER_LOST, TIMEOUT}
        print(f"precision_land did not land cleanly: {result.status.value}")
        print(f"final position: {result.final_position}")
    drone.close()
```

### Converting camera pixels to body offset

```python
from mavpilot.utils import pixel_to_body_offset

dx, dy = pixel_to_body_offset(
    px_norm_x=0.1,            # normalized [-1, 1]
    px_norm_y=-0.05,
    camera_hfov_deg=90.0,
    camera_vfov_deg=60.0,
    altitude_above_ground_m=drone.get_local_position().altitude,
    camera_mount_yaw_deg=0.0,
)
```

---

## CLI reference

```
python -m mavpilot [OPTIONS]

Options:
  --connection STR      MAVLink endpoint  [default: udp:127.0.0.1:14540]
  --mock                Hardware-free simulator mode
  --viz-port INT        Browser visualization port  [default: 8765]
  --viz-host STR        Interface the visualization server binds to  [default: 127.0.0.1]
                        Use 0.0.0.0 to expose on LAN (telemetry visible to everyone on the network)
  --no-viz              Disable browser visualization
  --precision-land      Use precision landing with a simulated marker
  --pattern {square,star}  Demo flight pattern  [default: square]
```

### Error handling and Ctrl-C

- **Ctrl-C** at any point during a mission calls `emergency_land()`. This chains: `AUTO_LAND` mode switch, wait up to 10 s for touchdown, send `MAV_CMD_NAV_LAND` if mode switch is stuck, and as a last resort `DO_FLIGHTTERMINATION` (immediate motor cut — drone falls).
- **RTL is not part of `emergency_land()`**. Return-to-launch is a separate nominal operation (`drone.return_to_launch()`), not an emergency procedure.
- Any unhandled exception in the mission body (including `KeyboardInterrupt`) also triggers `emergency_land()`.

### Telemetry watchdog & protocol safety (v0.2.0)

- **Telemetry watchdog** — `telemetry_watchdog_s` (default 2 s). If no fresh `LOCAL_POSITION_NED` arrives within this window, the streamer latches a watchdog flag and the next mission call (`takeoff`/`goto`/`set_yaw`/`land`/`return_to_launch`/`precision_land`) raises `DroneError`. `emergency_land()` deliberately ignores the flag — it is the recovery path the watchdog is meant to trigger.
- **EKF health gate** — `wait_until_ready()` now also validates EKF AHRS health (`SYS_STATUS` bit 5), not just position freshness.
- **`send_command_long()`** — exposes the COMMAND_ACK Future API: it awaits the terminal ACK keyed by `(cmd_id, target_sys, target_comp)`. `IN_PROGRESS` extends the deadline; a duplicate in-flight command, a timeout, or a non-`ACCEPTED` result each raise `DroneError`.
- **`get_yaw_deg()`** is normalized to `[-180, 180]`.

---

## API reference

### `DroneController(…)`

```python
DroneController(
    connection_string = "udp:127.0.0.1:14540",
    source_system     = 255,
    source_component  = MAV_COMP_ID_MISSIONPLANNER,
    loop_hz           = 50.0,       # setpoint publish rate
    enable_viz        = True,       # start browser viz server
    viz_port          = 8765,
    mock              = False,      # no-hardware simulator
    yaw_slew_rate_deg = 15.0,       # max yaw rate (deg/s)
)
```

### Flight methods

| Method | Description |
|---|---|
| `await connect(timeout_s)` | Open MAVLink and start background threads |
| `await apply_safe_params()` | Write recommended PX4 safety params |
| `await wait_until_ready(timeout_s)` | Block until EKF reports LOCAL_POSITION_NED |
| `await takeoff(altitude_m, timeout_s)` | Arm, enter OFFBOARD, climb |
| `await goto(x, y, z, yaw_deg, …)` | Fly to NED position |
| `await goto_relative(dx, dy, dz, …)` | NED offset from current position |
| `await goto_body_relative(fwd, right, down, …)` | Body FRD offset |
| `await set_yaw(yaw_deg, timeout_s)` | Rotate in-place |
| `await hover(duration_s)` | Hold position |
| `await land(timeout_s)` | Switch to AUTO_LAND, wait until on ground |
| `await precision_land(callback, …)` | Vision-guided descent; returns `PrecisionLandResult` |
| `await return_to_launch(timeout_s)` | Switch to AUTO_RTL, wait until landed |
| `await emergency_land()` | Chain: AUTO_LAND → NAV_LAND → DO_FLIGHTTERMINATION |
| `close()` | Stop all threads and close connection |

### Telemetry

| Method | Returns |
|---|---|
| `get_local_position()` | `Position(x, y, z)` in NED meters |
| `get_yaw_rad()` / `get_yaw_deg()` | Current heading |
| `is_armed()` | `bool` |
| `is_offboard()` | `bool` |
| `landed_state()` | `int` (1 = on ground, 2 = in air) |

### Data classes

```python
from mavpilot import Position, MarkerObservation

# NED position (x=North, y=East, z=Down)
pos: Position       # pos.altitude == -pos.z

# Marker offset in body FRD frame
obs: MarkerObservation  # dx=forward, dy=right, dz=down (optional)
```

---

## Coordinate system

mavpilot uses the **PX4 NED convention** from `LOCAL_POSITION_NED`:

| Axis | Direction | Note |
|---|---|---|
| x | North (+) | |
| y | East (+) | |
| z | Down (+) | altitude = `-z` |

Coordinate transform utilities:

```python
from mavpilot.utils import body_to_ned, ned_to_body, pixel_to_body_offset
```

---

## Visualization

A lightweight self-contained HTTP+SSE server serves a **Three.js 3D view** with no build step and no external package manager. Open `http://localhost:8765` while the drone is running.

The right-hand panel displays:
- Armed status and flight mode
- Live position, velocity, heading, battery
- Active setpoint
- Command log (takeoff, goto, land, …)
- PX4 STATUSTEXT messages

The UI is composed of native ES modules served from `mavpilot/viz/static/` (`index.html` + `styles.css` + `main.js`/`scene.js`/`sse.js`/`telemetry.js`/`log.js`) — no bundler, but a **modern browser with ES-module support** is required. The `max_clients` parameter (default 32) caps concurrent SSE connections; excess clients receive HTTP 503.

---

## Architecture

```
asyncio event loop  <-- your mission code
        |
        v
 DroneController
        |
        +-- heartbeat_thread   (1 Hz MAVLink heartbeat)
        +-- receiver_thread    (parses incoming MAVLink → self._tel)
        +-- streamer_thread    (publishes SET_POSITION_TARGET_LOCAL_NED @ 50 Hz)
        +-- viz_server         (optional HTTP+SSE → browser)
```

All shared state is protected by `_tel_lock` and `_setpoint_lock`. No asyncio primitives are needed in user mission code — the asyncio loop and background threads only touch shared dicts through these locks.

### Module layout (v0.2.0)

```
mavpilot/
├── controller.py          # DroneController facade (composition root)
├── _connection.py         # MAVLinkConnection — pymavlink + I/O lock + heartbeat/receiver
├── _telemetry.py          # Telemetry — incoming-message parsing + state cache
├── _commands.py           # CommandSender — COMMAND_LONG with asyncio.Future ACK routing
├── _streamer.py           # OffboardStreamer — setpoint thread + telemetry watchdog
├── _mission.py            # MissionOps — takeoff/goto/hover/land/rtl/emergency_land
├── _precision_land.py     # PrecisionLand — vision descent with altitude floor
├── _safety.py             # SafetyOps — wait_until_ready
├── _mock.py               # MockMavConnection + in-process simulator
├── errors.py              # DroneError
├── types.py               # Position, MarkerObservation, PrecisionLand{Status,Result}
├── utils.py               # coordinate transforms, pinhole, yaw normalization
├── constants.py           # PX4 mode bits, MAV_CMD ids, type_masks
├── cli.py                 # argparse entrypoint
└── viz.py                 # browser UI server (HTTP + SSE)
```

Every MAVLink send and recv goes through `MAVLinkConnection`, which holds the single threading lock. Each subsystem receives its dependencies via constructor injection — easy to mock in tests.

---

## Connecting to real hardware

```python
# Serial (Raspberry Pi <-> Pixhawk via UART)
drone = DroneController(connection_string="/dev/ttyAMA0")

# UDP (SITL or companion computer bridge)
drone = DroneController(connection_string="udp:192.168.1.10:14540")

# TCP
drone = DroneController(connection_string="tcp:127.0.0.1:5760")
```

**Recommended safety parameters** (set by `apply_safe_params()` defaults):

| Parameter | Value | Purpose |
|---|---|---|
| `COM_RCL_EXCEPT` | 7 | No failsafe in offboard / mission / hold |
| `COM_OBL_RC_ACT` | 4 | RC loss → hold, not RTL |
| `COM_OF_LOSS_T` | 2.0 s | Offboard loss timeout |
| `COM_RC_IN_MODE` | 1 | RC stick input not required |

---

## Development

```bash
# Install in editable mode with dev extras
pip install -e ".[dev]"

# Run tests
pytest -q

# Lint
ruff check mavpilot/

# Type check
mypy mavpilot/
```

---

## License

[MIT](LICENSE)
