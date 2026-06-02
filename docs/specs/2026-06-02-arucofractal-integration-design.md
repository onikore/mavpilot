# Design: arucofractal integration for precision landing

**Date:** 2026-06-02  
**Status:** Approved

## Goal

Integrate `arucofractal` (fractal ArUco marker detector) into `mavpilot` to provide a real camera-based `get_marker_offset` callback for `DroneController.precision_land()`.

## Scope

- Precision landing only (not hover-over-marker positioning).
- Camera orientation: straight down, camera top toward drone nose (configurable via `camera_yaw_deg`).
- Integration lives in `mavpilot/integrations/arucofractal.py` as an optional module.
- `arucofractal` is a soft dependency — not required for the rest of mavpilot.

## Architecture

### New files

```
mavpilot/
  integrations/
    __init__.py          (exports MarkerSource protocol)
    arucofractal.py      (ArucoFractalSource — default implementation)
tests/
  unit/
    test_arucofractal_integration.py
```

### Protocol: `MarkerSource`

Defined in `mavpilot/integrations/__init__.py`. Any object that implements this interface can be used as a marker source for `precision_land()` — either the built-in `ArucoFractalSource` or a fully custom implementation.

```python
# mavpilot/integrations/__init__.py
from typing import Protocol
from mavpilot.types import MarkerObservation

class MarkerSource(Protocol):
    def marker_callback(self) -> MarkerObservation | None: ...
```

`MarkerSource` is a structural `Protocol` — no inheritance required. Any class with a `marker_callback()` method of the right signature satisfies it automatically.

### Custom implementation example

```python
class MyOpenCVSource:
    """Custom marker source using plain OpenCV ArUco (not fractal)."""

    def __init__(self, camera_index: int = 0): ...
    async def __aenter__(self): ...
    async def __aexit__(self, *_): ...

    def marker_callback(self) -> MarkerObservation | None:
        # your detection logic here
        ...

async with MyOpenCVSource() as src:
    result = await drone.precision_land(src.marker_callback)
```

The `__aenter__`/`__aexit__` are recommended for lifecycle management but not enforced by the Protocol.

### Class: `ArucoFractalSource`

Default implementation. Async context manager that owns the `StreamReader` + `DetectionThread` lifecycle and exposes `marker_callback` for `precision_land()`.

```python
class ArucoFractalSource:
    def __init__(self, config: arucofractal.Config, camera_yaw_deg: float = 0.0): ...
    async def __aenter__(self) -> ArucoFractalSource: ...
    async def __aexit__(self, *_): ...
    def marker_callback(self) -> MarkerObservation | None: ...
```

`arucofractal` is imported lazily inside `__init__` so the rest of mavpilot is unaffected if the package is absent.

## Usage

### Default: ArucoFractalSource

```python
from arucofractal import Config as ArucoConfig
from mavpilot import DroneController
from mavpilot.integrations.arucofractal import ArucoFractalSource

aruco_cfg = ArucoConfig(
    source="rtsp",
    rtsp_url="rtsp://192.168.0.3:8080/h264_ulaw.sdp",
    marker_size=0.17,
)

async with ArucoFractalSource(aruco_cfg, camera_yaw_deg=0.0) as src:
    async with DroneController(connection_string="udp:127.0.0.1:14550") as drone:
        await drone.wait_until_ready()
        await drone.takeoff(altitude_m=5.0)
        result = await drone.precision_land(src.marker_callback)
        print(result.status)
```

### Custom implementation

```python
from mavpilot import DroneController
from mavpilot.types import MarkerObservation

class MyCustomSource:
    def marker_callback(self) -> MarkerObservation | None:
        # your detection logic — return None if marker not visible
        ...

src = MyCustomSource()
async with DroneController(...) as drone:
    result = await drone.precision_land(src.marker_callback)
```

## Data flow

```
[Camera] → StreamReader (thread)
               ↓ frame
         DetectionThread (thread) → DetectionResult {has_pose, tvec, rvec}
               ↓ .state  (thread-safe, lock inside DetectionThread)
         ArucoFractalSource.marker_callback()
               ↓ tvec → rotate(camera_yaw_deg) → MarkerObservation(dx, dy, dz)
         PrecisionLand._loop (asyncio event loop)
               ↓ MarkerObservation
         DroneController._set_setpoint_position(...)
```

## Coordinate transform

ArUco `tvec` is in the camera frame (X=right, Y=down-in-image, Z=depth).  
For a downward-facing camera with image-top toward drone nose (`camera_yaw_deg=0`):

| Camera frame | Body FRD frame | `MarkerObservation` field |
|---|---|---|
| `tvec[0]` (cam X / image right) | body Right (+Y) | `dy` |
| `tvec[1]` (cam Y / image down = ahead on ground) | body Forward (+X) | `dx` |
| `tvec[2]` (depth ≈ altitude) | body Down (+Z) | `dz` |

For non-zero `camera_yaw_deg`, a 2D rotation is applied to `(dx, dy)`:

```python
θ = radians(camera_yaw_deg)
dx_rot =  dx * cos(θ) - dy * sin(θ)
dy_rot =  dx * sin(θ) + dy * cos(θ)
```

## `marker_callback` return logic

| Condition | Returns |
|---|---|
| `state is None` (no frame yet) | `None` |
| `state.detected is False` | `None` |
| `state.has_pose is False` | `None` |
| All good | `MarkerObservation(dx, dy, dz)` |

Only a full pose (`has_pose=True`) produces a metric observation — without pose, only pixel corners are known, which cannot be converted to metric offsets.

## Error handling

- If `arucofractal` is not installed: `ImportError` with message `"Install arucofractal to use ArucoFractalSource (pip install arucofractal)"` raised in `__init__`.
- `__aexit__` calls `stream.stop()` and `detection.stop()` (both already implemented in arucofractal). Always called even if the mission raises.

## Dependencies

- `arucofractal` added to `pyproject.toml` as an optional extra: `mavpilot[aruco]`.
- No changes to mavpilot's required dependencies.

## Tests

File: `tests/unit/test_arucofractal_integration.py`

All tests use a mock `DetectionThread` with a `.state` attribute — no real camera or aruco binary needed.

1. `test_marker_callback_no_state` — returns `None` when `state=None`
2. `test_marker_callback_not_detected` — returns `None` when `detected=False`
3. `test_marker_callback_no_pose` — returns `None` when `has_pose=False`
4. `test_marker_callback_maps_tvec` — `tvec=[0.1, 0.2, 1.5]` → `MarkerObservation(dx=0.2, dy=0.1, dz=1.5)`
5. `test_camera_yaw_90deg` — 90° rotation produces correct rotated result
6. `test_aenter_aexit_stops_threads` — verifies `stop()` is called on both stream and detector
7. `test_marker_source_protocol` — verifies `MyCustomSource` satisfies `MarkerSource` Protocol via `isinstance` check
