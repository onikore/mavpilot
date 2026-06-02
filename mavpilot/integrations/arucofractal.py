from __future__ import annotations

import math

from mavpilot.types import MarkerObservation


class ArucoFractalSource:
    """Async context manager that drives precision_land() via arucofractal.

    Owns the StreamReader + DetectionThread lifecycle. Pass ``marker_callback``
    directly to ``DroneController.precision_land()``.

    Coordinate convention (camera_yaw_deg=0):
      - Camera mounted straight down, image-top toward drone nose.
      - tvec[0] (image right)  → body Right  (+Y FRD) → MarkerObservation.dy
      - tvec[1] (image down)   → body Back   (-X FRD) → MarkerObservation.dx = -tvec[1]
      - tvec[2] (depth)        → altitude             → MarkerObservation.dz

    If marker compensation goes in the wrong direction, try camera_yaw_deg=180.
    """

    def __init__(self, config, camera_yaw_deg: float = 0.0) -> None:
        try:
            import arucofractal as _af  # noqa: F401
        except ImportError as exc:
            raise ImportError(
                "Install arucofractal to use ArucoFractalSource "
                "(pip install arucofractal)"
            ) from exc
        self._config = config
        self._camera_yaw_deg = camera_yaw_deg
        self._stream = None
        self._detector = None

    async def __aenter__(self) -> ArucoFractalSource:
        import arucofractal as _af

        self._stream = _af.StreamReader(self._config)
        self._detector = _af.DetectionThread(self._stream, self._config)
        return self

    async def __aexit__(self, *_: object) -> None:
        if self._detector is not None:
            self._detector.stop()
        if self._stream is not None:
            self._stream.stop()

    def marker_callback(self) -> MarkerObservation | None:
        """Return current marker offset in body FRD, or None if not visible."""
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
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            dx, dy = dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t

        return MarkerObservation(dx=dx, dy=dy, dz=dz)
