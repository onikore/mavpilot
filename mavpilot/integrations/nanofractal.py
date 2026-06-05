"""nanofractal integration for precision landing.

Wraps the stateless `nanofractal` fractal-marker detector into a stream-driven
background worker that produces body-FRD :class:`MarkerObservation` offsets for
:meth:`mavpilot.DroneController.precision_land`.

``nanofractal`` and ``cv2`` are soft dependencies, imported lazily so the rest
of mavpilot is unaffected when they're absent. Install with
``pip install nanofractal`` (the wheel bundles OpenCV).

Pipeline per frame::

    frame = stream.read()                          # any object with read()
    res   = detector.detect(frame, with_inner_points=True)
    pose  = detector.estimate_pose(res, K, dist)   # (rvec, tvec, reproj_err) | None
    obs   = tvec_to_observation(pose[1], camera_yaw_deg)
"""

from __future__ import annotations

import logging
import math
import threading
import time

from mavpilot.types import MarkerObservation

log = logging.getLogger("drone")


def tvec_to_observation(tvec, camera_yaw_deg: float = 0.0) -> MarkerObservation | None:
    """Convert a camera-frame translation vector to a body-FRD observation.

    ``tvec`` is the solvePnP translation (camera frame) from
    ``FractalDetector.estimate_pose``; pass ``None`` when no pose was found.

    Convention (``camera_yaw_deg=0``): camera mounted straight down, image-top
    toward the drone nose.

    - ``tvec[0]`` (camera X / image right) → body Right (+Y FRD) → ``dy``
    - ``tvec[1]`` (camera Y / image down)  → body Back  (-X FRD) → ``dx = -tvec[1]``
    - ``tvec[2]`` (depth)                  → altitude            → ``dz``

    If lateral compensation goes the wrong way, try ``camera_yaw_deg=180``.
    """
    if tvec is None:
        return None
    import numpy as np  # type: ignore[import]

    t = np.asarray(tvec, dtype=float).reshape(-1)
    dx = -float(t[1])
    dy = float(t[0])
    dz = float(t[2])
    if camera_yaw_deg:
        theta = math.radians(camera_yaw_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        dx, dy = dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t
    return MarkerObservation(dx=dx, dy=dy, dz=dz)


def _make_fractal_detector(config: str, marker_size: float):
    try:
        import nanofractal as nf  # type: ignore[import]
    except ImportError as exc:
        raise ImportError(
            "Install nanofractal to use the fractal detector (pip install nanofractal)"
        ) from exc
    return nf.FractalDetector(config, marker_size)


class FractalDetectorWorker:
    """Background worker: stream frames → nanofractal detect → pose → observation.

    ``stream`` is any object with ``read() -> np.ndarray | None`` (e.g. the
    Gazebo :class:`~mavpilot.integrations.gazebo.ROS2ImageStream` or
    :class:`VideoCaptureStream`). Pass camera intrinsics for metric pose. Call
    :meth:`start` to spin the thread; :meth:`marker_callback` plugs into
    ``precision_land``.

    A ``detector`` may be injected (mainly for tests); otherwise a
    ``nanofractal.FractalDetector(config, marker_size)`` is created lazily.
    """

    def __init__(
        self,
        stream,
        camera_matrix,
        dist_coeffs,
        *,
        config: str = "FRACTAL_5L_6",
        marker_size: float = 0.17,
        camera_yaw_deg: float = 0.0,
        max_reproj_err_px: float | None = None,
        detector=None,
    ) -> None:
        import numpy as np  # type: ignore[import]

        self._stream = stream
        self._K = np.ascontiguousarray(camera_matrix, dtype=np.float64)
        self._dist = np.ascontiguousarray(dist_coeffs, dtype=np.float64)
        self._camera_yaw_deg = camera_yaw_deg
        self._max_err = max_reproj_err_px
        self._detector = (
            detector if detector is not None else _make_fractal_detector(config, marker_size)
        )

        self._lock = threading.Lock()
        self._result = None
        self._pose = None  # (rvec, tvec, reproj_err) | None
        self._fps = 0.0
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="nanofractal")
        self._thread.start()

    def _loop(self) -> None:
        last = time.time()
        while self._running:
            frame = self._stream.read()
            if frame is None:
                time.sleep(0.005)
                continue
            self._process_frame(frame)
            now = time.time()
            self._fps = 0.9 * self._fps + 0.1 / max(now - last, 1e-6)
            last = now

    def _process_frame(self, frame) -> None:
        """Detect + estimate pose for one frame and latch the result."""
        res = self._detector.detect(frame, with_inner_points=True)
        pose = self._detector.estimate_pose(res, self._K, self._dist)
        if pose is not None and self._max_err is not None and pose[2] > self._max_err:
            pose = None  # reject noisy pose
        with self._lock:
            self._result = res
            self._pose = pose

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def last_pose(self):
        """Latest ``(rvec, tvec, reproj_err)`` or ``None``."""
        with self._lock:
            return self._pose

    def marker_callback(self) -> MarkerObservation | None:
        """Return current marker offset in body FRD, or None if no pose."""
        with self._lock:
            pose = self._pose
        return tvec_to_observation(pose[1], self._camera_yaw_deg) if pose else None

    def draw_overlays(self, frame) -> None:
        """Draw marker outlines (+ axes when a pose is available) onto ``frame``."""
        with self._lock:
            res, pose = self._result, self._pose
        if res is None:
            return
        if pose is not None:
            self._detector.draw(frame, res, self._K, self._dist, pose[0], pose[1])
        else:
            self._detector.draw(frame, res)

    def stop(self) -> None:
        self._running = False


class VideoCaptureStream:
    """USB/RTSP frame source via OpenCV with a background reader + reconnect.

    ``source`` is a camera index (``int``) or an RTSP/file URL (``str``).
    Exposes ``read() -> np.ndarray | None`` like the Gazebo image stream.
    """

    def __init__(self, source) -> None:
        import cv2  # type: ignore[import]

        self._cv2 = cv2
        self._source = source
        self._frame = None
        self._lock = threading.Lock()
        self._running = True
        self._open()
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self) -> None:
        if isinstance(self._source, int):
            self._cap = self._cv2.VideoCapture(self._source)
        else:
            self._cap = self._cv2.VideoCapture(self._source, self._cv2.CAP_FFMPEG)
            self._cap.set(self._cv2.CAP_PROP_BUFFERSIZE, 1)

    def _loop(self) -> None:
        fail = 0
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None or frame.size == 0:
                fail += 1
                if fail > 10:
                    log.warning("video reconnecting...")
                    self._cap.release()
                    time.sleep(2.0)
                    self._open()
                    fail = 0
            else:
                with self._lock:
                    self._frame = frame
                fail = 0

    def read(self):
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._running = False
        self._cap.release()


class NanoFractalSource:
    """Async context manager: USB/RTSP camera → nanofractal fractal detection.

    Replaces the old ``ArucoFractalSource``. Owns a :class:`VideoCaptureStream`
    plus a :class:`FractalDetectorWorker`; pass :meth:`marker_callback` straight
    to :meth:`mavpilot.DroneController.precision_land`.

    Provide ``camera_matrix``/``dist_coeffs`` for accurate metric pose. If
    ``camera_matrix`` is omitted, a rough pinhole guess (focal ≈ frame width) is
    built from the first frame — pose scale will be approximate.

    Example::

        async with NanoFractalSource(source=0, marker_size=0.17) as src:
            async with DroneController(connection_string=conn) as drone:
                await drone.connect()
                await drone.wait_until_ready()
                await drone.precision_land(src.marker_callback)
    """

    def __init__(
        self,
        source=0,
        *,
        camera_matrix=None,
        dist_coeffs=None,
        config: str = "FRACTAL_5L_6",
        marker_size: float = 0.17,
        camera_yaw_deg: float = 0.0,
        max_reproj_err_px: float | None = None,
    ) -> None:
        self._source = source
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        self._config = config
        self._marker_size = marker_size
        self._camera_yaw_deg = camera_yaw_deg
        self._max_err = max_reproj_err_px

        self._stream: VideoCaptureStream | None = None
        self._worker: FractalDetectorWorker | None = None

    async def __aenter__(self) -> NanoFractalSource:
        import asyncio

        import numpy as np  # type: ignore[import]

        self._stream = VideoCaptureStream(self._source)

        K = self._camera_matrix
        if K is None:
            frame = await asyncio.to_thread(self._wait_first_frame)
            h, w = frame.shape[:2]
            f = float(w)
            K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=np.float64)
            log.warning(f"no camera_matrix given — using pinhole guess f={f:.0f}")
        dist = self._dist_coeffs if self._dist_coeffs is not None else np.zeros(5)

        self._worker = FractalDetectorWorker(
            self._stream,
            K,
            dist,
            config=self._config,
            marker_size=self._marker_size,
            camera_yaw_deg=self._camera_yaw_deg,
            max_reproj_err_px=self._max_err,
        )
        self._worker.start()
        return self

    async def __aexit__(self, *_: object) -> None:
        if self._worker:
            self._worker.stop()
        if self._stream:
            self._stream.stop()

    def _wait_first_frame(self, timeout_s: float = 10.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            frame = self._stream.read()
            if frame is not None:
                return frame
            time.sleep(0.05)
        raise TimeoutError(f"no frame from camera within {timeout_s}s")

    def marker_callback(self) -> MarkerObservation | None:
        """Return current marker offset in body FRD, or None if not visible."""
        return self._worker.marker_callback() if self._worker else None
