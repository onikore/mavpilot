"""Пример кастомной реализации MarkerSource.

Показывает, как подключить любой детектор маркеров к precision_land()
без использования arucofractal — достаточно класса с методом marker_callback().

Запуск:
    python examples/04_custom_marker_source.py --mock
"""

import asyncio
import argparse
import logging
import threading
import time

import cv2
import numpy as np

from mavpilot import DroneController
from mavpilot.types import MarkerObservation

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

MARKER_SIZE = 0.17  # метры


class OpenCVArucoSource:
    """Пример: обычный ArUco (не фрактал) через стандартный OpenCV.

    Удовлетворяет MarkerSource Protocol автоматически — нет наследования,
    достаточно метода marker_callback() с правильной сигнатурой.
    """

    def __init__(
        self,
        camera_index: int = 0,
        marker_size: float = MARKER_SIZE,
        camera_fx: float = 581.0,
        camera_fy: float = 582.0,
        camera_cx: float = 320.0,
        camera_cy: float = 240.0,
    ) -> None:
        self._marker_size = marker_size
        self._camera_matrix = np.array(
            [[camera_fx, 0, camera_cx], [0, camera_fy, camera_cy], [0, 0, 1]],
            dtype=np.float32,
        )
        self._dist = np.zeros((5, 1), dtype=np.float32)
        self._cap = cv2.VideoCapture(camera_index)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self._obs: MarkerObservation | None = None
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        s = self._marker_size / 2
        obj_pts = np.array(
            [[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32
        )
        while self._running:
            ret, frame = self._cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detector.detectMarkers(gray)

            obs = None
            if ids is not None and len(corners) > 0:
                img_pts = corners[0][0].astype(np.float32)
                ok, _, tvec = cv2.solvePnP(
                    obj_pts, img_pts, self._camera_matrix, self._dist
                )
                if ok:
                    t = tvec.flatten()
                    # Камера смотрит вниз, верх кадра к носу дрона:
                    # tvec[0] → dy (вправо), -tvec[1] → dx (вперёд), tvec[2] → dz
                    obs = MarkerObservation(dx=-float(t[1]), dy=float(t[0]), dz=float(t[2]))

            with self._lock:
                self._obs = obs

    def marker_callback(self) -> MarkerObservation | None:
        with self._lock:
            return self._obs

    def stop(self) -> None:
        self._running = False
        self._cap.release()


async def mission(connection: str, mock: bool) -> None:
    src = OpenCVArucoSource(camera_index=0)
    try:
        async with DroneController(connection_string=connection, mock=mock) as drone:
            await drone.connect(timeout_s=30.0)
            await drone.apply_safe_params()
            await drone.wait_until_ready(timeout_s=60.0)

            await drone.takeoff(altitude_m=5.0, timeout_s=30.0)

            result = await drone.precision_land(
                get_marker_offset=src.marker_callback,
                descent_rate_mps=0.3,
                final_altitude_m=0.5,
                timeout_s=120.0,
            )

            logging.getLogger("drone").info(f"Результат: {result.status.value}")

            if not result:
                await drone.land(timeout_s=30.0)
    finally:
        src.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Точная посадка с кастомным детектором")
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    asyncio.run(mission(args.connection, args.mock))


if __name__ == "__main__":
    main()
