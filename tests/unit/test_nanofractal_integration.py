"""Unit tests for mavpilot.integrations.nanofractal.

These exercise the coordinate transform and the worker's detect→pose→observation
mapping with a mock nanofractal detector — no real nanofractal/cv2 needed.
"""

import math
import sys
from unittest.mock import MagicMock

import numpy as np
import pytest

from mavpilot.integrations.nanofractal import (
    FractalDetectorWorker,
    tvec_to_observation,
)
from mavpilot.types import MarkerObservation

# ---------------------------------------------------------------------------
# tvec_to_observation — pure transform
# ---------------------------------------------------------------------------


def test_tvec_to_observation_none():
    assert tvec_to_observation(None) is None


def test_tvec_to_observation_maps_axes():
    # tvec = [x=0.1, y=-0.3, z=1.5] → dx=-(-0.3)=0.3, dy=0.1, dz=1.5
    obs = tvec_to_observation([0.1, -0.3, 1.5])
    assert obs is not None
    assert math.isclose(obs.dx, 0.3, abs_tol=1e-6)
    assert math.isclose(obs.dy, 0.1, abs_tol=1e-6)
    assert math.isclose(obs.dz, 1.5, abs_tol=1e-6)


def test_tvec_to_observation_accepts_numpy_column():
    obs = tvec_to_observation(np.array([[0.1], [-0.3], [1.5]]))
    assert obs is not None
    assert math.isclose(obs.dx, 0.3, abs_tol=1e-6)
    assert math.isclose(obs.dy, 0.1, abs_tol=1e-6)


def test_tvec_to_observation_yaw_90deg():
    # base (yaw=0): tvec=[0,-1,2] → dx=1.0, dy=0.0
    # 90°: dx_rot=0, dy_rot=1
    obs = tvec_to_observation([0.0, -1.0, 2.0], camera_yaw_deg=90.0)
    assert obs is not None
    assert math.isclose(obs.dx, 0.0, abs_tol=1e-6)
    assert math.isclose(obs.dy, 1.0, abs_tol=1e-6)
    assert math.isclose(obs.dz, 2.0, abs_tol=1e-6)


# ---------------------------------------------------------------------------
# FractalDetectorWorker — detect → estimate_pose → latch
# ---------------------------------------------------------------------------


def _worker(detector, *, camera_yaw_deg=0.0, max_reproj_err_px=None) -> FractalDetectorWorker:
    """Build a worker with an injected mock detector (no thread started)."""
    K = np.eye(3, dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)
    return FractalDetectorWorker(
        MagicMock(),  # stream (unused — we call _process_frame directly)
        K,
        dist,
        camera_yaw_deg=camera_yaw_deg,
        max_reproj_err_px=max_reproj_err_px,
        detector=detector,
    )


def test_worker_marker_callback_none_before_any_frame():
    w = _worker(MagicMock())
    assert w.marker_callback() is None
    assert w.last_pose is None


def test_worker_process_frame_latches_pose():
    detector = MagicMock()
    detector.detect.return_value = "RESULT"
    # estimate_pose → (rvec, tvec, reproj_err)
    detector.estimate_pose.return_value = (
        np.zeros(3),
        np.array([0.1, -0.3, 1.5]),
        0.2,
    )
    w = _worker(detector)
    w._process_frame(MagicMock())

    detector.detect.assert_called_once()
    obs = w.marker_callback()
    assert isinstance(obs, MarkerObservation)
    assert math.isclose(obs.dx, 0.3, abs_tol=1e-6)
    assert math.isclose(obs.dy, 0.1, abs_tol=1e-6)
    assert math.isclose(obs.dz, 1.5, abs_tol=1e-6)


def test_worker_process_frame_no_pose_returns_none():
    detector = MagicMock()
    detector.detect.return_value = "RESULT"
    detector.estimate_pose.return_value = None
    w = _worker(detector)
    w._process_frame(MagicMock())
    assert w.marker_callback() is None


def test_worker_rejects_high_reproj_error():
    detector = MagicMock()
    detector.detect.return_value = "RESULT"
    detector.estimate_pose.return_value = (np.zeros(3), np.array([0.0, 0.0, 1.0]), 9.0)
    w = _worker(detector, max_reproj_err_px=2.0)
    w._process_frame(MagicMock())
    # reproj_err 9.0 > 2.0 → pose rejected
    assert w.marker_callback() is None
    assert w.last_pose is None


def test_worker_draw_overlays_with_and_without_pose():
    detector = MagicMock()
    detector.detect.return_value = "RESULT"
    detector.estimate_pose.return_value = (np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.1)
    w = _worker(detector)
    w._process_frame(MagicMock())
    frame = MagicMock()
    w.draw_overlays(frame)
    # pose present → draw called with camera matrix + rvec/tvec (7 args incl. frame/result)
    assert detector.draw.call_args.args[0] is frame


def test_worker_import_error_when_nanofractal_missing(monkeypatch):
    monkeypatch.setitem(sys.modules, "nanofractal", None)
    with pytest.raises(ImportError, match="pip install nanofractal"):
        FractalDetectorWorker(MagicMock(), np.eye(3), np.zeros(5))
