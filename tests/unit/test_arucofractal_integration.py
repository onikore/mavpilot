"""ArUco fractal marker detector integration via MarkerSource Protocol."""

from __future__ import annotations

import math
import sys
from unittest.mock import MagicMock

import numpy as np
import pytest

from mavpilot.integrations import MarkerSource
from mavpilot.types import MarkerObservation


def test_marker_source_protocol_satisfied_by_custom_class():
    class _CustomSource:
        def marker_callback(self) -> MarkerObservation | None:
            return None

    assert isinstance(_CustomSource(), MarkerSource)


def test_marker_source_protocol_not_satisfied_without_method():
    class _BadSource:
        pass

    assert not isinstance(_BadSource(), MarkerSource)


def test_import_error_when_arucofractal_missing(monkeypatch):
    monkeypatch.setitem(sys.modules, "arucofractal", None)  # simulate missing package
    # Force re-import of the integration module with arucofractal absent
    monkeypatch.delitem(sys.modules, "mavpilot.integrations.arucofractal", raising=False)

    with pytest.raises(ImportError, match="pip install arucofractal"):
        from mavpilot.integrations.arucofractal import ArucoFractalSource  # noqa: F401
        ArucoFractalSource(MagicMock())


# Import ArucoFractalSource for None-case tests
from mavpilot.integrations.arucofractal import ArucoFractalSource  # noqa: E402


def _make_source(state) -> ArucoFractalSource:
    """Build an ArucoFractalSource bypassing __init__, injecting a mock detector."""
    src = ArucoFractalSource.__new__(ArucoFractalSource)
    src._camera_yaw_deg = 0.0
    mock_det = MagicMock()
    mock_det.state = state
    src._detector = mock_det
    src._stream = MagicMock()
    return src


def test_marker_callback_returns_none_when_detector_not_started():
    src = ArucoFractalSource.__new__(ArucoFractalSource)
    src._camera_yaw_deg = 0.0
    src._detector = None
    assert src.marker_callback() is None


def test_marker_callback_returns_none_when_state_is_none():
    src = _make_source(state=None)
    assert src.marker_callback() is None


def test_marker_callback_returns_none_when_not_detected():
    state = MagicMock()
    state.detected = False
    src = _make_source(state=state)
    assert src.marker_callback() is None


def test_marker_callback_returns_none_when_no_pose():
    state = MagicMock()
    state.detected = True
    state.has_pose = False
    src = _make_source(state=state)
    assert src.marker_callback() is None


def _make_source_with_tvec(tvec_values, yaw_deg: float = 0.0) -> ArucoFractalSource:
    """Build ArucoFractalSource with a fully-detected state and given tvec."""
    state = MagicMock()
    state.detected = True
    state.has_pose = True
    state.tvec = np.array(tvec_values, dtype=float).reshape(3, 1)

    src = ArucoFractalSource.__new__(ArucoFractalSource)
    src._camera_yaw_deg = yaw_deg
    mock_det = MagicMock()
    mock_det.state = state
    src._detector = mock_det
    src._stream = MagicMock()
    return src


def test_marker_callback_maps_tvec_to_marker_observation():
    # tvec = [tvec[0]=0.1, tvec[1]=-0.3, tvec[2]=1.5]
    # dx = -(-0.3) = 0.3  (negate camera Y → body forward)
    # dy = 0.1            (camera X → body right)
    # dz = 1.5
    src = _make_source_with_tvec([0.1, -0.3, 1.5])
    obs = src.marker_callback()
    assert obs is not None
    assert math.isclose(obs.dx, 0.3, abs_tol=1e-6)
    assert math.isclose(obs.dy, 0.1, abs_tol=1e-6)
    assert math.isclose(obs.dz, 1.5, abs_tol=1e-6)


def test_marker_callback_camera_yaw_90deg():
    # tvec = [0.0, -1.0, 2.0]
    # base (yaw=0): dx = -(-1.0) = 1.0, dy = 0.0
    # after 90° rotation: theta=π/2, cos≈0, sin=1
    #   dx_rot = 1.0*0 - 0.0*1 = 0.0
    #   dy_rot = 1.0*1 + 0.0*0 = 1.0
    src = _make_source_with_tvec([0.0, -1.0, 2.0], yaw_deg=90.0)
    obs = src.marker_callback()
    assert obs is not None
    assert math.isclose(obs.dx, 0.0, abs_tol=1e-6)
    assert math.isclose(obs.dy, 1.0, abs_tol=1e-6)
    assert math.isclose(obs.dz, 2.0, abs_tol=1e-6)


def test_marker_callback_zero_yaw_is_identity():
    src_no_yaw = _make_source_with_tvec([0.2, -0.4, 3.0], yaw_deg=0.0)
    src_zero_yaw = _make_source_with_tvec([0.2, -0.4, 3.0], yaw_deg=0.0)
    obs1 = src_no_yaw.marker_callback()
    obs2 = src_zero_yaw.marker_callback()
    assert obs1 is not None and obs2 is not None
    assert math.isclose(obs1.dx, obs2.dx, abs_tol=1e-9)
    assert math.isclose(obs1.dy, obs2.dy, abs_tol=1e-9)


@pytest.mark.asyncio
async def test_aenter_aexit_creates_and_stops_stream_and_detector(monkeypatch):
    mock_stream = MagicMock()
    mock_detector = MagicMock()
    mock_detector.state = None

    mock_af = MagicMock()
    mock_af.StreamReader.return_value = mock_stream
    mock_af.DetectionThread.return_value = mock_detector

    # Patch arucofractal in sys.modules so the lazy import inside __init__ succeeds
    monkeypatch.setitem(sys.modules, "arucofractal", mock_af)
    monkeypatch.delitem(sys.modules, "mavpilot.integrations.arucofractal", raising=False)

    from mavpilot.integrations.arucofractal import ArucoFractalSource

    config = MagicMock()
    src = ArucoFractalSource(config, camera_yaw_deg=0.0)

    async with src:
        assert src._stream is mock_stream
        assert src._detector is mock_detector

    mock_detector.stop.assert_called_once()
    mock_stream.stop.assert_called_once()
