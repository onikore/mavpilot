"""ArUco fractal marker detector integration via MarkerSource Protocol."""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

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
