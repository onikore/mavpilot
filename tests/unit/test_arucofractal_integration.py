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
