"""ArUco fractal marker detector integration via MarkerSource Protocol."""

from __future__ import annotations

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
