"""Structural protocols for marker detection sources and other integrations.

This module defines duck-typed protocols that enable third-party integrations
to be plugged into mavpilot without inheritance or boilerplate registration.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from mavpilot.types import MarkerObservation


@runtime_checkable
class MarkerSource(Protocol):
    """Structural protocol for any marker detector source.

    Any class with a ``marker_callback()`` of the right signature satisfies
    this protocol without inheritance — plain duck typing.

    This allows third-party marker detectors (e.g. arucofractal) to be used
    with :meth:`mavpilot.DroneController.precision_land` without needing to
    know about or inherit from this protocol.

    Example:
        >>> class MyDetector:
        ...     def marker_callback(self) -> MarkerObservation | None:
        ...         return MarkerObservation(dx=0.5, dy=-0.1) if detected else None
        >>> detector = MyDetector()
        >>> isinstance(detector, MarkerSource)  # True, despite no inheritance
        True
    """

    def marker_callback(self) -> MarkerObservation | None:
        """Attempt to detect and measure a landing marker.

        Returns:
            A :class:`MarkerObservation` with the marker's offset relative to
            the vehicle, or ``None`` if the marker is not currently visible.
        """
        ...


__all__ = ["MarkerSource"]
