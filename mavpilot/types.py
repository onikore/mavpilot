"""Public data types returned and accepted by :class:`mavpilot.DroneController`.

All coordinates use the PX4 **NED** convention (North-East-Down); marker
observations use the body **FRD** convention (Forward-Right-Down).
"""

from dataclasses import dataclass
from enum import Enum


@dataclass
class Position:
    """A local position in meters, in the PX4 NED frame.

    Returned by :meth:`mavpilot.DroneController.get_local_position`. The frame
    matches PX4's ``vehicle_local_position`` / ``LOCAL_POSITION_NED``:

    - ``x`` = North, ``y`` = East, ``z`` = Down.
    - Because *z* points down, altitude above the local origin is ``-z``
      (use the :attr:`altitude` convenience property).

    Example:
        >>> pos = Position(x=10.0, y=-5.0, z=-3.0)
        >>> pos.altitude
        3.0
    """

    x: float
    """North offset from the local origin, in meters (NED +x)."""

    y: float
    """East offset from the local origin, in meters (NED +y)."""

    z: float
    """Down offset from the local origin, in meters (NED +z). Negative when
    the vehicle is above the origin; ``altitude == -z``."""

    @property
    def altitude(self) -> float:
        """Height above the local origin in meters, i.e. ``-z``."""
        return -self.z


@dataclass
class MarkerObservation:
    """A landing-marker sighting expressed in the body **FRD** frame.

    This is what a :meth:`mavpilot.DroneController.precision_land` marker
    callback returns each time it sees the target. Offsets are relative to the
    vehicle body (not the world): they describe where the marker is *with
    respect to the drone*, so ``dx=0.3`` means "the pad is 0.3 m ahead of me".

    See :func:`mavpilot.utils.pixel_to_body_offset` for converting a camera
    pixel detection into these offsets.
    """

    dx: float
    """Forward offset to the marker, in meters (body +x / Forward)."""

    dy: float
    """Right offset to the marker, in meters (body +y / Right)."""

    dz: float | None = None
    """Down offset, in meters (body +z / Down). Optional and **reserved** —
    ``precision_land`` does not read it in this release."""


class PrecisionLandStatus(Enum):
    """Terminal outcome of a :meth:`mavpilot.DroneController.precision_land`
    call. Exactly one of these is reported. The first two are "good" outcomes
    (see :meth:`PrecisionLandResult.__bool__`); the rest require the caller to
    decide what to do next."""

    LANDED = "landed"
    """AUTO_LAND handoff completed AND ``landed_state`` was observed
    ON_GROUND — the vehicle is down."""

    HANDED_OFF = "handed_off"
    """Descended to the floor with the marker locked and centered and AUTO_LAND
    was triggered, but the call returned before touchdown was observed (e.g.
    the overall ``timeout_s`` was approaching). The drone is still descending
    under PX4's own AUTO_LAND."""

    ABORTED_AT_FLOOR = "aborted_at_floor"
    """Reached the floor altitude but the marker was lost or off-center at
    handoff time, so AUTO_LAND was **not** triggered. The drone is holding the
    floor altitude; the caller must decide the next action (retry, manual land,
    climb away, …)."""

    MARKER_LOST = "marker_lost"
    """The marker callback returned ``None`` for ``marker_lost_timeout_s``
    consecutive seconds *before* the floor was reached, so ``precision_land``
    fell through to a plain AUTO_LAND; this status reflects that AUTO_LAND
    outcome."""

    TIMEOUT = "timeout"
    """The overall ``timeout_s`` elapsed before any other terminal condition
    was met. No AUTO_LAND fallback was issued."""


@dataclass
class PrecisionLandResult:
    """Result of :meth:`mavpilot.DroneController.precision_land`.

    Truthy (``bool(result) is True``) only for :attr:`PrecisionLandStatus.LANDED`
    and :attr:`PrecisionLandStatus.HANDED_OFF`, so callers can branch with a
    simple ``if result:`` and inspect :attr:`status` for the detail.

    Example:
        >>> result = await drone.precision_land(get_marker)
        >>> if result:
        ...     print("down or handed off")
        ... else:
        ...     print("needs attention:", result.status.value)
    """

    status: PrecisionLandStatus
    """The terminal :class:`PrecisionLandStatus` for the attempt."""

    final_position: Position
    """The vehicle's NED :class:`Position` when the call returned."""

    iterations: int
    """How many control-loop iterations the descent ran for (useful for logs
    and tuning)."""

    def __bool__(self) -> bool:
        """``True`` when the vehicle is safely landed or has been handed off to
        PX4's AUTO_LAND controller (``LANDED`` or ``HANDED_OFF``)."""
        return self.status in (PrecisionLandStatus.LANDED, PrecisionLandStatus.HANDED_OFF)
