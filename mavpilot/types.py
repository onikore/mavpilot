"""Small data classes used across the package."""
from dataclasses import dataclass
from enum import Enum
from typing import Optional


@dataclass
class Position:
    """NED coordinates in meters.

    PX4 convention (vehicle_local_position):
      - x = North
      - y = East
      - z = Down (so altitude = -z)
    """

    x: float
    y: float
    z: float

    @property
    def altitude(self) -> float:
        return -self.z


@dataclass
class MarkerObservation:
    """Relative marker observation in body FRD frame.

    dx: forward (positive)
    dy: right (positive)
    dz: down (optional)  # Reserved for future vertical correction in
                         # precision_land; not read in v0.2.0.
    """

    dx: float
    dy: float
    dz: Optional[float] = None


class PrecisionLandStatus(Enum):
    """Terminal outcome of a precision_land() call. Mutually exclusive."""

    LANDED = "landed"
    """AUTO_LAND handoff completed AND landed_state observed ON_GROUND."""

    HANDED_OFF = "handed_off"
    """Descended to floor with marker locked + centered; AUTO_LAND triggered,
    but the call returned before landed_state was observed ON_GROUND
    (e.g. caller's overall timeout was approaching). Drone is still descending
    under PX4 AUTO_LAND."""

    ABORTED_AT_FLOOR = "aborted_at_floor"
    """Descended to floor altitude but marker was lost or off-center at
    handoff time. AUTO_LAND was NOT triggered. Drone is holding floor
    altitude; caller must decide next action (retry, manual land, etc.)."""

    MARKER_LOST = "marker_lost"
    """Marker callback returned None for ``marker_lost_timeout_s`` consecutive
    seconds before the floor was reached. precision_land() fell through to
    plain AUTO_LAND; result reflects that AUTO_LAND outcome."""

    TIMEOUT = "timeout"
    """Overall ``timeout_s`` reached before any other terminal condition."""


@dataclass
class PrecisionLandResult:
    status: PrecisionLandStatus
    final_position: Position
    iterations: int

    def __bool__(self) -> bool:
        """Truthy when terminal status indicates the vehicle is safely landed
        or handed off to PX4's own landing controller."""
        return self.status in (PrecisionLandStatus.LANDED, PrecisionLandStatus.HANDED_OFF)
