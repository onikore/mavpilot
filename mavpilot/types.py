"""Small data classes used across the package."""
from dataclasses import dataclass
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
    dz: down (optional)
    """

    dx: float
    dy: float
    dz: Optional[float] = None
