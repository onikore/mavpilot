"""Coordinate transforms and bit-casts.

Pure, dependency-free helpers shared across the package and useful on their own
when wiring a vision pipeline into :meth:`mavpilot.DroneController.precision_land`
or doing manual NED/body math.

Frames:

- **NED** (world): x=North, y=East, z=Down — PX4's local frame.
- **FRD** (body): x=Forward, y=Right, z=Down — relative to the vehicle's nose.
"""

import math
import struct


def int_to_float_bits(value: int) -> float:
    """Reinterpret an ``int32`` bit pattern as a ``float`` for PX4 ``PARAM_SET``.

    PX4 transports every parameter value in a 32-bit float field, even integer
    parameters: the four bytes of the int are sent verbatim and reinterpreted on
    the other side. This performs that bit-cast (``int32`` → ``float``) so an
    integer param can be packed into the float field without numeric conversion.

    Args:
        value: The integer parameter value (treated as a signed ``int32``).

    Returns:
        The float whose IEEE-754 bytes equal the int32's bytes — *not* the
        numeric value of ``value``.
    """
    return float(struct.unpack("<f", struct.pack("<i", int(value)))[0])


def pixel_to_body_offset(
    px_norm_x: float,
    px_norm_y: float,
    camera_hfov_deg: float,
    camera_vfov_deg: float,
    altitude_above_ground_m: float,
    camera_mount_yaw_deg: float = 0.0,
) -> tuple[float, float]:
    """Project a normalized camera pixel onto the ground as a body-frame offset.

    Uses a pinhole camera model for a **downward-facing** camera: a target seen
    at normalized pixel ``(px_norm_x, px_norm_y)`` is mapped to its position on
    the ground plane relative to the vehicle, scaled by the height above ground.
    The result is suitable for building a
    :class:`mavpilot.MarkerObservation` for ``precision_land``.

    The projection is the true pinhole relation
    ``offset = altitude * px_norm * tan(fov / 2)`` (not the small-angle
    approximation ``altitude * px_norm * (fov / 2)``), which matters at wide FOV
    — the two differ by ~17% at the edge of a 90° field of view.

    Args:
        px_norm_x: Horizontal pixel offset normalized to ``[-1, 1]``, where
            ``+1`` is the right image edge and ``0`` is the optical center.
        px_norm_y: Vertical pixel offset normalized to ``[-1, 1]``, where
            ``+1`` is the bottom image edge.
        camera_hfov_deg: Horizontal field of view, in degrees.
        camera_vfov_deg: Vertical field of view, in degrees.
        altitude_above_ground_m: Height of the camera above the ground plane,
            in meters (e.g. ``drone.get_local_position().altitude``).
        camera_mount_yaw_deg: Yaw of the camera mount relative to the vehicle
            nose, in degrees (0 = camera forward axis aligned with body +x).

    Returns:
        ``(dx, dy)`` in the body FRD frame, in meters: ``dx`` forward, ``dy``
        right — directly usable as ``MarkerObservation(dx=dx, dy=dy)``.
    """
    half_h = math.radians(camera_hfov_deg / 2.0)
    half_v = math.radians(camera_vfov_deg / 2.0)

    # Pinhole: ground-plane offset = altitude * px_norm * tan(half_fov)
    dy_cam = altitude_above_ground_m * px_norm_x * math.tan(half_h)
    dx_cam = -altitude_above_ground_m * px_norm_y * math.tan(half_v)

    cy = math.cos(math.radians(camera_mount_yaw_deg))
    sy = math.sin(math.radians(camera_mount_yaw_deg))
    dx = cy * dx_cam - sy * dy_cam
    dy = sy * dx_cam + cy * dy_cam
    return dx, dy


def body_to_ned(dx_body: float, dy_body: float, yaw_rad: float) -> tuple[float, float]:
    """Rotate a body FRD horizontal offset into the world NED frame.

    Applies the 2-D yaw rotation that takes a "forward/right" offset relative to
    the vehicle and expresses it as a "north/east" offset in the world. Inverse
    of :func:`ned_to_body`.

    Args:
        dx_body: Forward offset in the body frame, in meters.
        dy_body: Right offset in the body frame, in meters.
        yaw_rad: Vehicle heading in radians (0 = facing North, NED convention).

    Returns:
        ``(ned_x, ned_y)`` — north and east offsets, in meters.
    """
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    ned_x = cy * dx_body - sy * dy_body
    ned_y = sy * dx_body + cy * dy_body
    return ned_x, ned_y


def ned_to_body(ned_dx: float, ned_dy: float, yaw_rad: float) -> tuple[float, float]:
    """Rotate a world NED horizontal offset into the body FRD frame.

    Inverse of :func:`body_to_ned`: given a north/east offset in the world and
    the vehicle heading, returns the forward/right offset relative to the nose.

    Args:
        ned_dx: North offset in the world frame, in meters.
        ned_dy: East offset in the world frame, in meters.
        yaw_rad: Vehicle heading in radians (0 = facing North, NED convention).

    Returns:
        ``(body_dx, body_dy)`` — forward and right offsets, in meters.
    """
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    body_dx = cy * ned_dx + sy * ned_dy
    body_dy = -sy * ned_dx + cy * ned_dy
    return body_dx, body_dy


def normalize_yaw_deg(yaw_deg: float) -> float:
    """Wrap an angle in degrees to the half-open range ``[-180, 180)``.

    Args:
        yaw_deg: Any angle in degrees (may be negative or exceed ±360).

    Returns:
        The equivalent angle wrapped into ``[-180, 180)`` — e.g. ``190`` → ``-170``.
    """
    y = yaw_deg % 360.0
    if y > 180.0:
        y -= 360.0
    return y
