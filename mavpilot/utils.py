"""Utility functions: bit-casts and coordinate transforms."""
import math
import struct
from typing import Tuple


def int_to_float_bits(value: int) -> float:
    """Bit-cast int32 -> float for PX4 PARAM_SET INT32 params."""
    return struct.unpack("<f", struct.pack("<i", int(value)))[0]


def pixel_to_body_offset(
    px_norm_x: float,
    px_norm_y: float,
    camera_hfov_deg: float,
    camera_vfov_deg: float,
    altitude_above_ground_m: float,
    camera_mount_yaw_deg: float = 0.0,
) -> Tuple[float, float]:
    """Convert normalized pixel offset to body offset in meters.

    Pinhole camera model. ``px_norm_x``/``px_norm_y`` are in ``[-1, 1]`` where
    ``±1`` is the image edge. Returns (dx, dy) in body FRD frame (meters),
    assuming a downward-facing camera.
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


def body_to_ned(dx_body: float, dy_body: float, yaw_rad: float) -> Tuple[float, float]:
    """Convert body FRD offset to NED offset using current heading."""
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    ned_x = cy * dx_body - sy * dy_body
    ned_y = sy * dx_body + cy * dy_body
    return ned_x, ned_y


def ned_to_body(ned_dx: float, ned_dy: float, yaw_rad: float) -> Tuple[float, float]:
    """Convert NED offset to body FRD offset using current heading."""
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    body_dx = cy * ned_dx + sy * ned_dy
    body_dy = -sy * ned_dx + cy * ned_dy
    return body_dx, body_dy


def normalize_yaw_deg(yaw_deg: float) -> float:
    """Wrap yaw in degrees to [-180, 180]."""
    y = yaw_deg % 360.0
    if y > 180.0:
        y -= 360.0
    return y
