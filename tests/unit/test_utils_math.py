"""Unit tests for mavpilot.utils — coordinate transforms and pinhole projection."""

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from mavpilot.utils import (
    body_to_ned,
    int_to_float_bits,
    ned_to_body,
    normalize_yaw_deg,
    pixel_to_body_offset,
)


class TestPixelToBodyOffset:
    def test_center_pixel_returns_zero_offset(self):
        dx, dy = pixel_to_body_offset(
            px_norm_x=0.0,
            px_norm_y=0.0,
            camera_hfov_deg=90.0,
            camera_vfov_deg=70.0,
            altitude_above_ground_m=10.0,
        )
        assert dx == pytest.approx(0.0, abs=1e-9)
        assert dy == pytest.approx(0.0, abs=1e-9)

    def test_off_center_pinhole_90deg_fov(self):
        # px=0.5 of half-FOV in a 90° HFOV camera at 10 m altitude.
        # Correct pinhole: dy = alt * px_norm * tan(half_hfov) = 10 * 0.5 * tan(45°) = 5.0 m.
        # The old (incorrect) formula gave 10 * tan(0.5 * 45°) = 4.142 m.
        dx, dy = pixel_to_body_offset(
            px_norm_x=0.5,
            px_norm_y=0.0,
            camera_hfov_deg=90.0,
            camera_vfov_deg=90.0,
            altitude_above_ground_m=10.0,
        )
        assert dy == pytest.approx(5.0, abs=0.01)
        assert dx == pytest.approx(0.0, abs=1e-9)

    def test_edge_pixel_returns_full_half_fov_distance(self):
        # px=1.0 (image edge): dy = alt * tan(half_hfov).
        dx, dy = pixel_to_body_offset(
            px_norm_x=1.0,
            px_norm_y=0.0,
            camera_hfov_deg=60.0,
            camera_vfov_deg=60.0,
            altitude_above_ground_m=5.0,
        )
        expected_dy = 5.0 * math.tan(math.radians(30.0))
        assert dy == pytest.approx(expected_dy, abs=1e-6)

    def test_negative_pixel_y_maps_to_positive_body_x(self):
        # Image +y (downward in image, "forward" of vehicle when downward camera).
        # By convention dx_cam = -alt * px_norm_y * tan(half_vfov),
        # so positive image y → negative body x (vehicle moves backward to centre).
        dx, dy = pixel_to_body_offset(
            px_norm_x=0.0,
            px_norm_y=-0.5,
            camera_hfov_deg=90.0,
            camera_vfov_deg=90.0,
            altitude_above_ground_m=10.0,
        )
        assert dx == pytest.approx(5.0, abs=0.01)
        assert dy == pytest.approx(0.0, abs=1e-9)


class TestBodyNedRoundtrip:
    @given(
        dx=st.floats(min_value=-100, max_value=100, allow_nan=False, allow_infinity=False),
        dy=st.floats(min_value=-100, max_value=100, allow_nan=False, allow_infinity=False),
        yaw=st.floats(min_value=-math.pi, max_value=math.pi, allow_nan=False),
    )
    def test_body_ned_roundtrip(self, dx, dy, yaw):
        ned_x, ned_y = body_to_ned(dx, dy, yaw)
        bx, by = ned_to_body(ned_x, ned_y, yaw)
        assert bx == pytest.approx(dx, abs=1e-9)
        assert by == pytest.approx(dy, abs=1e-9)

    def test_forward_at_yaw_zero_maps_to_north(self):
        ned_x, ned_y = body_to_ned(1.0, 0.0, 0.0)
        assert ned_x == pytest.approx(1.0)
        assert ned_y == pytest.approx(0.0, abs=1e-9)

    def test_forward_at_yaw_90deg_maps_to_east(self):
        ned_x, ned_y = body_to_ned(1.0, 0.0, math.pi / 2)
        assert ned_x == pytest.approx(0.0, abs=1e-9)
        assert ned_y == pytest.approx(1.0)


class TestIntToFloatBits:
    @pytest.mark.parametrize("v", [0, 1, -1, 42, -42, 2**30, -(2**30), 2**31 - 1, -(2**31)])
    def test_int_to_float_bits_roundtrip(self, v):
        import struct

        as_float = int_to_float_bits(v)
        as_int_back = struct.unpack("<i", struct.pack("<f", as_float))[0]
        assert as_int_back == v


class TestNormalizeYawDeg:
    def test_in_range_unchanged(self):
        assert normalize_yaw_deg(0.0) == pytest.approx(0.0)
        assert normalize_yaw_deg(90.0) == pytest.approx(90.0)
        assert normalize_yaw_deg(-90.0) == pytest.approx(-90.0)

    def test_above_180_wraps(self):
        assert normalize_yaw_deg(181.0) == pytest.approx(-179.0)
        assert normalize_yaw_deg(360.0) == pytest.approx(0.0)
        assert normalize_yaw_deg(540.0) == pytest.approx(180.0)

    def test_below_neg180_wraps(self):
        assert normalize_yaw_deg(-181.0) == pytest.approx(179.0)
        assert normalize_yaw_deg(-360.0) == pytest.approx(0.0)
