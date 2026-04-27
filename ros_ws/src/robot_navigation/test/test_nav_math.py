import math

from robot_navigation.utils.nav_math import clamp, normalize_angle, quaternion_from_yaw, yaw_from_quaternion


def test_clamp_bounds():
    assert clamp(5.0, 0.0, 3.0) == 3.0
    assert clamp(-1.0, 0.0, 3.0) == 0.0
    assert clamp(2.5, 0.0, 3.0) == 2.5


def test_normalize_angle_range():
    val = normalize_angle(4.0 * math.pi + 0.3)
    assert -math.pi <= val <= math.pi
    assert math.isclose(val, 0.3, rel_tol=1e-6, abs_tol=1e-6)


def test_yaw_quaternion_roundtrip():
    yaw = 1.2
    _, _, z, w = quaternion_from_yaw(yaw)
    out = normalize_angle(yaw_from_quaternion(z, w))
    assert math.isclose(out, normalize_angle(yaw), abs_tol=1e-6)
