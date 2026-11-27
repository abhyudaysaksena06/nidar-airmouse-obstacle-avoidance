import math

import numpy as np

from nidar_control.ev_bridge import enu_flu_to_ned_frd


def yaw_of(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def test_facing_north_is_ned_yaw_0():
    c = math.cos(math.pi / 4)
    p, q, v = enu_flu_to_ned_frd([1.0, 2.0, 3.0], (c, 0, 0, c), [1.0, 0.5, 0.2])
    assert np.allclose(p, [2.0, 1.0, -3.0])          # north, east, down
    assert abs(yaw_of(q)) < 1e-6
    assert np.allclose(v, [1.0, -0.5, -0.2])          # body FLU -> FRD


def test_facing_east_is_ned_yaw_90():
    _, q, _ = enu_flu_to_ned_frd([0, 0, 0], (1, 0, 0, 0), [0, 0, 0])
    assert abs(yaw_of(q) - 90) < 1e-6


def test_world_velocity_option():
    _, _, v = enu_flu_to_ned_frd([0, 0, 0], (1, 0, 0, 0), [1.0, 2.0, 3.0], body_velocity=False)
    assert np.allclose(v, [2.0, 1.0, -3.0])
