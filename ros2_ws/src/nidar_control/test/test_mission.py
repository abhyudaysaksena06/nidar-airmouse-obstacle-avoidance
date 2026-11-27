import math

from nidar_control.mission import Carrot, Waypoint, build_mission


def test_missions_build():
    for kind in ("hover", "square", "step", "arena_demo"):
        m = build_mission(kind, 2.0, 4.0, 1.0, 5.0)
        assert m and all(w.z < 0 for w in m)   # NED: up is negative z


def test_carrot_moves_at_speed():
    c = Carrot(0, 0, 0)
    wp = Waypoint(10, 0, 0)
    for _ in range(20):          # 1 s at 20 Hz
        c.step(wp, 2.0, 0.05)
    assert math.isclose(c.p[0], 2.0, abs_tol=1e-9)
    assert math.isclose(c.v[0], 2.0)


def test_carrot_reaches_and_stops():
    c = Carrot(0, 0, 0)
    wp = Waypoint(0.05, 0, 0)
    c.step(wp, 2.0, 0.05)
    assert c.at(wp) and c.v == [0.0, 0.0, 0.0]


def test_jump_is_instant():
    c = Carrot(0, 0, -2)
    wp = Waypoint(1, 0, -2, jump=True)
    c.step(wp, 0.1, 0.05)
    assert c.at(wp)


def test_face_travel_points_nose_along_path():
    c = Carrot(0, 0, -1)
    c.step(Waypoint(0, 5, -1), 1.0, 0.05, face_travel=True)   # moving east
    assert math.isclose(c.yaw, math.pi / 2)


def test_decel_slows_into_corner():
    c = Carrot(0, 0, -1)
    wp = Waypoint(0.1, 0, -1)
    c.step(wp, 1.0, 0.05, decel=0.5)          # 0.1 m away: max speed sqrt(2*0.5*0.1)=0.32
    assert c.v[0] < 0.33


def test_ramp_up_limits_acceleration():
    c = Carrot(0, 0, -1)
    c.step(Waypoint(10, 0, -1), 0.8, 0.05, decel=0.5)
    assert c.v[0] <= 0.5 * 0.05 + 1e-9          # first tick: 0.025 m/s, not 0.8
