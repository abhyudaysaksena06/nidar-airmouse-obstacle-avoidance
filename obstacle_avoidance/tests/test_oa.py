"""Unit tests. Run:  python tests/test_oa.py   (or pytest if installed)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from oa import AvoidanceConfig, ObstacleAvoider  # noqa: E402
from oa.cbf import project_2d, filter_velocity  # noqa: E402
from oa.tracker import DynamicTracker  # noqa: E402
from oa.vfh import VFHPlus  # noqa: E402


def wall(x0, y0, x1, y1, step=0.03):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    return np.column_stack((np.linspace(x0, x1, n), np.linspace(y0, y1, n)))


def scan_of(pts, n=360, rmax=8.0):
    """Render body-frame obstacle points into a 360-beam scan."""
    ang = np.linspace(-np.pi, np.pi, n, endpoint=False)
    rng = np.full(n, np.inf)
    a = np.arctan2(pts[:, 1], pts[:, 0])
    d = np.hypot(pts[:, 0], pts[:, 1])
    k = np.round((a + np.pi) / (2 * np.pi / n)).astype(int) % n
    for i, dd in zip(k, d):
        if dd < rng[i] and dd < rmax:
            rng[i] = dd
    return ang, rng


def test_projection_matches_brute_force():
    rng = np.random.default_rng(0)
    g = np.linspace(-2, 2, 801)
    X, Y = np.meshgrid(g, g)
    G = np.column_stack((X.ravel(), Y.ravel()))
    for _ in range(30):
        m = rng.integers(1, 7)
        th = rng.uniform(-np.pi, np.pi, m)
        A = np.column_stack((np.cos(th), np.sin(th)))
        b = rng.uniform(-0.3, 1.0, m)
        u0 = rng.uniform(-1.5, 1.5, 2)
        u, ok = project_2d(u0, A, b)
        feas = np.all(G @ A.T <= b + 1e-9, axis=1)
        if not feas.any():
            continue
        assert ok
        best = G[feas][np.argmin(np.hypot(*(G[feas] - u0).T))]
        assert np.hypot(*(u - u0)) <= np.hypot(*(best - u0)) + 0.01, (u, best)
        assert np.all(A @ u <= b + 1e-6)


def test_cbf_never_breaches_clearance():
    """Point-mass drone told to fly straight into a wall at full speed."""
    cfg = AvoidanceConfig()
    W = wall(3.0, -3, 3.0, 3)                     # wall at x = 3 m (world)
    x, dt, closest = 0.0, 0.05, 99.0
    v = 0.0
    for _ in range(200):
        pts = W - np.array([x, 0.0])
        u, _ = filter_velocity(np.array([1.0, 0.0]), pts, [], cfg)
        # first-order velocity response with acceleration limit
        a = np.clip((u[0] - v) / 0.25, -cfg.a_brake, cfg.a_brake)
        v += a * dt
        x += v * dt
        closest = min(closest, 3.0 - x)
    assert closest >= cfg.drone_radius, closest       # never touches
    assert 3.0 - x < cfg.r_safe + 0.15                # but still gets close (not over-conservative)


def test_vfh_finds_gap():
    """Wall 1 m ahead with a doorway to the left: choose the doorway side."""
    cfg = AvoidanceConfig()
    pts = np.vstack((wall(1.0, -3, 1.0, 0.5), wall(1.0, 1.7, 1.0, 4)))
    v = VFHPlus(cfg)
    v.histogram(pts)
    d = v.select(0.0)
    assert d is not None and d > 0.2, d                  # turns left towards the door
    assert v.binary[np.argmin(np.abs(v.centers - d))] == 0


def test_vfh_blocked_when_boxed_in():
    cfg = AvoidanceConfig()
    a = np.linspace(-np.pi, np.pi, 400)
    v = VFHPlus(cfg)
    v.histogram(np.column_stack((0.6 * np.cos(a), 0.6 * np.sin(a))))
    assert v.select(0.0) is None


def test_tracker_estimates_velocity():
    cfg = AvoidanceConfig()
    tr = DynamicTracker(cfg)
    dt = 0.1
    ang = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    for k in range(30):
        c = np.array([3.0, -1.0 + 0.5 * k * dt])        # person walking at 0.5 m/s (+y)
        person = np.column_stack((c[0] + 0.25 * np.cos(ang), c[1] + 0.25 * np.sin(ang)))
        visible = person[person[:, 0] <= c[0]]           # lidar sees the near half
        order = np.argsort(np.arctan2(visible[:, 1], visible[:, 0]))
        tr.update(visible[order], (0.0, 0.0, 0.0), dt)
    mov = tr.moving_obstacles((0.0, 0.0, 0.0))
    assert len(mov) == 1
    _, vel, _ = mov[0]
    assert abs(vel[1] - 0.5) < 0.1 and abs(vel[0]) < 0.1, vel


def test_avoider_status_and_failsafes():
    cfg = AvoidanceConfig()
    av = ObstacleAvoider(cfg)
    # no scan yet -> STALE, zero velocity
    out = av.update(0.0, (0, 0, 0), (0.8, 0))
    assert out.status == "STALE" and out.vx == 0
    # open space -> OK, passes the command through
    out = av.update(0.1, (0, 0, 0), (0.8, 0), scan=scan_of(wall(6, -6, 6, 6)))
    assert out.status == "OK" and abs(out.vx - 0.8) < 1e-6
    # obstacle very close -> EMERGENCY, moves away
    out = av.update(0.2, (0, 0, 0), (0.8, 0), scan=scan_of(wall(0.27, -2, 0.27, 2)))
    assert out.status == "EMERGENCY" and out.vx < 0
    # scan stops arriving -> STALE again
    out = av.update(1.0, (0, 0, 0), (0.8, 0))
    assert out.status == "STALE"


def test_compute_time_budget():
    cfg = AvoidanceConfig()
    av = ObstacleAvoider(cfg)
    rng = np.random.default_rng(1)
    pts = np.vstack((wall(2, -3, 2, 3), wall(-3, 1.2, 3, 1.2), rng.uniform(-3, 3, (40, 2))))
    sc = scan_of(pts)
    ms = []
    for k in range(50):
        av.update(0.1 * k, (0.01 * k, 0, 0), (0.8, 0.1), scan=sc)
        ms.append(av.compute_ms)
    print(f"  mean update {np.mean(ms):.2f} ms, max {np.max(ms):.2f} ms (desktop CPU)")
    assert np.mean(ms) < 20


def test_points3d_classification():
    """Wall -> horizontal; beam with free space seen below -> hanging; low box -> low."""
    from oa.points3d import PointCloudFrontEnd
    cfg = AvoidanceConfig()
    fe = PointCloudFrontEnd(cfg)
    z = 1.0
    zs = np.linspace(-0.9, 0.9, 19)
    wall = np.array([[2.0, y, zz] for y in np.linspace(-0.5, 0.5, 6) for zz in zs])
    beam = np.array([[0.6, y, zz] for y in np.linspace(0.3, 0.8, 6) for zz in (0.2, 0.3)])   # bottom 1.2 m abs
    box = np.array([[1.2, y, zz] for y in np.linspace(-1.6, -1.2, 5) for zz in (-0.9, -0.7, -0.5)])  # top 0.5 m abs
    h, low, hang = fe.split(np.vstack((wall, beam, box)), z, vfov=(-0.5, 0.5))
    assert len(h) and np.all(np.abs(h[:, 0] - 2.0) < 0.1)                 # only the wall is horizontal
    assert len(hang) and np.all(np.abs(hang[:, 0] - 0.6) < 0.1)
    assert len(low) and np.all(np.abs(low[:, 0] - 1.2) < 0.1)
    zt, _ = fe.altitude_band(low, hang, z, cruise=1.0, motion_dir=math.atan2(0.55, 0.6))
    assert zt < 1.2 - cfg.half_height - cfg.vertical_clearance + 1e-6       # ducks under the beam
    zt2, _ = fe.altitude_band(low, np.zeros((0, 3)), z, cruise=1.0, motion_dir=math.atan2(-1.4, 1.2))
    assert zt2 >= 1.0                                                       # box top 0.5 m: cruise is fine


def test_depth_image_projection():
    from oa.points3d import depth_image_to_points
    W, H, f = 64, 48, 40.0
    depth = np.full((H, W), 2.0)
    p = depth_image_to_points(depth, f, f, W / 2, H / 2, pitch=0.0, stride=1)
    centre = p[np.argmin(np.hypot(p[:, 1], p[:, 2]))]
    assert abs(centre[0] - 2.0) < 0.05 and abs(centre[1]) < 0.06 and abs(centre[2]) < 0.06
    left = p[np.argmax(p[:, 1])]
    assert left[1] > 0                                                      # image left = body +y (FLU)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for f in tests:
        f()
        print("PASS", f.__name__)
    print(f"{len(tests)} tests passed")
