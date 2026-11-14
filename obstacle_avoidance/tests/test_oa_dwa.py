"""Unit tests for Algo 1. Run:  python tests/test_oa_dwa.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from oa_dwa import DWAAvoider, DWAConfig  # noqa: E402
from oa_dwa.costmap import LocalCostmap  # noqa: E402
from oa_dwa.frames import PoseBuffer  # noqa: E402
from oa_dwa.perception import filter_scan  # noqa: E402

ANG = np.linspace(-math.pi, math.pi, 360, endpoint=False)


def cast_circles(x, y, yaw, circles, rmax=8.0):
    """360-beam scan of circular obstacles [(cx, cy, r)] from pose (x, y, yaw)."""
    rng = np.full(len(ANG), np.inf)
    for cx, cy, r in circles:
        rel = np.array([cx - x, cy - y])
        dirs = np.column_stack((np.cos(ANG + yaw), np.sin(ANG + yaw)))
        b = dirs @ rel
        disc = b * b - (rel @ rel - r * r)
        hit = (disc >= 0) & (b > 0)
        d = np.where(hit, b - np.sqrt(np.maximum(disc, 0)), np.inf)
        rng = np.minimum(rng, d)
    rng[rng > rmax] = np.inf
    return rng


def fly(av, circles, goal, steps=400, dt=0.05, frame_sign=1.0):
    """Unicycle point-mass flying the avoider's commands towards `goal`."""
    x = y = yaw = 0.0
    closest = math.inf
    for k in range(steps):
        t = k * dt
        gx, gy = goal[0] - x, goal[1] - y
        if math.hypot(gx, gy) < 0.3:
            return True, closest
        rel = math.atan2(gy, gx) - yaw
        u = (0.8 * math.cos(rel), 0.8 * math.sin(rel) * frame_sign)
        scan = None
        if k % 2 == 0:
            r = cast_circles(x, y, yaw, circles)
            scan = (ANG * frame_sign, r)
        out = av.update(t, (x, y * frame_sign, yaw * frame_sign), u, scan=scan)
        w = (out.yaw_rate or 0.0) * frame_sign
        vx, vy = out.vx, out.vy * frame_sign
        x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
        y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
        yaw += w * dt
        for cx, cy, r in circles:
            closest = min(closest, math.hypot(cx - x, cy - y) - r)
    return False, closest


def test_pose_interpolation():
    pb = PoseBuffer(2.0)
    pb.add(0.0, (0.0, 0.0, 3.0))
    pb.add(1.0, (1.0, 2.0, -3.0))        # yaw crosses +-pi
    x, y, th = pb.at(0.5)
    assert abs(x - 0.5) < 1e-9 and abs(y - 1.0) < 1e-9
    assert abs(abs(th) - math.pi) < 0.2   # interpolated the short way round


def test_spike_rejection():
    cfg = DWAConfig(lidar_fov_deg=360)
    r = np.full(360, 3.0)
    r[100] = 0.4                            # single bogus return
    pts, _, rk, q = filter_scan(ANG, r, cfg)
    assert np.all(rk > 2.9) and q < 1.0


def test_candidate_then_confirmed():
    cfg = DWAConfig()
    cm = LocalCostmap(cfg)
    hit = np.array([[0.8, 0.0]])
    cm.integrate(0.0, np.zeros(2), hit)
    assert len(cm.obstacle_points(np.zeros(2), 3.0)) == 1      # candidate, but close -> avoided
    far = np.array([[2.5, 0.0]])
    cm2 = LocalCostmap(cfg)
    cm2.integrate(0.0, np.zeros(2), far)
    assert len(cm2.obstacle_points(np.zeros(2), 3.0)) == 0     # single far return ignored
    cm2.integrate(0.1, np.zeros(2), far)
    assert len(cm2.obstacle_points(np.zeros(2), 3.0)) == 1     # confirmed after 2 hits
    assert len(cm2.obstacle_report(min_cells=1)) == 1


def test_avoids_pillar_and_reaches_goal():
    av = DWAAvoider(DWAConfig(lidar_fov_deg=180))
    ok, closest = fly(av, [(2.5, 0.0, 0.3)], (5.0, 0.0))
    assert ok, "did not reach goal"
    assert closest > av.cfg.drone_radius, closest


def test_frd_mirror_equivalent():
    """FRD interface = mirrored FLU: same physical flight, no collision."""
    av = DWAAvoider(DWAConfig(lidar_fov_deg=180, body_frame="FRD"))
    ok, closest = fly(av, [(2.5, 0.1, 0.3)], (5.0, 0.0), frame_sign=-1.0)
    assert ok and closest > av.cfg.drone_radius


def test_blocked_and_stale():
    cfg = DWAConfig(lidar_fov_deg=360)
    av = DWAAvoider(cfg)
    assert av.update(0.0, (0, 0, 0), (0.8, 0)).status == "STALE"
    ring = [(0.55 * math.cos(a), 0.55 * math.sin(a), 0.05) for a in np.linspace(-math.pi, math.pi, 40)]
    out = None
    for k in range(4):
        out = av.update(0.05 * k, (0, 0, 0), (0.8, 0), scan=(ANG, cast_circles(0, 0, 0, ring)))
    assert out.status == "BLOCKED" and out.replan and out.vx == 0.0
    assert av.report()["blocked_events"]
    assert av.update(2.0, (0, 0, 0), (0.8, 0)).status == "STALE"


def test_compute_time():
    av = DWAAvoider(DWAConfig(lidar_fov_deg=180))
    circles = [(2.0, 0.5, 0.3), (1.5, -1.0, 0.4), (3.0, 1.5, 0.3)]
    ms = []
    for k in range(40):
        out = av.update(0.05 * k, (0.0, 0.0, 0.0), (0.8, 0.0), scan=(ANG, cast_circles(0, 0, 0, circles)))
        ms.append(out.info["compute_ms"])
    print(f"  mean update {np.mean(ms):.2f} ms, max {np.max(ms):.2f} ms (desktop CPU)")
    assert np.mean(ms) < 40


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for f in tests:
        f()
        print("PASS", f.__name__)
    print(f"{len(tests)} tests passed")
