"""Unit tests for OA-Stack. Run:  python tests/test_oa_stack.py

Synthetic ToF frames are rendered by casting the guide's 64 zone rays at axis-aligned
boxes (drone frame, z relative to base_link) and the floor."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from oa_stack import GuideBaseline, StackAvoider, StackConfig  # noqa: E402
from oa_stack.lidar import clean_scan  # noqa: E402
from oa_stack.tof import parse_line, zone_directions  # noqa: E402

ANG = np.linspace(-math.pi, math.pi, 500, endpoint=False)   # C1: ~0.72 deg


def ray_boxes(origin, dirs, boxes, floor_z, max_r=3.0):
    """Distances (m) along each unit ray to the nearest box or the floor (z = floor_z)."""
    out = np.full(len(dirs), np.inf)
    for k, d in enumerate(dirs):
        best = math.inf
        if d[2] < -1e-6:
            tf = (floor_z - origin[2]) / d[2]
            if tf > 0:
                best = tf
        for lo, hi in boxes:
            tmin, tmax = -math.inf, math.inf
            for ax in range(3):
                if abs(d[ax]) < 1e-9:
                    if not (lo[ax] <= origin[ax] <= hi[ax]):
                        tmin, tmax = 1, 0
                        break
                else:
                    t1, t2 = (lo[ax] - origin[ax]) / d[ax], (hi[ax] - origin[ax]) / d[ax]
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if tmax >= max(tmin, 0) and tmin > 0:
                best = min(best, tmin)
        out[k] = best
    return out


def frames(cfg, boxes, height):
    res = []
    for tag, pitch, xyz in (("U", cfg.up_pitch_deg, cfg.up_xyz), ("D", cfg.down_pitch_deg, cfg.down_xyz)):
        dirs = zone_directions(pitch, cfg.tof_fov_deg)
        d = ray_boxes(np.array(xyz), dirs, boxes, -height, cfg.tof_max_range)
        res.append((tag, [int(v * 1000) if v <= cfg.tof_max_range else -1 for v in d]))
    return res


def scan_of_boxes(boxes, z=0.08):
    """Lidar ranges (plane at z) to boxes that cross the plane."""
    dirs = np.column_stack((np.cos(ANG), np.sin(ANG), np.zeros_like(ANG)))
    crossing = [b for b in boxes if b[0][2] <= z <= b[1][2]]
    return ray_boxes(np.array([0.0, 0.0, z]), dirs, crossing, -99, 12.0)


def test_parse_line_and_geometry():
    tag, mm = parse_line("U," + ",".join(["1000"] * 64))
    assert tag == "U" and len(mm) == 64
    assert parse_line("X,1,2") is None and parse_line("U," + ",".join(["a"] * 64)) is None
    d = zone_directions(20.0)
    mean = d.mean(0)
    assert abs(math.degrees(math.atan2(mean[2], mean[0])) - 20.0) < 1.0     # axis points 20 deg up
    assert d[0, 2] > d[63, 2]                                                # row 0 = top


def test_tilt_correction():
    cfg = StackConfig()
    r = np.full(500, 2.0)
    _, rh, q, tilt = clean_scan(ANG, r, cfg, roll=0.0, pitch=math.radians(15))
    k0 = int(np.argmin(np.abs(ANG)))
    assert abs(rh[k0] - 2.0 * math.cos(math.radians(15))) < 0.01             # forward beam shortened
    k90 = int(np.argmin(np.abs(ANG - math.pi / 2)))
    assert abs(rh[k90] - 2.0) < 0.01                                         # side beam unaffected
    assert abs(tilt - 15) < 0.5 and q == 1.0


def test_spike_rejected():
    cfg = StackConfig()
    r = np.full(500, 3.0)
    r[123] = 0.3
    _, rh, q, _ = clean_scan(ANG, r, cfg)
    assert not np.isfinite(rh[123]) and q < 1.0


def _run(boxes, height=1.0, steps=3, cfg=None):
    cfg = cfg or StackConfig()
    av = StackAvoider(cfg)
    out = None
    for k in range(steps):
        out = av.update(0.1 * k, (0.0, 0.0, 0.0), (0.6, 0.0), scan=(ANG, scan_of_boxes(boxes)),
                        tof_frames=frames(cfg, boxes, height), height=height, cruise_alt=1.0)
    return av, out


def test_floor_and_wall_ignored():
    wall = ((1.2, -2.0, -1.0), (1.3, 2.0, 1.4))           # floor-to-ceiling wall 1.2 m ahead
    av, out = _run([wall])
    assert out.z_target == 1.0 and out.info["vertical"] == "cruise"      # no climb / duck for a wall
    assert len(av.hazards.items) == 0                                    # floor + wall both ignored


def test_duct_ducks_under():
    duct = ((0.6, -0.8, 0.13), (0.9, 0.8, 0.19))          # guide sim: duct 1.13-1.19 m at cruise 1.0 m
    av, out = _run([duct])
    assert out.info["vertical"] == "duck", out.info
    assert abs(out.z_target - (1.13 - 0.25)) < 0.05                       # guide: 25 cm below the duct
    assert "DUCK" in out.flags and out.vx < 0.6 * 0.5                     # slowed until ducked


def test_low_box_climbs_over():
    box = ((0.8, -0.3, -1.0), (1.2, 0.3, 0.0))            # guide sim: 1.0 m box, cruise 1.0 m
    av, out = _run([box])
    assert out.info["vertical"] == "climb", out.info
    assert abs(out.z_target - (1.0 + 0.35)) < 0.06                        # guide: 35 cm above its top


def test_inband_pole_blocks_horizontally():
    """Thin pole the lidar misses (dark / narrower than a beam gap) but the ToF sees."""
    pole = ((0.7, -0.05, -1.0), (0.8, 0.05, 1.4))   # 10 cm (zone rays are ~6 cm apart at 0.6 m)
    cfg = StackConfig()
    av = StackAvoider(cfg)
    empty = np.full(500, np.inf)                           # lidar sees nothing
    out = None
    for k in range(3):
        out = av.update(0.1 * k, (0.0, 0.0, 0.0), (0.8, 0.0), scan=(ANG, empty),
                        tof_frames=frames(cfg, [pole], 1.0), height=1.0)
    assert out.info["extra_obstacles"] > 0
    assert out.vx < 0.8 or abs(out.vy) > 0.05                             # not flying straight into it


def test_memory_keeps_duct_after_it_leaves_view():
    duct = ((0.6, -0.8, 0.13), (0.9, 0.8, 0.19))
    cfg = StackConfig()
    av = StackAvoider(cfg)
    for k in range(3):
        av.update(0.1 * k, (0.0, 0.0, 0.0), (0.6, 0.0), scan=(ANG, scan_of_boxes([duct])),
                  tof_frames=frames(cfg, [duct], 1.0), height=1.0)
    # drone now under the duct, ToF sees nothing new; height already lowered
    out = av.update(0.5, (0.75, 0.0, 0.0), (0.6, 0.0), scan=(ANG, np.full(500, np.inf)),
                    tof_frames=frames(cfg, [], 0.88), height=0.88)
    assert out.info["vertical"] == "duck" and out.z_target < 0.9            # stays low while under it


def test_failsafes():
    cfg = StackConfig()
    av = StackAvoider(cfg)
    out = av.update(0.0, (0, 0, 0), (0.6, 0.0))                           # nothing received yet
    assert out.status == "STALE" and out.vx == 0.0 and "LIDAR_DOWN" in out.flags
    out = av.update(0.1, (0, 0, 0), (0.6, 0.0), scan=(ANG, np.full(500, 5.0)), height=None)
    assert "NO_HEIGHT" in out.flags and "TOF_DOWN" in out.flags and out.vx <= 0.6 * 0.4 + 1e-6


def test_guide_baseline_reacts_like_the_guide():
    duct = ((0.6, -0.8, 0.13), (0.9, 0.8, 0.19))
    cfg = StackConfig()
    b = GuideBaseline(cfg)
    o = b.update(0.0, (0, 0, 0), (0.6, 0.0), scan=(ANG, scan_of_boxes([duct])),
                 tof_frames=frames(cfg, [duct], 1.0), height=1.0)
    assert abs(o["z_target"] - 0.88) < 0.05


def test_compute_time():
    cfg = StackConfig()
    av = StackAvoider(cfg)
    boxes = [((1.2, -2.0, -1.0), (1.3, 2.0, 1.4)), ((0.6, -0.8, 0.13), (0.9, 0.8, 0.19)),
             ((0.8, 0.5, -1.0), (1.0, 0.9, 0.0))]
    fr, sc = frames(cfg, boxes, 1.0), scan_of_boxes(boxes)
    ms = []
    for k in range(60):
        out = av.update(0.05 * k, (0.01 * k, 0.0, 0.0), (0.6, 0.1), scan=(ANG, sc) if k % 2 == 0 else None,
                        tof_frames=fr if k % 3 == 0 else (), height=1.0)
        ms.append(out.info["compute_ms"])
    print(f"  mean update {np.mean(ms):.2f} ms, max {np.max(ms):.2f} ms (desktop CPU)")
    assert np.mean(ms) < 15


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for f in tests:
        f()
        print("PASS", f.__name__)
    print(f"{len(tests)} tests passed")
