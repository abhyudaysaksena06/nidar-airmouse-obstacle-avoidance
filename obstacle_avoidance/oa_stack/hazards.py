"""Stages B-D - ToF points -> hazard columns -> memory -> altitude band.

Each ToF point (drone frame, levelled) is binned into a vertical column of
`column_res` x `column_res` m. Heights are taken relative to base_link (dz) and
above the floor (h = height + dz). Per column, from its LOWEST point zmin and
HIGHEST point zmax:

  floor        : h < low_min_h for every point              -> ignored (guide low_min_h)
  known wall   : column within wall_radius of a lidar return
                 (current scan) or a strong /map cell         -> ignored (guide wall check)
  far above    : zmin > up_max_dz                             -> ignored (ceiling etc.)
  DUCT         : up_min_dz < zmin <= up_max_dz                -> must pass UNDER it
  LOW          : zmax < low_max_dz (top below the lidar plane) -> must pass OVER it
  IN-BAND      : anything else that reaches our own height and
                 the lidar did not see (thin pole, dark/glass) -> horizontal obstacle

The guide's tof_bridge.classify() implements DUCT and LOW (nearest point, one decision
per sensor). OA-Stack keeps the same windows but works per column, keeps every hazard
in a short memory (the ToF only looks forward: once the drone is under a duct or over a
box the sensor no longer sees it), and adds the IN-BAND class.
"""
from __future__ import annotations

import math

import numpy as np

from .config import StackConfig


def _odom(points_xy, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + c * points_xy[:, 0] - s * points_xy[:, 1],
                            y + s * points_xy[:, 0] + c * points_xy[:, 1]))


def _body(points_xy, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = points_xy[:, 0] - x, points_xy[:, 1] - y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def near_points(xy, ref_xy, radius):
    """True for each xy within `radius` of any ref point (both body frame)."""
    if len(xy) == 0 or len(ref_xy) == 0:
        return np.zeros(len(xy), dtype=bool)
    d2 = ((xy[:, None, :] - ref_xy[None, :, :]) ** 2).sum(-1)
    return (d2 <= radius * radius).any(1)


def near_map_wall(xy_odom, grid, radius, occ_threshold):
    """Guide's near_wall(): a strongly occupied /map cell within radius.
    grid = (data 2D 0..100/-1, resolution, origin_x, origin_y)."""
    data, res, ox, oy = grid
    k = int(round(radius / res))
    out = np.zeros(len(xy_odom), dtype=bool)
    H, W = data.shape
    for i, (x, y) in enumerate(xy_odom):
        c, r = int((x - ox) / res), int((y - oy) / res)
        win = data[max(r - k, 0):min(r + k + 1, H), max(c - k, 0):min(c + k + 1, W)]
        out[i] = win.size > 0 and bool((win >= occ_threshold).any())
    return out


class HazardMap:
    """Classified ToF hazards kept in the odometry frame for hazard_memory_s."""

    def __init__(self, cfg: StackConfig):
        self.cfg = cfg
        # rows: x, y (odom), value (abs height: duct bottom / low top), t, kind (0 duct, 1 low, 2 inband)
        self.items = np.zeros((0, 5))

    def classify(self, pts, height, pose, lidar_xy_body, grid=None, kind_hint="up"):
        """pts: (N,3) levelled drone-frame ToF points of ONE sensor frame.
        Returns dict counts and adds hazards to memory with the current time set later."""
        cfg = self.cfg
        out = {"duct": [], "low": [], "inband": []}
        if len(pts) == 0:
            return out
        h_abs = height + pts[:, 2]
        keep = h_abs >= cfg.low_min_h                                   # floor
        horiz = np.hypot(pts[:, 0], pts[:, 1])
        reach = cfg.up_reach if kind_hint == "up" else cfg.low_reach
        keep &= horiz <= max(cfg.up_reach, cfg.low_reach)
        p = pts[keep]
        if len(p) == 0:
            return out
        key = np.floor(p[:, :2] / cfg.column_res).astype(np.int64)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.ravel()
        n = len(uniq)
        zmin = np.full(n, np.inf)
        zmax = np.full(n, -np.inf)
        np.minimum.at(zmin, inv, p[:, 2])
        np.maximum.at(zmax, inv, p[:, 2])
        centre = (uniq + 0.5) * cfg.column_res
        dist = np.hypot(centre[:, 0], centre[:, 1])
        if kind_hint == "down" and cfg.unconfirmed_top_margin > 0:
            # The DOWN sensor cannot look above its top edge. A column that reaches that edge may be
            # taller than what we saw (e.g. a 1.0 m box at a 1.0 m cruise, under the lidar plane):
            # raise its top to just above the edge, so it is a hard obstacle until we climb and see over it.
            sx, sz = cfg.down_xyz[0], cfg.down_xyz[2]
            edge = math.tan(math.radians(cfg.down_pitch_deg + cfg.tof_fov_deg / 2))
            z_edge = sz + np.hypot(p[:, 0] - sx, p[:, 1]) * edge
            at_edge = np.zeros(n, dtype=bool)
            np.logical_or.at(at_edge, inv, p[:, 2] >= z_edge - cfg.unconfirmed_top_margin)
            zmax = np.where(at_edge, np.minimum(np.maximum(zmax, sz + 0.10), cfg.low_max_dz - 1e-3), zmax)
        # wall check: lidar already explains it (current scan, and /map if given)
        wall = np.zeros(n, dtype=bool)
        if cfg.use_wall_check:
            rad = cfg.wall_radius_up if kind_hint == "up" else cfg.wall_radius_down
            wall |= near_points(centre, lidar_xy_body, rad)
            if grid is not None and cfg.use_map:
                wall |= near_map_wall(_odom(centre, pose), grid, rad, cfg.occ_threshold)
        far_above = zmin > cfg.up_max_dz
        duct = ~wall & ~far_above & (zmin > cfg.up_min_dz) & (dist <= cfg.up_reach)
        low = ~wall & ~duct & (zmax < cfg.low_max_dz) & (dist <= cfg.low_reach)
        inband = ~wall & ~far_above & ~duct & ~low & (zmin <= cfg.up_min_dz) & (dist <= reach)
        if not cfg.use_inband:
            inband[:] = False
        out["duct"] = np.column_stack((centre[duct], height + zmin[duct])) if duct.any() else np.zeros((0, 3))
        out["low"] = np.column_stack((centre[low], height + zmax[low])) if low.any() else np.zeros((0, 3))
        out["inband"] = np.column_stack((centre[inband], height + zmin[inband])) if inband.any() else np.zeros((0, 3))
        return out

    def add(self, found, pose, t):
        rows = []
        for kind, k in (("duct", 0), ("low", 1), ("inband", 2)):
            c = found[kind]
            if len(c):
                xy = _odom(c[:, :2], pose)
                rows.append(np.column_stack((xy, c[:, 2], np.full(len(c), t), np.full(len(c), k))))
        if rows:
            self.items = self._dedupe(np.vstack([self.items] + rows))

    def _dedupe(self, items):
        """One entry per 10 cm cell and kind: newest time; lowest duct bottom,
        highest low-obstacle top (the conservative value)."""
        if len(items) == 0:
            return items
        r = self.cfg.column_res
        key = np.column_stack((np.floor(items[:, 0] / r), np.floor(items[:, 1] / r), items[:, 4])).astype(np.int64)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.ravel()
        n = len(uniq)
        out = np.zeros((n, 5))
        out[:, 0] = (uniq[:, 0] + 0.5) * r
        out[:, 1] = (uniq[:, 1] + 0.5) * r
        out[:, 4] = uniq[:, 2]
        lo = np.full(n, np.inf)
        hi = np.full(n, -np.inf)
        np.minimum.at(lo, inv, items[:, 2])
        np.maximum.at(hi, inv, items[:, 2])
        out[:, 2] = np.where(out[:, 4] == 1, hi, lo)       # low: highest top; duct/inband: lowest bottom
        tmax = np.full(n, -np.inf)
        np.maximum.at(tmax, inv, items[:, 3])
        out[:, 3] = tmax
        return out

    def prune(self, t, pose):
        cfg = self.cfg
        if len(self.items) == 0:
            return
        age_ok = (t - self.items[:, 3]) <= (cfg.hazard_memory_s if cfg.use_memory else 0.2)
        d = np.hypot(self.items[:, 0] - pose[0], self.items[:, 1] - pose[1])
        self.items = self.items[age_ok & (d < 4.0)]

    def body_items(self, pose, kind):
        sel = self.items[self.items[:, 4] == kind]
        if len(sel) == 0:
            return np.zeros((0, 3))
        return np.column_stack((_body(sel[:, :2], pose), sel[:, 2]))


def altitude_band(ducts_b, lows_b, height, cruise, motion_dir, cfg: StackConfig, lidar_min_up=None):
    """Altitude target from remembered hazards near the drone or along its motion.
       hi = min(duct bottom) - duct_descend        (guide: descend 25 cm below a duct)
       lo = max(low top)     + low_climb           (guide: climb 35 cm above its top)
       z* = clip(cruise, lo, hi), then min/max altitude and the climb limit.
    Returns (z_target, speed_scale, reason)."""
    reach = cfg.drone_radius + 0.45
    ax, ay = 0.7 * math.cos(motion_dir), 0.7 * math.sin(motion_dir)

    def near(c):
        if len(c) == 0:
            return c
        d0 = np.hypot(c[:, 0], c[:, 1])
        d1 = np.hypot(c[:, 0] - ax, c[:, 1] - ay)
        return c[(d0 < reach) | (d1 < reach)]
    nd, nl = near(ducts_b), near(lows_b)
    hi = float(nd[:, 2].min()) - cfg.duct_descend if len(nd) else math.inf
    lo = float(nl[:, 2].max()) + cfg.low_climb if len(nl) else -math.inf
    zt, reason = cruise, "cruise"
    if lo > zt:
        zt, reason = min(lo, cruise + cfg.max_climb), "climb"
    if hi < zt:
        zt, reason = hi, "duck"
    if lo > hi:   # cannot pass over AND under -> hold height, let the planner route round
        zt, reason = height, "conflict"
    zt = min(max(zt, cfg.min_alt), cfg.max_alt)
    scale = cfg.vert_slow_scale if abs(zt - height) > cfg.vert_done_tol else 1.0
    return zt, scale, reason
