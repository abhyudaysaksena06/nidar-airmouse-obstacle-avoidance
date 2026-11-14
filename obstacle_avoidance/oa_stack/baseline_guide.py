"""Baseline: the obstacle handling described in the zip's guide (Sections 9.6, 13),
re-implemented with the same inputs so it can be compared fairly.

  * LiDAR proximity avoidance: push away from any return closer than 0.55 m
    (like ArduPilot AVOID_MARGIN), plus the explorer's slow-down near walls (< ~0.6 m)
  * UP ToF guard (guide classify 'duct'): nearest point 6-34 cm above base_link, < 1 m
    ahead, not on a mapped wall -> descend 25 cm below it and slow down
  * DOWN ToF guard ('low'): point > 20 cm above the floor, < 1.3 m ahead, below the lidar
    plane (dz < 0.05), not on a mapped wall -> climb 35 cm above its top, remember the spot
No memory beyond the 'climb zone', no in-band class, no braking model, no CBF.
"""
from __future__ import annotations

import math

import numpy as np

from .config import StackConfig
from .hazards import near_map_wall, near_points
from .tof import level_points, zone_directions, zones_to_points


class GuideBaseline:
    def __init__(self, cfg: StackConfig | None = None, push_margin=0.55):
        self.cfg = cfg or StackConfig()
        c = self.cfg
        self.dirs = {"U": zone_directions(c.up_pitch_deg, c.tof_fov_deg, **c.up_calib),
                     "D": zone_directions(c.down_pitch_deg, c.tof_fov_deg, **c.down_calib)}
        self.xyz = {"U": np.array(c.up_xyz), "D": np.array(c.down_xyz)}
        self.push_margin = push_margin
        self.scan = None
        self.duct_until = -1e9
        self.duct_z = None
        self.climb_zones = []   # (x, y, top_abs) odom

    def update(self, t, pose, u_des, scan=None, tof_frames=(), height=None, roll=0.0, pitch=0.0,
               cruise_alt=1.0, grid=None):
        c = self.cfg
        vx, vy = float(u_des[0]), float(u_des[1])
        if scan is not None:
            self.scan = (np.asarray(scan[0], float), np.asarray(scan[1], float))
        lidar_xy = np.zeros((0, 2))
        if self.scan is not None:
            a, r = self.scan
            ok = np.isfinite(r) & (r >= c.lidar_min_range)
            lidar_xy = np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))
            # proximity push-away + slow-down near walls
            d = np.hypot(*lidar_xy.T) if len(lidar_xy) else np.array([np.inf])
            close = d < self.push_margin
            if close.any():
                n = -(lidar_xy[close] / d[close, None]).sum(0)
                n /= max(np.hypot(*n), 1e-6)
                push = 1.2 * float((self.push_margin - d[close]).max())
                vx, vy = vx + push * n[0], vy + push * n[1]
            dmin = float(d.min())
            slow = float(np.clip((dmin - c.drone_radius) / 0.35, 0.2, 1.0))
            vx, vy = vx * slow, vy * slow
        z_target = cruise_alt
        if height is not None:
            for tag, mm in tof_frames:
                pts = level_points(zones_to_points(mm, self.dirs[tag], self.xyz[tag]), roll, pitch)
                if len(pts) == 0:
                    continue
                pts = pts[np.linalg.norm(pts - self.xyz[tag], axis=1) <= c.tof_max_range]
                hz = np.hypot(pts[:, 0], pts[:, 1])
                if tag == "U":
                    sel = (pts[:, 2] > c.up_min_dz) & (pts[:, 2] < c.up_max_dz) & (hz < c.up_reach)
                    rad = c.wall_radius_up
                else:
                    sel = ((height + pts[:, 2]) > c.low_min_h) & (hz < c.low_reach) & (pts[:, 2] < c.low_max_dz)
                    rad = c.wall_radius_down
                if sel.any():
                    idx = np.where(sel)[0]
                    wall = near_points(pts[idx, :2], lidar_xy, rad)
                    if grid is not None:
                        th = pose[2]
                        xy = np.column_stack((pose[0] + math.cos(th) * pts[idx, 0] - math.sin(th) * pts[idx, 1],
                                              pose[1] + math.sin(th) * pts[idx, 0] + math.cos(th) * pts[idx, 1]))
                        wall |= near_map_wall(xy, grid, rad, c.occ_threshold)
                    sel[idx[wall]] = False
                if not sel.any():
                    continue
                if tag == "U":
                    self.duct_z = height + float(pts[sel, 2].min()) - c.duct_descend
                    self.duct_until = t + 2.0
                else:
                    top = height + float(pts[sel, 2].max())
                    i = int(np.argmin(np.where(sel, hz, np.inf)))
                    th = pose[2]
                    self.climb_zones.append((pose[0] + math.cos(th) * pts[i, 0] - math.sin(th) * pts[i, 1],
                                             pose[1] + math.sin(th) * pts[i, 0] + math.cos(th) * pts[i, 1], top))
        for x, y, top in self.climb_zones:
            if math.hypot(x - pose[0], y - pose[1]) < 0.8:
                z_target = max(z_target, top + c.low_climb)
        if t < self.duct_until and self.duct_z is not None:
            z_target = min(z_target, self.duct_z)
            vx, vy = vx * 0.5, vy * 0.5
        z_target = min(max(z_target, c.min_alt), c.max_alt)
        return {"vx": vx, "vy": vy, "z_target": z_target}
