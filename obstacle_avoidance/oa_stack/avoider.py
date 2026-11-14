"""OA-Stack: obstacle avoidance for the zip's budget sensor stack.

Per control cycle (20 Hz suggested):
  A  lidar clean + tilt correction            lidar.clean_scan
  B  ToF zones -> levelled 3D points          tof.zones_to_points + tof.level_points
  C  classify: floor / wall / duct / low / in-band     hazards.HazardMap.classify
  D  hazard memory (odom frame, 5 s)           hazards.HazardMap.add / prune
  E  altitude band: climb over / duck under    hazards.altitude_band
  F  horizontal decision: VFH+ + braking limit + CBF safety filter + people tracking
                                               oa.ObstacleAvoider (shared core)
  G  hazards still at our height -> hard obstacles for F (until the climb / duck is done)
  H  failsafes: lidar stale, ToF stale, no height, tilt too large, blocked / stuck / emergency
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from oa import AvoidanceConfig, ObstacleAvoider

from .config import StackConfig
from .hazards import HazardMap, altitude_band
from .lidar import clean_scan
from .tof import level_points, zone_directions, zones_to_points


@dataclass
class StackOutput:
    vx: float                    # body-frame velocity setpoint (m/s)
    vy: float
    z_target: float | None       # altitude target above the floor (m)
    status: str                  # core status (+ flags)
    flags: list = field(default_factory=list)   # TOF_DOWN, NO_HEIGHT, TILT, CLIMB, DUCK, CONFLICT
    replan: bool = False
    min_clearance: float = math.inf
    info: dict = field(default_factory=dict)


class StackAvoider:
    def __init__(self, cfg: StackConfig | None = None, core_overrides: dict | None = None):
        self.cfg = cfg or StackConfig()
        c = self.cfg
        core = dict(drone_radius=c.drone_radius, margin=c.margin, v_max=c.v_max, a_brake=c.a_brake,
                    latency_s=c.latency_s, lidar_fov_deg=360.0, lidar_min_range=c.lidar_min_range,
                    lidar_max_range=c.lidar_max_range, window=c.vfh_window)
        core.update(core_overrides or {})
        self.core = ObstacleAvoider(AvoidanceConfig(**core))
        self.hazards = HazardMap(c)
        self.dirs = {"U": zone_directions(c.up_pitch_deg, c.tof_fov_deg, **c.up_calib),
                     "D": zone_directions(c.down_pitch_deg, c.tof_fov_deg, **c.down_calib)}
        self.xyz = {"U": np.array(c.up_xyz), "D": np.array(c.down_xyz)}
        self.lidar_xy = np.zeros((0, 2))
        self.lidar_pose = (0.0, 0.0, 0.0)
        self.last_tof_t = None
        self.compute_ms = 0.0

    # ------------------------------------------------------------------
    def update(self, t, pose, u_des, scan=None, tof_frames=(), height=None, roll=0.0, pitch=0.0,
               cruise_alt=1.0, grid=None):
        """
        t          : time (s)
        pose       : smooth local pose (x, y, yaw) - EKF/odometry, never a jumping SLAM pose
        u_des      : explorer's desired body velocity (vx, vy)
        scan       : (angles, ranges) of a NEW /scan, else None
        tof_frames : iterable of ('U'|'D', [64 mm values]) that arrived since the last call
        height     : EKF height above the floor (m) or None
        roll, pitch: attitude (rad, ROS FLU convention)
        cruise_alt : the explorer's flying height (m)
        grid       : optional Cartographer map (data 0..100/-1, res, origin_x, origin_y)
        """
        t0 = time.perf_counter()
        c = self.cfg
        flags = []
        u_des = np.asarray(u_des, float)

        # A. lidar
        core_scan = None
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        if scan is not None:
            a, r_h, quality, tilt, z_rel = clean_scan(scan[0], scan[1], c, roll, pitch, with_height=True)
            # wall evidence = returns truly at lidar height; a dipped beam that clipped the top
            # of a low obstacle must not hide that obstacle from the ToF guard
            ok = np.isfinite(r_h) & (z_rel >= c.low_max_dz + c.wall_evidence_margin)
            self.lidar_xy = np.column_stack((r_h[ok] * np.cos(a[ok]), r_h[ok] * np.sin(a[ok])))
            self.lidar_pose = pose
            if tilt <= c.tilt_max_deg:
                core_scan = (a, r_h)
        if tilt > c.tilt_warn_deg:
            flags.append("TILT")

        # B-D. ToF frames -> hazards
        counts = {"duct": 0, "low": 0, "inband": 0}
        if height is not None:
            lidar_now = self._lidar_in_current_frame(pose)
            for tag, mm in tof_frames:
                if tag not in self.dirs:
                    continue
                pts = zones_to_points(mm, self.dirs[tag], self.xyz[tag])
                if len(pts):
                    rng = np.linalg.norm(pts - self.xyz[tag], axis=1)
                    pts = pts[rng <= c.tof_max_range]
                pts = level_points(pts, roll, pitch)
                found = self.hazards.classify(pts, height, pose, lidar_now, grid, "up" if tag == "U" else "down")
                for k in counts:
                    counts[k] += len(found[k])
                self.hazards.add(found, pose, t)
                self.last_tof_t = t
        else:
            flags.append("NO_HEIGHT")
        self.hazards.prune(t, pose)
        tof_ok = self.last_tof_t is not None and t - self.last_tof_t <= c.tof_stale_s
        if not tof_ok:
            flags.append("TOF_DOWN")

        # E. altitude band
        ducts = self.hazards.body_items(pose, 0)
        lows = self.hazards.body_items(pose, 1)
        inband = self.hazards.body_items(pose, 2)
        speed = float(np.hypot(*u_des))
        mdir = math.atan2(u_des[1], u_des[0]) if speed > 0.05 else 0.0
        z_target, scale, reason = None, 1.0, "none"
        if height is not None:
            z_target, scale, reason = altitude_band(ducts, lows, height, cruise_alt, mdir, c)
            if reason in ("climb", "duck", "conflict"):
                flags.append(reason.upper())
        else:
            scale = 0.5                       # no height: cannot judge ducts/low obstacles -> slow
        if not tof_ok:
            scale = min(scale, 0.4)           # blind above/below the lidar slice -> slow

        # G. hazards still inside our height band are hard horizontal obstacles
        extra = [inband[:, :2]] if len(inband) else []
        if height is not None:
            top, bottom = height + c.prop_top_dz + 0.03, height + c.body_bottom_dz - 0.03
            if len(ducts):
                extra.append(ducts[ducts[:, 2] < top, :2])
            if len(lows):
                extra.append(lows[lows[:, 2] > bottom, :2])
        extra = np.vstack(extra) if extra else np.zeros((0, 2))

        # F + H. horizontal decision on the shared core
        out = self.core.update(t, pose, tuple(u_des * scale), scan=core_scan, extra_static=extra)
        if out.status == "STALE":
            flags.append("LIDAR_DOWN")
        res = StackOutput(out.vx, out.vy, z_target, out.status, flags, out.replan, out.min_clearance)
        self.compute_ms = 1000 * (time.perf_counter() - t0)
        res.info = {"compute_ms": self.compute_ms, "core_ms": out.info.get("compute_ms", 0.0),
                    "vertical": reason, "new": counts, "memory": len(self.hazards.items),
                    "extra_obstacles": len(extra)}
        return res

    def _lidar_in_current_frame(self, pose):
        """Last lidar points re-expressed in the current body frame (for the wall check)."""
        if len(self.lidar_xy) == 0:
            return self.lidar_xy
        x0, y0, th0 = self.lidar_pose
        x1, y1, th1 = pose
        c0, s0, c1, s1 = math.cos(th0), math.sin(th0), math.cos(th1), math.sin(th1)
        wx = x0 + c0 * self.lidar_xy[:, 0] - s0 * self.lidar_xy[:, 1]
        wy = y0 + s0 * self.lidar_xy[:, 0] + c0 * self.lidar_xy[:, 1]
        dx, dy = wx - x1, wy - y1
        return np.column_stack((c1 * dx + s1 * dy, -s1 * dx + c1 * dy))
