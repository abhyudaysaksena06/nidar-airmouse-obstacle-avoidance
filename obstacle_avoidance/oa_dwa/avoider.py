"""Algo 1 - layered costmap + DWA obstacle avoidance (Layer 5: fallbacks + output).

Pipeline (mirrors the team's tagging README structure):
  L1 robust measurement (perception.py) -> L2 time-aligned transforms (frames.py)
  -> L3 local costmap with candidate/confirmed obstacles (costmap.py)
  -> L4 DWA with temporal consistency (dwa.py) -> L5 fallbacks, status, reports (here)

Same call signature as Algo 2 (oa.ObstacleAvoider), so the planner,
benchmark and simulator can swap them.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .config import DWAConfig
from .costmap import LocalCostmap
from .dwa import DWAPlanner
from .frames import FrameAdapter, PoseBuffer, body_to_odom, odom_to_body, wrap
from .perception import filter_depth, filter_scan
from .subgoal import local_subgoal, segment_blocked

log = logging.getLogger("nidar.oa_dwa")  # swap for cv.logger at integration

OK, SLOWED, AVOIDING, BLOCKED, STUCK, EMERGENCY, STALE, DEGRADED, IDLE = (
    "OK", "SLOWED", "AVOIDING", "BLOCKED", "STUCK", "EMERGENCY", "STALE", "DEGRADED", "IDLE")


def _closest_per_bearing(pts: np.ndarray, deg: float) -> np.ndarray:
    """Keep the nearest point in each `deg` bearing bin."""
    if len(pts) == 0:
        return pts
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    dist = np.hypot(pts[:, 0], pts[:, 1])
    k = np.floor((ang + np.pi) / math.radians(deg)).astype(int)
    order = np.lexsort((dist, k))
    ks = k[order]
    first = np.ones(len(order), dtype=bool)
    first[1:] = ks[1:] != ks[:-1]
    return pts[order[first]]


@dataclass
class AvoidanceOutput:
    """Command + status for the flight controller, planner and GCS."""

    vx: float                       # body-frame velocity (team body frame)
    vy: float
    status: str
    yaw_rate: float | None = None   # DWA always commands yaw (drone faces travel)
    z_target: float | None = None
    replan: bool = False
    min_clearance: float = math.inf
    direction: float = 0.0
    info: dict = field(default_factory=dict)


class DWAAvoider:
    """Algo 1. Call update() every control cycle."""

    def __init__(self, cfg: DWAConfig | None = None) -> None:
        self.cfg = cfg or DWAConfig()
        self.frames = FrameAdapter(self.cfg.body_frame)
        self.poses = PoseBuffer(self.cfg.pose_buffer_s)
        self.costmap = LocalCostmap(self.cfg)
        self.dwa = DWAPlanner(self.cfg)
        self.last_scan_t: float | None = None
        self.lidar_quality = 1.0
        self.stuck_since: float | None = None
        self.v = 0.0
        self.w = 0.0
        self.blocked_events: list[dict] = []
        self._sub_odom: np.ndarray | None = None
        self._sub_t = -1e9
        self.compute_ms = 0.0

    # ------------------------------------------------------------------
    def update(self, t: float, pose: tuple[float, float, float], u_des: tuple[float, float],
               scan: tuple[np.ndarray, np.ndarray] | None = None, scan_t: float | None = None,
               depth_points: np.ndarray | None = None, depth_t: float | None = None,
               z: float | None = None, **_unused) -> AvoidanceOutput:
        """
        t            : current time (s)
        pose         : odometry pose (x, y, yaw) - smooth local frame, never the SLAM pose
        u_des        : desired body velocity (vx, vy) from the planner / pure pursuit
        scan         : (angles, ranges) when a NEW lidar scan arrived, else None
        scan_t       : capture time of that scan (default t)  -> time alignment
        depth_points : optional (N,3) depth-camera points, body frame, z relative to drone
        depth_t      : capture time of the depth frame
        z            : height above floor (needed to reject floor depth points)
        """
        t0 = time.perf_counter()
        cfg, fr = self.cfg, self.frames
        pose = fr.pose_in(pose)
        u_des = fr.vec_in(u_des)
        self.poses.add(t, pose)
        self.costmap.recentre(pose[0], pose[1])

        # L1 + L2: measurements placed with the pose at THEIR capture time
        if scan is not None:
            ts = t if scan_t is None else scan_t
            p_s = self.poses.at(ts) or pose
            pts, ang, rng, self.lidar_quality = filter_scan(fr.angles_in(scan[0]), scan[1], cfg)
            hits = body_to_odom(pts, p_s) if len(pts) else np.zeros((0, 2))
            # free-space clearing along valid beams and along no-return beams (to max range)
            a_all = fr.angles_in(scan[0])
            r_all = np.asarray(scan[1], float)
            fov = np.abs(a_all) <= math.radians(cfg.lidar_fov_deg) / 2
            r_free = np.where(np.isfinite(r_all), np.minimum(r_all, cfg.map_size / 2),
                              min(cfg.lidar_max_range, cfg.map_size / 2))[fov][::2]
            a_free = a_all[fov][::2]
            ends = body_to_odom(np.column_stack((r_free * np.cos(a_free), r_free * np.sin(a_free))), p_s)
            self.costmap.integrate(ts, np.array(p_s[:2]), hits, ends)
            self.last_scan_t = t
        depth_ok = depth_points is not None
        if depth_ok:
            td = t if depth_t is None else depth_t
            p_d = self.poses.at(td) or pose
            dp = filter_depth(fr.points_in(depth_points) if len(depth_points) else depth_points,
                              z if z is not None else 1.0, cfg)
            if len(dp):
                self.costmap.integrate(td, np.array(p_d[:2]), body_to_odom(dp, p_d))

        # L5 fallback: lidar not reporting -> hover
        if self.last_scan_t is None or t - self.last_scan_t > cfg.scan_stale_s:
            return self._done(AvoidanceOutput(0.0, 0.0, STALE, yaw_rate=0.0, replan=False), t0)

        obs_odom = self.costmap.obstacle_points(np.array(pose[:2]), cfg.obstacle_range)
        obs = odom_to_body(obs_odom, pose) if len(obs_odom) else np.zeros((0, 2))
        obs = _closest_per_bearing(obs, 2.0)  # bounded DWA cost on a Pi
        dmin = float(np.min(np.hypot(*obs.T))) if len(obs) else math.inf

        # EMERGENCY: something inside our body + 5 cm -> back away slowly
        if dmin < cfg.drone_radius + cfg.emergency_dist:
            p = obs[int(np.argmin(np.hypot(*obs.T)))]
            n = -p / max(np.hypot(*p), 1e-6) * cfg.backoff_speed
            self.v, self.w = 0.0, 0.0
            return self._done(self._out(n[0], n[1], 0.0, EMERGENCY, True, dmin), t0)

        speed = float(np.hypot(*u_des))
        if speed < 0.05:
            self.stuck_since = None
            self.v, self.w = 0.0, 0.0
            return self._done(self._out(0.0, 0.0, 0.0, IDLE, False, dmin), t0)

        target = math.atan2(u_des[1], u_des[0])
        goal = cfg.goal_distance * np.array([math.cos(target), math.sin(target)])
        # L4a: goal hidden behind an obstacle -> local A* sub-goal (Nav2 structure)
        detour = False
        if segment_blocked(goal, obs, cfg.r_inf + 0.05):
            if self._sub_odom is None or t - self._sub_t > 0.5:
                sg, _ = local_subgoal(goal, obs, cfg)
                self._sub_odom = None if sg is None else body_to_odom(sg[None, :], pose)[0]
                self._sub_t = t
            if self._sub_odom is not None:
                goal = odom_to_body(self._sub_odom[None, :], pose)[0]
                detour = True
        else:
            self._sub_odom = None
        v, w, info = self.dwa.plan(obs, goal, self.v, self.w, speed)
        info["detour"] = detour
        if v is not None and info.get("n_moving_admissible", 1) == 0:
            v = None  # only "spin in place" is safe: no way forward -> BLOCKED
        if v is None:  # nothing admissible -> stop, rotate to search, ask to replan
            self.v, self.w = 0.0, float(np.sign(target) or 1.0) * cfg.rotate_search_w
            status, replan = BLOCKED, True
            self._blocked(t, pose)
            vx, vy, w = 0.0, 0.0, self.w
        else:
            if abs(v) < 0.03 and abs(w) < 0.1 and len(obs):
                # stopped facing an obstacle (head-on local minimum): turn
                # towards the side with more free space so new arcs open up
                left = obs[obs[:, 1] > 0]
                right = obs[obs[:, 1] <= 0]
                dl = np.hypot(*left.T).min() if len(left) else np.inf
                dr = np.hypot(*right.T).min() if len(right) else np.inf
                w = cfg.rotate_search_w if dl >= dr else -cfg.rotate_search_w
            self.v, self.w = v, w
            vx, vy = v, 0.0
            heading_err = abs(float(wrap(target)))
            status = OK if v >= 0.9 * min(speed, cfg.v_max) and heading_err < 0.35 else \
                (AVOIDING if heading_err >= 0.35 or abs(w) > 0.5 else SLOWED)
            replan = False
        if status in (OK, SLOWED, AVOIDING) and self.lidar_quality < 0.8:
            status = DEGRADED
        # STUCK: asked to move, cannot make way
        if speed > 0.2 and abs(self.v) < 0.05:
            if self.stuck_since is None:
                self.stuck_since = t
            elif t - self.stuck_since > cfg.stuck_s:
                status, replan = STUCK, True
        else:
            self.stuck_since = None
        out = self._out(vx, vy, w, status, replan, dmin)
        out.direction = float(fr.s * target)
        out.info.update(info)
        out.info["depth_used"] = depth_ok
        return self._done(out, t0)

    # ------------------------------------------------------------------
    def _out(self, vx, vy, w, status, replan, dmin) -> AvoidanceOutput:
        ox, oy, ow = self.frames.vel_out(float(vx), float(vy), float(w))
        return AvoidanceOutput(ox, oy, status, yaw_rate=ow, replan=replan,
                               min_clearance=dmin - self.cfg.drone_radius)

    def _blocked(self, t: float, pose) -> None:
        x, y = self.frames.xy_out(pose[0], pose[1])
        if not self.blocked_events or t - self.blocked_events[-1]["t"] > 2.0:
            self.blocked_events.append({"t": round(t, 2), "x": round(x, 2), "y": round(y, 2)})
            log.info("path BLOCKED at (%.2f, %.2f)", x, y)

    def _done(self, out: AvoidanceOutput, t0: float) -> AvoidanceOutput:
        self.compute_ms = 1000 * (time.perf_counter() - t0)
        out.info["compute_ms"] = self.compute_ms
        return out

    def report(self) -> dict:
        """For mapping (Jainil) / GCS: confirmed obstacles + blocked events."""
        obs = self.costmap.obstacle_report()
        for o in obs:
            o["centroid"] = list(self.frames.xy_out(*o["centroid"]))
        return {"obstacles": obs, "blocked_events": list(self.blocked_events),
                "lidar_quality": round(self.lidar_quality, 2)}
