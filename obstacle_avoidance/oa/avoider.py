"""ObstacleAvoider: the full local-avoidance pipeline.

    planner / pure pursuit  --u_des-->  [ObstacleAvoider]  --u_safe-->  flight controller

    1. scan -> body points (FOV-limited)           scan.py
    2. moving-obstacle tracking (alpha-beta)       tracker.py
    3. obstacle memory for blind sectors            scan.py
    4. EMERGENCY check (already too close)
    5. VFH+ : free direction nearest to u_des       vfh.py
    6. braking-distance speed limit along it
    7. CBF safety filter (hard clearance guarantee) cbf.py
    8. FOV guard + yaw hint for limited-FOV lidars
    9. failsafes: STALE scan, BLOCKED, STUCK -> replan request
   10. vertical guard from forward ToF beams        vertical.py
"""
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .cbf import filter_velocity, sector_reduce
from .config import AvoidanceConfig
from .scan import ObstacleMemory, scan_to_points, wrap
from .tracker import DynamicTracker
from .points3d import PointCloudFrontEnd
from .vertical import VerticalGuard
from .vfh import VFHPlus, corridor_clearance

# status codes (also a good set of states for the GCS / event log)
OK, AVOIDING, SLOWED, BLOCKED, STUCK, EMERGENCY, STALE, IDLE = (
    "OK", "AVOIDING", "SLOWED", "BLOCKED", "STUCK", "EMERGENCY", "STALE", "IDLE")


@dataclass
class AvoidanceOutput:
    vx: float                      # body-frame velocity command (m/s)
    vy: float
    status: str
    yaw_rate: float = None         # suggested yaw rate (only for limited-FOV lidars)
    z_target: float = None         # altitude target from the vertical guard (if ToF given)
    replan: bool = False           # ask the global planner for a new path
    min_clearance: float = math.inf
    direction: float = 0.0         # chosen travel direction (rad, body)
    info: dict = field(default_factory=dict)


class ObstacleAvoider:
    def __init__(self, cfg: AvoidanceConfig = None):
        self.cfg = cfg or AvoidanceConfig()
        self.vfh = VFHPlus(self.cfg)
        self.tracker = DynamicTracker(self.cfg)
        self.memory = ObstacleMemory(self.cfg)
        self.vertical = VerticalGuard(self.cfg)
        self.cloud = PointCloudFrontEnd(self.cfg)
        self.p3_horiz = np.zeros((0, 2))       # 3D-sensor obstacles at our height (odom frame)
        self.p3_low = np.zeros((0, 3))         # (x, y, top_abs) odom
        self.p3_hang = np.zeros((0, 3))        # (x, y, bottom_abs) odom
        self.p3_t = -1e9
        self.last_scan_t = None
        self.last_scan_pts = np.zeros((0, 2))
        self.last_scan_pose = (0.0, 0.0, 0.0)
        self.moving_mask = np.zeros(0, dtype=bool)
        self.stuck_since = None
        self.slow_since = None
        self.base_margin = self.cfg.margin
        self.u_prev = np.zeros(2)
        self.compute_ms = 0.0

    # ------------------------------------------------------------------
    def update(self, t, pose, u_des, scan=None, tof=None, z=None, cruise_alt=None, points3d=None,
               points3d_vfov=(-0.5, 0.5), extra_static=None, **_):
        """
        t       : time (s)
        pose    : (x, y, yaw) odometry pose (any consistent local frame)
        u_des   : (vx, vy) desired BODY-frame velocity from the planner
        scan    : (angles, ranges) body-frame lidar scan if a NEW scan
                  arrived this step, else None
        tof     : optional {"fwd_down", "fwd_up", "up"} ranges for the vertical guard
        z       : current height estimate (needed with tof / points3d)
        points3d: optional NEW 3D points (N,3) body frame, z relative to the drone,
                  from a depth camera or multi-zone ToF sensors (see points3d.py)
        extra_static: optional (M,2) body-frame obstacle points from another
                  front-end (e.g. oa_stack's ToF in-band hazards) - treated like lidar walls
        """
        t0 = time.perf_counter()
        cfg = self.cfg
        u_des = np.asarray(u_des, float)
        dt = 0.0 if self.last_scan_t is None else t - self.last_scan_t

        if scan is not None:
            pts, _, _ = scan_to_points(scan[0], scan[1], cfg)
            self.moving_mask = self.tracker.update(pts, pose, max(dt, 1e-3)) if cfg.use_tracker                 else np.zeros(len(pts), dtype=bool)
            if cfg.use_memory:
                self.memory.add(pts, pose, t)
            self.last_scan_pts, self.last_scan_pose, self.last_scan_t = pts, pose, t

        if points3d is not None and z is not None:
            h, lo, hg = self.cloud.split(points3d, z, points3d_vfov)
            c, s_ = math.cos(pose[2]), math.sin(pose[2])

            def to_odom(q):
                return np.column_stack((pose[0] + c * q[:, 0] - s_ * q[:, 1], pose[1] + s_ * q[:, 0] + c * q[:, 1]))
            self.p3_horiz = to_odom(h) if len(h) else np.zeros((0, 2))
            self.p3_low = np.column_stack((to_odom(lo[:, :2]), z + lo[:, 2])) if len(lo) else np.zeros((0, 3))
            self.p3_hang = np.column_stack((to_odom(hg[:, :2]), z + hg[:, 2])) if len(hg) else np.zeros((0, 3))
            self.p3_t = t
            if cfg.use_memory and len(h):
                self.memory.add(h, pose, t)

        if self.last_scan_t is None or t - self.last_scan_t > cfg.scan_stale_s:
            return self._done(AvoidanceOutput(0.0, 0.0, STALE), t0)

        # current obstacles in the CURRENT body frame (compensate motion since the scan)
        pts = self._rebase(self.last_scan_pts, self.last_scan_pose, pose)
        static = pts[~self.moving_mask] if len(self.moving_mask) == len(pts) else pts
        fresh3d = t - self.p3_t < 0.5
        if fresh3d:
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            band = cfg.half_height + cfg.vertical_clearance * 0.6
            blocking = [self.p3_horiz]
            if z is not None:  # not yet climbed over / ducked under -> still an obstacle
                if len(self.p3_low):
                    blocking.append(self.p3_low[self.p3_low[:, 2] > z - band, :2])
                if len(self.p3_hang):
                    blocking.append(self.p3_hang[self.p3_hang[:, 2] < z + band, :2])
            B = np.vstack(blocking) if blocking else np.zeros((0, 2))
            if len(B):
                dx, dy = B[:, 0] - pose[0], B[:, 1] - pose[1]
                static = np.vstack((static, np.column_stack((c * dx + s_ * dy, -s_ * dx + c * dy))))
        if cfg.use_memory:
            static = np.vstack((static, self.memory.outside_fov(pose)))
        if extra_static is not None and len(extra_static):
            static = np.vstack((static, np.asarray(extra_static, float).reshape(-1, 2)))
        # closest point per 1 deg: all the avoider needs, bounded cost on a Pi
        static = sector_reduce(static, 1.0)
        moving = self.tracker.moving_obstacles(pose) if cfg.use_tracker else []
        all_pts = np.vstack([static] + [self._circle_pts(c, r) for c, _, r in moving]) if moving else static
        dmin = float(np.min(np.hypot(*all_pts.T))) if len(all_pts) else math.inf

        # vertical guard
        z_target, v_scale = None, 1.0
        if fresh3d and z is not None:   # 3D sensor: climb over / duck under from the cloud
            c, s_ = math.cos(pose[2]), math.sin(pose[2])

            def to_body(q):
                if len(q) == 0:
                    return np.zeros((0, 3))
                dx, dy = q[:, 0] - pose[0], q[:, 1] - pose[1]
                return np.column_stack((c * dx + s_ * dy, -s_ * dx + c * dy, q[:, 2] - z))
            mdir = math.atan2(u_des[1], u_des[0]) if np.hypot(*u_des) > 0.05 else 0.0
            up = tof.get("up") if tof else None
            z_target, v_scale = self.cloud.altitude_band(to_body(self.p3_low), to_body(self.p3_hang),
                                                         z, cruise_alt or z, mdir, up)
        elif tof is not None and z is not None:
            fr = pts[(np.abs(np.arctan2(pts[:, 1], pts[:, 0])) < math.radians(5))] if len(pts) else pts
            front = float(np.min(np.hypot(*fr.T))) if len(fr) else math.inf
            z_target, v_scale = self.vertical.update(tof, z, pose, front, cruise_alt or z)

        # EMERGENCY: already inside the body + small margin -> back away
        if dmin < cfg.drone_radius + cfg.emergency_dist:
            p = all_pts[int(np.argmin(np.hypot(*all_pts.T)))]
            n = p / max(np.hypot(*p), 1e-6)
            # escape faster than anything closing in on us, then let the CBF
            # make sure the escape does not run into something else
            closing = max([0.0] + [float(-(c / max(np.hypot(*c), 1e-6)) @ v) for c, v, _ in moving])
            away = -n * min(cfg.v_max, cfg.escape_speed + closing)
            if cfg.use_cbf:
                away, _ = filter_velocity(away, static, moving, cfg)
            self.u_prev = away
            return self._done(AvoidanceOutput(float(away[0]), float(away[1]), EMERGENCY, replan=True,
                                              z_target=z_target, min_clearance=dmin - cfg.drone_radius), t0)

        speed_des = float(np.hypot(*u_des))
        # narrow-passage mode: slow progress -> shrink the margin, cap the speed
        if self.slow_since is not None and t - self.slow_since > cfg.squeeze_after_s:
            cfg.margin = max(cfg.min_margin, self.base_margin - cfg.squeeze_rate * (t - self.slow_since - cfg.squeeze_after_s))
        else:
            cfg.margin = self.base_margin
        squeezing = cfg.margin < self.base_margin - 1e-9
        if squeezing:
            speed_des = min(speed_des, cfg.squeeze_speed)
            u_des = u_des * (speed_des / max(np.hypot(*u_des), 1e-9))
        if speed_des < 0.05:
            self.stuck_since = None
            u = np.zeros(2)
            status = IDLE
            direction = 0.0
        else:
            target = math.atan2(u_des[1], u_des[0])
            # predicted positions of moving obstacles also block directions
            hist_pts = all_pts
            if moving:
                pred = [self._circle_pts(c + v * k * cfg.predict_s / 3, r) for c, v, r in moving for k in (1, 2, 3)]
                hist_pts = np.vstack([all_pts] + pred)
            if cfg.use_vfh:
                # target direction is fine if its corridor is free for the
                # stopping distance + margin (far walls must not force detours)
                need = speed_des ** 2 / (2 * cfg.a_brake) + speed_des * cfg.latency_s + cfg.margin + 0.6
                clear = corridor_clearance(hist_pts, target, cfg.r_safe) >= need
                self.vfh.histogram(hist_pts)
                direction = self.vfh.select(target, target_clear=clear, t=t)
            else:
                direction = target
            if direction is None:
                status, u, direction = BLOCKED, np.zeros(2), target
            else:
                status = OK if abs(wrap(direction - target)) < 1e-6 else AVOIDING
                # speed: braking distance along the chosen corridor
                free = corridor_clearance(hist_pts, direction, cfg.r_safe)
                usable = max(0.0, free - cfg.margin)
                v_brake = math.sqrt(2 * cfg.a_brake * usable)
                v = min(speed_des, cfg.v_max, v_brake) * v_scale
                if status == AVOIDING:  # big detours slower
                    v *= max(0.4, math.cos(0.5 * abs(wrap(direction - target))))
                if v < speed_des - 1e-3 and status == OK:
                    status = SLOWED
                u = v * np.array([math.cos(direction), math.sin(direction)])
            # hard guarantee
            if cfg.use_cbf:
                u, qp = filter_velocity(u, static, moving, cfg, latency_speed=float(np.hypot(*self.u_prev)))
                if qp["active"] and status == OK:
                    status = SLOWED
        # FOV guard: a front lidar must look where it flies
        yaw_rate = None
        if cfg.lidar_fov_deg < 359:
            sp = float(np.hypot(*u))
            if sp > 0.05:
                ang = math.atan2(u[1], u[0])
                yaw_rate = float(np.clip(cfg.yaw_gain * ang, -cfg.yaw_rate_max, cfg.yaw_rate_max))
                lim = math.radians(cfg.lidar_fov_deg) / 2 - math.radians(cfg.fov_guard_deg)
                if abs(ang) > lim:
                    u = u * max(0.0, math.cos(abs(ang) - lim))
        # progress monitor (drives narrow-passage mode)
        if speed_des > 0.2 and np.hypot(*u) < 0.3 * speed_des and status != EMERGENCY:
            if self.slow_since is None:
                self.slow_since = t
        elif speed_des > 0.2 and np.hypot(*u) > 0.6 * speed_des:
            self.slow_since = None
        if squeezing:
            status = status + "+SQUEEZE" if status in (AVOIDING, SLOWED, OK) else status
        # STUCK: we want to move but cannot
        replan = status == BLOCKED
        if speed_des > 0.2 and np.hypot(*u) < 0.05:
            if self.stuck_since is None:
                self.stuck_since = t
            elif t - self.stuck_since > cfg.stuck_s:
                status, replan = STUCK, True
        else:
            self.stuck_since = None
        self.u_prev = u
        return self._done(AvoidanceOutput(float(u[0]), float(u[1]), status, yaw_rate=yaw_rate,
                                          z_target=z_target, replan=replan,
                                          min_clearance=dmin - cfg.drone_radius, direction=direction), t0)

    # ------------------------------------------------------------------
    def _done(self, out, t0):
        self.compute_ms = 1000 * (time.perf_counter() - t0)
        out.info["compute_ms"] = self.compute_ms
        return out

    @staticmethod
    def _rebase(pts, pose_then, pose_now):
        if len(pts) == 0 or pose_then == pose_now:
            return pts
        x0, y0, th0 = pose_then
        x1, y1, th1 = pose_now
        c0, s0 = math.cos(th0), math.sin(th0)
        wx, wy = x0 + c0 * pts[:, 0] - s0 * pts[:, 1], y0 + s0 * pts[:, 0] + c0 * pts[:, 1]
        c1, s1 = math.cos(th1), math.sin(th1)
        dx, dy = wx - x1, wy - y1
        return np.column_stack((c1 * dx + s1 * dy, -s1 * dx + c1 * dy))

    @staticmethod
    def _circle_pts(c, r, n=8):
        a = np.arange(n) * 2 * np.pi / n
        return np.column_stack((c[0] + r * np.cos(a), c[1] + r * np.sin(a)))
