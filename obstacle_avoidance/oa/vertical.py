"""Vertical guard: obstacles above/below the 2D lidar plane.

A 2D lidar only sees obstacles crossing its scan plane. Two single-point ToF
sensors pitched +-theta from horizontal (plus an upward ToF) sample the band
just above and below. A forward beam of range d hits at height
    z_hit = z +- d * sin(theta),  horizontal distance  d * cos(theta).
If z_hit lies inside the drone's height band [z - hh - c, z + hh + c] and the
lidar does NOT see anything at that horizontal distance (i.e. it is not just
a wall), it is a low box / hanging beam: we remember the point and keep the
altitude target above low points and below hanging ones while near them.
"""
import math


class VerticalGuard:
    def __init__(self, cfg):
        self.cfg = cfg
        self.low, self.hang = [], []   # (x, y, z_hit) in odom frame
        self.z_target = None

    def update(self, tof, z, pose, front_range, cruise_alt):
        """tof: dict with fwd_down, fwd_up, up (m, inf if no return).
        Returns (z_target, speed_scale)."""
        cfg = self.cfg
        th = math.radians(cfg.tof_pitch_deg)
        hh, c = cfg.half_height, cfg.vertical_clearance
        for key, sign, store in (("fwd_down", -1, self.low), ("fwd_up", 1, self.hang)):
            d = tof.get(key, math.inf)
            if not math.isfinite(d):
                continue
            z_hit, horiz = z + sign * d * math.sin(th), d * math.cos(th)
            if front_range < horiz + 0.35:
                continue  # lidar sees it too -> a wall, handled horizontally
            if abs(z_hit - z) < hh + c and z_hit > 0.1:
                store.append((pose[0] + horiz * math.cos(pose[2]), pose[1] + horiz * math.sin(pose[2]), z_hit))
                del store[:-400]
        ax, ay = pose[0] + 0.6 * math.cos(pose[2]), pose[1] + 0.6 * math.sin(pose[2])
        reach = cfg.drone_radius + 0.45

        def near(pts):
            return [zz for x, y, zz in pts if math.hypot(x - pose[0], y - pose[1]) < reach
                    or math.hypot(x - ax, y - ay) < reach]
        lo = max(near(self.low), default=-1.0) + hh + c
        hi = min(near(self.hang), default=99.0) - hh - c
        up = tof.get("up", math.inf)
        if math.isfinite(up):
            hi = min(hi, z + up - hh - c)
        zt = max(cruise_alt, lo)
        if hi >= cfg.min_alt:
            zt = min(zt, hi)
        self.z_target = max(cfg.min_alt, zt)
        # do not fly into the obstacle before the climb / duck is done
        scale = 0.25 if abs(self.z_target - z) > 0.2 else 1.0
        return self.z_target, scale
