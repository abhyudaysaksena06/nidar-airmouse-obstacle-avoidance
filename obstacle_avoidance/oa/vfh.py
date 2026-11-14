"""VFH+ direction selection (Ulrich & Borenstein, 1998), adapted to a
holonomic multirotor: we choose a free *direction of travel*, not a steering
angle, so no trajectory masking is needed."""
import math

import numpy as np

from .scan import wrap


class VFHPlus:
    def __init__(self, cfg):
        self.cfg = cfg
        self.n = int(round(360.0 / cfg.sector_deg))
        self.centers = wrap(np.arange(self.n) * 2 * np.pi / self.n)
        self.binary = np.zeros(self.n, dtype=bool)
        self.prev = 0.0
        self.side, self.side_t = 0, -1e9   # committed detour side (+1 left, -1 right)
        self.H = np.zeros(self.n)

    def histogram(self, pts):
        """Polar obstacle histogram. Each obstacle point at distance d and
        bearing b has magnitude m = (1 - d/W)^2 and covers every sector within
        the enlargement angle gamma = asin(r_safe / d) of b (robot treated as a
        point). A sector takes the MAX magnitude over its points, so the
        thresholds are distances: m > tau  <=>  d < W (1 - sqrt(tau))."""
        cfg = self.cfg
        H = np.zeros(self.n)
        if len(pts):
            d = np.hypot(pts[:, 0], pts[:, 1])
            sel = d < cfg.window
            d, p = d[sel], pts[sel]
            if len(d):
                b = np.arctan2(p[:, 1], p[:, 0])
                m = (1.0 - d / cfg.window) ** 2
                gamma = np.arcsin(np.clip(cfg.r_safe / np.maximum(d, 1e-3), 0, 1))
                half = math.radians(cfg.sector_deg) / 2
                diff = np.abs(wrap(self.centers[None, :] - b[:, None]))
                H = (m[:, None] * (diff <= gamma[:, None] + half)).max(axis=0)
        self.H = H
        # binary histogram with hysteresis
        self.binary = np.where(H > cfg.tau_high, True, np.where(H < cfg.tau_low, False, self.binary))
        return self.binary

    def select(self, target, target_clear=False, t=0.0):
        """Free direction closest to `target` (rad, body frame), also
        preferring the current heading (0) and the previous choice.
        target_clear=True: the caller verified the target corridor is free
        for the stopping distance -> keep the target even if far obstacles
        mark its sector. Returns None if no acceptable direction exists."""
        cfg = self.cfg
        k_t = int(np.argmin(np.abs(wrap(self.centers - target))))
        if target_clear or not self.binary[k_t]:
            self.prev = target
            return target
        free = np.flatnonzero(~self.binary)
        if free.size == 0:
            return None
        c = self.centers[free]
        dt_ = np.abs(wrap(c - target))
        ok = dt_ <= math.radians(cfg.max_detour_deg)
        if not ok.any():
            return None
        cost = cfg.mu_target * dt_ + cfg.mu_heading * np.abs(wrap(c)) + cfg.mu_prev * np.abs(wrap(c - self.prev))
        cost[~ok] = np.inf
        if self.side and t - self.side_t < cfg.commit_s:  # stay on the committed side
            same = np.sign(wrap(c - target)) == self.side
            if np.isfinite(cost[same]).any():
                cost[~same] = np.inf
        best = float(c[int(np.argmin(cost))])
        side = int(np.sign(wrap(best - target)))
        if side != self.side or t - self.side_t >= cfg.commit_s:
            self.side, self.side_t = side, t
        self.prev = best
        return best


def corridor_clearance(pts, direction, width):
    """Distance travelled along `direction` before a point comes within
    `width` laterally (free length of a corridor of half-width `width`)."""
    if len(pts) == 0:
        return math.inf
    u = np.array([math.cos(direction), math.sin(direction)])
    lon = pts @ u
    lat = np.abs(pts @ np.array([-u[1], u[0]]))
    # a wall we are already flying alongside (closer than `width` beside us)
    # is not "ahead": shrink the corridor to the current side clearance
    beside = np.abs(lon) < 0.3
    if beside.any():
        width = min(width, max(0.05, float(lat[beside].min()) - 0.02))
    hit = (lon > 0) & (lat < width)
    if not hit.any():
        return math.inf
    return float((lon[hit] - np.sqrt(np.maximum(width ** 2 - lat[hit] ** 2, 0))).min())
