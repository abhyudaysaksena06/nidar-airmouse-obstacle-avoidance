"""Moving-obstacle tracking: scan segmentation + alpha-beta filtered tracks.

Only compact clusters (people, rolling debris, small boxes) are tracked; long
clusters are walls whose apparent centroid moves as more of them comes into
view, so tracking them would invent fake velocities.
"""
import math

import numpy as np

from .scan import body_to_world, world_to_body


def segment(pts, gap):
    """Split angle-ordered scan points at range jumps > gap. Returns index lists."""
    if len(pts) == 0:
        return []
    jumps = np.hypot(*np.diff(pts, axis=0).T) > gap
    cuts = np.flatnonzero(jumps) + 1
    segs = np.split(np.arange(len(pts)), cuts)
    # a 360 deg scan wraps around: merge last and first segment if they touch
    if len(segs) > 1 and np.hypot(*(pts[0] - pts[-1])) <= gap:
        segs[0] = np.concatenate((segs[-1], segs[0]))
        segs.pop()
    return segs


class Track:
    __slots__ = ("id", "pos", "vel", "radius", "hits", "misses", "origin")

    def __init__(self, tid, pos, radius):
        self.id, self.pos, self.vel = tid, np.array(pos, float), np.zeros(2)
        self.origin = np.array(pos, float)
        self.radius, self.hits, self.misses = radius, 1, 0


class DynamicTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = []
        self._next = 0

    def update(self, pts_body, pose, dt):
        """pts_body: angle-ordered scan points. Returns a boolean mask over
        pts_body marking points that belong to MOVING tracks."""
        cfg = self.cfg
        segs = segment(pts_body, cfg.cluster_gap)
        meas = []  # (centroid_world, radius, indices)
        for idx in segs:
            if len(idx) < 3:
                continue
            p = pts_body[idx]
            ext = np.hypot(*(p.max(0) - p.min(0)))
            if ext > cfg.track_max_extent:
                continue
            cw = body_to_world(p.mean(0, keepdims=True), pose)[0]
            meas.append((cw, max(0.1, ext / 2), idx))

        # predict
        for tr in self.tracks:
            tr.pos = tr.pos + tr.vel * dt
        # greedy nearest-neighbour association
        pairs = sorted((np.hypot(*(m[0] - tr.pos)), ti, mi)
                       for ti, tr in enumerate(self.tracks) for mi, m in enumerate(meas))
        used_t, used_m, assign = set(), set(), {}
        for d, ti, mi in pairs:
            if d <= cfg.track_gate and ti not in used_t and mi not in used_m:
                used_t.add(ti)
                used_m.add(mi)
                assign[mi] = ti
        mask = np.zeros(len(pts_body), dtype=bool)
        for mi, (cw, rad, idx) in enumerate(meas):
            if mi in assign:
                tr = self.tracks[assign[mi]]
                r = cw - tr.pos
                tr.pos = tr.pos + cfg.track_alpha * r
                if dt > 1e-6:
                    tr.vel = tr.vel + (cfg.track_beta / dt) * r
                tr.radius = 0.7 * tr.radius + 0.3 * rad
                tr.hits += 1
                tr.misses = 0
                if self._moving(tr):
                    mask[idx] = True
            else:
                self.tracks.append(Track(self._next, cw, rad))
                self._next += 1
        n_old = len(self.tracks) - sum(1 for mi in range(len(meas)) if mi not in assign)
        for ti in range(n_old):
            if ti not in used_t:
                self.tracks[ti].misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= 5]
        return mask

    def _moving(self, tr):
        """Fast AND actually displaced: wall ends / door jambs show apparent
        centroid motion as the viewpoint changes but never travel far."""
        c = self.cfg
        return (tr.hits >= c.dyn_min_hits and np.hypot(*tr.vel) > c.dyn_speed
                and np.hypot(*(tr.pos - tr.origin)) > c.dyn_min_travel)

    def moving_obstacles(self, pose):
        """[(centre_body, velocity_body, radius)] for tracks judged moving."""
        out = []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for tr in self.tracks:
            if self._moving(tr) and tr.misses <= 2:
                cb = world_to_body(tr.pos[None, :], pose)[0]
                vb = np.array([c * tr.vel[0] + s * tr.vel[1], -s * tr.vel[0] + c * tr.vel[1]])
                out.append((cb, vb, tr.radius))
        return out
