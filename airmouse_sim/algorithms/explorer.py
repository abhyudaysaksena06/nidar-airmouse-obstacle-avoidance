"""Baseline AirMouse mission ("baseline first" - beat it!).

  State machine : PREFLIGHT -> TAKEOFF -> ENTER -> EXPLORE (+LOOK) -> RETURN
                  -> EXIT -> LAND -> DONE, with failsafes HOLD (lidar lost),
                  RTL (battery/time budget) and EMERGENCY_LAND (critical).
  Localisation  : flow + gyro dead reckoning, heading snap to the maze axes,
                  correlative scan-to-map matching (Hector-SLAM style).
  Mapping       : log-odds occupancy grid at the lidar plane + a camera
                  coverage grid (which floor has the camera actually seen).
  Exploration   : utility = unknown area + unseen-by-camera floor - path cost
                  (Dijkstra), A* path, LOOK turns towards unseen floor.
  Survivors     : bbox -> bearing (+ ground-plane or depth range), clustered,
                  multi-view least-squares triangulation, consistency filter.
  Safety        : lidar braking + repulsion bubble, forward ToF beams for low
                  boxes / hanging beams (climb or duck), ceiling ToF.
"""
import heapq
import math

import numpy as np

from algorithms.base import BaseAlgorithm

RES = 0.1
SIZE = 800  # 80 m x 80 m map centred on the start


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def dilate(g, r):
    out = g.copy()
    H, W = g.shape
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r * r:
                continue
            out[max(0, di):H - max(0, -di), max(0, dj):W - max(0, -dj)] |= \
                g[max(0, -di):H - max(0, di), max(0, -dj):W - max(0, dj)]
    return out


def box_sum(a, r):
    """Sum of a over a (2r+1)^2 window around every cell (integral image)."""
    p = np.pad(a.astype(np.float32), ((r + 1, r), (r + 1, r)))
    c = p.cumsum(0).cumsum(1)
    return c[2 * r + 1:, 2 * r + 1:] - c[:-2 * r - 1, 2 * r + 1:] - c[2 * r + 1:, :-2 * r - 1] + c[:-2 * r - 1, :-2 * r - 1]


class Algorithm(BaseAlgorithm):
    SPEED = 0.6
    LAMBDA = 25.0        # utility cost per metre of path
    MIN_GAIN = 25.0      # stop exploring when nothing is worth this much
    CAM_RANGE = 3.5      # floor considered "seen" by the camera up to here

    def reset(self, info):
        super().reset(info)
        self.L = np.zeros((SIZE, SIZE), dtype=np.float32)       # log-odds occupancy
        self.cam_seen = np.zeros((SIZE, SIZE), dtype=bool)       # camera coverage
        self.lf = np.zeros((SIZE, SIZE), dtype=np.float32)       # likelihood field
        self.origin = (-SIZE * RES / 2, -SIZE * RES / 2)
        self.pose = [0.0, 0.0, 0.0]
        self.v_last = (0.0, 0.0)
        self.state, self.t_state = "PREFLIGHT", 0.0
        self.clusters = []
        self.path, self.goal, self.last_plan = None, None, -99.0
        self.banned, self.stall_t, self.stall_xy = [], 0.0, (0.0, 0.0)
        self.look_yaw = None
        self.n = self.n_scans = 0
        self.entry_x = info["start_outside"]
        self.z_target = info["cruise_alt"]
        self.z_est, self.baro_off = 0.0, None
        self.low_pts, self.hang_pts = [], []
        self.front_range = math.inf
        self.lidar_lost_since = None
        self.match_score = 0.0
        self.avg_power = 200.0
        self.msg = ""

    # =============================================================== helpers
    def _cell(self, x, y):
        return int((y - self.origin[1]) / RES), int((x - self.origin[0]) / RES)

    def _set(self, st, t):
        if st != self.state:
            self.state, self.t_state = st, t
            self.path = None

    # ========================================================= localisation
    def _predict(self, odo, dt):
        p = self.pose
        p[2] = wrap(p[2] + odo["yaw_rate"] * dt)
        if odo["vx"] is not None:
            self.v_last = (odo["vx"], odo["vy"])
        else:  # flow invalid: coast and decay
            self.v_last = (self.v_last[0] * 0.98, self.v_last[1] * 0.98)
        vx, vy = self.v_last
        c, s = math.cos(p[2]), math.sin(p[2])
        p[0] += (c * vx - s * vy) * dt
        p[1] += (s * vx + c * vy) * dt

    def _scan_points(self, ang, rng):
        ok = np.isfinite(rng) & (rng < self.info["lidar_max_range"] - 0.1)
        r, a = rng[ok], ang[ok]
        return r * np.cos(a), r * np.sin(a)

    def _heading_snap(self, bx, by):
        """Maze walls are square to the entry wall: wall directions must be
        multiples of 90 deg in the start frame."""
        dx, dy = np.diff(bx), np.diff(by)
        seg = np.hypot(dx, dy)
        good = (seg > 0.02) & (seg < 0.3)
        if good.sum() < 30:
            return
        th = np.arctan2(dy[good], dx[good]) + self.pose[2]
        z = np.exp(4j * th).mean()
        if abs(z) < 0.3:
            return
        err = np.angle(z) / 4
        if abs(err) < math.radians(15):
            self.pose[2] = wrap(self.pose[2] - 0.3 * err)

    def _scan_match(self, bx, by):
        """Correlative scan-to-map matching on a likelihood field."""
        if len(bx) < 40 or (self.L > 0.5).sum() < 300:
            return
        sel = slice(None, None, max(1, len(bx) // 120))
        bx, by = bx[sel], by[sel]
        p = self.pose
        best = (-1.0, 0.0, 0.0, 0.0)
        for dth in np.radians([-2.0, -1.0, 0.0, 1.0, 2.0]):
            th = p[2] + dth
            c, s = math.cos(th), math.sin(th)
            wx = p[0] + c * bx - s * by
            wy = p[1] + s * bx + c * by
            offs = np.arange(-0.15, 0.151, 0.05)
            X = wx[None, None, :] + offs[None, :, None]
            Y = wy[None, None, :] + offs[:, None, None]
            iy = ((Y - self.origin[1]) / RES).astype(int).clip(0, SIZE - 1)
            ix = ((X - self.origin[0]) / RES).astype(int).clip(0, SIZE - 1)
            sc = self.lf[iy, ix].sum(axis=2)  # [dy, dx]
            k = np.unravel_index(sc.argmax(), sc.shape)
            if sc[k] > best[0]:
                best = (sc[k], offs[k[1]], offs[k[0]], dth)
        self.match_score = best[0] / len(bx)
        if self.match_score > 0.45:  # confident match -> pull pose towards it
            p[0] += 0.5 * best[1]
            p[1] += 0.5 * best[2]
            p[2] = wrap(p[2] + 0.5 * best[3])

    def _update_lf(self):
        occ = self.L > 0.5
        self.lf = occ * 1.0 + (dilate(occ, 1) & ~occ) * 0.6 + (dilate(occ, 2) & ~dilate(occ, 1)) * 0.25
        self.lf = self.lf.astype(np.float32)

    def get_pose(self):
        return tuple(self.pose)

    # ============================================================== mapping
    def _update_map(self, ang, rng, rmax):
        o = self.pose
        a = ang[::2] + o[2]
        r = rng[::2]
        hit = np.isfinite(r) & (r < rmax - 0.1)
        rr = np.where(np.isfinite(r), np.minimum(r, rmax), rmax)
        ts = np.arange(0, rmax, RES)
        mask = ts[None, :] < (rr[:, None] - RES)
        xs = o[0] + np.cos(a)[:, None] * ts[None, :]
        ys = o[1] + np.sin(a)[:, None] * ts[None, :]
        iy = ((ys - self.origin[1]) / RES).astype(int)[mask]
        ix = ((xs - self.origin[0]) / RES).astype(int)[mask]
        np.add.at(self.L, (iy, ix), -0.4)
        hx = o[0] + np.cos(a[hit]) * r[hit]
        hy = o[1] + np.sin(a[hit]) * r[hit]
        np.add.at(self.L, (((hy - self.origin[1]) / RES).astype(int),
                           ((hx - self.origin[0]) / RES).astype(int)), 0.9)
        np.clip(self.L, -4, 4, out=self.L)

    def _update_cam_coverage(self, ang, rng):
        hf = self.info["camera"]["hfov"] / 2
        sel = np.abs(ang) < hf
        o = self.pose
        a = ang[sel] + o[2]
        r = np.minimum(np.where(np.isfinite(rng[sel]), rng[sel], 99), self.CAM_RANGE)
        ts = np.arange(0.8, self.CAM_RANGE, RES)
        mask = ts[None, :] < r[:, None]
        xs = o[0] + np.cos(a)[:, None] * ts[None, :]
        ys = o[1] + np.sin(a)[:, None] * ts[None, :]
        self.cam_seen[((ys - self.origin[1]) / RES).astype(int)[mask],
                      ((xs - self.origin[0]) / RES).astype(int)[mask]] = True

    def get_map(self):
        known = np.abs(self.L) > 0.1
        if not known.any():
            return None
        ys, xs = np.where(known)
        i0, i1, j0, j1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
        P = 1 / (1 + np.exp(-self.L[i0:i1, j0:j1]))
        return {"grid": P, "res": RES,
                "origin": (self.origin[0] + j0 * RES, self.origin[1] + i0 * RES)}

    # ============================================================ survivors
    def _survivors(self, dets):
        cam = self.info["camera"]
        o = self.pose
        for c in self.clusters:  # count frames in which a cluster should be visible
            d = math.hypot(c["x"] - o[0], c["y"] - o[1])
            b = wrap(math.atan2(c["y"] - o[1], c["x"] - o[0]) - o[2])
            if 1.0 < d < self.CAM_RANGE and abs(b) < cam["hfov"] * 0.4:
                c["views"] += 1
        for d in dets:
            if d["confidence"] < 0.35:
                continue
            rng_g, bearing = self.bbox_ground_range(d["bbox"], cam, max(0.3, self.z_est))
            if "depth" in d:
                rng_g = math.sqrt(max(0.04, d["depth"] ** 2 - (self.z_est - 0.15) ** 2))
            if rng_g is None or rng_g > 7.0:
                continue
            a = o[2] + bearing
            x, y = o[0] + rng_g * math.cos(a), o[1] + rng_g * math.sin(a)
            ob = (o[0], o[1], a, rng_g, d["confidence"], "depth" in d)
            for c in self.clusters:
                if math.hypot(c["x"] - x, c["y"] - y) < 1.2:
                    c["obs"].append(ob)
                    c["hits"] += 1
                    if c["hits"] % 3 == 0:
                        self._triangulate(c)
                    break
            else:
                self.clusters.append({"x": x, "y": y, "obs": [ob], "hits": 1, "views": 1})

    @staticmethod
    def _triangulate(c):
        """Least squares over all views: bearing rays + (weak) ranges."""
        obs = c["obs"][-60:]
        x, y = c["x"], c["y"]
        for _ in range(5):
            H = np.zeros((2, 2))
            g = np.zeros(2)
            for px, py, a, r, w, has_depth in obs:
                dx, dy = x - px, y - py
                n = np.array([-math.sin(a), math.cos(a)])          # perp to the ray
                e_b = n @ np.array([dx, dy])
                wb = w / max(0.05, 0.03 * r) ** 2
                H += wb * np.outer(n, n)
                g += wb * n * e_b
                d = math.hypot(dx, dy) + 1e-6
                u = np.array([dx, dy]) / d
                wr = w / ((0.05 if has_depth else 0.2) * r) ** 2
                H += wr * np.outer(u, u)
                g += wr * u * (d - r)
            try:
                step = np.linalg.solve(H + 1e-6 * np.eye(2), g)
            except np.linalg.LinAlgError:
                return
            x, y = x - step[0], y - step[1]
            if np.hypot(*step) < 1e-3:
                break
        c["x"], c["y"] = float(x), float(y)

    def get_survivors(self):
        good = [c for c in self.clusters if c["hits"] >= 6 and c["hits"] >= 0.3 * c["views"]
                and np.mean([ob[4] for ob in c["obs"]]) >= 0.5]
        good.sort(key=lambda c: -c["hits"])
        out = []
        for c in good:  # merge duplicates of the same person
            if all(math.hypot(c["x"] - a, c["y"] - b) > 1.3 for a, b in out):
                out.append((c["x"], c["y"]))
        return out[:6]

    # =========================================================== altitude
    def _altitude(self, obs, t):
        """Height estimate (ToF + baro across box tops) and vertical hazards."""
        tof, info = obs["tof"], self.info
        h = tof["down"]
        baro = obs["baro_z"]
        if math.isfinite(h):
            if self.baro_off is None:
                self.baro_off = baro - h
            over_box = (baro - self.baro_off) - h > 0.25
            if over_box:
                self.z_est = baro - self.baro_off
            else:
                self.z_est = h
                self.baro_off = 0.98 * self.baro_off + 0.02 * (baro - h)
        else:
            self.z_est = baro - (self.baro_off or 0.0)
        hh, p = info["half_height"], info["tof_fwd_pitch"]
        o = self.pose
        # forward beams hit something in our height band -> remember it
        for key, sign, store in (("fwd_down", -1, self.low_pts), ("fwd_up", 1, self.hang_pts)):
            d = tof[key]
            if not math.isfinite(d):
                continue
            hit_z = self.z_est + sign * d * math.sin(p)
            horiz = d * math.cos(p)
            if self.front_range < horiz + 0.35:
                continue  # the lidar already sees it at our height: a wall, not a 3D hazard
            if abs(hit_z - self.z_est) < hh + 0.25 and 0.1 < hit_z:
                hx = o[0] + d * math.cos(p) * math.cos(o[2])
                hy = o[1] + d * math.cos(p) * math.sin(o[2])
                store.append((hx, hy, hit_z))
                del store[:-400]
        # height band allowed here: above remembered low obstacles, below hanging ones
        ax, ay = o[0] + 0.6 * math.cos(o[2]), o[1] + 0.6 * math.sin(o[2])  # look-ahead point
        reach = info["drone_radius"] + 0.45

        def nearby(pts):
            return [z for x, y, z in pts
                    if math.hypot(x - o[0], y - o[1]) < reach or math.hypot(x - ax, y - ay) < reach]
        lo = max(nearby(self.low_pts), default=-1.0) + hh + 0.25
        hi = min(nearby(self.hang_pts), default=99.0) - hh - 0.25
        if math.isfinite(tof["up"]):
            hi = min(hi, self.z_est + tof["up"] - hh - 0.25)
        zt = info["cruise_alt"]
        zt = max(zt, lo)
        zt = min(zt, hi) if hi >= 0.4 else zt
        self.z_target = max(0.4, zt)
        return float(np.clip(1.5 * (self.z_target - self.z_est), -info["max_vz"], info["max_vz"]))

    # ============================================================== control
    def _repulse(self, ang, rng):
        """Safety bubble: push away from the closest obstacles. Direction is
        the weighted mean over close beams, strength set by the closest."""
        bubble = self.info["drone_radius"] + 0.22
        close = rng < bubble
        if not close.any():
            return 0.0, 0.0
        wgt = bubble - rng[close]
        dx, dy = -(wgt * np.cos(ang[close])).sum(), -(wgt * np.sin(ang[close])).sum()
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return 0.0, 0.0
        mag = 2.5 * float(wgt.max())
        return mag * dx / n, mag * dy / n

    def _brake(self, vx, ang, rng):
        ahead = rng[np.abs(ang) < math.radians(35)]
        gap = float(ahead.min()) - self.info["drone_radius"] - 0.15 if ahead.size else 9.0
        return min(vx, max(0.0, 0.8 * gap))

    def _follow(self, speed, look=0.6):
        o = self.pose
        dists = [math.hypot(x - o[0], y - o[1]) for x, y in self.path]
        self.path = self.path[int(np.argmin(dists)):]
        tgt = self.path[-1]
        for x, y in self.path:
            if math.hypot(x - o[0], y - o[1]) >= look:
                tgt = (x, y)
                break
        err = wrap(math.atan2(tgt[1] - o[1], tgt[0] - o[0]) - o[2])
        v = speed * max(0.0, math.cos(err)) ** 2 if abs(err) < math.pi / 2 else 0.0
        return v, float(np.clip(1.8 * err, -1.2, 1.2))

    def _gap_steer(self, ang, rng):
        """Final approach: steer through the opening the lidar actually sees."""
        o = self.pose
        home = wrap(math.atan2(-o[1], -o[0]) - o[2])
        open_ = rng > 2.0
        best, best_cost, n, i = None, 1e9, len(ang), 0
        while i < n:
            if not open_[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and open_[j + 1]:
                j += 1
            mid = wrap((ang[i] + ang[j]) / 2)
            cost = abs(wrap(mid - home))
            if j - i + 1 >= 6 and cost < math.radians(60) and cost < best_cost:
                best, best_cost = mid, cost
            i = j + 1
        if best is None:
            return None
        return 0.35 * max(0.0, math.cos(best)), float(np.clip(2.0 * best, -1.0, 1.0))

    # ============================================================= planning
    def _crop(self, pad=10):
        known = np.abs(self.L) > 0.1
        ys, xs = np.where(known)
        if ys.size == 0:
            return None
        i0, i1 = max(ys.min() - pad, 0) // 2 * 2, min(ys.max() + pad, SIZE) // 2 * 2
        j0, j1 = max(xs.min() - pad, 0) // 2 * 2, min(xs.max() + pad, SIZE) // 2 * 2
        return i0, i1, j0, j1

    def _blocked(self, Lc, i0, j0, inside_only):
        occ = Lc > 0.4
        blocked = dilate(occ, int(round((self.info["drone_radius"] + 0.05) / RES)))
        if inside_only:
            xs = self.origin[0] + (j0 + np.arange(Lc.shape[1]) + 0.5) * RES
            blocked[:, xs < self.entry_x + 0.4] = True
        return blocked, occ

    def _astar(self, start, goal, inside_only=False, max_expand=150000):
        cr = self._crop()
        if cr is None:
            return None
        i0, i1, j0, j1 = cr
        Lc = self.L[i0:i1, j0:j1]
        blocked, occ = self._blocked(Lc, i0, j0, inside_only)
        near = dilate(occ, int(math.ceil((self.info["drone_radius"] + 0.5) / RES))) & ~blocked
        unknown = np.abs(Lc) <= 0.1
        H, W = blocked.shape

        def cell(x, y):
            return int((y - self.origin[1]) / RES) - i0, int((x - self.origin[0]) / RES) - j0
        s, g = cell(*start), cell(*goal)
        if not (0 <= s[0] < H and 0 <= s[1] < W and 0 <= g[0] < H and 0 <= g[1] < W):
            return None
        blocked[max(s[0] - 2, 0):s[0] + 3, max(s[1] - 2, 0):s[1] + 3] = False
        blocked[g] = False
        openq = [(0.0, 0.0, s)]
        came, cost = {s: None}, {s: 0.0}
        steps = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        n_exp = 0
        while openq and n_exp < max_expand:
            _, c, cur = heapq.heappop(openq)
            if cur == g:
                break
            if c > cost[cur]:
                continue
            n_exp += 1
            for di, dj, sc in steps:
                n = (cur[0] + di, cur[1] + dj)
                if not (0 <= n[0] < H and 0 <= n[1] < W) or blocked[n]:
                    continue
                nc = c + sc * (3.0 if unknown[n] else 1.0) * (2.5 if near[n] else 1.0)
                if nc < cost.get(n, 1e18):
                    cost[n], came[n] = nc, cur
                    heapq.heappush(openq, (nc + math.hypot(n[0] - g[0], n[1] - g[1]), nc, n))
        if g not in came:
            return None
        path, cur = [], g
        while cur is not None:
            path.append((self.origin[0] + (cur[1] + j0 + 0.5) * RES,
                         self.origin[1] + (cur[0] + i0 + 0.5) * RES))
            cur = came[cur]
        return path[::-1]

    def _choose_goal(self):
        """Best (gain - cost) goal: unknown space + floor the camera hasn't seen."""
        cr = self._crop(pad=16)
        if cr is None:
            return None, 0.0
        i0, i1, j0, j1 = cr
        Lc = self.L[i0:i1, j0:j1]
        blocked, _ = self._blocked(Lc, i0, j0, inside_only=True)
        free = Lc < -0.4
        trav = free & ~blocked
        # Dijkstra on 0.2 m cells (a coarse cell is passable if any sub-cell is)
        h2, w2 = trav.shape[0] // 2, trav.shape[1] // 2
        tc = trav[:h2 * 2, :w2 * 2].reshape(h2, 2, w2, 2).any(axis=(1, 3))
        o = self.pose
        si = (int((o[1] - self.origin[1]) / RES) - i0) // 2
        sj = (int((o[0] - self.origin[0]) / RES) - j0) // 2
        if not (0 <= si < h2 and 0 <= sj < w2):
            return None, 0.0
        tc[max(si - 1, 0):si + 2, max(sj - 1, 0):sj + 2] = True
        dist = np.full((h2, w2), np.inf)
        dist[si, sj] = 0.0
        q = [(0.0, si, sj)]
        while q:
            c, i, j = heapq.heappop(q)
            if c > dist[i, j]:
                continue
            for di, dj, sc in ((1, 0, 0.2), (-1, 0, 0.2), (0, 1, 0.2), (0, -1, 0.2),
                               (1, 1, 0.283), (1, -1, 0.283), (-1, 1, 0.283), (-1, -1, 0.283)):
                a, b = i + di, j + dj
                if 0 <= a < h2 and 0 <= b < w2 and tc[a, b] and c + sc < dist[a, b]:
                    dist[a, b] = c + sc
                    heapq.heappush(q, (c + sc, a, b))
        unknown = np.abs(Lc) <= 0.1
        unseen = free & ~self.cam_seen[i0:i1, j0:j1]
        gain = box_sum(unknown, 12) + 1.5 * box_sum(unseen, 12)
        gain_c = gain[:h2 * 2:2, :w2 * 2:2] + 0.0
        for bx, by in self.banned:
            bi = (int((by - self.origin[1]) / RES) - i0) // 2
            bj = (int((bx - self.origin[0]) / RES) - j0) // 2
            gain_c[max(bi - 6, 0):bi + 7, max(bj - 6, 0):bj + 7] = 0
        gain_c[dist < 0.6] = 0
        util = np.where(np.isfinite(dist) & (gain_c >= self.MIN_GAIN), gain_c - self.LAMBDA * dist, -np.inf)
        k = np.unravel_index(np.argmax(util), util.shape)
        if not np.isfinite(util[k]):
            return None, 0.0
        best = float(util[k])
        if self.goal is not None:  # hysteresis: keep the current goal unless clearly beaten
            gi = (int((self.goal[1] - self.origin[1]) / RES) - i0) // 2
            gj = (int((self.goal[0] - self.origin[0]) / RES) - j0) // 2
            if 0 <= gi < h2 and 0 <= gj < w2 and np.isfinite(util[gi, gj]) and                     util[gi, gj] > best - 0.3 * abs(best) - 20:
                return self.goal, float(gain_c[gi, gj])
        gx = self.origin[0] + (j0 + k[1] * 2 + 1) * RES
        gy = self.origin[1] + (i0 + k[0] * 2 + 1) * RES
        return (gx, gy), float(gain_c[k])

    def _look_direction(self):
        """Yaw towards the centroid of nearby floor the camera hasn't seen."""
        o = self.pose
        ci, cj = self._cell(o[0], o[1])
        r = 30
        free = self.L[ci - r:ci + r, cj - r:cj + r] < -0.4
        unseen = free & ~self.cam_seen[ci - r:ci + r, cj - r:cj + r]
        if unseen.sum() < 40:
            return None
        ii, jj = np.nonzero(unseen)
        return math.atan2((ii.mean() - r) * RES, (jj.mean() - r) * RES)

    # ============================================================ budgets
    def _must_return(self, obs):
        o = self.pose
        d_home = math.hypot(o[0], o[1])
        t_home = 1.8 * d_home / 0.4 + 30.0
        self.avg_power = 0.98 * self.avg_power + 0.02 * obs["battery"]["power_w"]
        e_need = self.avg_power * t_home / 3600.0 + 0.12 * self.info["battery_capacity_wh"]
        if obs["battery"]["energy_wh"] < e_need:
            self.msg = "RTL: battery"
            return True
        if obs["time_left"] < t_home:
            self.msg = "RTL: time"
            return True
        return False

    # ================================================================= step
    def step(self, obs):
        t, dt = obs["t"], obs["dt"]
        self.n += 1
        self._predict(obs["odom"], dt)
        lid = obs["lidar"]
        if lid is not None and lid["new"]:
            bx, by = self._scan_points(lid["angles"], lid["ranges"])
            if self.z_est > 0.4:
                self._heading_snap(bx, by)
                self._scan_match(bx, by)
                self._update_map(lid["angles"], lid["ranges"], lid["max_range"])
                if obs["detections"] is not None:
                    self._update_cam_coverage(lid["angles"], lid["ranges"])
                self.n_scans += 1
                if self.n_scans % 10 == 0:
                    self._update_lf()
        if obs["detections"]:
            self._survivors(obs["detections"])
        if lid is not None:
            fr = lid["ranges"][np.abs(lid["angles"]) < math.radians(5)]
            self.front_range = float(fr.min()) if fr.size else math.inf
        vz = self._altitude(obs, t)
        o = self.pose
        st = self.state
        vx = w = 0.0

        # ---------------- failsafes (highest priority) ----------------
        if obs["battery"]["frac"] < 0.06 and st not in ("LAND", "EMERGENCY_LAND", "DONE"):
            self._set("EMERGENCY_LAND", t)
        if lid is None and st in ("ENTER", "EXPLORE", "RETURN", "EXIT"):
            if self.lidar_lost_since is None:
                self.lidar_lost_since, self.resume = t, st
            self._set("HOLD", t)
        if self.state == "HOLD":
            if lid is not None:
                self.lidar_lost_since = None
                self._set(self.resume, t)
            elif t - self.lidar_lost_since > 10.0:
                self._set("EMERGENCY_LAND", t)
        st = self.state

        # ---------------- mission state machine -----------------------
        if st == "PREFLIGHT":
            vz = 0.0
            if t > 1.0:
                self._set("TAKEOFF", t)
        elif st == "TAKEOFF":
            if abs(self.z_est - self.info["cruise_alt"]) < 0.1:
                self._set("ENTER", t)
        elif st == "ENTER":
            vx, w = 0.45, float(np.clip(-1.5 * o[1], -0.5, 0.5) - 1.0 * o[2])
            if o[0] > self.entry_x + 1.2:
                self._set("EXPLORE", t)
        elif st in ("EXPLORE", "LOOK"):
            if self._must_return(obs):
                self._set("RETURN", t)
            elif st == "LOOK":
                err = wrap(self.look_yaw - o[2])
                w = float(np.clip(2.0 * err, -1.0, 1.0))
                if abs(err) < 0.15 or t - self.t_state > 6.0:
                    self._set("EXPLORE", t)
            else:
                reached = self.path and math.hypot(self.path[-1][0] - o[0], self.path[-1][1] - o[1]) < 0.4
                if reached:
                    self.goal = None
                    look = self._look_direction()
                    if look is not None and abs(wrap(look - o[2])) > 0.6:
                        self.look_yaw = look
                        self._set("LOOK", t)
                        return self._out(0.0, 0.0, vz, 0.0, lid)
                if t - self.last_plan > 2.5 or reached or not self.path:
                    if t - self.last_plan > 0.5:
                        goal, gain = self._choose_goal()
                        self.last_plan = t
                        if goal is None:
                            self.msg = "exploration complete"
                            self._set("RETURN", t)
                        else:
                            p = self._astar((o[0], o[1]), goal, inside_only=True)
                            if p:
                                self.path, self.goal = p, goal
                            else:
                                self.banned.append(goal)
                                self.goal = None
                if self.state == "EXPLORE" and self.path:
                    spd = self.SPEED if obs["odom"]["flow_quality"] >= 0.3 else 0.3
                    vx, w = self._follow(spd)
                    if t - self.stall_t > 6.0:
                        if math.hypot(o[0] - self.stall_xy[0], o[1] - self.stall_xy[1]) < 0.4:
                            self.banned.append(self.path[-1])
                            self.path, self.goal = None, None
                        self.stall_t, self.stall_xy = t, (o[0], o[1])
        elif st == "RETURN":
            if t - self.last_plan > (5.0 if self.path else 1.5):
                p = self._astar((o[0], o[1]), (self.entry_x + 0.8, 0.0))
                if p:
                    self.path = p + [(self.entry_x - 0.5, 0.0), (0.0, 0.0)]
                self.last_plan = t
            if math.hypot(o[0] - self.entry_x, o[1]) < 1.6 and o[0] > self.entry_x - 0.3:
                self._set("EXIT", t)
            elif self.path:
                vx, w = self._follow(0.5)
        elif st == "EXIT":
            gap = self._gap_steer(lid["angles"], lid["ranges"]) if lid else None
            if o[0] < self.entry_x - 0.4:  # outside: head for the take-off spot
                err = wrap(math.atan2(-o[1], -o[0]) - o[2])
                vx, w = 0.35 * max(0.0, math.cos(err)), float(np.clip(1.8 * err, -1, 1))
            elif gap:
                vx, w = gap
            if math.hypot(o[0], o[1]) < 0.35:
                self._set("LAND", t)
        elif st in ("LAND", "EMERGENCY_LAND"):
            vz = -0.35 if self.z_est > 0.3 else -0.2
            if obs["landed"]:
                self._set("DONE", t)
        elif st == "DONE":
            return {"vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0, "done": True, "state": "DONE"}
        elif st == "HOLD":
            pass
        return self._out(vx, 0.0, vz, w, lid)

    def _out(self, vx, vy, vz, w, lid):
        if lid is not None and self.state not in ("PREFLIGHT", "TAKEOFF", "LAND", "DONE"):
            ang, rng = lid["angles"], lid["ranges"]
            vx = self._brake(vx, ang, rng)
            rx, ry = self._repulse(ang, rng)
            if self.state in ("EXIT", "RETURN") and math.hypot(self.pose[0], self.pose[1]) < 2.5:
                rx, ry = rx * 0.3, ry * 0.3  # squeezing through the entry
            vx, vy = vx + rx, vy + ry
        if abs(self.z_target - self.z_est) > 0.2:
            vx = min(vx, 0.15)  # wait for the climb/duck before moving on
        return {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": w, "state": self.state, "done": False}

    def debug_text(self):
        return (f"{self.state} z={self.z_est:.2f}->{self.z_target:.2f} match={self.match_score:.2f} "
                f"clusters={len(self.clusters)} {self.msg}")
