"""Obstacle-avoidance benchmark in the AirMouse simulator.

Role split being tested (same as the team pipeline):
  * global planner (A* + pure pursuit) knows only the WALLS of the maze
  * the avoider must deal with everything else: boxes, pillars, hanging
    beams, moving people - using only the lidar / ToF / odometry

Each trial: random start/goal 6-14 m apart (path length), fly the planner's
path at 0.8 m/s through the clutter. Success = reach the goal; failure =
collision or timeout.

  python bench/bench_avoidance.py                       # all methods, clutter scenario
  python bench/bench_avoidance.py --scenario 3d --trials 40
  python bench/bench_avoidance.py --methods full --fov 180
"""
import argparse
import copy
import heapq
import json
import math
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIM = os.path.join(os.path.dirname(ROOT), "airmouse_sim")
sys.path.insert(0, ROOT)
sys.path.insert(0, SIM)

from oa import AvoidanceConfig, ObstacleAvoider  # noqa: E402
from oa.points3d import virtual_scan  # noqa: E402
from oa_dwa import DWAAvoider, DWAConfig  # noqa: E402
from sim.engine import Simulation  # noqa: E402
from sim.sensors import ray3d  # noqa: E402
from sim.world import RES, generate_world, inflate  # noqa: E402

SCENARIOS = {
    # every obstacle tall enough for the 2D lidar: pure horizontal avoidance
    "clutter": {"obstacles": {"static_count": 16, "box_height_m": [1.3, 2.2], "hanging_count": 0,
                              "moving_count": 3, "moving_speed_mps": 0.5}},
    # full 2.5D: low boxes and hanging beams the lidar cannot see
    "3d": {"obstacles": {"static_count": 12, "hanging_count": 5, "moving_count": 2, "moving_speed_mps": 0.4}},
    # narrow corridors/doors
    "narrow": {"arena": {"cell_m": 2.0, "door_min_m": 0.9, "door_max_m": 1.1},
               "obstacles": {"static_count": 10, "box_height_m": [1.3, 2.2], "hanging_count": 0,
                             "moving_count": 2, "moving_speed_mps": 0.4}},
}
METHODS = ["none", "brake_repulse", "cbf_only", "vfh_only", "full", "algo1", "algoD", "algoL"]
# "full"  = Algo 2: VFH+ + CBF, 360 lidar + 2 forward ToF beams (oa/)
# "algo1" = layered costmap + DWA, lidar + depth camera (oa_dwa/)
# "algoD" = VFH+ + CBF core driven by a DEPTH CAMERA ONLY (87 x 58 deg)
# "algoL" = VFH+ + CBF core, 360 lidar + ring of 4 multi-zone ToF (8x8, 45 deg)

DEPTH = {"hfov": 87.0, "vfov": 58.0, "nu": 48, "nv": 20, "min": 0.3, "max": 6.0}   # D435-class
MZTOF = {"fov": 45.0, "zones": 8, "max": 3.5, "yaws": (0.0, 90.0, 180.0, 270.0), "pitch": 0.0}


def ray_batch(world, x, y, z, yaw_dirs, pitch_dirs, max_r, step=0.05):
    """Vectorised 3D ray march (static obstacles + walking people)."""
    cp = np.cos(pitch_dirs)
    d = np.column_stack((cp * np.cos(yaw_dirs), cp * np.sin(yaw_dirs), np.sin(pitch_dirs)))
    ts = np.arange(step, max_r + step, step)
    P = np.array([x, y, z])[None, None, :] + d[:, None, :] * ts[None, :, None]
    ix = (P[..., 0] / RES).astype(int)
    iy = (P[..., 1] / RES).astype(int)
    H, W = world.zlo.shape
    inb = (ix >= 0) & (iy >= 0) & (ix < W) & (iy < H)
    ixc, iyc = np.clip(ix, 0, W - 1), np.clip(iy, 0, H - 1)
    zl, zh = world.zlo[iyc, ixc], world.zhi[iyc, ixc]
    hit = ~inb | (P[..., 2] <= 0.0) | (P[..., 2] >= world.ceiling) | ((zl <= P[..., 2]) & (P[..., 2] <= zh))
    for m in world.movers:
        hit |= (np.hypot(P[..., 0] - m.x, P[..., 1] - m.y) <= m.r) & (P[..., 2] <= m.h)
    has = hit.any(1)
    r = np.where(has, ts[hit.argmax(1)], np.inf)
    return d, r


def deep_update(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
def plan_walls_only(world, start, goal, inflate_m, extra=None):
    """A* on a 0.2 m grid containing only walls (what a map-based planner
    knows) plus optional `extra` obstacle cells seen in flight (replanning)."""
    g = world.walls
    if extra is not None:
        g = g | extra
    g = g[: g.shape[0] // 2 * 2, : g.shape[1] // 2 * 2]
    g = g.reshape(g.shape[0] // 2, 2, g.shape[1] // 2, 2).any(axis=(1, 3))
    R = RES * 2
    blocked = inflate(g, inflate_m / R)
    H, W = blocked.shape
    s = (int(start[1] / R), int(start[0] / R))
    t = (int(goal[1] / R), int(goal[0] / R))
    if blocked[s] or blocked[t]:
        return None
    openq, came, cost = [(0.0, 0.0, s)], {s: None}, {s: 0.0}
    while openq:
        _, c, cur = heapq.heappop(openq)
        if cur == t:
            break
        if c > cost[cur]:
            continue
        for di, dj, sc in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                           (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)):
            n = (cur[0] + di, cur[1] + dj)
            if 0 <= n[0] < H and 0 <= n[1] < W and not blocked[n] and c + sc < cost.get(n, 1e18):
                cost[n], came[n] = c + sc, cur
                heapq.heappush(openq, (c + sc + math.hypot(n[0] - t[0], n[1] - t[1]), c + sc, n))
    if t not in came:
        return None
    path, cur = [], t
    while cur is not None:
        path.append(((cur[1] + 0.5) * R, (cur[0] + 0.5) * R))
        cur = came[cur]
    return path[::-1]


def path_length(p):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))


# ---------------------------------------------------------------------------
class BenchPilot:
    """Planner (pure pursuit on the walls-only path, ground-truth pose: this
    benchmark isolates AVOIDANCE from localisation) + the method under test."""

    def __init__(self, method, path, goal, fov_deg, use_vertical):
        self.method, self.path, self.goal = method, path, goal
        self.fov = math.radians(fov_deg)
        self.use_vertical = use_vertical
        self.sim = None
        cfg = AvoidanceConfig(lidar_fov_deg=fov_deg)
        if method == "cbf_only":
            cfg.use_vfh = False
        elif method == "vfh_only":
            cfg.use_cbf = False
        self.oa = ObstacleAvoider(cfg) if method in ("cbf_only", "vfh_only", "full") else None
        if method == "algo1":
            self.oa = DWAAvoider(DWAConfig(lidar_fov_deg=fov_deg))
        if method == "algoD":
            self.oa = ObstacleAvoider(AvoidanceConfig(lidar_fov_deg=DEPTH["hfov"], lidar_max_range=DEPTH["max"]))
        if method == "algoL":
            self.oa = ObstacleAvoider(AvoidanceConfig(lidar_fov_deg=fov_deg))
        self.ms = []
        self.statuses = {}
        self.seen = None
        self.replans = 0
        self.last_replan = -99.0
        self.prog_t, self.prog_xy = 0.0, None

    def reset(self, info):
        self.info = info

    def get_survivors(self):
        return []

    def debug_text(self):
        return f"method {self.method}"

    def _pursuit(self, d, speed=0.8, look=0.8):
        px, py = d.x, d.y
        dists = [math.hypot(x - px, y - py) for x, y in self.path]
        k = int(np.argmin(dists))
        self.path = self.path[k:]
        tgt = self.path[-1]
        for x, y in self.path:
            if math.hypot(x - px, y - py) >= look:
                tgt = (x, y)
                break
        ang = math.atan2(tgt[1] - py, tgt[0] - px)
        rel = wrap(ang - d.yaw)
        return speed * math.cos(rel), speed * math.sin(rel), rel

    def _record_obstacles(self):
        """Planner-side map update from the lidar (stand-in for the SLAM map)."""
        d, sim = self.sim.drone, self.sim
        if self.seen is None:
            self.seen = np.zeros_like(sim.w.walls)
        r = sim.last_ranges
        hit = r < sim.cs["lidar_max_range_m"] - 0.05
        a = sim.lidar_angles[hit] + d.yaw
        x = ((d.x + r[hit] * np.cos(a)) / RES).astype(int)
        y = ((d.y + r[hit] * np.sin(a)) / RES).astype(int)
        ok = (x >= 0) & (y >= 0) & (x < self.seen.shape[1]) & (y < self.seen.shape[0])
        self.seen[y[ok], x[ok]] = True

    def _maybe_replan(self, t, requested):
        d = self.sim.drone
        if self.prog_xy is None:
            self.prog_t, self.prog_xy = t, (d.x, d.y)
        no_progress = False
        if t - self.prog_t > 4.0:
            no_progress = math.hypot(d.x - self.prog_xy[0], d.y - self.prog_xy[1]) < 0.5
            self.prog_t, self.prog_xy = t, (d.x, d.y)
        if (requested or no_progress) and t - self.last_replan > 3.0:
            self.last_replan = t
            p = plan_walls_only(self.sim.w, (d.x, d.y), self.goal, 0.4, extra=self.seen)
            if p:
                self.path = p
                self.replans += 1

    def step(self, obs):
        d = self.sim.drone
        vz_cruise = self.info["cruise_alt"]
        if self.sim.steps % 2 == 0:
            self._record_obstacles()
        requested = False
        vx, vy, rel = self._pursuit(d)
        w = float(np.clip(1.5 * rel, -1.2, 1.2))   # face the direction of travel
        z_target = vz_cruise
        lid = obs["lidar"]
        if lid is not None and self.fov < 2 * math.pi - 1e-6:  # emulate a front lidar
            r = lid["ranges"].copy()
            r[np.abs(lid["angles"]) > self.fov / 2] = np.inf
            lid = dict(lid, ranges=r)
        m = self.method
        if m == "brake_repulse" and lid is not None:
            vx, vy = self._brake_repulse(vx, vy, lid["angles"], lid["ranges"])
        elif self.oa is not None:
            o = obs["odom"]
            scan = (lid["angles"], lid["ranges"]) if lid is not None and lid["new"] else None
            z_here = obs["tof"]["down"] if math.isfinite(obs["tof"]["down"]) else obs["baro_z"]
            extra = {}
            if m == "algo1" and self.sim.steps % 2 == 0:
                extra = {"depth_points": self._depth_cloud(), "depth_t": obs["t"]}
            if m == "algoD":          # depth camera is the ONLY obstacle sensor
                scan = None
                if self.sim.steps % 2 == 0:
                    p3 = self._depth_main_cloud()
                    vf = (math.radians(-DEPTH["vfov"] / 2), math.radians(DEPTH["vfov"] / 2))
                    extra = {"points3d": p3, "points3d_vfov": vf}
                    hz = self.oa.cloud.split(p3, z_here, vf)[0]
                    scan = virtual_scan(hz, 360, DEPTH["max"])
            if m == "algoL" and self.sim.steps % 3 == 0:   # ~7 Hz multi-zone ToF ring
                vf = (math.radians(MZTOF["pitch"] - MZTOF["fov"] / 2), math.radians(MZTOF["pitch"] + MZTOF["fov"] / 2))
                extra = {"points3d": self._mztof_cloud(), "points3d_vfov": vf}
            out = self.oa.update(obs["t"], (o["x"], o["y"], o["yaw"]), (vx, vy), scan=scan,
                                 tof=obs["tof"] if self.use_vertical else None,
                                 z=z_here, cruise_alt=vz_cruise, **extra)
            self.ms.append(self.oa.compute_ms)
            self.statuses[out.status] = self.statuses.get(out.status, 0) + 1
            vx, vy = out.vx, out.vy
            if out.z_target is not None:
                z_target = out.z_target
            if out.yaw_rate is not None:
                w = out.yaw_rate
            requested = out.replan
        self._maybe_replan(obs["t"], requested)
        h = obs["tof"]["down"] if math.isfinite(obs["tof"]["down"]) else obs["baro_z"]
        vz = float(np.clip(1.5 * (z_target - h), -0.5, 0.5))
        return {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": w}

    def _depth_main_cloud(self):
        """D435-class depth camera, level: nu x nv rays, sigma = 1 cm * d^2."""
        d = self.sim.drone
        az = np.radians(np.linspace(-DEPTH["hfov"] / 2, DEPTH["hfov"] / 2, DEPTH["nu"]))
        el = np.radians(np.linspace(-DEPTH["vfov"] / 2, DEPTH["vfov"] / 2, DEPTH["nv"]))
        A, E = np.meshgrid(az, el)
        A, E = A.ravel(), E.ravel()
        dirs, r = ray_batch(self.sim.w, d.x, d.y, d.z, d.yaw + A, E, DEPTH["max"])
        ok = np.isfinite(r) & (r >= DEPTH["min"])
        r = r[ok] * (1 + 0.01 * r[ok] * np.random.randn(ok.sum()))
        ce = np.cos(E[ok])
        return np.column_stack((r * ce * np.cos(A[ok]), r * ce * np.sin(A[ok]), r * np.sin(E[ok])))

    def _mztof_cloud(self):
        """Ring of 4 multi-zone ToF sensors (8x8 zones over 45 x 45 deg each)."""
        d = self.sim.drone
        g = np.radians(np.linspace(-MZTOF["fov"] / 2, MZTOF["fov"] / 2, MZTOF["zones"]))
        A0, E = np.meshgrid(g, g)
        A, El = [], []
        for yw in MZTOF["yaws"]:
            A.append(A0.ravel() + math.radians(yw))
            El.append(E.ravel() + math.radians(MZTOF["pitch"]))
        A, El = np.concatenate(A), np.concatenate(El)
        dirs, r = ray_batch(self.sim.w, d.x, d.y, d.z, d.yaw + A, El, MZTOF["max"])
        ok = np.isfinite(r)
        r = r[ok] + np.random.randn(ok.sum()) * (0.005 + 0.01 * r[ok])
        ce = np.cos(El[ok])
        return np.column_stack((r * ce * np.cos(A[ok]), r * ce * np.sin(A[ok]), r * np.sin(El[ok])))

    def _depth_cloud(self, nu=16, nv=12, hfov=70.0, vfov=55.0, max_r=4.0):
        """Forward depth camera (Algo 1 hardware): nu x nv rays -> body-frame
        points (x fwd, y left, z relative to the drone), 1% range noise."""
        d, w = self.sim.drone, self.sim.w
        pts = []
        for el in np.radians(np.linspace(-vfov / 2, vfov / 2, nv)):
            for az in np.radians(np.linspace(-hfov / 2, hfov / 2, nu)):
                r = ray3d(w, d.x, d.y, d.z, d.yaw + az, el, max_r)
                if math.isfinite(r):
                    r *= 1 + 0.01 * np.random.randn()
                    ce = math.cos(el)
                    pts.append((r * ce * math.cos(az), r * ce * math.sin(az), r * math.sin(el)))
        return np.array(pts) if pts else np.zeros((0, 3))

    @staticmethod
    def _brake_repulse(vx, vy, ang, rng):
        """The simulator baseline's reactive layer (explorer.py) for comparison."""
        ahead = rng[np.abs(ang) < math.radians(35)]
        gap = float(ahead.min()) - 0.25 - 0.15 if ahead.size else 9.0
        vx = min(vx, max(0.0, 0.8 * gap))
        bubble = 0.25 + 0.22
        close = rng < bubble
        if close.any():
            wgt = bubble - rng[close]
            dx, dy = -(wgt * np.cos(ang[close])).sum(), -(wgt * np.sin(ang[close])).sum()
            n = math.hypot(dx, dy)
            if n > 1e-6:
                mag = 2.5 * float(wgt.max())
                vx, vy = vx + mag * dx / n, vy + mag * dy / n
        return vx, vy


# ---------------------------------------------------------------------------
def make_trial(cfg, seed):
    """Random world + start/goal with a walls-only path of 6-14 m."""
    rng = random.Random(seed * 31 + 7)
    world = generate_world(copy.deepcopy(cfg), seed)
    ax0, ay0, ax1, ay1 = world.arena
    occ = inflate(world.grid, 0.6 / RES)
    for _ in range(300):
        s = (rng.uniform(ax0 + 0.6, ax1 - 0.6), rng.uniform(ay0 + 0.6, ay1 - 0.6))
        g = (rng.uniform(ax0 + 0.6, ax1 - 0.6), rng.uniform(ay0 + 0.6, ay1 - 0.6))
        if occ[int(s[1] / RES), int(s[0] / RES)] or occ[int(g[1] / RES), int(g[0] / RES)]:
            continue
        if any(math.hypot(m.x - s[0], m.y - s[1]) < 1.5 for m in world.movers):
            continue
        p = plan_walls_only(world, s, g, cfg["drone"]["radius_m"] + 0.15)
        if p and 6.0 <= path_length(p) <= 14.0:
            return world, s, g, p
    return None


def run_trial(args):
    cfg, seed, method, fov, use_vertical = args
    tr = make_trial(cfg, seed)
    if tr is None:
        return None
    world, s, g, path = tr
    pilot = BenchPilot(method, list(path), g, fov, use_vertical)
    sim = Simulation(world, pilot, seed=seed)
    pilot.sim = sim
    d = sim.drone
    d.x, d.y, d.z, d.landed = s[0], s[1], cfg["drone"]["cruise_alt_m"], False
    d.yaw = math.atan2(path[min(5, len(path) - 1)][1] - s[1], path[min(5, len(path) - 1)][0] - s[0])
    limit = 20.0 + path_length(path) / 0.8 * 2.0
    min_clear, result = math.inf, "timeout"
    while sim.t < limit:
        sim.step()
        min_clear = min(min_clear, float(np.min(sim.last_ranges)) - cfg["drone"]["radius_m"])
        if sim.crashed:
            result = "collision: " + sim.end_reason.replace("CRASH - ", "")
            break
        if math.hypot(d.x - g[0], d.y - g[1]) < 0.5:
            result = "success"
            break
    return {"seed": seed, "method": method, "result": result, "time_s": round(sim.t, 1),
            "path_m": round(path_length(path), 1), "min_clearance_m": round(min_clear, 3),
            "compute_ms": round(float(np.mean(pilot.ms)), 3) if pilot.ms else 0.0,
            "replans": pilot.replans,
            "statuses": pilot.statuses}


def summarize(rows):
    n = len(rows)
    succ = [r for r in rows if r["result"] == "success"]
    coll = [r for r in rows if r["result"].startswith("collision")]
    return {
        "trials": n,
        "success_pct": round(100 * len(succ) / n, 1),
        "collision_pct": round(100 * len(coll) / n, 1),
        "timeout_pct": round(100 * (n - len(succ) - len(coll)) / n, 1),
        "mean_speed_mps": round(float(np.mean([r["path_m"] / r["time_s"] for r in succ])), 2) if succ else None,
        "min_clearance_m": round(float(np.median([r["min_clearance_m"] for r in rows])), 3),
        "compute_ms": round(float(np.mean([r["compute_ms"] for r in rows])), 2),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="clutter", choices=list(SCENARIOS))
    ap.add_argument("--methods", default=",".join(METHODS))
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--fov", type=float, default=360.0, help="lidar FOV in degrees (e.g. 180 front lidar)")
    ap.add_argument("--no-vertical", action="store_true", help="disable the ToF vertical guard")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--out", default=None, help="write all trial rows to this JSON file")
    ap.add_argument("--sim-set", action="append", default=[],
                    help='simulator config override, e.g. sensors.lidar_layers_z_m=[-0.15,0,0.15]')
    a = ap.parse_args()

    with open(os.path.join(SIM, "config.json")) as f:
        cfg = json.load(f)
    deep_update(cfg, SCENARIOS[a.scenario])
    for kv in a.sim_set:
        key, val = kv.split("=", 1)
        node = cfg
        for k in key.split(".")[:-1]:
            node = node[k]
        node[key.split(".")[-1]] = json.loads(val)
    cfg["mission"]["time_limit_s"] = 1e9
    methods = a.methods.split(",")
    jobs = [(cfg, a.seed + k, m, a.fov, not a.no_vertical) for m in methods for k in range(a.trials)]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:
        rows = [r for r in ex.map(run_trial, jobs) if r]
    print(f"\nscenario={a.scenario}  fov={a.fov:.0f}  trials/method={a.trials}  ({time.time() - t0:.0f}s)\n")
    hdr = ["method", "success_pct", "collision_pct", "timeout_pct", "mean_speed_mps", "min_clearance_m", "compute_ms"]
    print(" | ".join(f"{h:>15}" for h in hdr))
    for m in methods:
        s = summarize([r for r in rows if r["method"] == m])
        print(" | ".join(f"{str(v):>15}" for v in [m] + [s[h] for h in hdr[1:]]))
    fails = {}
    for r in rows:
        if r["result"] != "success":
            fails.setdefault(r["method"], {}).setdefault(r["result"], 0)
            fails[r["method"]][r["result"]] += 1
    print("\nfailure breakdown:", json.dumps(fails))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(rows, f, indent=1)


if __name__ == "__main__":
    main()
