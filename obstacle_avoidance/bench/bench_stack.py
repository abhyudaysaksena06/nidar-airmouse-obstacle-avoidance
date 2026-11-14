"""OA-Stack benchmark: the zip's budget sensor stack in the AirMouse simulator.

Sensors simulated exactly as the guide describes them (values marked GUIDE in oa_stack/config.py):
  * RPLIDAR C1: 500 beams (0.72 deg), 10 Hz, 0.15-12 m, sigma 15 mm (+-30 mm), 1% dropouts,
    scan plane 8 cm above base_link; the plane TILTS with the drone's roll/pitch
  * VL53L5CX UP (+20 deg) and DOWN (-35 deg): the guide's zone_directions() 8x8 rays, rotated
    with the drone attitude, 3 m range (1.5 m in the "bright" scenario), ~15 Hz,
    noise 5 mm + 1%, 5% invalid zones, ESP32 'U,'/'D,' integer-mm format
  * EKF height above the floor: true height + slowly varying error (sigma 4.7 cm, tau 4 s),
    as in the guide's MuJoCo simulation
  * pose: SLAM stand-in = true pose + slowly drifting error (sigma 1.5 cm), as in the guide's sim
  * /map: occupancy built from accumulated lidar hits (stand-in for Cartographer)
Attitude is derived from the commanded acceleration (a multirotor tilts to accelerate).

Planner: A* on the walls-only map + pure pursuit at 0.8 m/s (replans on request or no progress).
Methods: guide (the guide's push-away + ToF guards), stack (OA-Stack), and ablations.

  python bench/bench_stack.py --scenario guide_maze --trials 30
  python bench/bench_stack.py --scenario guide_people --methods guide,stack
"""
import argparse
import copy
import json
import math
import os
import random
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SIM = os.path.join(os.path.dirname(ROOT), "airmouse_sim")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, SIM)

from bench_avoidance import deep_update, path_length, plan_walls_only, ray_batch  # noqa: E402
from oa_stack import GuideBaseline, StackAvoider, StackConfig  # noqa: E402
from oa_stack.tof import zone_directions  # noqa: E402
from sim.engine import Simulation  # noqa: E402
from sim.world import RES, generate_world, inflate  # noqa: E402

G = 9.81
# The guide's MuJoCo world: 1.6 m cells (~1.5 m corridors), 2.4 m walls, 1.0 m low boxes,
# thin ducts 1.13-1.19 m above the floor, cruise 1.0 m.
BASE = {
    "arena": {"cell_m": 1.6, "wall_height_m": 2.4, "width_m": 16.0, "height_m": 12.8, "rooms": 2,
              "door_min_m": 1.2, "door_max_m": 1.2, "dark_cell_frac": 0.0, "textureless_zones": 0},
    "obstacles": {"static_count": 6, "box_size_m": [0.4, 0.8], "box_height_m": [1.0, 1.0],
                  "cylinder_radius_m": [0.15, 0.3], "hanging_count": 5, "hanging_length_m": [1.2, 1.6],
                  "hanging_thickness_m": [0.06, 0.06], "hanging_bottom_m": [1.13, 1.13], "moving_count": 0},
    "drone": {"half_height_m": 0.04, "cruise_alt_m": 1.0, "max_speed_mps": 1.0},
    "sensors": {"lidar_beams": 500, "lidar_rate_hz": 10.0, "lidar_max_range_m": 12.0,
                "lidar_min_range_m": 0.15, "lidar_noise_m": 0.015, "lidar_dropout_prob": 0.01,
                "lidar_layers_z_m": [0.08]},
    "wind": {"gust_std_mps": 0.1},
}
SCENARIOS = {
    "guide_maze": {},
    "guide_people": {"obstacles": {"moving_count": 2, "moving_speed_mps": 0.4}},
    "bright_hall": {"_tof_range": 1.5},
    "mixed_clutter": {"obstacles": {"static_count": 12, "box_height_m": [0.5, 2.0], "hanging_count": 5,
                                    "hanging_bottom_m": [1.1, 1.5], "moving_count": 1}},
}
METHODS = ["guide", "stack", "stack_nomem", "stack_nowall", "stack_noinband", "stack_map", "stack_noedge"]


class OU:
    """Slowly varying error (Ornstein-Uhlenbeck), like a barometer or SLAM drift."""

    def __init__(self, sigma, tau, rng, dim=1):
        self.s, self.tau, self.rng, self.x = sigma, tau, rng, np.zeros(dim)

    def step(self, dt):
        self.x += -self.x / self.tau * dt + self.s * math.sqrt(2 * dt / self.tau) * self.rng.standard_normal(len(self.x))
        return self.x


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class StackPilot:
    def __init__(self, method, path, goal, scfg, tof_range, seed):
        self.method, self.path, self.goal = method, path, goal
        self.cfg = scfg
        self.tof_range = tof_range
        self.np = np.random.default_rng(seed + 991)
        self.h_err = OU(0.047, 4.0, self.np)
        self.p_err = OU(0.015, 10.0, self.np, 2)
        c = copy.deepcopy(scfg)
        if method == "stack_nomem":
            c.use_memory = False
        if method == "stack_nowall":
            c.use_wall_check = False
        if method == "stack_noinband":
            c.use_inband = False
        if method == "stack_nomap":
            c.use_map = False
        if method == "stack_map":
            c.use_map = True
        if method == "stack_noedge":
            c.unconfirmed_top_margin = 0.0
        self.alg = GuideBaseline(c) if method == "guide" else StackAvoider(c)
        self.dirs = {"U": zone_directions(scfg.up_pitch_deg, scfg.tof_fov_deg),
                     "D": zone_directions(scfg.down_pitch_deg, scfg.tof_fov_deg)}
        self.xyz = {"U": np.array(scfg.up_xyz), "D": np.array(scfg.down_xyz)}
        self.sim = None
        self.seen = None
        self.replans, self.last_replan = 0, -99.0
        self.prog_t, self.prog_xy = 0.0, None
        self.ms, self.statuses, self.vert = [], {}, {}
        self.roll = self.pitch = 0.0

    # --- simulator protocol ---------------------------------------------------
    def reset(self, info):
        self.info = info

    def get_survivors(self):
        return []

    def debug_text(self):
        return self.method

    # --- sensors ----------------------------------------------------------------
    def _attitude(self):
        d = self.sim.drone
        c, s = math.cos(d.yaw), math.sin(d.yaw)
        axb, ayb = c * d.ax + s * d.ay, -s * d.ax + c * d.ay
        self.pitch = math.atan2(axb, G)          # nose down to accelerate forward (FLU +pitch)
        self.roll = -math.atan2(ayb, G)

    def _lidar(self, obs):
        lid = obs["lidar"]
        if lid is None or not lid["new"]:
            return None
        a, r = lid["angles"], lid["ranges"].copy()
        # the real scan plane tilts with the drone: walls look farther (guide 11.1)
        cr, sr, cp, sp = math.cos(self.roll), math.sin(self.roll), math.cos(self.pitch), math.sin(self.pitch)
        dz = -sp * np.cos(a) + cp * sr * np.sin(a)
        r = np.where(np.isfinite(r), r / np.sqrt(np.clip(1 - dz * dz, 0.05, 1)), r)
        return a, r

    def _tof(self):
        d = self.sim.drone
        out = []
        R = rot(self.roll, self.pitch, d.yaw)
        for tag in ("U", "D"):
            if self.np.random() > 0.75:          # ~15 Hz at a 20 Hz control loop
                continue
            org = np.array([d.x, d.y, d.z]) + R @ self.xyz[tag]
            dirs = self.dirs[tag] @ R.T
            yaw = np.arctan2(dirs[:, 1], dirs[:, 0])
            el = np.arcsin(np.clip(dirs[:, 2], -1, 1))
            _, r = ray_batch(self.sim.w, org[0], org[1], org[2], yaw, el, self.tof_range)
            r = r + self.np.standard_normal(64) * (0.005 + 0.01 * np.where(np.isfinite(r), r, 0))
            r[self.np.random(64) < 0.05] = np.inf
            out.append((tag, [int(v * 1000) if math.isfinite(v) and v > 0.02 else -1 for v in r]))
        return out

    def _map(self):
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

    # --- planner ------------------------------------------------------------------
    def _pursuit(self, speed=0.8, look=0.8):
        d = self.sim.drone
        dists = [math.hypot(x - d.x, y - d.y) for x, y in self.path]
        self.path = self.path[int(np.argmin(dists)):]
        tgt = self.path[-1]
        for x, y in self.path:
            if math.hypot(x - d.x, y - d.y) >= look:
                tgt = (x, y)
                break
        rel = (math.atan2(tgt[1] - d.y, tgt[0] - d.x) - d.yaw + math.pi) % (2 * math.pi) - math.pi
        return speed * math.cos(rel), speed * math.sin(rel), rel

    def _maybe_replan(self, t, requested):
        d = self.sim.drone
        if self.prog_xy is None:
            self.prog_t, self.prog_xy = t, (d.x, d.y)
        stuck = False
        if t - self.prog_t > 4.0:
            stuck = math.hypot(d.x - self.prog_xy[0], d.y - self.prog_xy[1]) < 0.5
            self.prog_t, self.prog_xy = t, (d.x, d.y)
        if (requested or stuck) and t - self.last_replan > 3.0:
            self.last_replan = t
            p = plan_walls_only(self.sim.w, (d.x, d.y), self.goal, 0.4, extra=self.seen)
            if p:
                self.path = p
                self.replans += 1

    # --- control --------------------------------------------------------------------
    def step(self, obs):
        sim, d, dt = self.sim, self.sim.drone, obs["dt"]
        self._attitude()
        if sim.steps % 2 == 0:
            self._map()
        vx, vy, rel = self._pursuit()
        w = float(np.clip(1.5 * rel, -1.2, 1.2))
        height = d.z + float(self.h_err.step(dt)[0])
        pe = self.p_err.step(dt)
        pose = (d.x + pe[0], d.y + pe[1], d.yaw)
        grid = None
        if self.seen is not None:
            grid = (self.seen.astype(np.int16) * 100, RES, 0.0, 0.0)
        scan = self._lidar(obs)
        frames = self._tof()
        t0 = time.perf_counter()
        out = self.alg.update(obs["t"], pose, (vx, vy), scan=scan, tof_frames=frames, height=height,
                              roll=self.roll, pitch=self.pitch, cruise_alt=self.info["cruise_alt"], grid=grid)
        self.ms.append(1000 * (time.perf_counter() - t0))
        if isinstance(out, dict):
            vx, vy, zt, requested = out["vx"], out["vy"], out["z_target"], False
        else:
            vx, vy, zt, requested = out.vx, out.vy, out.z_target, out.replan
            self.statuses[out.status] = self.statuses.get(out.status, 0) + 1
            v = out.info.get("vertical", "none")
            self.vert[v] = self.vert.get(v, 0) + 1
        self._maybe_replan(obs["t"], requested)
        zt = self.info["cruise_alt"] if zt is None else zt
        vz = float(np.clip(1.5 * (zt - height), -0.5, 0.5))
        return {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": w}


def make_trial(cfg, seed):
    """Random world + start/goal 5-14 m apart along the walls-only path (1.5 m corridors)."""
    rng = random.Random(seed * 31 + 7)
    world = generate_world(copy.deepcopy(cfg), seed)
    ax0, ay0, ax1, ay1 = world.arena
    occ = inflate(world.grid, 0.42 / RES)
    for _ in range(800):
        s = (rng.uniform(ax0 + 0.5, ax1 - 0.5), rng.uniform(ay0 + 0.5, ay1 - 0.5))
        g = (rng.uniform(ax0 + 0.5, ax1 - 0.5), rng.uniform(ay0 + 0.5, ay1 - 0.5))
        if occ[int(s[1] / RES), int(s[0] / RES)] or occ[int(g[1] / RES), int(g[0] / RES)]:
            continue
        if any(math.hypot(m.x - s[0], m.y - s[1]) < 1.5 for m in world.movers):
            continue
        p = plan_walls_only(world, s, g, cfg["drone"]["radius_m"] + 0.12)
        if p and 5.0 <= path_length(p) <= 14.0:
            return world, s, g, p
    return None


def crash_kind(world, x, y):
    best, kind = 9.0, "wall"
    for r in world.rects:
        dx = max(r.x0 - x, 0, x - r.x1)
        dy = max(r.y0 - y, 0, y - r.y1)
        dd = math.hypot(dx, dy)
        if dd < best:
            best, kind = dd, {"wall": "wall", "fence": "wall", "box": "low box", "hanging": "duct"}[r.kind]
    for c in world.cylinders:
        dd = math.hypot(c.x - x, c.y - y) - c.r
        if dd < best:
            best, kind = dd, "pillar"
    return kind


def build_cfg(scenario):
    with open(os.path.join(SIM, "config.json")) as f:
        cfg = json.load(f)
    deep_update(cfg, copy.deepcopy(BASE))
    sc = copy.deepcopy(SCENARIOS[scenario])
    tof_range = sc.pop("_tof_range", 3.0)
    deep_update(cfg, sc)
    cfg["mission"]["time_limit_s"] = 1e9
    return cfg, tof_range


def run_trial(args):
    scenario, seed, method = args
    cfg, tof_range = build_cfg(scenario)
    tr = make_trial(cfg, seed)
    if tr is None:
        return None
    world, s, g, path = tr
    scfg = StackConfig(drone_radius=cfg["drone"]["radius_m"], tof_max_range=tof_range)
    pilot = StackPilot(method, list(path), g, scfg, tof_range, seed)
    sim = Simulation(world, pilot, seed=seed)
    pilot.sim = sim
    d = sim.drone
    d.x, d.y, d.z, d.landed = s[0], s[1], cfg["drone"]["cruise_alt_m"], False
    d.yaw = math.atan2(path[min(5, len(path) - 1)][1] - s[1], path[min(5, len(path) - 1)][0] - s[0])
    limit = 25.0 + path_length(path) / 0.8 * 2.5
    min_clear, result = math.inf, "timeout"
    while sim.t < limit:
        sim.step()
        min_clear = min(min_clear, float(np.min(sim.last_ranges)) - cfg["drone"]["radius_m"])
        if sim.crashed:
            who = "person" if "moving" in sim.end_reason else crash_kind(world, d.x, d.y)
            result = "collision: " + who
            break
        if math.hypot(d.x - g[0], d.y - g[1]) < 0.5:
            result = "success"
            break
    return {"scenario": scenario, "seed": seed, "method": method, "result": result, "time_s": round(sim.t, 1),
            "path_m": round(path_length(path), 1), "min_clearance_m": round(min_clear, 3),
            "compute_ms_mean": round(float(np.mean(pilot.ms)), 3) if pilot.ms else 0.0,
            "compute_ms_p99": round(float(np.percentile(pilot.ms, 99)), 3) if pilot.ms else 0.0,
            "replans": pilot.replans, "statuses": pilot.statuses, "vertical": pilot.vert}


def wilson(k, n, z=1.96):
    if n == 0:
        return (0, 0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return round(100 * (c - h)), round(100 * (c + h))


def summarize(rows):
    n = len(rows)
    s = sum(r["result"] == "success" for r in rows)
    c = sum(r["result"].startswith("collision") for r in rows)
    sp = [r["path_m"] / r["time_s"] for r in rows if r["result"] == "success"]
    return {"trials": n, "success_pct": round(100 * s / n, 1), "success_ci": wilson(s, n),
            "collision_pct": round(100 * c / n, 1), "collision_ci": wilson(c, n),
            "timeout_pct": round(100 * (n - s - c) / n, 1),
            "mean_speed_mps": round(statistics.mean(sp), 2) if sp else None,
            "median_min_clearance_m": round(statistics.median([r["min_clearance_m"] for r in rows]), 3),
            "compute_ms_mean": round(statistics.mean([r["compute_ms_mean"] for r in rows]), 2),
            "compute_ms_p99": round(max(r["compute_ms_p99"] for r in rows), 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="guide_maze", choices=list(SCENARIOS))
    ap.add_argument("--methods", default="guide,stack")
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--seed", type=int, default=200)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    methods = a.methods.split(",")
    jobs = [(a.scenario, a.seed + k, m) for m in methods for k in range(a.trials)]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:
        rows = [r for r in ex.map(run_trial, jobs) if r]
    print(f"\nscenario={a.scenario}  trials/method={a.trials}  ({time.time() - t0:.0f}s)")
    hdr = ["method", "success_pct", "success_ci", "collision_pct", "collision_ci", "timeout_pct",
           "mean_speed_mps", "median_min_clearance_m", "compute_ms_mean", "compute_ms_p99"]
    print(" | ".join(f"{h:>14}" for h in hdr))
    for m in methods:
        sm = summarize([r for r in rows if r["method"] == m])
        print(" | ".join(f"{str(v):>14}" for v in [m] + [sm[h] for h in hdr[1:]]))
    fails = {}
    for r in rows:
        if r["result"] != "success":
            fails.setdefault(r["method"], {}).setdefault(r["result"], 0)
            fails[r["method"]][r["result"]] += 1
    print("failures:", json.dumps(fails))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(rows, f, indent=1)


if __name__ == "__main__":
    main()
