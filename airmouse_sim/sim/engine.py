"""Mission loop: dynamics, sensors, faults, battery, radio link, GCS
telemetry, event log and competition-style scoring."""
import math
import random

import numpy as np

from .dynamics import Drone
from .faults import Faults
from .sensors import Sensors
from .world import RES, _circle_hits


class Simulation:
    def __init__(self, world, algo, seed=0):
        self.w, self.algo = world, algo
        cfg = world.cfg
        self.cfg = cfg
        self.cd, self.cs, self.cm = cfg["drone"], cfg["sensors"], cfg["mission"]
        self.cb, self.cl, self.sc = cfg["battery"], cfg["link"], cfg["scoring"]
        self.dt = self.cm["dt_s"]
        self.rng = random.Random(seed)
        self.nprng = np.random.default_rng(seed)
        sx, sy, syaw = world.start
        self.drone = Drone(sx, sy, syaw, self.cd, cfg["wind"], self.dt, self.rng)
        self.faults = Faults(cfg["faults"], self.cm["time_limit_s"], seed)
        self.sens = Sensors(world, self.cs, self.dt, self.rng, self.nprng, self.faults)
        self.lidar_angles = self.sens.angles
        self.t, self.steps = 0.0, 0
        self.collisions, self.crashed, self.finished = 0, False, False
        self.end_reason = ""
        self.energy_wh = self.cb["capacity_wh"]
        self.power_w = 0.0
        self.link_up, self._next_link = True, 0.0
        self.gcs = {"t": None, "map": None, "pose": None, "survivors": [], "state": "", "battery": 1.0}
        self._next_tel = 0.0
        self.events = []
        self._tagged = []
        self._state = ""
        self._batt_marks = [0.5, 0.2, 0.1]
        self.true_trail, self.est_trail = [(sx, sy)], [(0.0, 0.0)]
        self.alt_trail = []
        self.last_ranges = self.sens.true_ranges
        self.last_cmd = (0.0, 0.0, 0.0, 0.0)
        self.tof = {}
        self.flow_quality = 1.0
        self._in_contact = False
        self.cov_res = 0.2
        self.seen = np.zeros((int(world.height / self.cov_res) + 1,
                              int(world.width / self.cov_res) + 1), dtype=bool)
        self._free_cells = self._arena_free_mask()
        c = self.cs
        algo.reset({
            "dt": self.dt,
            "time_limit": self.cm["time_limit_s"],
            "drone_radius": self.cd["radius_m"],
            "half_height": self.cd["half_height_m"],
            "cruise_alt": self.cd["cruise_alt_m"],
            "max_speed": self.cd["max_speed_mps"],
            "max_vz": self.cd["max_vz_mps"],
            "max_yaw_rate": self.cd["max_yaw_rate_rps"],
            "lidar_angles": self.lidar_angles.copy(),
            "lidar_max_range": c["lidar_max_range_m"],
            "lidar_rate_hz": c["lidar_rate_hz"],
            "tof_fwd_pitch": math.radians(c["tof_fwd_pitch_deg"]),
            "tof_max_range": c["tof_max_range_m"],
            "camera": {"type": c["camera_type"], "width": self.sens.cam_W, "height": self.sens.cam_H,
                       "f_px": self.sens.f, "hfov": math.radians(c["camera_hfov_deg"]),
                       "vfov": math.radians(c["camera_vfov_deg"]),
                       "pitch": math.radians(c["camera_pitch_deg"]),
                       "range_hint": float(c["detector_recall_curve"][-1][0])},
            "battery_capacity_wh": self.cb["capacity_wh"],
            "start_outside": self.cd["start_outside_m"],
            "exit_radius": self.cm["exit_radius_m"],
        })
        self.log("Mission start (on ground, outside the entry)")

    # ---- utilities ----------------------------------------------------------
    def log(self, msg):
        self.events.append((round(self.t, 1), msg))

    def start_to_world(self, x, y):
        sx, sy, syaw = self.w.start
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * x - s * y, sy + s * x + c * y

    def algo_pose(self):
        try:
            p = self.algo.get_pose()
        except Exception:
            p = None
        o = self.sens
        return p if p else (o.ox, o.oy, o.oyaw)

    def _arena_free_mask(self):
        ax0, ay0, ax1, ay1 = self.w.arena
        r = self.cov_res
        m = np.zeros_like(self.seen)
        occ = self.w.occ_at(self.cd["cruise_alt_m"])
        for i in range(m.shape[0]):
            y = (i + 0.5) * r
            if not ay0 < y < ay1:
                continue
            for j in range(m.shape[1]):
                x = (j + 0.5) * r
                if ax0 < x < ax1 and not occ[int(y / RES), int(x / RES)]:
                    m[i, j] = True
        return m

    def _mark_seen(self, ranges):
        d = self.drone
        ang = self.lidar_angles[::4] + d.yaw
        rr = np.minimum(ranges[::4], self.cs["lidar_max_range_m"])
        for k in range(len(ang)):
            ts = np.arange(0, rr[k], self.cov_res)
            xs = ((d.x + np.cos(ang[k]) * ts) / self.cov_res).astype(int)
            ys = ((d.y + np.sin(ang[k]) * ts) / self.cov_res).astype(int)
            ok = (xs >= 0) & (ys >= 0) & (xs < self.seen.shape[1]) & (ys < self.seen.shape[0])
            self.seen[ys[ok], xs[ok]] = True

    def _update_link(self):
        if self.t < self._next_link:
            return
        self._next_link = self.t + 0.5
        d, (sx, sy, _) = self.drone, self.w.start
        dist = math.hypot(d.x - sx, d.y - sy)
        n = max(2, int(dist / 0.05))
        xs = np.linspace(sx, d.x, n)
        ys = np.linspace(sy, d.y, n)
        on = self.w.walls[(ys / RES).astype(int), (xs / RES).astype(int)]
        walls = int(np.count_nonzero(on[1:] & ~on[:-1]))
        up = dist <= self.cl["range_m"] and walls <= self.cl["max_walls"] and \
            not self.faults.active(self.t, "link_outage")
        if up != self.link_up:
            self.log("GCS link REGAINED" if up else f"GCS link LOST ({walls} walls, {dist:.0f} m)")
        self.link_up = up

    # ---- observation ----------------------------------------------------------
    def _observe(self):
        d, s = self.drone, self.sens
        scan = s.lidar(d, self.t)
        if isinstance(scan, np.ndarray):
            self.last_ranges = s.true_ranges
            self._scan = scan
            if self.steps % 4 == 0:
                self._mark_seen(s.true_ranges)
        lidar_ok = not isinstance(scan, str) and not self.faults.active(self.t, "lidar_blackout")
        cam = s.camera(d, self.t)
        cam_ok = cam != "blackout" and not self.faults.active(self.t, "camera_blackout")
        self.tof = s.tof(d)
        odo = s.odometry(d, self.t, self.dt)
        self.flow_quality = odo["flow_quality"]
        return {
            "t": self.t,
            "dt": self.dt,
            "time_left": self.cm["time_limit_s"] - self.t,
            "lidar": None if not lidar_ok or not hasattr(self, "_scan") else
            {"angles": self.lidar_angles, "ranges": self._scan,
             "max_range": self.cs["lidar_max_range_m"], "new": isinstance(scan, np.ndarray)},
            "odom": odo,
            "tof": dict(self.tof),
            "baro_z": s.baro(d, self.dt),
            "landed": d.landed,
            "detections": None if not cam_ok else (cam or []),
            "battery": {"frac": max(0.0, self.energy_wh / self.cb["capacity_wh"]),
                        "power_w": self.power_w, "energy_wh": self.energy_wh},
            "link": self.link_up,
            "health": {"lidar": lidar_ok, "camera": cam_ok,
                       "flow_quality": odo["flow_quality"]},
        }

    # ---- main step ----------------------------------------------------------
    def step(self):
        if self.finished:
            return
        d, dt = self.drone, self.dt
        obs = self._observe()
        cmd = self.algo.step(obs) or {}
        if isinstance(cmd, (tuple, list)):
            cmd = dict(zip(("vx", "vy", "vz", "yaw_rate", "done"), cmd))
        st = cmd.get("state")
        if st and st != self._state:
            self.log(f"STATE {self._state or '-'} -> {st}")
            self._state = st
        if cmd.get("done"):
            self._finish("algorithm declared mission complete")
            return
        c = (float(cmd.get("vx", 0)), float(cmd.get("vy", 0)),
             float(cmd.get("vz", 0)), float(cmd.get("yaw_rate", 0)))
        self.last_cmd = c
        was_landed = d.landed
        floor = self.w.surface_below(d.x, d.y, d.z + 0.05)
        ceil = self.w.surface_above(d.x, d.y, d.z)
        hit = d.step(c, dt, floor, ceil)
        if was_landed and not d.landed:
            self.log("Takeoff")
        elif d.landed and not was_landed:
            self.log(f"Landed (z={d.z:.2f} m)")
        self.w.step_movers(dt)
        prev_t = self.t
        self.t += dt
        self.steps += 1
        for m in self.faults.transitions(prev_t, self.t):
            self.log(m)

        # battery
        factor = self.faults.value(self.t, "battery_fast_drain", 1.0)
        self.power_w = d.power_w(self.cb) * factor
        self.energy_wh -= self.power_w * dt / 3600.0
        frac = self.energy_wh / self.cb["capacity_wh"]
        while self._batt_marks and frac <= self._batt_marks[0]:
            self.log(f"Battery {int(self._batt_marks.pop(0) * 100)}%")

        self._update_link()
        self._telemetry()

        if self.steps % 2 == 0:
            self.true_trail.append((d.x, d.y))
            ap = self.algo_pose()
            self.est_trail.append((ap[0], ap[1]))
            self.alt_trail.append(d.z)
            self.alt_trail = self.alt_trail[-600:]

        # collisions (footprint vs obstacles in the drone's height band)
        hh = self.cd["half_height_m"]
        coll = hit
        if not d.landed:
            if _circle_hits(self.w.occ_at(d.z, hh * 0.8), d.x, d.y, self.cd["radius_m"]):
                coll = coll or "obstacle"
            for m in self.w.movers:
                if d.z - hh < m.h and math.hypot(m.x - d.x, m.y - d.y) < m.r + self.cd["radius_m"]:
                    coll = coll or "moving obstacle"
        if coll and not self._in_contact:
            self.collisions += 1
            self.log(f"COLLISION with {coll} at z={d.z:.2f} m")
            if self.cm["crash_ends_mission"]:
                self.crashed = True
                self._finish(f"CRASH - {coll}")
                return
        self._in_contact = bool(coll)
        if self.energy_wh <= 0:
            self._finish("BATTERY DEPLETED - forced landing")
        elif self.t >= self.cm["time_limit_s"]:
            self._finish("time limit")

    def _telemetry(self):
        """What the ground station receives (only while the link is up)."""
        if self.steps % 10 == 0:
            for r in self.reported_world():
                if all(math.hypot(r[0] - a, r[1] - b) > 1.0 for a, b in self._tagged):
                    self._tagged.append(r)
                    self.log(f"Survivor tag #{len(self._tagged)} at ({r[0]:.1f}, {r[1]:.1f})")
        if not self.link_up or self.t < self._next_tel:
            return
        self._next_tel = self.t + 1.0 / self.cl["telemetry_rate_hz"]
        try:
            m = self.algo.get_map()
        except Exception:
            m = None
        try:
            local = list(self.algo.get_survivors() or [])
        except Exception:
            local = []
        self.gcs = {"t": self.t, "map": m, "pose": self.algo_pose(), "survivors": local,
                    "state": self._state, "battery": self.energy_wh / self.cb["capacity_wh"]}

    def kill(self):
        self.log("OPERATOR KILL SWITCH")
        self._finish("operator kill switch")

    def _finish(self, why):
        if not self.finished:
            self.finished, self.end_reason = True, why
            self.log(f"END: {why}")

    def run(self, max_steps=None):
        while not self.finished and (max_steps is None or self.steps < max_steps):
            self.step()
        return self.score()

    # ---- scoring ------------------------------------------------------------
    def reported_world(self):
        try:
            reps = list(self.algo.get_survivors() or [])
        except Exception:
            reps = []
        return [self.start_to_world(p[0], p[1]) for p in reps]

    def match(self):
        """Greedy nearest matching of reported survivors to true ones."""
        tol = self.cm["survivor_match_tol_m"]
        reps = self.reported_world()
        pairs = sorted(((math.hypot(r[0] - s.x, r[1] - s.y), ri, s.id)
                        for ri, r in enumerate(reps) for s in self.w.survivors))
        used_r, used_s, matched = set(), set(), []
        for dist, ri, sid in pairs:
            if dist <= tol and ri not in used_r and sid not in used_s:
                used_r.add(ri)
                used_s.add(sid)
                matched.append((sid, dist))
        return reps, matched

    def score(self):
        reps, matched = self.match()
        d, sc, cm = self.drone, self.sc, self.cm
        sx, sy, syaw = self.w.start
        at_exit = math.hypot(d.x - sx, d.y - sy) <= cm["exit_radius_m"]
        landed = d.landed or d.z < cm["landed_alt_m"]
        exited = at_exit and landed and self.t > 5 and not self.crashed
        cov = (self.seen & self._free_cells).sum() / max(1, self._free_cells.sum())
        c, s = math.cos(-syaw), math.sin(-syaw)
        tx, ty = d.x - sx, d.y - sy
        tsx, tsy = c * tx - s * ty, s * tx + c * ty
        drift = math.hypot(self.sens.ox - tsx, self.sens.oy - tsy)
        ap = self.algo_pose()
        pose_err = math.hypot(ap[0] - tsx, ap[1] - tsy)
        batt_dead = self.end_reason.startswith("BATTERY")

        # competition-style points (weights in config.json "scoring")
        pts = 0.0
        full, tol = sc["full_accuracy_m"], cm["survivor_match_tol_m"]
        for _, e in matched:
            pts += sc["per_survivor"] * (1.0 if e <= full else 1.0 - 0.5 * (e - full) / max(1e-6, tol - full))
        pts += sc["per_false_report"] * (len(reps) - len(matched))
        pts += sc["per_collision"] * self.collisions + (sc["crash"] if self.crashed else 0)
        if exited:
            pts += sc["exit_and_land_bonus"] + sc["time_bonus_per_s_left"] * max(0.0, cm["time_limit_s"] - self.t)
        pts += sc["coverage_bonus_max"] * cov
        if batt_dead:
            pts += sc["battery_depleted"]
        return {
            "seed": self.w.seed,
            "score": round(pts, 1),
            "survivors_total": len(self.w.survivors),
            "survivors_found": len(matched),
            "false_reports": len(reps) - len(matched),
            "mean_loc_error_m": round(sum(e for _, e in matched) / len(matched), 3) if matched else None,
            "collisions": self.collisions,
            "crashed": self.crashed,
            "exited_and_landed": bool(exited),
            "time_s": round(self.t, 1),
            "battery_left_pct": round(100 * max(0.0, self.energy_wh) / self.cb["capacity_wh"], 1),
            "coverage_pct": round(100 * cov, 1),
            "odom_drift_m": round(drift, 2),
            "pose_error_m": round(pose_err, 2),
            "end_reason": self.end_reason or "running",
        }
