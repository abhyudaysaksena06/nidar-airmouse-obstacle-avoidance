"""OA-Stack flying the drone in Gazebo, with a live evaluation in the terminal.

    gz sim -r /tmp/airmouse.sdf &                         # world from gz_world.py (same scenario/seed)
    python3 gazebo/oa_gazebo.py --scenario mixed_clutter --seed 207

    python3 gazebo/oa_gazebo.py --selftest                # no Gazebo: same loop on ray-cast sensors

Every 50 ms: latest Gazebo lidar scan + ToF frames + drone pose -> OA-Stack -> velocity command
(/model/airmouse/cmd_vel). Gazebo's gpu_lidar 8x8 scans are re-ordered into the VL53L5CX zone order
(tof.zone_directions), so OA-Stack receives exactly what the ESP32 would send.
Evaluation (success / collision / timeout, clearance, speed, sensor rates) is judged against the
known world geometry, not against what the drone believes.
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "bench"))
sys.path.insert(0, os.path.join(HERE, ".."))

import bench_stack as B  # noqa: E402
from oa_stack import StackAvoider, StackConfig  # noqa: E402
from oa_stack.tof import zone_directions  # noqa: E402
from sim.world import RES  # noqa: E402

WORLD = "airmouse"
TOF_HALF = math.radians(45.0 / 2)              # guide zone_directions(): outer zone centres at +-22.5 deg


class Scan:
    """The fields of gz.msgs.LaserScan that we use (the fake sensors fill the same ones)."""

    def __init__(self, angle_min, angle_step, count, vmin, vstep, vcount, ranges, range_max):
        self.angle_min, self.angle_step, self.count = angle_min, angle_step, count
        self.vertical_angle_min, self.vertical_angle_step, self.vertical_count = vmin, vstep, vcount
        self.ranges, self.range_max = ranges, range_max


def beam_dirs(msg, pitch_up_deg):
    """Body-frame unit vector of every beam of a gpu_lidar scan (ranges index = row * count + col)."""
    vc = max(1, msg.vertical_count)
    h = msg.angle_min + msg.angle_step * np.arange(msg.count)
    v = msg.vertical_angle_min + msg.vertical_angle_step * np.arange(vc)
    V, Hh = np.meshgrid(v, h, indexing="ij")
    d = np.stack((np.cos(V) * np.cos(Hh), np.cos(V) * np.sin(Hh), np.sin(V)), -1).reshape(-1, 3)
    p = math.radians(pitch_up_deg)
    return np.column_stack((math.cos(p) * d[:, 0] - math.sin(p) * d[:, 2], d[:, 1],
                            math.sin(p) * d[:, 0] + math.cos(p) * d[:, 2]))


class TofReorder:
    """gpu_lidar 8x8 -> 64 mm values in VL53L5CX zone order (-1 = no target)."""

    def __init__(self, pitch_up_deg, fov_deg, calib):
        self.pitch, self.zones = pitch_up_deg, zone_directions(pitch_up_deg, fov_deg, **calib)
        self.index = None

    def __call__(self, msg):
        if self.index is None:
            self.index = np.argmax(self.zones @ beam_dirs(msg, self.pitch).T, axis=1)
        r = np.asarray(msg.ranges, float)[self.index]
        ok = np.isfinite(r) & (r > 0.02) & (r < msg.range_max - 1e-3)
        return [int(v * 1000) if k else -1 for v, k in zip(r, ok)]


def collided(world, x, y, z, radius, bottom, top):
    """True if the drone's cylinder overlaps any obstacle cell or a person."""
    k = int(math.ceil(radius / RES))
    i0, j0 = int(y / RES), int(x / RES)
    H, W = world.zlo.shape
    for i in range(max(i0 - k, 0), min(i0 + k + 1, H)):
        for j in range(max(j0 - k, 0), min(j0 + k + 1, W)):
            cx, cy = (j + 0.5) * RES, (i + 0.5) * RES
            if math.hypot(cx - x, cy - y) > radius:
                continue
            if world.zlo[i, j] <= z + top and world.zhi[i, j] >= z + bottom:
                return True
    return any(math.hypot(m.x - x, m.y - y) < radius + m.r and z + bottom <= m.h for m in world.movers)


# ---------------------------------------------------------------------------------- Gazebo I/O
class GzIO:
    def __init__(self, n_movers):
        mods = None
        for t, m in (("transport13", "msgs10"), ("transport14", "msgs11"), ("transport12", "msgs9")):
            try:
                tr = __import__(f"gz.{t}", fromlist=["Node"])
                mods = {k: __import__(f"gz.{m}.{k}_pb2", fromlist=[k]) for k in ("laserscan", "pose_v", "twist", "pose", "boolean")}
                break
            except ImportError:
                continue
        if mods is None:
            raise SystemExit("Gazebo Python bindings not found: sudo apt install python3-gz-transport13 python3-gz-msgs10")
        self.LaserScan, self.Pose_V = mods["laserscan"].LaserScan, mods["pose_v"].Pose_V
        self.Twist, self.Pose, self.Boolean = mods["twist"].Twist, mods["pose"].Pose, mods["boolean"].Boolean
        self.node = tr.Node()
        self.lock = threading.Lock()
        self.pose, self.scan, self.tof = None, None, []
        self.counts = {"lidar": 0, "tof_up": 0, "tof_down": 0}
        self.node.subscribe(self.Pose_V, f"/world/{WORLD}/dynamic_pose/info", self._pose)
        self.node.subscribe(self.LaserScan, "/airmouse/lidar", self._lidar)
        self.node.subscribe(self.LaserScan, "/airmouse/tof_up", lambda m: self._tof("U", "tof_up", m))
        self.node.subscribe(self.LaserScan, "/airmouse/tof_down", lambda m: self._tof("D", "tof_down", m))
        self.pub = self.node.advertise("/model/airmouse/cmd_vel", self.Twist)
        self.can_move_people = n_movers > 0

    def _pose(self, msg):
        for p in msg.pose:
            if p.name == "airmouse":
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
                with self.lock:
                    self.pose = (t, p.position.x, p.position.y, p.position.z, yaw)

    def _lidar(self, msg):
        with self.lock:
            self.scan = msg
            self.counts["lidar"] += 1

    def _tof(self, tag, key, msg):
        with self.lock:
            self.tof.append((tag, msg))
            self.counts[key] += 1

    def read(self):
        with self.lock:
            out = (self.pose, self.scan, self.tof)
            self.scan, self.tof = None, []
            return out

    def command(self, vx, vy, vz, wz):
        m = self.Twist()
        m.linear.x, m.linear.y, m.linear.z, m.angular.z = vx, vy, vz, wz
        self.pub.publish(m)

    def move_person(self, i, x, y, h):
        if not self.can_move_people:
            return
        req = self.Pose()
        req.name, req.position.x, req.position.y, req.position.z = f"person_{i}", x, y, h / 2
        try:
            self.node.request(f"/world/{WORLD}/set_pose", req, self.Pose, self.Boolean, 200)
        except Exception as e:                     # older/newer binding signature: people stay still
            print(f"\n[warn] cannot move people through set_pose ({e}); they stay in place")
            self.can_move_people = False

    def wait(self, dt):
        time.sleep(dt)


# ------------------------------------------------------------------------------- fake I/O (self-test)
class FakeIO:
    """Kinematic drone + ray-cast sensors with the SAME message layout as the Gazebo sensors."""

    def __init__(self, world, start, z, yaw, cfg, seed):
        from bench_avoidance import ray_batch
        self.ray, self.w, self.cfg = ray_batch, world, cfg
        self.rng = np.random.default_rng(seed)
        self.x, self.y, self.z, self.yaw, self.t = start[0], start[1], z, yaw, 0.0
        self.v = (0.0, 0.0, 0.0, 0.0)
        self.counts = {"lidar": 0, "tof_up": 0, "tof_down": 0}
        self.next = {"lidar": 0.0, "tof_up": 0.0, "tof_down": 0.0}
        n = 500
        self.lid = Scan(-math.pi, 2 * math.pi / n, n, 0.0, 0.0, 1, None, cfg.lidar_max_range)
        st = 2 * TOF_HALF / 7
        self.tof_tpl = Scan(-TOF_HALF, st, 8, -TOF_HALF, st, 8, None, cfg.tof_max_range)
        self.dirs = {"tof_up": beam_dirs(self.tof_tpl, cfg.up_pitch_deg), "tof_down": beam_dirs(self.tof_tpl, cfg.down_pitch_deg)}
        self.xyz = {"tof_up": np.array(cfg.up_xyz), "tof_down": np.array(cfg.down_xyz)}

    def _cast(self, org, dirs_body, max_r):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        d = np.column_stack((c * dirs_body[:, 0] - s * dirs_body[:, 1], s * dirs_body[:, 0] + c * dirs_body[:, 1], dirs_body[:, 2]))
        _, r = self.ray(self.w, org[0], org[1], org[2], np.arctan2(d[:, 1], d[:, 0]), np.arcsin(np.clip(d[:, 2], -1, 1)), max_r)
        return r

    def read(self):
        scan, tof = None, []
        if self.t >= self.next["lidar"]:
            self.next["lidar"] += 0.1
            a = self.lid.angle_min + self.lid.angle_step * np.arange(self.lid.count)
            r = self._cast((self.x, self.y, self.z + self.cfg.lidar_xyz[2]), np.column_stack((np.cos(a), np.sin(a), 0 * a)), 12.0)
            r = np.where(np.isfinite(r), r + self.rng.normal(0, 0.015, r.shape), np.inf)
            scan = Scan(self.lid.angle_min, self.lid.angle_step, self.lid.count, 0, 0, 1, r, self.lid.range_max)
            self.counts["lidar"] += 1
        for key, tag in (("tof_up", "U"), ("tof_down", "D")):
            if self.t >= self.next[key]:
                self.next[key] += 1 / 15
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                o = self.xyz[key]
                org = (self.x + c * o[0] - s * o[1], self.y + s * o[0] + c * o[1], self.z + o[2])
                r = self._cast(org, self.dirs[key], self.cfg.tof_max_range)
                r = np.where(np.isfinite(r), r + self.rng.normal(0, 0.008, r.shape), np.inf)
                t = self.tof_tpl
                tof.append((tag, Scan(t.angle_min, t.angle_step, 8, t.vertical_angle_min, t.vertical_angle_step, 8, r, t.range_max)))
                self.counts[key] += 1
        return (self.t, self.x, self.y, self.z, self.yaw), scan, tof

    def command(self, vx, vy, vz, wz):
        self.v = (vx, vy, vz, wz)

    def move_person(self, i, x, y, h):
        pass

    def wait(self, dt):
        vx, vy, vz, wz = self.v
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        self.z += vz * dt
        self.yaw += wz * dt
        self.t += dt
        self.w.step_movers(dt)


# ---------------------------------------------------------------------------------- the run
class Mission:
    def __init__(self, args):
        cfg, tof_range = B.build_cfg(args.scenario)
        tr = B.make_trial(cfg, args.seed)
        if tr is None:
            raise SystemExit(f"seed {args.seed}: no valid start/goal")
        self.world, self.start, self.goal, self.path = tr
        self.path0 = list(self.path)
        self.cruise, self.radius = cfg["drone"]["cruise_alt_m"], cfg["drone"]["radius_m"]
        self.cfg = StackConfig(drone_radius=self.radius, tof_max_range=tof_range)
        for kv in getattr(args, "set", None) or []:             # --set name=value (OA-Stack parameters)
            k, v = kv.split("=", 1)
            setattr(self.cfg, k, type(getattr(self.cfg, k))(v))
        self.alg = StackAvoider(self.cfg)
        self.tof = {"U": TofReorder(self.cfg.up_pitch_deg, self.cfg.tof_fov_deg, self.cfg.up_calib),
                    "D": TofReorder(self.cfg.down_pitch_deg, self.cfg.tof_fov_deg, self.cfg.down_calib)}
        self.limit = 25.0 + B.path_length(self.path) / 0.8 * 2.5
        rng = np.random.default_rng(args.seed + 991)
        self.h_err, self.p_err = B.OU(0.047, 4.0, rng), B.OU(0.015, 10.0, rng, 2)
        p = self.path
        self.yaw0 = math.atan2(p[min(5, len(p) - 1)][1] - self.start[1], p[min(5, len(p) - 1)][0] - self.start[0])

    def pursuit(self, x, y, yaw, speed=0.8, look=0.8):
        d = [math.hypot(px - x, py - y) for px, py in self.path]
        self.path = self.path[int(np.argmin(d)):]
        tgt = self.path[-1]
        for px, py in self.path:
            if math.hypot(px - x, py - y) >= look:
                tgt = (px, py)
                break
        rel = (math.atan2(tgt[1] - y, tgt[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        return speed * math.cos(rel), speed * math.sin(rel), rel

    def progress(self, x, y):
        d = [math.hypot(px - x, py - y) for px, py in self.path0]
        return int(np.argmin(d)) / max(1, len(self.path0) - 1)


def run(args, io, mission, tally):
    m, c = mission, mission.cfg
    t0_wall, t_start, last_t = time.time(), None, None
    stats = {"min_clear": math.inf, "speed_sum": 0.0, "n": 0, "ms": [], "replans": 0, "status": "-", "vert": "-", "flags": []}
    prog_t, prog_xy, last_replan, mover_t, shown = 0.0, None, -99.0, 0.0, 0.0
    seen = np.zeros_like(m.world.walls)
    result = None
    while result is None:
        pose, scan, tof = io.read()
        if pose is None:
            if time.time() - t0_wall > 60:
                raise SystemExit("no pose from Gazebo after 60 s - is `gz sim -r <world>` running with this world?")
            io.wait(0.05)
            continue
        t, x, y, z, yaw = pose
        t_start = t if t_start is None else t_start
        tr = t - t_start
        if last_t is not None and t <= last_t and scan is None and not tof:
            io.wait(0.01)
            continue
        dt = 0.05 if last_t is None else max(1e-3, t - last_t)
        last_t = t
        # sensors as OA-Stack receives them (height and pose with EKF/SLAM-like error)
        height = z + float(m.h_err.step(dt)[0])
        pe = m.p_err.step(dt)
        est = (x + pe[0], y + pe[1], yaw)
        sc = None
        if scan is not None:
            a = scan.angle_min + scan.angle_step * np.arange(scan.count)
            r = np.asarray(scan.ranges, float)
            sc = (a, np.where(np.isfinite(r) & (r < scan.range_max - 1e-3), r, np.inf))
        frames = [(tag, m.tof[tag](msg)) for tag, msg in tof]
        vx, vy, rel = m.pursuit(x, y, yaw)
        tic = time.perf_counter()
        out = m.alg.update(tr, est, (vx, vy), scan=sc, tof_frames=frames, height=height, cruise_alt=m.cruise)
        stats["ms"].append(1000 * (time.perf_counter() - tic))
        zt = m.cruise if out.z_target is None else out.z_target
        vz = float(np.clip(1.5 * (zt - height), -0.5, 0.5))
        # nose (and so both ToF sensors) toward where the drone is actually going: the ToF only looks
        # forward, so sliding sideways under avoidance would fly blind into ducts
        heading = math.atan2(out.vy, out.vx) if math.hypot(out.vx, out.vy) > 0.15 else rel
        wz = float(np.clip(1.5 * heading, -1.2, 1.2))
        io.command(out.vx, out.vy, vz, wz)
        stats.update(status=out.status, vert=out.info.get("vertical", "-"), flags=out.flags)
        # replan when asked or stuck (walls-only A*, as in the benchmark)
        # what the drone has seen (lidar hits + remembered ToF hazards) goes into the replanning map,
        # as in the benchmark; otherwise a replan on the walls-only map returns the same blocked path
        if sc is not None:
            hit = np.isfinite(sc[1])
            wa = sc[0][hit] + est[2]
            ix = ((est[0] + sc[1][hit] * np.cos(wa)) / RES).astype(int)
            iy = ((est[1] + sc[1][hit] * np.sin(wa)) / RES).astype(int)
            ok = (ix >= 0) & (iy >= 0) & (ix < seen.shape[1]) & (iy < seen.shape[0])
            seen[iy[ok], ix[ok]] = True
        # stuck = less than 0.3 m closer to the goal (along the path) in 5 s
        togo = B.path_length([(x, y)] + m.path)
        if prog_xy is None:
            prog_t, prog_xy = tr, togo
        stuck = False
        if tr - prog_t > 5.0:
            stuck = prog_xy - togo < 0.3
            prog_t, prog_xy = tr, togo
        if (out.replan or stuck) and tr - last_replan > 3.0:
            last_replan = tr
            extra = seen.copy()
            for hx, hy, _, _, _ in m.alg.hazards.items:
                j, i = int(hx / RES), int(hy / RES)
                if 0 <= i < extra.shape[0] and 0 <= j < extra.shape[1]:
                    extra[i, j] = True
            k = int(0.7 / RES)                     # the drone stands next to what it saw: free its own spot
            i0, j0 = int(y / RES), int(x / RES)
            extra[max(i0 - k, 0):i0 + k + 1, max(j0 - k, 0):j0 + k + 1] = False
            infl = m.radius + 0.12                 # the clearance the start-goal path was planned with
            p = B.plan_walls_only(m.world, (x, y), m.goal, infl, extra=extra) \
                or B.plan_walls_only(m.world, (x, y), m.goal, infl)
            if p:
                m.path = p
                stats["replans"] += 1
        # people
        if tr - mover_t >= 0.1:
            if not isinstance(io, FakeIO):
                m.world.step_movers(tr - mover_t)
            mover_t = tr
            for i, p in enumerate(m.world.movers):
                io.move_person(i, p.x, p.y, p.h)
        # evaluation against the true geometry
        if sc is not None:
            stats["min_clear"] = min(stats["min_clear"], float(np.min(sc[1])) - m.radius)
        stats["speed_sum"] += math.hypot(out.vx, out.vy)
        stats["n"] += 1
        if collided(m.world, x, y, z, m.radius, c.body_bottom_dz, c.prop_top_dz):
            result = "collision: " + ("person" if any(math.hypot(p.x - x, p.y - y) < m.radius + p.r + 0.05 for p in m.world.movers)
                                      else B.crash_kind(m.world, x, y))
        elif math.hypot(x - m.goal[0], y - m.goal[1]) < 0.5:
            result = "success"
        elif tr > m.limit:
            result = "timeout"
        if result or time.time() - shown > 0.25 or isinstance(io, FakeIO) and stats["n"] % 20 == 0:
            shown = time.time()
            dashboard(args, m, io, stats, tr, (x, y, z), result, tally)
        io.wait(0.05)
    io.command(0.0, 0.0, 0.0, 0.0)
    return {"scenario": args.scenario, "seed": args.seed, "result": result, "time_s": round(tr, 1),
            "min_clearance_m": round(stats["min_clear"], 3), "mean_speed_mps": round(stats["speed_sum"] / max(1, stats["n"]), 2),
            "compute_ms_mean": round(float(np.mean(stats["ms"])), 2), "replans": stats["replans"]}


def dashboard(args, m, io, st, tr, xyz, result, tally):
    rates = {k: v / max(tr, 1e-3) for k, v in io.counts.items()}
    bar = int(30 * m.progress(xyz[0], xyz[1]))
    clear_now = st["min_clear"]
    lines = [
        f"OA-Stack in Gazebo   world {args.scenario}  seed {args.seed}   run {args.run}/{args.runs}",
        "-" * 72,
        f"time      {tr:6.1f} s / limit {m.limit:.0f} s        result: {result or 'flying...'}",
        f"status    {st['status']:<10} height mode {st['vert']:<9} flags {' '.join(st['flags']) or 'none'}",
        f"position  x {xyz[0]:6.2f}  y {xyz[1]:6.2f}  z {xyz[2]:5.2f} m      replans {st['replans']}",
        f"speed     {st['speed_sum'] / max(1, st['n']):4.2f} m/s mean     closest approach so far {max(0.0, clear_now):.2f} m",
        f"compute   {np.mean(st['ms'][-40:]):4.1f} ms/cycle        hazards in memory {len(m.alg.hazards.items)}",
        f"sensors   lidar {rates['lidar']:4.1f} Hz   ToF up {rates['tof_up']:4.1f} Hz   ToF down {rates['tof_down']:4.1f} Hz",
        f"progress  [{'#' * bar}{'.' * (30 - bar)}] {100 * bar // 30:3d}%",
        "-" * 72,
        f"session   runs {tally['runs']}   success {tally['success']}   collision {tally['collision']}   timeout {tally['timeout']}",
    ]
    if any(v < 1 for v in rates.values()) and tr > 3:
        lines.append("WARNING   a sensor topic is silent - check the world was made by gz_world.py")
    if sys.stdout.isatty():
        sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lines) + "\n")
    else:
        sys.stdout.write(lines[2] + " | " + lines[3] + "\n")
    sys.stdout.flush()


def load_tally(path):
    t = {"runs": 0, "success": 0, "collision": 0, "timeout": 0}
    if path and os.path.exists(path):
        with open(path) as f:
            for row in csv.DictReader(f):
                t["runs"] += 1
                t[row["result"].split(":")[0]] += 1
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="mixed_clutter", choices=list(B.SCENARIOS))
    ap.add_argument("--seed", type=int, default=207)
    ap.add_argument("--csv", default=None, help="append the result here (the session tally is read from it)")
    ap.add_argument("--run", type=int, default=1)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--selftest", action="store_true", help="no Gazebo: kinematic drone + ray-cast sensors")
    ap.add_argument("--set", action="append", default=[], help="OA-Stack parameter override, e.g. --set margin=0.2")
    a = ap.parse_args()
    mission = Mission(a)
    if a.selftest:
        io = FakeIO(mission.world, mission.start, mission.cruise, mission.yaw0, mission.cfg, a.seed)
    else:
        io = GzIO(len(mission.world.movers))
    tally = load_tally(a.csv)
    res = run(a, io, mission, tally)
    print("\nRESULT", res)
    if a.csv:
        new = not os.path.exists(a.csv)
        os.makedirs(os.path.dirname(os.path.abspath(a.csv)), exist_ok=True)
        with open(a.csv, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(res))
            if new:
                w.writeheader()
            w.writerow(res)


if __name__ == "__main__":
    main()
