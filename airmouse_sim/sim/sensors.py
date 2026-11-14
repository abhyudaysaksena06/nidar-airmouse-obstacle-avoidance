"""GPS-denied sensor suite: 2D/multi-layer lidar, ToF rangefinders, baro,
optical flow + gyro odometry, and a camera person detector (bounding boxes)."""
import math

import numpy as np

from .world import RES


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def raycast(world, ox, oy, z, angles, max_r, step=RES):
    """Ray march in the horizontal plane at height z (sees only obstacles
    crossing that plane) + analytic moving cylinders."""
    grid = world.occ_at(z)
    ts = np.arange(step, max_r + step, step)
    ca, sa = np.cos(angles), np.sin(angles)
    xs = ox + ca[:, None] * ts
    ys = oy + sa[:, None] * ts
    ix, iy = (xs / RES).astype(int), (ys / RES).astype(int)
    H, W = grid.shape
    out = (ix < 0) | (iy < 0) | (ix >= W) | (iy >= H)
    hit = out.copy()
    ok = ~out
    hit[ok] = grid[iy[ok], ix[ok]]
    has = hit.any(axis=1)
    r = np.where(has, ts[hit.argmax(axis=1)], np.inf)
    for m in world.movers:
        if z > m.h:
            continue
        dx, dy = m.x - ox, m.y - oy
        b = dx * ca + dy * sa
        c = dx * dx + dy * dy - m.r * m.r
        disc = b * b - c
        valid = (disc >= 0) & (b > 0)
        tt = b - np.sqrt(np.where(valid, disc, 0))
        r = np.where(valid & (tt > 0) & (tt < r), tt, r)
    return np.minimum(r, max_r + 1.0)


def ray3d(world, x, y, z, yaw, pitch, max_r, step=0.05):
    """Single 3D ray (for tilted ToF beams). pitch > 0 points up."""
    cp = math.cos(pitch)
    dx, dy, dz = cp * math.cos(yaw), cp * math.sin(yaw), math.sin(pitch)
    t = step
    while t <= max_r:
        px, py, pz = x + dx * t, y + dy * t, z + dz * t
        if pz <= 0.0 or pz >= world.ceiling:
            return t
        iy, ix = int(py / RES), int(px / RES)
        if not (0 <= iy < world.zlo.shape[0] and 0 <= ix < world.zlo.shape[1]):
            return t
        if world.zlo[iy, ix] <= pz <= world.zhi[iy, ix]:
            return t
        t += step
    return math.inf


class Sensors:
    def __init__(self, world, cfg, dt, rng, nprng, faults):
        self.w, self.c, self.dt, self.rng, self.np, self.faults = world, cfg, dt, rng, nprng, faults
        n = cfg["lidar_beams"]
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        self.lidar_period = 1.0 / cfg["lidar_rate_hz"]
        self.cam_period = 1.0 / cfg["camera_rate_hz"]
        self.next_lidar = self.next_cam = 0.0
        self.last_scan = None
        self.true_ranges = np.full(n, cfg["lidar_max_range_m"])
        # odometry integrator (what a PX4 EKF would publish, flow + gyro)
        self.ox = self.oy = self.oyaw = 0.0
        self.bias = rng.choice([-1, 1]) * cfg["gyro_bias_rps"]
        self.baro_offset = 0.0
        self.mvx = self.mvy = 0.0
        W, H = cfg["camera_resolution"]
        self.cam_W, self.cam_H = W, H
        self.f = (W / 2) / math.tan(math.radians(cfg["camera_hfov_deg"]) / 2)

    # ------------------------------------------------------------------ lidar
    def lidar(self, d, t):
        c = self.c
        if t + 1e-9 < self.next_lidar:
            return None
        self.next_lidar = t + self.lidar_period
        if self.faults.active(t, "lidar_blackout"):
            return "blackout"
        r = None
        for dz in c["lidar_layers_z_m"]:
            rr = raycast(self.w, d.x, d.y, max(0.02, d.z + dz), self.angles + d.yaw, c["lidar_max_range_m"])
            r = rr if r is None else np.minimum(r, rr)
        self.true_ranges = r
        noisy = r + self.np.normal(0, c["lidar_noise_m"], r.shape)
        noisy[(r > c["lidar_max_range_m"]) | (noisy < c["lidar_min_range_m"])] = np.inf
        noisy[self.np.random(r.shape) < c["lidar_dropout_prob"]] = np.inf
        return noisy

    # -------------------------------------------------------------- rangers
    def tof(self, d):
        c, g = self.c, self.rng.gauss
        mx = c["tof_max_range_m"]
        below = d.z - self.w.surface_below(d.x, d.y, d.z)
        above = self.w.surface_above(d.x, d.y, d.z) - d.z
        p = math.radians(c["tof_fwd_pitch_deg"])
        fd = ray3d(self.w, d.x, d.y, d.z, d.yaw, -p, mx)
        fu = ray3d(self.w, d.x, d.y, d.z, d.yaw, p, mx)

        def meas(v):
            return v + g(0, c["tof_noise_m"]) if v <= mx else math.inf
        return {"down": meas(below), "up": meas(above), "fwd_down": meas(fd), "fwd_up": meas(fu)}

    def baro(self, d, dt):
        self.baro_offset += self.c["baro_drift_mps"] * dt * self.rng.gauss(0, 1)
        return d.z + self.baro_offset + self.rng.gauss(0, self.c["baro_noise_m"])

    # ------------------------------------------------------------ odometry
    def odometry(self, d, t, dt):
        c, g = self.c, self.rng.gauss
        bvx, bvy = d.body_vel()
        height = d.z - self.w.surface_below(d.x, d.y, d.z)
        quality = 1.0
        if self.w.is_textureless(d.x, d.y):
            quality = 0.1
        if height < c["flow_min_height_m"]:
            quality = min(quality, 0.2)
        if self.w.is_dark(d.x, d.y) and not c["drone_has_light"]:
            quality = min(quality, 0.5)
        if self.faults.active(t, "flow_loss"):
            quality = 0.0
        valid = quality >= 0.3
        sp = math.hypot(bvx, bvy)
        sd = c["flow_vel_noise_frac"] * sp + c["flow_vel_noise_abs"]
        if valid:
            self.mvx, self.mvy = bvx + g(0, sd), bvy + g(0, sd)
        else:  # EKF coasting on IMU: velocity estimate wanders
            self.mvx += g(0, 0.05)
            self.mvy += g(0, 0.05)
        bias = self.bias + self.faults.value(t, "gyro_bias_jump", 0.0)
        mwz = d.wz + bias + g(0, c["gyro_noise_rps"])
        self.oyaw = wrap(self.oyaw + mwz * dt)
        cy, sy = math.cos(self.oyaw), math.sin(self.oyaw)
        self.ox += (cy * self.mvx - sy * self.mvy) * dt
        self.oy += (sy * self.mvx + cy * self.mvy) * dt
        return {"x": self.ox, "y": self.oy, "yaw": self.oyaw,
                "vx": self.mvx if valid else None, "vy": self.mvy if valid else None,
                "yaw_rate": mwz, "flow_quality": round(quality, 2)}

    # -------------------------------------------------------------- camera
    def camera(self, d, t):
        """Person detector output: bounding boxes in pixels (+ depth if a
        depth camera is configured)."""
        c, g = self.c, self.rng.gauss
        if t + 1e-9 < self.next_cam:
            return None
        self.next_cam = t + self.cam_period
        if self.faults.active(t, "camera_blackout"):
            return "blackout"
        pitch = math.radians(c["camera_pitch_deg"])
        W, H, f = self.cam_W, self.cam_H, self.f
        curve = np.array(c["detector_recall_curve"])
        dets = []
        for s in self.w.survivors:
            dx, dy = s.x - d.x, s.y - d.y
            fwd = dx * math.cos(d.yaw) + dy * math.sin(d.yaw)
            left = -dx * math.sin(d.yaw) + dy * math.cos(d.yaw)
            down = d.z - 0.15
            xc = fwd * math.cos(pitch) + down * math.sin(pitch)
            yc = -fwd * math.sin(pitch) + down * math.cos(pitch)
            if xc < 0.3:
                continue
            u, v = W / 2 - f * left / xc, H / 2 + f * yc / xc
            if not (0 <= u < W and 0 <= v < H):
                continue
            dist = math.sqrt(dx * dx + dy * dy + down * down)
            # occlusion: line of sight to head, centre and feet
            vis = 0
            for k in (-0.8, 0.0, 0.8):
                px, py = s.x + k * math.cos(s.yaw), s.y + k * math.sin(s.yaw)
                ang = math.atan2(py - d.y, px - d.x)
                hd = math.hypot(px - d.x, py - d.y)
                r = raycast(self.w, d.x, d.y, max(0.3, d.z * 0.5), np.array([ang]), hd)[0]
                vis += r >= hd - 0.15
            if vis == 0:
                continue
            recall = float(np.interp(dist, curve[:, 0], curve[:, 1], right=0.0))
            recall *= vis / 3
            if self.w.is_dark(s.x, s.y) and not c["drone_has_light"]:
                recall *= c["detector_dark_factor"]
            if self.rng.random() > recall:
                continue
            rel = wrap(s.yaw - math.atan2(dy, dx))
            bw = f * max(0.45, 1.7 * abs(math.sin(rel))) / xc
            bh = f * max(0.35, 1.7 * abs(math.cos(rel)) * math.sin(math.atan2(down, max(fwd, 0.1)))) / xc
            n = c["detector_bbox_noise_px"]
            box = [round(u - bw / 2 + g(0, n), 1), round(v - bh / 2 + g(0, n), 1),
                   round(bw * (1 + g(0, 0.1)), 1), round(bh * (1 + g(0, 0.1)), 1)]
            det = {"cls": "person", "bbox": box,
                   "confidence": round(min(0.99, max(0.25, recall + g(0, 0.08))), 2)}
            if c["camera_type"] == "depth":
                det["depth"] = round(dist * (1 + g(0, c["depth_noise_frac"])), 2)
            dets.append(det)
        if self.rng.random() < c["detector_false_positives_per_frame"]:
            u, v = self.rng.uniform(0, W), self.rng.uniform(H * 0.3, H)
            s = self.rng.uniform(20, 80)
            det = {"cls": "person", "bbox": [round(u, 1), round(v, 1), round(s, 1), round(s * 0.6, 1)],
                   "confidence": round(self.rng.uniform(0.25, 0.6), 2)}
            if c["camera_type"] == "depth":
                det["depth"] = round(self.rng.uniform(0.5, 5.0), 2)
            dets.append(det)
        return dets
