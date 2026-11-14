"""Camera + Canny-edge obstacle avoidance, for comparison with OA-Stack (not part of OA-Stack).

Camera: forward pinhole, 96x72 px, 70 deg horizontal FOV, tilted 10 deg down, 10 Hz, 3D ray cast
in the same world. Pixels are shaded by surface type, face direction and distance, plus noise
(good indoor lighting; no motion blur). Two avoiders use it:

  canny_reactive  classic edge-density steering: Canny edges, image split into 7 vertical strips,
                  fly toward the strip with the fewest edges near the goal direction, slow down
                  when the centre strip is busy.
  canny_core      ground-plane free space (Lorigo/Ulrich style): in every image column the first
                  Canny edge from the bottom is taken as where an obstacle meets the floor; its
                  angle below the horizon and the camera height give a distance. These distances
                  replace the lidar in the SAME VFH+ / braking-limit / CBF core OA-Stack uses.
Neither can see ducts (they do not touch the floor) or judge height, so both fly at cruise.
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

from bench_avoidance import ray_batch
from sim.world import RES

W, H = 96, 72
HFOV, TILT = math.radians(70.0), math.radians(-10.0)
VFOV = 2 * math.atan(math.tan(HFOV / 2) * H / W)
MAX_R = 8.0
ALBEDO = {0: 0.70, 1: 0.70, 2: 0.55, 3: 0.50, 4: 0.45, "person": 0.35, "floor": 0.62, "ceiling": 0.85}


def body_rays():
    """Unit ray per pixel in the body frame (x fwd, y left, z up), camera tilted by TILT."""
    tx = math.tan(HFOV / 2) * (1 - 2 * np.arange(W) / (W - 1))
    ty = math.tan(VFOV / 2) * (1 - 2 * np.arange(H) / (H - 1))
    X, Y = np.meshgrid(tx, ty)
    d = np.stack((np.ones_like(X), X, Y), -1).reshape(-1, 3)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    c, s = math.cos(TILT), math.sin(TILT)
    return np.column_stack((c * d[:, 0] - s * d[:, 2], d[:, 1], s * d[:, 0] + c * d[:, 2]))


def kind_grid(world):
    g = np.zeros(world.zlo.shape, dtype=np.uint8)
    code = {"wall": 1, "fence": 1, "box": 2, "hanging": 3}
    for r in world.rects:
        g[max(int(r.y0 / RES), 0):int(math.ceil(r.y1 / RES)), max(int(r.x0 / RES), 0):int(math.ceil(r.x1 / RES))] = code.get(r.kind, 1)
    yy, xx = np.mgrid[0:g.shape[0], 0:g.shape[1]]
    for c in world.cylinders:
        g[np.hypot((xx + 0.5) * RES - c.x, (yy + 0.5) * RES - c.y) <= c.r + RES] = 4
    return g


class Camera:
    def __init__(self, world, rng):
        self.w, self.rng = world, rng
        self.kinds = kind_grid(world)
        self.rays = body_rays()

    def image(self, x, y, z, R):
        """Grey image in [0,1] (H, W) and the world ray directions."""
        dirs = self.rays @ R.T
        yaw = np.arctan2(dirs[:, 1], dirs[:, 0])
        el = np.arcsin(np.clip(dirs[:, 2], -1, 1))
        d, r = ray_batch(self.w, x, y, z, yaw, el, MAX_R)
        fin = np.isfinite(r)
        rr = np.where(fin, r, MAX_R)
        P = np.array([x, y, z]) + d * rr[:, None]
        Q = np.array([x, y, z]) + d * np.maximum(rr - 0.05, 0)[:, None]      # just before the hit
        ix = np.clip((P[:, 0] / RES).astype(int), 0, self.kinds.shape[1] - 1)
        iy = np.clip((P[:, 1] / RES).astype(int), 0, self.kinds.shape[0] - 1)
        k = self.kinds[iy, ix]
        alb = np.choose(np.clip(k, 0, 4), [ALBEDO[i] for i in range(5)]).astype(float)
        face = np.where((Q[:, 0] / RES).astype(int) != ix, 1.0, 0.78)          # x-faces lit, y-faces darker
        top = Q[:, 2] > self.w.zhi[iy, ix] - 0.01                               # top of a box
        face = np.where(top, 1.12, face)
        floor = P[:, 2] <= 0.03
        ceil = P[:, 2] >= self.w.ceiling - 0.03
        alb = np.where(floor, ALBEDO["floor"], np.where(ceil, ALBEDO["ceiling"], alb))
        face = np.where(floor | ceil, 1.0, face)
        for m in self.w.movers:
            on = (np.hypot(P[:, 0] - m.x, P[:, 1] - m.y) <= m.r + 0.06) & (P[:, 2] <= m.h) & ~floor
            alb, face = np.where(on, ALBEDO["person"], alb), np.where(on, 1.0, face)
        img = alb * face * (0.55 + 0.45 * np.exp(-rr / 6.0))
        img = np.where(fin, img, 0.15)
        img = img + self.rng.standard_normal(img.shape) * 0.012
        return np.clip(img, 0, 1).reshape(H, W), dirs.reshape(H, W, 3)


def canny(img, sigma=1.0, lo=0.05, hi=0.12):
    """Canny (1986): Gaussian blur, Sobel gradient, non-maximum suppression, hysteresis."""
    g = ndimage.gaussian_filter(img, sigma)
    gx, gy = ndimage.sobel(g, 1), ndimage.sobel(g, 0)
    mag = np.hypot(gx, gy)
    ang = (np.rad2deg(np.arctan2(gy, gx)) + 180) % 180
    q = np.zeros_like(ang, dtype=int)
    q[(ang >= 22.5) & (ang < 67.5)] = 1
    q[(ang >= 67.5) & (ang < 112.5)] = 2
    q[(ang >= 112.5) & (ang < 157.5)] = 3
    p = np.pad(mag, 1)
    nb = {0: ((0, 1), (0, -1)), 1: ((1, 1), (-1, -1)), 2: ((1, 0), (-1, 0)), 3: ((1, -1), (-1, 1))}
    keep = np.zeros_like(mag, dtype=bool)
    for k, ((a1, b1), (a2, b2)) in nb.items():
        n1 = p[1 + a1:1 + a1 + mag.shape[0], 1 + b1:1 + b1 + mag.shape[1]]
        n2 = p[1 + a2:1 + a2 + mag.shape[0], 1 + b2:1 + b2 + mag.shape[1]]
        keep |= (q == k) & (mag >= n1) & (mag >= n2)
    nms = mag * keep
    strong, weak = nms >= hi, nms >= lo
    lab, n = ndimage.label(weak, structure=np.ones((3, 3)))
    good = np.zeros(n + 1, dtype=bool)
    good[np.unique(lab[strong])] = True
    good[0] = False
    return good[lab]


def ground_ranges(edges, dirs_body, cam_h):
    """First edge from the bottom of each column -> horizontal distance on a flat floor.
    Returns body-frame (angles, ranges) like a 96-beam front scan."""
    rows = np.where(edges.any(0), H - 1 - np.argmax(edges[::-1], 0), -1)
    ang = np.arctan2(dirs_body[H // 2, :, 1], dirs_body[H // 2, :, 0])
    rng = np.full(W, np.inf)
    for c in range(W):
        if rows[c] < 0:
            continue
        d = dirs_body[rows[c], c]
        if d[2] < -0.02:                                  # below the horizon -> on the floor
            rng[c] = min(cam_h / -d[2] * math.hypot(d[0], d[1]), MAX_R)
    return ang, rng


class CannyReactive:
    """Edge-density steering on 7 strips of the lower 2/3 of the image."""
    def __init__(self, v_max=0.8):
        self.v_max = v_max
        self.centres = np.linspace(HFOV / 2, -HFOV / 2, 7 * 2 + 1)[1::2]     # strip centre angles, left->right

    def command(self, edges, goal_rel):
        lower = edges[H // 3:, :]
        dens = np.array([s.mean() for s in np.array_split(lower, 7, axis=1)])
        cost = dens / 0.08 + 0.6 * np.abs(np.clip(goal_rel, -HFOV / 2, HFOV / 2) - self.centres)
        k = int(np.argmin(cost))
        busy = dens[3]
        speed = self.v_max * float(np.clip(1 - busy / 0.12, 0.0, 1.0))
        if dens.min() > 0.12:                              # everything busy: stop and turn toward the goal
            return 0.0, 0.0, "BLOCKED"
        a = self.centres[k] if abs(goal_rel) < HFOV / 2 else goal_rel
        return speed * math.cos(a), speed * math.sin(a), "OK"
