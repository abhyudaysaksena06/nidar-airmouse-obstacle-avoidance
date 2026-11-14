"""3D point-cloud front-end shared by the depth-camera (Algo D) and the
lidar + multi-zone-ToF (Algo L) configurations.

Input: points (N,3) in the body frame (x fwd, y left, z up RELATIVE to the
drone), from any 3D sensor. Points are binned into vertical columns of
`col_res` x `col_res` m. For each column we look at the height span of its
points relative to the drone's protected band B = [-(h_h + c), +(h_h + c)]:

  * column overlaps B                      -> HORIZONTAL obstacle (wall, pillar,
                                              box face at our height): goes to VFH+/CBF
  * column entirely below B (and not floor) -> LOW obstacle, top = max z
  * column entirely above B (and not ceiling far away) -> HANGING obstacle,
                                              bottom = min z

Low/hanging columns near the drone (or just ahead) bound the altitude:
    z* in [ max(top_low) + h_h + c ,  min(bottom_hang) - h_h - c ]
so the drone climbs over low boxes and ducks under beams instead of
treating them as walls (which would block a corridor).
"""
import math

import numpy as np


class PointCloudFrontEnd:
    def __init__(self, cfg, col_res=0.10, floor_z=0.08):
        self.cfg = cfg
        self.col_res = col_res
        self.floor_z = floor_z

    def split(self, pts3, z_abs, vfov=(-0.5, 0.5), ceiling_guess=2.3, max_climb=0.8):
        """Returns (horizontal_pts (M,2), low_cols (K,3)=(x,y,top_rel),
        hang_cols (L,3)=(x,y,bottom_rel)) in the body frame.

        vfov: sensor elevation limits (rad). At horizontal distance r the
        sensor observes heights r*tan(el_min) .. r*tan(el_max) (relative).
        A column whose lowest point is clearly ABOVE the lowest observable
        height had free space observed beneath it -> hanging (duck under),
        if there is room for the drone below it. Likewise a column whose top
        is clearly below the highest observable height -> low (climb over).
        Never climb more than `max_climb` m (a partly seen tall obstacle must
        not be "climbed" towards the ceiling). Everything else that touches
        our height band is a wall/pillar."""
        e2, e3 = np.zeros((0, 2)), np.zeros((0, 3))
        if pts3 is None or len(pts3) == 0:
            return e2, e3, e3
        cfg = self.cfg
        p = np.asarray(pts3, float)
        p = p[z_abs + p[:, 2] > self.floor_z]                 # drop the floor
        if len(p) == 0:
            return e2, e3, e3
        hh, c = cfg.half_height, cfg.vertical_clearance
        band = hh + c
        key = np.floor(p[:, :2] / self.col_res).astype(np.int64)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.ravel()
        n = len(uniq)
        zmin = np.full(n, np.inf)
        zmax = np.full(n, -np.inf)
        np.minimum.at(zmin, inv, p[:, 2])
        np.maximum.at(zmax, inv, p[:, 2])
        centre = (uniq + 0.5) * self.col_res
        r = np.hypot(centre[:, 0], centre[:, 1])
        obs_bot = np.maximum(r * math.tan(vfov[0]), -z_abs)          # floor bounds the view
        obs_top = r * math.tan(vfov[1])
        seen_below = zmin > obs_bot + 0.15                            # free space seen under it
        seen_above = zmax < obs_top - 0.15                            # free space seen over it
        room_below = z_abs + zmin - band >= cfg.min_alt               # drone fits under it
        room_above = (z_abs + zmax + band <= ceiling_guess) & (zmax + band <= max_climb)  # fits over it
        hang = (zmin > band) | (seen_below & room_below)
        low = ~hang & ((zmax < -band) | (seen_above & room_above & (zmin <= obs_bot + 0.15)))
        horiz = ~hang & ~low & (zmax >= -band) & (zmin <= band)
        return (centre[horiz],
                np.column_stack((centre[low], zmax[low])),
                np.column_stack((centre[hang], zmin[hang])))

    def altitude_band(self, low, hang, z_abs, cruise, motion_dir=0.0, up_clear=None):
        """Altitude target respecting low/hanging columns near us or ahead.
        Returns (z_target_abs, speed_scale)."""
        cfg = self.cfg
        hh, c = cfg.half_height, cfg.vertical_clearance
        reach = cfg.drone_radius + 0.45
        ax, ay = 0.7 * math.cos(motion_dir), 0.7 * math.sin(motion_dir)

        def near(cols):
            if len(cols) == 0:
                return cols
            d0 = np.hypot(cols[:, 0], cols[:, 1])
            d1 = np.hypot(cols[:, 0] - ax, cols[:, 1] - ay)
            return cols[(d0 < reach) | (d1 < reach)]
        nl, nh = near(low), near(hang)
        lo = (z_abs + float(nl[:, 2].max()) + hh + c) if len(nl) else -1.0
        hi = (z_abs + float(nh[:, 2].min()) - hh - c) if len(nh) else 99.0
        if up_clear is not None and math.isfinite(up_clear):
            hi = min(hi, z_abs + up_clear - hh - c)
        zt = max(cruise, lo)
        if hi >= cfg.min_alt:
            zt = min(zt, hi)
        zt = max(cfg.min_alt, zt)
        scale = 0.25 if abs(zt - z_abs) > 0.2 else 1.0
        return zt, scale


def virtual_scan(horiz_pts, n_beams=360, max_range=8.0):
    """2D 'laser scan' from horizontal obstacle points: nearest point per
    bearing bin (inf where nothing). Lets a depth camera drive the same
    VFH+/CBF/tracker pipeline as a lidar."""
    ang = np.linspace(-np.pi, np.pi, n_beams, endpoint=False)
    rng = np.full(n_beams, np.inf)
    if len(horiz_pts):
        a = np.arctan2(horiz_pts[:, 1], horiz_pts[:, 0])
        d = np.hypot(horiz_pts[:, 0], horiz_pts[:, 1])
        k = np.round((a + np.pi) / (2 * np.pi / n_beams)).astype(int) % n_beams
        np.minimum.at(rng, k, d)
        rng[rng > max_range] = np.inf
    return ang, rng


def depth_image_to_points(depth, fx, fy, cx, cy, pitch=0.0, stride=4, mount=(0.0, 0.0, 0.0),
                          min_d=0.3, max_d=6.0):
    """Depth image (H,W metres, 0/nan = invalid) -> body-frame points (N,3).
    Camera optical frame: z forward, x right, y down; pitched down by `pitch`."""
    d = np.asarray(depth, float)[::stride, ::stride]
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    u = u * stride
    v = v * stride
    ok = np.isfinite(d) & (d >= min_d) & (d <= max_d)
    z = d[ok]
    xo = (u[ok] - cx) / fx * z          # right
    yo = (v[ok] - cy) / fy * z          # down
    # optical -> FLU body (before pitch): x = z, y = -xo, z = -yo
    bx, by, bz = z, -xo, -yo
    cp, sp = math.cos(pitch), math.sin(pitch)
    X = cp * bx + sp * bz
    Z = -sp * bx + cp * bz
    return np.column_stack((X + mount[0], by + mount[1], Z + mount[2]))
