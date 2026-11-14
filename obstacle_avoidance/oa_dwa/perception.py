"""Layer 1 - robust obstacle measurement.

Lidar (primary, per chat: "2D front LiDAR -> mapping + obstacles"):
validity limits, FOV mask, isolated-spike rejection.
Depth camera (Jainil README: primary 3D sensor): keep only points inside the
drone's height band (+- depth_band), remove floor returns, voxel-downsample.
Every measurement keeps a quality value, as in the README's Layer 1.
"""
from __future__ import annotations

import math

import numpy as np

from .config import DWAConfig


def filter_scan(angles: np.ndarray, ranges: np.ndarray, cfg: DWAConfig):
    """Returns (points_body (N,2), angles_kept, ranges_kept, quality 0..1).

    quality = fraction of in-FOV beams that are valid AND not spikes; a low
    value flags a degraded lidar (dust, glass, sensor fault)."""
    a = np.asarray(angles, float)
    r = np.asarray(ranges, float)
    in_fov = np.abs(a) <= math.radians(cfg.lidar_fov_deg) / 2 + 1e-9
    valid = in_fov & np.isfinite(r) & (r >= cfg.lidar_min_range) & (r < cfg.lidar_max_range - 0.05)
    # isolated spike: a beam far from BOTH neighbours while neighbours agree
    rf = np.where(valid, r, np.nan)
    prev, nxt = np.roll(rf, 1), np.roll(rf, -1)
    with np.errstate(invalid="ignore"):
        spike = (np.abs(rf - prev) > cfg.spike_jump) & (np.abs(rf - nxt) > cfg.spike_jump) \
            & (np.abs(prev - nxt) < cfg.spike_jump)
    keep = valid & ~np.nan_to_num(spike, nan=False).astype(bool)
    n_fov = max(1, int(in_fov.sum()))
    # "no return" in open space is normal, so quality only counts spikes
    quality = 1.0 - float(np.nan_to_num(spike, nan=False).sum()) / n_fov
    ak, rk = a[keep], r[keep]
    return np.column_stack((rk * np.cos(ak), rk * np.sin(ak))), ak, rk, quality


def filter_depth(points_xyz: np.ndarray, z_drone: float, cfg: DWAConfig) -> np.ndarray:
    """Depth-camera points (body frame, x fwd, y left, z up RELATIVE to the
    drone) -> 2D obstacle points that intersect the drone's flight band."""
    if points_xyz is None or len(points_xyz) == 0:
        return np.zeros((0, 2))
    p = np.asarray(points_xyz, float)
    d = np.linalg.norm(p, axis=1)
    ok = np.isfinite(d) & (d >= cfg.depth_min_range) & (d <= cfg.depth_max_range)
    z_abs = z_drone + p[:, 2]
    ok &= z_abs > cfg.depth_ground_z                       # floor is not an obstacle
    ok &= np.abs(p[:, 2]) <= cfg.half_height + cfg.depth_band  # inside our height band
    q = p[ok, :2]
    if len(q) == 0:
        return q
    keys = np.floor(q / cfg.map_res).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return q[np.sort(idx)]
