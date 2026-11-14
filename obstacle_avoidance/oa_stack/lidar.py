"""Stage A - RPLIDAR C1 scan cleaning and tilt correction.

Tilt (guide Section 11.1): the C1 measures in its own plane. When the drone rolls/pitches,
a beam with body azimuth a points at elevation e = asin(d_z), d = R(roll, pitch) [cos a, sin a, 0].
A vertical wall at horizontal distance h is then measured at r = h / cos(e). The horizontal
distance is recovered as h = r * cos(e) (the guide's d/cos(theta), solved per beam).
"""
from __future__ import annotations

import math

import numpy as np

from .config import StackConfig


def clean_scan(angles, ranges, cfg: StackConfig, roll=0.0, pitch=0.0, with_height=False):
    """Returns (angles_body, ranges_horizontal, quality, tilt_deg) [, z_rel].

    with_height=True also returns z_rel: the height of each return relative to base_link,
    z = lidar_z + r * sin(e). A tilted scan plane dips: beams that hit BELOW the normal
    plane (e.g. the top of a 1.0 m box while the drone pitches to accelerate) must not be
    taken as proof of a full-height wall (see hazards wall check).

    * angles are rotated by the mounting yaw (LASER_YAW_DEG) into the body frame
    * ranges outside [min, max] -> inf (no return)
    * isolated single-beam spikes (disagree with both neighbours, which agree) -> inf
    * each range is projected to the horizontal (tilt correction)
    quality = 1 - (fraction of beams rejected as spikes)."""
    a = np.asarray(angles, float) + math.radians(cfg.lidar_yaw_deg)
    r = np.asarray(ranges, float).copy()
    bad = ~np.isfinite(r) | (r < cfg.lidar_min_range) | (r > cfg.lidar_max_range)
    r[bad] = np.inf
    rf = np.where(np.isfinite(r), r, np.nan)
    prev, nxt = np.roll(rf, 1), np.roll(rf, -1)
    with np.errstate(invalid="ignore"):
        spike = (np.abs(rf - prev) > cfg.spike_jump) & (np.abs(rf - nxt) > cfg.spike_jump) & \
                (np.abs(prev - nxt) < cfg.spike_jump)
    spike = np.nan_to_num(spike, nan=False).astype(bool)
    r[spike] = np.inf
    quality = 1.0 - spike.sum() / max(1, len(r))
    # tilt correction (per-beam elevation of the scan plane)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    bx, by = np.cos(a), np.sin(a)
    # d = Ry(pitch) Rx(roll) [bx, by, 0]  ->  dz = -sp*bx + cp*sr*by
    dz = -sp * bx + cp * sr * by
    cos_e = np.sqrt(np.clip(1.0 - dz * dz, 0.0, 1.0))
    r_h = np.where(np.isfinite(r), r * cos_e, np.inf)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, cr * cp))))
    if with_height:
        fin = np.isfinite(r)
        z_rel = np.where(fin, cfg.lidar_xyz[2] + np.where(fin, r, 0.0) * dz, np.nan)
        return a, r_h, float(quality), tilt, z_rel
    return a, r_h, float(quality), tilt
