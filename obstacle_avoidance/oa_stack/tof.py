"""VL53L5CX zone geometry and ESP32 line parsing.

zone_directions() and zones_to_points() are taken unchanged from the guide's
tof_bridge.py (NIDAR_code_files.zip), so the bench calibration in guide Section 6
(transpose / flip_rows / flip_cols) applies as-is.
"""
from __future__ import annotations

import math

import numpy as np

FOV_DEG = 45.0  # VL53L5CX: about 45 deg horizontal / vertical (guide)


def zone_directions(pitch_deg, fov_deg=FOV_DEG, transpose=False, flip_rows=False, flip_cols=False):
    """Unit vectors (64 x 3) of the 8x8 zones in the drone frame (x forward, y left, z up).
    pitch_deg: sensor axis angle above horizontal (+20 = tilted up, -35 = tilted down).
    Zone index = row*8 + col. Row 0 = top of the view, col 0 = left (seen from behind)."""
    p = math.radians(pitch_deg)
    axis = np.array([math.cos(p), 0.0, math.sin(p)])
    up = np.array([-math.sin(p), 0.0, math.cos(p)])
    left = np.cross(up, axis)
    t = np.tan(np.radians(np.linspace(-fov_deg / 2, fov_deg / 2, 8)))
    dirs = np.zeros((8, 8, 3))
    for row in range(8):
        for col in range(8):
            v = axis + t[7 - row] * up + t[7 - col] * left
            dirs[row, col] = v / np.linalg.norm(v)
    if transpose:
        dirs = dirs.transpose(1, 0, 2)
    if flip_rows:
        dirs = dirs[::-1]
    if flip_cols:
        dirs = dirs[:, ::-1]
    return dirs.reshape(64, 3)


def zones_to_points(mm, dirs, sensor_xyz):
    """64 readings in mm (-1 = none) -> (N x 3) points in the drone frame."""
    d = np.asarray(mm, float) / 1000.0
    ok = d > 0
    return np.asarray(sensor_xyz) + dirs[ok] * d[ok][:, None]


def parse_line(line: str):
    """'U,<64 mm>' / 'D,<64 mm>' from tof_dual.ino -> ('U'|'D', list of 64 ints) or None."""
    parts = line.strip().split(",")
    if len(parts) != 65 or parts[0] not in ("U", "D"):
        return None
    try:
        return parts[0], [int(v) for v in parts[1:]]
    except ValueError:
        return None


def level_points(pts_body, roll, pitch):
    """Rotate drone-frame points by the drone's roll/pitch so z is truly 'up'
    (the guide's classify() assumes the drone is level enough; we correct it)."""
    if len(pts_body) == 0:
        return pts_body
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    # body -> level frame: R = Ry(pitch) * Rx(roll)   (yaw not needed)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return pts_body @ (Ry @ Rx).T
