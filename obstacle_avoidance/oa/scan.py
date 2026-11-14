"""Scan preprocessing and short-term obstacle memory."""
import math

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def scan_to_points(angles, ranges, cfg):
    """Body-frame points (N,2) from a 2D scan, keeping valid returns inside
    the lidar field of view. Returns (points, angles_kept, ranges_kept)."""
    angles = np.asarray(angles, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    ok = np.isfinite(ranges) & (ranges >= cfg.lidar_min_range) & (ranges < cfg.lidar_max_range - 0.05)
    ok &= np.abs(wrap(angles)) <= math.radians(cfg.lidar_fov_deg) / 2 + 1e-9
    a, r = angles[ok], ranges[ok]
    return np.column_stack((r * np.cos(a), r * np.sin(a))), a, r


def body_to_world(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))


def world_to_body(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def voxel_downsample(pts, voxel):
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


class ObstacleMemory:
    """Keeps recently seen obstacle points in the odometry frame so that
    obstacles leaving a limited lidar FOV (or hidden by a dropout) are still
    avoided for `memory_s` seconds. Odometry drift over that horizon is small."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.pts = np.zeros((0, 2))
        self.stamp = np.zeros(0)

    def add(self, pts_body, pose, t):
        if len(pts_body):
            w = voxel_downsample(body_to_world(pts_body, pose), self.cfg.voxel)
            self.pts = np.vstack((self.pts, w))
            self.stamp = np.concatenate((self.stamp, np.full(len(w), t)))
        keep = (t - self.stamp) <= self.cfg.memory_s
        d = np.hypot(self.pts[:, 0] - pose[0], self.pts[:, 1] - pose[1])
        keep &= d <= self.cfg.memory_radius
        self.pts, self.stamp = self.pts[keep], self.stamp[keep]

    def outside_fov(self, pose):
        """Remembered points (body frame) that the current scan cannot see."""
        if len(self.pts) == 0:
            return self.pts
        b = world_to_body(self.pts, pose)
        ang = np.arctan2(b[:, 1], b[:, 0])
        half = math.radians(self.cfg.lidar_fov_deg) / 2
        if half >= math.pi - 1e-6:
            return b[:0]
        return b[np.abs(ang) > half]
