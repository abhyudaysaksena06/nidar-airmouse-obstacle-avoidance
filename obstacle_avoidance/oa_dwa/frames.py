"""Layer 2 - time-aligned coordinate handling.

Frames (team conventions.md): map/odom ENU, origin at the maze entry.
Internally Algo 1 works in FLU body / ENU-like odom. If the team adopts FRD
body / NED world, inputs are MIRRORED (y -> -y, yaw -> -yaw, angles -> -angles)
at the interface and outputs mirrored back: a mirror maps a right-handed
FRD/NED pair onto a consistent FLU/ENU-like pair, so all geometry stays valid.
"""
from __future__ import annotations

import bisect
import math

import numpy as np


def wrap(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class FrameAdapter:
    """Converts between the team body frame and the internal FLU frame."""

    def __init__(self, body_frame: str = "FLU") -> None:
        if body_frame not in ("FLU", "FRD"):
            raise ValueError("body_frame must be 'FLU' or 'FRD'")
        self.s = 1.0 if body_frame == "FLU" else -1.0

    def pose_in(self, pose: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, yaw = pose
        return (x, self.s * y, self.s * yaw)

    def angles_in(self, angles: np.ndarray) -> np.ndarray:
        return self.s * np.asarray(angles, float)

    def vec_in(self, v: tuple[float, float]) -> np.ndarray:
        return np.array([v[0], self.s * v[1]], float)

    def points_in(self, pts: np.ndarray) -> np.ndarray:
        p = np.array(pts, float, copy=True)
        if len(p):
            p[:, 1] *= self.s
        return p

    def vel_out(self, vx: float, vy: float, w: float) -> tuple[float, float, float]:
        return vx, self.s * vy, self.s * w

    def xy_out(self, x: float, y: float) -> tuple[float, float]:
        return x, self.s * y


class PoseBuffer:
    """Stamped odometry poses for time-aligned transforms: a scan or depth
    frame captured at t_c is placed with the pose interpolated at t_c,
    not the latest pose (Jainil README, Layer 2)."""

    def __init__(self, horizon_s: float) -> None:
        self.horizon = horizon_s
        self.t: list[float] = []
        self.p: list[tuple[float, float, float]] = []

    def add(self, t: float, pose: tuple[float, float, float]) -> None:
        if self.t and t <= self.t[-1]:
            self.t[-1], self.p[-1] = t, pose
        else:
            self.t.append(t)
            self.p.append(pose)
        while self.t and self.t[0] < t - self.horizon:
            self.t.pop(0)
            self.p.pop(0)

    def at(self, t: float) -> tuple[float, float, float] | None:
        """Pose at time t (linear interpolation, yaw on the circle)."""
        if not self.t:
            return None
        if t <= self.t[0]:
            return self.p[0]
        if t >= self.t[-1]:
            return self.p[-1]
        k = bisect.bisect_left(self.t, t)
        t0, t1 = self.t[k - 1], self.t[k]
        a = (t - t0) / (t1 - t0)
        (x0, y0, h0), (x1, y1, h1) = self.p[k - 1], self.p[k]
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), float(wrap(h0 + a * wrap(h1 - h0))))


def body_to_odom(pts: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))


def odom_to_body(pts: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))
