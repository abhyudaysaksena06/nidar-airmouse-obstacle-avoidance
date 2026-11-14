"""Layer 3 - local obstacle map (rolling log-odds costmap, odom frame).

Each cell holds log-odds L of being occupied:
    L <- clip(L + l_hit)   on a lidar/depth hit
    L <- clip(L + l_miss)  for cells a lidar beam passed through (free space)
    L <- L - sign(L) * decay * dt   for unobserved cells (moving things fade)
Tag status per cell (Jainil README, Layer 4 "candidate / confirmed"):
    confirmed  L >= l_confirmed   (>= 2 consistent hits)
    candidate  l_candidate <= L < l_confirmed
Candidates are only avoided when close (<= candidate_radius): a single noisy
return far away does not reroute us, but nothing close is ever ignored.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from .config import DWAConfig


class LocalCostmap:
    def __init__(self, cfg: DWAConfig) -> None:
        self.cfg = cfg
        self.n = int(round(cfg.map_size / cfg.map_res))
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self.origin = np.array([-cfg.map_size / 2, -cfg.map_size / 2])  # odom xy of cell [0,0]
        self.t_last = None

    # ------------------------------------------------------------------
    def recentre(self, x: float, y: float) -> None:
        """Keep the window centred on the drone (shift whole cells)."""
        res = self.cfg.map_res
        centre = self.origin + self.n * res / 2
        di, dj = int(round((y - centre[1]) / res)), int(round((x - centre[0]) / res))
        if di == 0 and dj == 0:
            return
        L = np.zeros_like(self.L)
        n = self.n
        si0, si1 = max(0, di), min(n, n + di)
        sj0, sj1 = max(0, dj), min(n, n + dj)
        L[si0 - di:si1 - di, sj0 - dj:sj1 - dj] = self.L[si0:si1, sj0:sj1]
        self.L = L
        self.origin = self.origin + np.array([dj * res, di * res])

    def _idx(self, pts: np.ndarray):
        ij = np.floor((pts - self.origin) / self.cfg.map_res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < self.n) & (ij[:, 1] < self.n)
        return ij[ok, 1], ij[ok, 0]  # (row = y, col = x)

    def integrate(self, t: float, sensor_xy: np.ndarray, hits_odom: np.ndarray,
                  free_rays_odom: np.ndarray | None = None) -> None:
        """hits_odom: obstacle points (odom). free_rays_odom: (K,2) ray end
        points for free-space clearing from sensor_xy (lidar only)."""
        cfg = self.cfg
        self.seen[:] = False
        if free_rays_odom is not None and len(free_rays_odom):
            d = free_rays_odom - sensor_xy
            length = np.hypot(d[:, 0], d[:, 1])
            ts = np.arange(0.0, float(length.max()) + cfg.map_res, cfg.map_res * 0.8)
            frac = ts[None, :] / np.maximum(length[:, None], 1e-6)
            m = frac < 1.0 - cfg.map_res / np.maximum(length[:, None], 1e-6)
            px = sensor_xy[0] + d[:, 0:1] * frac
            py = sensor_xy[1] + d[:, 1:2] * frac
            r, c = self._idx(np.column_stack((px[m], py[m])))
            self.L[r, c] += cfg.l_miss
            self.seen[r, c] = True
        if len(hits_odom):
            r, c = self._idx(hits_odom)
            np.add.at(self.L, (r, c), cfg.l_hit)
            self.seen[r, c] = True
        if self.t_last is not None:
            dt = max(0.0, t - self.t_last)
            unseen = ~self.seen
            self.L[unseen] -= np.sign(self.L[unseen]) * np.minimum(np.abs(self.L[unseen]), cfg.decay_per_s * dt)
        self.t_last = t
        np.clip(self.L, cfg.l_min, cfg.l_max, out=self.L)

    # ------------------------------------------------------------------
    def obstacle_points(self, pose_xy: np.ndarray, max_range: float) -> np.ndarray:
        """Cell centres (odom) to avoid: confirmed everywhere in range,
        candidates only within candidate_radius."""
        cfg = self.cfg
        rows, cols = np.nonzero(self.L >= cfg.l_candidate)
        if rows.size == 0:
            return np.zeros((0, 2))
        pts = self.origin + (np.column_stack((cols, rows)) + 0.5) * cfg.map_res
        d = np.hypot(pts[:, 0] - pose_xy[0], pts[:, 1] - pose_xy[1])
        conf = self.L[rows, cols] >= cfg.l_confirmed
        keep = (d <= max_range) & (conf | (d <= cfg.candidate_radius))
        return pts[keep]

    def obstacle_report(self, min_cells: int = 2) -> list[dict]:
        """Connected components of CONFIRMED cells -> obstacles for the map /
        GCS ("map what problems/obstacles exist on each path", team chat)."""
        cfg = self.cfg
        occ = self.L >= cfg.l_confirmed
        seen = np.zeros_like(occ)
        out = []
        for r0, c0 in zip(*np.nonzero(occ)):
            if seen[r0, c0]:
                continue
            q, cells = deque([(r0, c0)]), []
            seen[r0, c0] = True
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for a, b in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if 0 <= a < self.n and 0 <= b < self.n and occ[a, b] and not seen[a, b]:
                        seen[a, b] = True
                        q.append((a, b))
            if len(cells) < min_cells:
                continue
            arr = np.array(cells)
            xy = self.origin + (arr[:, ::-1] + 0.5) * cfg.map_res
            out.append({"centroid": xy.mean(0).round(2).tolist(),
                        "extent_m": round(float(np.hypot(*(xy.max(0) - xy.min(0)))) + cfg.map_res, 2),
                        "cells": len(cells),
                        "confidence": round(float(self.L[arr[:, 0], arr[:, 1]].mean() / cfg.l_max), 2)})
        return out
