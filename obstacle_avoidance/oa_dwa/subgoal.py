"""Layer 4a - local sub-goal (Nav2 structure: costmap path -> DWA follows it).

DWA alone has a well-known local minimum: if the local goal lies behind an
obstacle, arcs pointed straight at it score best until it is too late to turn.
In Nav2 the global planner routes around costmap obstacles and the local
planner follows that path. Here, when the straight segment to the local goal
is blocked, a small 8-connected A* on the local costmap (body frame, inflated
by r_inf) finds a way round and DWA tracks a point `lookahead` metres along it.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .config import DWAConfig


def segment_blocked(goal: np.ndarray, obs: np.ndarray, radius: float) -> bool:
    """Does the segment origin->goal pass within `radius` of any obstacle?"""
    if len(obs) == 0:
        return False
    L = float(np.hypot(*goal))
    if L < 1e-6:
        return False
    u = goal / L
    lon = np.clip(obs @ u, 0.0, L)
    closest = np.outer(lon, u)
    return bool((np.hypot(*(obs - closest).T) < radius).any())


def _disk_dilate(g: np.ndarray, r: int) -> np.ndarray:
    out = g.copy()
    H, W = g.shape
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r * r:
                continue
            out[max(0, di):H - max(0, -di), max(0, dj):W - max(0, -dj)] |= \
                g[max(0, -di):H - max(0, di), max(0, -dj):W - max(0, dj)]
    return out


def local_subgoal(goal: np.ndarray, obs: np.ndarray, cfg: DWAConfig, lookahead: float = 1.5):
    """Returns (subgoal_body or None if no path, path_length)."""
    res = cfg.map_res
    n = int(round(cfg.map_size / res))
    half = n // 2
    occ = np.zeros((n, n), dtype=bool)
    if len(obs):
        ij = np.floor(obs / res).astype(int) + half
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < n) & (ij[:, 1] < n)
        occ[ij[ok, 1], ij[ok, 0]] = True
    blocked = _disk_dilate(occ, int(math.ceil(cfg.r_inf / res)))
    s = (half, half)
    blocked[half - 1:half + 2, half - 1:half + 2] = False
    # goal: clip into the window, then pull back along the ray until free
    g = goal * min(1.0, (cfg.map_size / 2 - 0.2) / max(float(np.hypot(*goal)), 1e-6))
    for frac in np.linspace(1.0, 0.2, 17):
        gi = tuple((np.floor(g * frac / res).astype(int) + half)[::-1])
        if not blocked[gi]:
            break
    else:
        return None, math.inf
    openq, came, cost = [(0.0, 0.0, s)], {s: None}, {s: 0.0}
    steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
    while openq:
        _, c, cur = heapq.heappop(openq)
        if cur == gi:
            break
        if c > cost[cur]:
            continue
        for di, dj, sc in steps:
            nb = (cur[0] + di, cur[1] + dj)
            if 0 <= nb[0] < n and 0 <= nb[1] < n and not blocked[nb] and c + sc < cost.get(nb, 1e18):
                cost[nb], came[nb] = c + sc, cur
                heapq.heappush(openq, (c + sc + math.hypot(nb[0] - gi[0], nb[1] - gi[1]), c + sc, nb))
    if gi not in came:
        return None, math.inf
    path, cur = [], gi
    while cur is not None:
        path.append(((cur[1] - half + 0.5) * res, (cur[0] - half + 0.5) * res))
        cur = came[cur]
    path = np.array(path[::-1])
    seg = np.hypot(*np.diff(path, axis=0).T)
    acc = np.concatenate(([0.0], np.cumsum(seg)))
    k = int(np.searchsorted(acc, lookahead))
    return path[min(k, len(path) - 1)], float(acc[-1])
