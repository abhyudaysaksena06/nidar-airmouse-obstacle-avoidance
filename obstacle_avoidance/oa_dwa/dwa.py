"""Layer 4 - Dynamic Window Approach (Fox, Burgard & Thrun, 1997).

Model (front-facing sensor -> the drone faces where it flies):
    x' = v cos(th),  y' = v sin(th),  th' = w        (body frame at t=0)

1. Dynamic window   V_d = [v_c - a dT, v_c + a dT] x [w_c - b dT, w_c + b dT]
                     intersected with [v_min, v_max] x [-w_max, w_max]
2. Sample n_v x n_w pairs; each is simulated as "turn at (v, w) for turn_time,
   then straight" over sim_time (two-segment trajectories, see rollout()).
3. Admissible:  no trajectory point closer than r_inf to an obstacle within the
   stopping horizon, and  |v| <= sqrt(2 a d_coll)  where d_coll is the travel
   distance until the arc first comes within r_inf of an obstacle.
4. Score (ABSOLUTE terms in [0, 1]; not normalised over the sample set):
       G = w_h * heading + w_v * velocity - w_o * obstacle - w_s * change
   heading   = 0.7 * progress + 0.3 * (1 - |angle from final pose to goal| / pi)
               progress = (|g| - |g - p_end|) / (v_max T)   (distance gained)
   obstacle  = 0.5 max_t c(t) + 0.5 mean_t c(t),  c = exp(-max(d - r_inf, 0) / lambda)
   velocity  = v / v_max
   change    = |v - v_prev| / v_max + |w - w_prev| / w_max  (temporal consistency)
"""
from __future__ import annotations

import math

import numpy as np

from .config import DWAConfig


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def rollout(v: np.ndarray, w: np.ndarray, T: float, dt: float, t_turn: float = 1.0) -> np.ndarray:
    """Two-segment trajectories: turn at (v, w) for t_turn, then fly straight
    at v for the rest of the horizon. A single constant arc cannot express
    "turn a bit, then go" - with a long horizon every fast arc either hits the
    obstacle or curls back, and DWA creeps. Returns (K, S, 3) of x, y, th."""
    t = np.arange(dt, T + 1e-9, dt)
    th = w[:, None] * np.minimum(t, t_turn)[None, :]                    # (K,S)
    th_mid = th - 0.5 * w[:, None] * dt * (t <= t_turn)[None, :]          # midpoint heading
    x = np.cumsum(v[:, None] * np.cos(th_mid) * dt, axis=1)
    y = np.cumsum(v[:, None] * np.sin(th_mid) * dt, axis=1)
    return np.stack((x, y, th), axis=-1)


class DWAPlanner:
    def __init__(self, cfg: DWAConfig) -> None:
        self.cfg = cfg
        self.v_prev = 0.0
        self.w_prev = 0.0

    def window(self, v_c: float, w_c: float) -> tuple[np.ndarray, np.ndarray]:
        c = self.cfg
        v_lo, v_hi = max(c.v_min, v_c - c.a_lin * c.window_dt), min(c.v_max, v_c + c.a_lin * c.window_dt)
        w_lo, w_hi = max(-c.w_max, w_c - c.a_ang * c.window_dt), min(c.w_max, w_c + c.a_ang * c.window_dt)
        vs = np.linspace(v_lo, v_hi, c.n_v)
        ws = np.linspace(w_lo, w_hi, c.n_w)
        if v_lo <= 0.0 <= v_hi:      # always allow "stop"
            vs = np.unique(np.append(vs, 0.0))
        if w_lo <= 0.0 <= w_hi:      # and "go straight"
            ws = np.unique(np.append(ws, 0.0))
        V, W = np.meshgrid(vs, ws, indexing="ij")
        return V.ravel(), W.ravel()

    def plan(self, obstacles: np.ndarray, goal: np.ndarray, v_c: float, w_c: float, v_goal: float):
        """obstacles: (M,2) body frame; goal: local goal (body); v_goal: speed cap.
        Returns (v, w, info) or (None, None, info) if nothing is admissible."""
        c = self.cfg
        V, W = self.window(v_c, w_c)
        V = np.minimum(V, max(v_goal, 0.0))  # never faster than the planner asked
        traj = rollout(V, W, c.sim_time, c.sim_step, c.turn_time)  # (K,S,3)
        K, S, _ = traj.shape
        if len(obstacles):
            P = traj[:, :, :2].reshape(-1, 2)
            d = np.sqrt(((P[:, None, :] - obstacles[None, :, :]) ** 2).sum(-1)).min(1).reshape(K, S)
        else:
            d = np.full((K, S), np.inf)
        step_len = np.abs(V)[:, None] * c.sim_step * np.arange(1, S + 1)[None, :]
        coll = d < c.r_inf
        first = np.where(coll.any(1), coll.argmax(1), S)
        d_coll = np.where(first < S, step_len[np.arange(K), np.minimum(first, S - 1)], np.inf)
        d_coll = np.where(first == 0, 0.0, d_coll)
        # hard collision with the BODY anywhere inside the stopping horizon
        t_stop = np.abs(V) / c.a_lin + 0.3
        idx_stop = np.minimum((t_stop / c.sim_step).astype(int) + 1, S)
        body_hit = np.array([(d[k, :idx_stop[k]] < c.drone_radius + 0.02).any() for k in range(K)])
        brake_ok = np.abs(V) <= np.sqrt(2 * c.a_lin * np.maximum(d_coll - 0.05, 0.0)) + 1e-9
        stopped = np.abs(V) < 1e-6
        brakeable = (~body_hit & brake_ok) | (stopped & (d[:, 0] >= c.drone_radius))
        # Prefer arcs that stay outside r_inf for the WHOLE horizon (Nav2 DWB
        # style). The textbook "can still brake" rule alone lets an arc aimed
        # straight at an obstacle win every cycle until it is too late to turn.
        clear_horizon = brakeable & (first == S)
        admissible = clear_horizon if clear_horizon.any() else brakeable
        info = {"n_samples": int(K), "n_admissible": int(admissible.sum()),
                "n_moving_admissible": int((admissible & (np.abs(V) > 0.05)).sum()),
                "fallback_braking": bool(not clear_horizon.any())}
        if not admissible.any():
            return None, None, info
        # scores
        fx, fy, fth = traj[:, -1, 0], traj[:, -1, 1], traj[:, -1, 2]
        ang = np.abs(wrap(np.arctan2(goal[1] - fy, goal[0] - fx) - fth))
        # progress: how much closer the arc END gets to the goal (a pure heading
        # term rewards stopping while pointing at the goal - classic DWA trap)
        g = float(np.hypot(goal[0], goal[1]))
        progress = (g - np.hypot(goal[0] - fx, goal[1] - fy)) / max(c.v_max * c.sim_time, 1e-6)
        heading = 0.7 * progress + 0.3 * (1.0 - ang / math.pi)
        # obstacle COST (Nav2-style inflation): rises exponentially near obstacles.
        # A "reward clearance" term makes hovering in open space win, because a
        # point keeps more clearance than any arc that makes progress.
        e = np.exp(-np.maximum(d - c.r_inf, 0.0) / c.inflation_decay)          # (K,S) in (0,1]
        obstacle = 0.5 * e.max(1) + 0.5 * e.mean(1)
        velocity = V / c.v_max
        change = 0.5 * (np.abs(V - self.v_prev) / c.v_max + np.abs(W - self.w_prev) / c.w_max)
        # ABSOLUTE terms in [0, 1]: normalising each term over the sample set
        # (classic DWA) blows centimetre differences up to the full range.
        G = c.w_heading * heading + c.w_velocity * velocity - c.w_obstacle * obstacle - c.w_smooth * change
        clear = d.min(1) - c.drone_radius
        G[~admissible] = -np.inf
        k = int(np.argmax(G))
        self.v_prev, self.w_prev = float(V[k]), float(W[k])
        info.update({"score": float(G[k]), "clearance": float(d[k].min() - c.drone_radius)})
        return float(V[k]), float(W[k]), info
