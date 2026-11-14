"""Control Barrier Function safety filter.

For every nearby obstacle i (body frame, relative position p_i, own velocity
v_i, radius rho_i) define the barrier

    h_i = ||p_i|| - (r_safe + rho_i)            (>= 0  <=>  safe)

With drone velocity u, d/dt p_i = v_i - u, so  dh_i/dt = n_i . (v_i - u)
with n_i = p_i / ||p_i||. Enforcing  dh_i/dt >= -kappa(h_i)  keeps h_i >= 0
forever (forward invariance). This is linear in u:

    n_i . u  <=  kappa(h_i) + n_i . v_i

kappa(h) = min(alpha * h, sqrt(2 * a_brake * h))  for h >= 0, alpha*h for h<0:
the approach speed towards an obstacle never exceeds the speed from which
the vehicle can still brake to a stop within h (and pushes it out if h < 0).

The filtered command is the closest admissible velocity to the nominal one:

    u* = argmin ||u - u_nom||^2   s.t.  A u <= b,  plus a polygonal |u| <= v_max

solved EXACTLY in 2D (no solver library): the optimum is u_nom itself, its
projection onto one constraint line, or a vertex of two lines.
"""
import math

import numpy as np


def kappa(h, cfg):
    hp = np.maximum(h, 0.0)
    inside = np.maximum(cfg.cbf_alpha_inside * h, -cfg.cbf_push_max)  # gentle, capped push-out
    return np.where(h >= 0, np.minimum(cfg.cbf_alpha * hp, np.sqrt(2 * cfg.a_brake * hp)), inside)


def sector_reduce(pts, sector_deg=5.0):
    """Keep the closest point per angular sector (bounded QP size)."""
    if len(pts) == 0:
        return pts
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    d = np.hypot(pts[:, 0], pts[:, 1])
    k = np.floor((ang + np.pi) / math.radians(sector_deg)).astype(int)
    order = np.lexsort((d, k))
    k_sorted = k[order]
    first = np.ones(len(order), dtype=bool)
    first[1:] = k_sorted[1:] != k_sorted[:-1]
    return pts[order[first]]


def build_constraints(static_pts, moving, cfg, latency_speed=0.0):
    """Rows (a_i, b_i) of  a_i . u <= b_i ; a_i unit vectors."""
    A, b = [], []
    # latency: during the delay we keep moving at the old speed
    r_eff = cfg.r_safe + latency_speed * cfg.latency_s
    if len(static_pts):
        p = sector_reduce(static_pts)
        d = np.hypot(p[:, 0], p[:, 1])
        sel = (d < cfg.cbf_range) & (d > 1e-3)
        p, d = p[sel], d[sel]
        n = p / d[:, None]
        h = d - r_eff
        A.append(n)
        b.append(kappa(h, cfg))
    for c, v, rho in moving:
        d = float(np.hypot(*c))
        if d < 1e-3 or d > cfg.cbf_range + 1.5:
            continue
        n = c / d
        h = d - r_eff - rho
        A.append(n[None, :])
        b.append(np.atleast_1d(kappa(np.array(h), cfg) + n @ v))
    # limited-FOV lidar: we cannot see what is behind/beside us, so motion
    # into the blind sector is capped to a slow creep (memory points above
    # still forbid it entirely where obstacles were seen)
    half = math.radians(cfg.lidar_fov_deg) / 2
    if half < math.pi - 1e-6:
        # half-planes e.u <= b also cap motion *towards* neighbouring directions,
        # so only constrain directions well behind the sensor (>= max(FOV/2, 90) + 15 deg);
        # the FOV guard (yaw towards motion) covers the sides
        start = max(half, math.pi / 2) + math.radians(15)
        th = np.arange(-np.pi, np.pi, math.radians(15))
        th = th[np.abs(th) >= start - 1e-9]
        A.append(np.column_stack((np.cos(th), np.sin(th))))
        b.append(np.full(len(th), cfg.blind_speed))
    # speed limit as a 16-gon (inscribed polygon -> conservative)
    th = np.arange(16) * 2 * np.pi / 16
    A.append(np.column_stack((np.cos(th), np.sin(th))))
    b.append(np.full(16, cfg.v_max * math.cos(np.pi / 16)))
    return np.vstack(A), np.concatenate(b)


def project_2d(u0, A, b, tol=1e-7):
    """Exact Euclidean projection of u0 onto {u : A u <= b} in 2D.
    Returns (u, feasible)."""
    u0 = np.asarray(u0, float)
    if np.all(A @ u0 <= b + tol):
        return u0, True
    cands = [u0 - (A[i] @ u0 - b[i]) * A[i] for i in range(len(b))]  # line projections
    m = len(b)
    if m >= 2:
        i, j = np.triu_indices(m, 1)
        det = A[i, 0] * A[j, 1] - A[i, 1] * A[j, 0]
        ok = np.abs(det) > 1e-9
        i, j, det = i[ok], j[ok], det[ok]
        x = (b[i] * A[j, 1] - b[j] * A[i, 1]) / det
        y = (A[i, 0] * b[j] - A[j, 0] * b[i]) / det
        cands.extend(np.column_stack((x, y)))
    C = np.array(cands)
    feas = np.all(C @ A.T <= b[None, :] + 1e-6, axis=1)
    if not feas.any():
        return None, False
    C = C[feas]
    k = int(np.argmin(np.hypot(*(C - u0).T)))
    return C[k], True


def filter_velocity(u_nom, static_pts, moving, cfg, latency_speed=0.0):
    """Returns (u_safe, info). If constraints are mutually infeasible (boxed
    in), the farthest obstacles are relaxed first until feasible."""
    A, b = build_constraints(static_pts, moving, cfg, latency_speed)
    half = math.radians(cfg.lidar_fov_deg) / 2
    start = max(half, math.pi / 2) + math.radians(15)
    n_blind = 0 if half >= math.pi - 1e-6 else         int(np.sum(np.abs(np.arange(-np.pi, np.pi, math.radians(15))) >= start - 1e-9))
    n_obs = len(b) - 16 - n_blind
    u, ok = project_2d(u_nom, A, b)
    relaxed = 0
    if not ok:
        # drop obstacle rows with the largest slack first (keep the closest)
        order = np.argsort(-b[:n_obs])
        k = 1
        while k <= n_obs:                      # drop 1, 2, 4, 8, ... farthest rows (bounded cost)
            keep = np.ones(len(b), dtype=bool)
            keep[order[:k]] = False
            u, ok = project_2d(u_nom, A[keep], b[keep])
            if ok:
                relaxed = k
                break
            k = k * 2 if k < n_obs else n_obs + 1
        if not ok:
            u = np.zeros(2)
    active = int(np.sum(A[:n_obs] @ u >= b[:n_obs] - 1e-4)) if n_obs else 0
    return u, {"n_constraints": n_obs, "active": active, "relaxed": relaxed}
