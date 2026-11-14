"""Configuration for Algo 1 (layered costmap + DWA).

Defaults live in configs/oa_dwa.yaml (team convention: tuneable values in
YAML under configs/). All units SI: metres, seconds, radians unless *_deg.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from pathlib import Path


@dataclass
class DWAConfig:
    """All tuneable parameters of Algo 1."""

    # --- interface ---------------------------------------------------------
    body_frame: str = "FLU"          # "FLU" (ROS REP 103) or "FRD" (aviation); see frames.py
    # --- vehicle -------------------------------------------------------------
    drone_radius: float = 0.25       # m, incl. prop guards
    margin: float = 0.15             # m -> inflation radius r_inf = radius + margin
    half_height: float = 0.10        # m, half of the drone's vertical extent
    v_max: float = 1.0               # m/s forward
    v_min: float = -0.15             # m/s, slow reverse only (front lidar cannot see behind)
    w_max: float = 1.2               # rad/s yaw rate
    a_lin: float = 1.5               # m/s^2 linear acceleration / braking the FC delivers
    a_ang: float = 3.0               # rad/s^2
    # --- L1 robust measurement -------------------------------------------------
    lidar_fov_deg: float = 180.0     # chat: "2D front LiDAR"
    lidar_min_range: float = 0.15
    lidar_max_range: float = 8.0
    spike_jump: float = 0.5          # m, isolated single-beam spikes rejected (median-of-3 test)
    depth_min_range: float = 0.2
    depth_max_range: float = 4.0
    depth_band: float = 0.30         # m above/below the drone height band that counts as obstacle
    depth_ground_z: float = 0.10     # m, depth points this close to the floor are ground
    # --- L2 time alignment -----------------------------------------------------
    pose_buffer_s: float = 2.0
    scan_stale_s: float = 0.35
    # --- L3 local costmap ------------------------------------------------------
    map_size: float = 8.0            # m, rolling window centred on the drone
    map_res: float = 0.10            # m/cell
    l_hit: float = 0.85              # log-odds increments
    l_miss: float = -0.60
    l_min: float = -2.0
    l_max: float = 3.5
    l_confirmed: float = 1.2         # >= 2 consistent hits
    l_candidate: float = 0.4
    candidate_radius: float = 1.0    # m, candidates closer than this are treated as real
    decay_per_s: float = 0.50        # log-odds decay of unobserved cells (moving things fade)
    # --- L4 DWA -----------------------------------------------------------------
    window_dt: float = 0.30          # s, reachable velocities: v +- a * window_dt
    sim_time: float = 3.0            # s, forward simulation horizon
    sim_step: float = 0.15
    turn_time: float = 1.0           # s, turning segment of each trajectory, then straight
    n_v: int = 7
    n_w: int = 11
    goal_distance: float = 2.0       # m, local goal placed along the desired velocity
    w_heading: float = 1.0           # score weights (normalised terms)
    w_obstacle: float = 0.6
    w_velocity: float = 0.3
    w_smooth: float = 0.15
    inflation_decay: float = 0.20    # m, lambda of the exponential obstacle cost
    obstacle_range: float = 3.0      # m, obstacles considered by DWA
    # --- L5 failsafes ------------------------------------------------------------
    emergency_dist: float = 0.05     # m beyond radius
    backoff_speed: float = 0.15
    stuck_s: float = 3.0
    rotate_search_w: float = 0.6     # rad/s when BLOCKED: rotate to look for a way

    @property
    def r_inf(self) -> float:
        """Inflation radius used for collision checking."""
        return self.drone_radius + self.margin

    def to_dict(self) -> dict:
        """Plain dict (for logging / reports)."""
        return asdict(self)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "DWAConfig":
        """Load from YAML; unknown keys raise, missing keys keep defaults."""
        import yaml  # local import: only needed when loading files

        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        flat: dict = {}
        for v in data.values() if all(isinstance(v, dict) for v in data.values()) else [data]:
            flat.update(v)
        known = {f.name for f in fields(cls)}
        unknown = set(flat) - known
        if unknown:
            raise KeyError(f"unknown config keys: {sorted(unknown)}")
        return cls(**flat)
