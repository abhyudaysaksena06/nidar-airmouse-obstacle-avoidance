"""All tunable parameters of the obstacle-avoidance stack (SI units)."""
from dataclasses import dataclass, asdict


@dataclass
class AvoidanceConfig:
    # --- vehicle ---------------------------------------------------------
    drone_radius: float = 0.25      # m, incl. prop guards
    margin: float = 0.15            # m, extra clearance -> r_safe = radius + margin
    v_max: float = 1.0              # m/s horizontal speed limit
    a_brake: float = 1.5            # m/s^2 deceleration we trust the FC to deliver
    latency_s: float = 0.10         # s, sensing + compute + actuation delay
    # --- sensor ----------------------------------------------------------
    lidar_fov_deg: float = 360.0    # 360 = spinning lidar; e.g. 180/270 for a front lidar
    lidar_min_range: float = 0.15
    lidar_max_range: float = 8.0
    scan_stale_s: float = 0.35      # no scan for this long -> STALE (hover)
    # --- obstacle memory (covers lidar blind sectors) ---------------------
    memory_s: float = 5.0
    memory_radius: float = 3.0
    voxel: float = 0.08
    # --- VFH+ direction selection ----------------------------------------
    window: float = 3.0             # m, active window of the polar histogram
    sector_deg: float = 5.0
    tau_low: float = 0.15           # histogram hysteresis thresholds
    tau_high: float = 0.35
    mu_target: float = 5.0          # cost weights: target dir, current heading, previous choice
    mu_heading: float = 1.0
    mu_prev: float = 2.0
    max_detour_deg: float = 135.0   # beyond this from the target -> BLOCKED
    commit_s: float = 2.0           # keep detouring on the same side this long (anti-oscillation)
    # --- CBF safety filter -------------------------------------------------
    cbf_alpha: float = 2.0          # 1/s, class-K gain: allowed approach rate = alpha * h
    cbf_range: float = 2.5          # m, only obstacles this close enter the QP
    cbf_alpha_inside: float = 1.0   # 1/s, push-out gain when already inside the margin (h < 0)
    cbf_push_max: float = 0.4       # m/s, cap on that push-out
    # --- dynamic obstacle tracking ------------------------------------------
    cluster_gap: float = 0.30       # m, split clusters at range jumps
    track_max_extent: float = 0.9   # m, only compact clusters (people, debris) are tracked
    track_gate: float = 0.6         # m, association gate
    track_alpha: float = 0.5        # alpha-beta filter gains
    track_beta: float = 0.2
    dyn_speed: float = 0.15         # m/s, above this a track is "moving"
    dyn_min_hits: int = 5
    dyn_min_travel: float = 0.5     # m a track must actually travel before it counts as moving
    predict_s: float = 1.0          # s, look-ahead for moving obstacles in VFH
    # --- failsafes -----------------------------------------------------------
    emergency_dist: float = 0.05    # m beyond drone_radius -> EMERGENCY escape
    escape_speed: float = 0.3
    stuck_s: float = 3.0            # wanted to move but could not -> STUCK, ask to replan
    squeeze_after_s: float = 1.5    # slow progress this long -> narrow-passage mode
    min_margin: float = 0.08        # margin shrinks down to this in narrow-passage mode
    squeeze_rate: float = 0.05      # m/s of margin reduction
    squeeze_speed: float = 0.3      # m/s cap while squeezing
    # --- yaw policy (limited-FOV lidar must look where it flies) ------------
    yaw_gain: float = 1.5
    yaw_rate_max: float = 1.2
    fov_guard_deg: float = 10.0
    blind_speed: float = 0.15       # m/s max speed towards directions the lidar cannot see
    # --- vertical guard (forward-pitched ToF beams) -------------------------
    half_height: float = 0.10
    tof_pitch_deg: float = 10.0
    vertical_clearance: float = 0.25
    min_alt: float = 0.40
    # --- layer switches (ablation studies / debugging) -----------------------
    use_vfh: bool = True
    use_cbf: bool = True
    use_tracker: bool = True
    use_memory: bool = True

    @property
    def r_safe(self):
        return self.drone_radius + self.margin

    def to_dict(self):
        return asdict(self)
