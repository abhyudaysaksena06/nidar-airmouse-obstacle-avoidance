"""OA-Stack parameters.

Every value is either copied from the zip's guide (NIDAR_Budget_Sensor_Stack_Guide.pdf v1.2,
its launch file and tof_bridge.py) -> marked GUIDE, or a placeholder -> marked TBD, to be
replaced with measurements during testing. Units: metres, seconds, degrees where named *_deg.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass
class StackConfig:
    # --- drone body (TBD: measure the real drone) ---------------------------
    drone_radius: float = 0.25          # TBD  horizontal radius incl. prop guards
    body_bottom_dz: float = -0.035      # GUIDE sim: underside 0.965 m when base_link at 1.0 m
    prop_top_dz: float = 0.044          # GUIDE sim: prop tops ~8.6 cm under a 1.13 m duct at 1.0 m
    margin: float = 0.15                # TBD  extra horizontal clearance
    v_max: float = 1.0                  # GUIDE "fly slowly (under 1 m/s)"
    a_brake: float = 1.5                # TBD  braking deceleration the FC achieves
    latency_s: float = 0.10             # TBD  sensing + compute + FC response
    vfh_window: float = 2.0             # TBD  VFH+ window, sized for the guide's 1.5 m corridors:
                                        #      blocked under 0.82 m, free beyond 1.23 m

    # --- RPLIDAR C1 -----------------------------------------------------------
    lidar_xyz: tuple = (0.0, 0.0, 0.08)  # GUIDE launch file: 8 cm above base_link
    lidar_yaw_deg: float = 0.0          # GUIDE launch file LASER_YAW_DEG
    lidar_min_range: float = 0.15       # GUIDE launch / Cartographer min_range
    lidar_max_range: float = 12.0       # GUIDE (6 m on black targets)
    lidar_rate_hz: float = 10.0         # GUIDE typical
    spike_jump: float = 0.5             # TBD  single-beam spike rejection threshold
    tilt_warn_deg: float = 10.0         # TBD  above this, lidar plane error grows (guide 11.1)
    tilt_max_deg: float = 20.0          # TBD  above this, the scan is not trusted (hover)

    # --- VL53L5CX ToF pair (guide launch-file example values) --------------------
    tof_fov_deg: float = 45.0           # GUIDE tof_bridge.FOV_DEG
    tof_max_range: float = 3.0          # GUIDE sim (4 m only in a dark room; ~1.5 m bright hall)
    up_xyz: tuple = (0.10, 0.0, 0.045)  # GUIDE launch example
    up_pitch_deg: float = 20.0          # GUIDE
    down_xyz: tuple = (0.15, 0.0, 0.0)  # GUIDE launch example
    down_pitch_deg: float = -35.0       # GUIDE
    up_calib: dict = field(default_factory=lambda: {"transpose": False, "flip_rows": False, "flip_cols": False})
    down_calib: dict = field(default_factory=lambda: {"transpose": False, "flip_rows": False, "flip_cols": False})
    tof_stale_s: float = 0.5            # TBD  no ToF frame for this long -> ToF considered down

    # --- hazard windows (GUIDE tof_bridge.DEFAULTS) --------------------------------
    up_min_dz: float = 0.06             # GUIDE  duct band starts this far above base_link (= prop tops + 2 cm)
    up_max_dz: float = 0.34             # GUIDE  ...and ends here
    up_reach: float = 1.0               # GUIDE  duct considered within this horizontal distance
    low_min_h: float = 0.20             # GUIDE  points lower than this above the floor = floor/ignored
    low_max_dz: float = 0.05            # GUIDE  low obstacle = top below this (relative to base_link)
    low_reach: float = 1.3              # GUIDE
    wall_radius_up: float = 0.15        # GUIDE  ToF point this close to a known wall is ignored
    wall_radius_down: float = 0.20      # GUIDE
    occ_threshold: int = 65             # GUIDE  "strong wall" in Cartographer's /map (0-100)

    # --- reactions (GUIDE section 9.6 / 13) ------------------------------------------
    duct_descend: float = 0.25          # GUIDE  "descend 25 cm below a duct"
    low_climb: float = 0.35             # GUIDE  "climb 35 cm above its top"
    max_climb: float = 0.8              # TBD  never climb more than this above cruise
    min_alt: float = 0.40               # TBD
    max_alt: float = 2.0                # TBD  (guide sim walls 2.4 m)
    vert_slow_scale: float = 0.25       # TBD  forward speed factor until a climb/duck is done
    vert_done_tol: float = 0.08         # TBD

    # --- memory / tracking ---------------------------------------------------------------
    hazard_memory_s: float = 5.0        # TBD  ToF hazards remembered (they leave the ToF view when under/over them)
    column_res: float = 0.10            # TBD
    use_map: bool = False               # Cartographer /map in the wall check. OFF by default: a map built
                                        # while the drone dipped or tilted can contain the TOP of a low
                                        # obstacle, which then hides that obstacle (found in simulation)
    wall_evidence_margin: float = 0.02  # TBD  lidar returns below low_max_dz + this are not wall evidence
    unconfirmed_top_margin: float = 0.0  # OFF. >0 (e.g. 0.04): DOWN points this close to the view's top edge
                                        #      -> top unknown, treat as solid and climb. A level drone cannot see
                                        #      the top of a 1.0 m obstacle at 1.0 m cruise; in the benchmark the
                                        #      rule added no safety and raised timeouts 17% -> 30%, so it is off
    use_memory: bool = True             # ablation switch
    use_wall_check: bool = True         # ablation switch
    use_inband: bool = True             # ablation switch (ToF obstacles at our height the lidar missed)

    def to_dict(self) -> dict:
        return asdict(self)
