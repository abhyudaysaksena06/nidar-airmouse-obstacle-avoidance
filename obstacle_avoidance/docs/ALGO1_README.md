# NIDAR AirMouse: Obstacle Avoidance, Algo 1 (Layered Costmap + DWA)

**Owner:** Abhyuday Saksena · **Module:** obstacle avoidance (local planning) · **Code:** `oa_dwa/` · **Config:** `configs/oa_dwa.yaml`

## Purpose
Keep the drone collision-free while it follows the path from the navigation module (A* + pure pursuit) through the GPS-denied indoor maze. Use the 2D front LiDAR (primary) and the depth camera (3D obstacles above or below the lidar plane). Report blocked paths and confirmed obstacles to mapping and the GCS.

This module covers **local obstacle avoidance only**. Human detection, SLAM mapping, survivor tagging and global path planning are separate modules.

## Priority
**Never collide; never stop silently.** Every cycle produces either a safe command or an explicit status (BLOCKED / STUCK / STALE / EMERGENCY) plus a replan request. Collision-free motion takes priority over speed and over following the planner exactly.

## Architecture

**Baseline:** the **Dynamic Window Approach** (Fox, Burgard & Thrun, 1997) on a rolling local costmap, the standard ROS 2 Nav2-style local planner, chosen as the team baseline ("baseline first → test → identify limitations → innovate").

**Layer 1: Robust obstacle measurement** (`perception.py`)
* Lidar: range limits, FOV mask (180° front lidar by default), rejection of isolated single-beam spikes (a beam that disagrees with both neighbours by > 0.5 m while they agree).
* Depth camera: keep points inside the drone's height band, $|z_{rel}| \le h_h + 0.30$ m; reject floor returns ($z_{abs} < 0.10$ m); voxel-downsample to 0.1 m.
* A **quality** value (spike fraction) is kept with each scan. Quality < 0.8 sets the status to DEGRADED.

**Layer 2: Time-aligned transforms** (`frames.py`)
* Odometry poses are buffered for 2 s. Each scan or depth frame is placed using the pose **interpolated at its capture time** $t_c$ (yaw interpolated on the circle), not the latest pose.
* $p_{odom} = R(\psi(t_c))\,p_{body} + [x(t_c), y(t_c)]^\top$
* Body frame FLU by default. `body_frame: FRD` mirrors inputs and outputs ($y \to -y$, $\psi \to -\psi$) so the same code serves an FRD/NED convention.

**Layer 3: Local costmap registration** (`costmap.py`)
* Rolling 8 × 8 m window at 0.1 m, centred on the drone, stored in the **odometry** frame (smooth; it never jumps on SLAM loop closure).
* Log-odds per cell: hit $+0.85$, ray pass-through $-0.6$ (free-space clearing), clamped to $[-2, 3.5]$. Unobserved cells decay towards 0 at 0.5 per s, so obstacles that moved away fade.
* **Candidate → confirmed** status, as in the tagging design:
  * confirmed: $L \ge 1.2$ (at least 2 consistent hits)
  * candidate: $0.4 \le L < 1.2$
  * Candidates are avoided **only when within 1 m**. A single noisy return far away doesn't reroute the drone, but nothing close is ever ignored.

**Layer 4: Local planning with temporal consistency** (`subgoal.py`, `dwa.py`)
* **4a Sub-goal.** The local goal sits 2 m along the planner's desired velocity. If the segment to it passes within $r_{inf} + 0.05$ of an obstacle, an 8-connected A* on the inflated local costmap finds a way round, and DWA tracks the point 1.5 m along that path. This is the Nav2 structure (costmap path, followed by DWA) and avoids DWA's local minimum when the goal is behind an obstacle.
* **4b DWA.** Unicycle model ($\dot x = v\cos\theta$, $\dot y = v\sin\theta$, $\dot\theta = \omega$), so the front sensor always looks where the drone flies.
  * Dynamic window: $V_d = [v_c \pm a\,\Delta T] \times [\omega_c \pm \alpha\,\Delta T] \cap [v_{min}, v_{max}] \times [\pm\omega_{max}]$, with $\Delta T = 0.3$ s, $a = 1.5$ m/s², $\alpha = 3$ rad/s². Sampled 7 × 11, plus "stop" and "straight".
  * Trajectories: **turn at $(v, \omega)$ for 1 s, then straight**, over a 3 s horizon (a single constant arc can't express "turn a bit, then go").
  * **Admissible:** no point within $r_{inf} = r + m$ over the **whole horizon**, and $|v| \le \sqrt{2 a\, d_{coll}}$. If no arc is clear for the horizon, the fallback is braking-feasible arcs. If no admissible arc *moves*, the status is BLOCKED.
  * Score, with absolute terms in [0, 1]:
    $$G = w_h\,\big(0.7\,\text{progress} + 0.3\,\text{heading}\big) + w_v\,\frac{v}{v_{max}} - w_o\,\text{obstacle} - w_s\,\text{change}$$
    * $\text{progress} = (|g| - |g - p_{end}|)/(v_{max} T)$ (distance gained towards the goal)
    * $\text{heading} = 1 - |\angle(g - p_{end}) - \theta_{end}|/\pi$
    * $\text{obstacle} = \tfrac12\max_t c + \tfrac12\,\overline{c}$ with $c = e^{-\max(d - r_{inf}, 0)/\lambda}$, $\lambda = 0.2$ m (Nav2-style inflation cost)
    * $\text{change} = \tfrac12\big(|v - v_{prev}|/v_{max} + |\omega - \omega_{prev}|/\omega_{max}\big)$ (temporal consistency)
    * Weights: $w_h = 1$, $w_v = 0.3$, $w_o = 0.6$, $w_s = 0.15$
* **Head-on rule:** if DWA chooses "stop" with an obstacle ahead, rotate towards the side with more clearance so new arcs open up.

**Layer 5: Fallback handling and live output** (`avoider.py`)
* Status every cycle, safe command, replan flag.
* `report()` gives confirmed obstacles (connected components: centroid, extent, confidence) and blocked-path events (time, position) for mapping and the GCS. The chat asked for this: "map not just the path, but also which paths are possible and what problems/obstacles exist".

## Processing Pipeline
lidar scan + stamp → L1 filtering + quality → L2 pose at capture time → odom points → L3 costmap update (hits, free-space clearing, decay) → candidate/confirmed cells → body-frame obstacles (closest per 2°) → L4a sub-goal (if blocked) → L4b DWA (window → trajectories → admissibility → score) → L5 status, failsafes, report → $(v, \omega)$ to the flight controller.

Depth frames enter at L1 and are registered into the same costmap.

## Inputs
* **Lidar:** angles, ranges (`inf` = no return), capture time.
* **Odometry:** smooth local pose $(x, y, \psi)$ at ≥ 20 Hz (FC/EKF). **Not** the SLAM pose: loop-closure jumps would misplace costmap cells.
* **Planner:** desired body velocity $(v_x, v_y)$ (pure pursuit) or an equivalent local goal.
* **Optional:** depth points (body frame, $z$ relative to the drone) + capture time; height above the floor.

## Outputs
* $v_x$ (forward), $v_y = 0$, yaw rate $\omega$, in the team body frame.
* **Status:** OK, SLOWED, AVOIDING, DEGRADED, BLOCKED, STUCK, EMERGENCY, STALE, IDLE.
* `replan` flag; minimum clearance; compute time.
* `report()`: confirmed obstacles + blocked-path events (for mapping and the GCS).

## Fallback and Supporting Methods
| Condition (team failsafe list) | Detection | Response |
|---|---|---|
| Sensor data unreliable | no scan for 0.35 s / spike quality < 0.8 | **STALE:** hover / **DEGRADED:** continue, flagged |
| Planned path blocked | no admissible *moving* arc | **BLOCKED:** stop, rotate to search, **replan**, event logged |
| Navigation fails | asked > 0.2 m/s, moving < 0.05 for 3 s | **STUCK:** **replan** |
| Too close (e.g. a person stepped in) | obstacle within $r + 5$ cm | **EMERGENCY:** back away at 0.15 m/s |
| GCS communication lost | n/a | avoidance runs fully onboard and is unaffected |

These are conditional fallbacks. None of them forces motion from unreliable data.

## Sensors and Hardware
* **2D front LiDAR:** primary. ≥ 10 Hz, ≥ 6 m. 180° FOV by default; 360° improves results (see Validation).
* **Depth camera:** 3D obstacles (low boxes, hanging beams) in the drone's height band. Optional: Algo 1 runs lidar-only if it's absent (the status reports `depth_used`).
* **Onboard computer:** Raspberry Pi 5 likely. About 4 ms per update on a desktop CPU in the unit tests, about 10-20 ms in the full benchmark. Measure on the Pi.
* **Flight controller:** accepts body-frame velocity + yaw-rate setpoints at ≥ 20 Hz.

## Software Stack
* Python 3.10+ and NumPy only (no solver libraries); PyYAML for the config.
* Proposed ROS 2 integration: a node wrapping `DWAAvoider.update()`, with tf2 for the odometry ↔ body transforms (not yet built).
* Standard `logging` (logger `nidar.oa_dwa`), to be swapped for the team's `cv.logger` at integration.

## Interfaces (proposed; to be agreed and placed in `interfaces/schemas/`)
| Direction | Proposed topic | Content |
|---|---|---|
| in | `/scan` | LaserScan (angles, ranges, stamp) |
| in | `/odom` | smooth local odometry pose (not `/slam_pose`) |
| in | `/cmd_vel_des` | planner's desired body velocity |
| in | `/depth/points` | depth points, body frame (optional) |
| out | `/cmd_vel_safe` | $v_x$, $\omega$ to the flight controller |
| out | `/oa/status` | status, min clearance, replan flag |
| out | `/oa/report` | confirmed obstacles + blocked events (1 Hz) |

## Coordinate Frames
* **Odometry/map:** ENU, origin at the maze entry (team `conventions.md`).
* **Body:** FLU internally; FRD via `body_frame: FRD` (team decision pending).
* **Lidar:** assumed at the body origin, angles CCW positive in FLU. Mounting offsets would go into Layer 2.
* **Timestamps:** seconds internally; reports convert to UTC ISO 8601 at the logging layer.

## What Other Team Members Should Provide
* **Navigation (Mahima):** desired velocity / local goal at ≥ 10 Hz. **A* inflation ≥ $r_{inf}$ (0.40 m)**, otherwise the avoider fights every path. A handler for `replan`.
* **Mapping (Jainil):** smooth odometry separate from the SLAM pose. Optionally, consume `/oa/report` to annotate the map.
* **Electronics:** lidar model, FOV and rate; depth camera choice (or none); the onboard computer for timing.
* **Mechanical / flight control:** drone radius with guards; achievable braking $a$; command latency.

## Status
* **Design:** complete. **Implementation:** complete in Python (`oa_dwa/`), with 7 unit tests passing.
* **Validation:** simulation (AirMouse simulator) done; hardware pending.
* **ROS 2 node:** not built yet.

## Validation (done in simulation; see `results/final/` and `docs/COMPARATIVE_STUDY.md`)
Planner knows only the walls; the avoider handles every obstacle; 108 trials pooled over four scenarios.

| Scenario | Success | Collisions | Timeouts |
|---|---|---|---|
| clutter (360°) | 63% | 7% | 30% |
| 3D obstacles (depth camera) | 37% | 10% | 53% |
| narrow doors | 50% | 6% | 44% |
| front 180° lidar | 63% | 10% | 27% |
| **pooled** | **54%** | **8%** (vs 24% existing reactive layer) | 38% |

**Limitations found:**
* No moving-obstacle model.
* Conservative in clutter and narrow doors (timeouts).
* The unicycle model can't sidestep.

## Planned Validation (hardware)
1. Bench test with props off: real lidar scans, check statuses and commands.
2. Measure the update time on the Pi 5 (`python tests/test_oa_dwa.py`).
3. Tethered hover with a static obstacle placed ahead.
4. Corridor flight with boxes; then a person walking across.
5. Measure $a_{lin}$ and latency in flight, then update `configs/oa_dwa.yaml`.
