# Obstacle Avoidance: Comparative Study (Algo 1 vs Algo 2)

**NIDAR AirMouse 2026-27 · Automation · Abhyuday Saksena · data: `results/final/` (raw per-trial JSON)**

## 1. The two algorithms at a glance
| | **Algo 1: Layered Costmap + DWA** (`oa_dwa/`) | **Algo 2: VFH+ + CBF safety filter** (`oa/`) |
|---|---|---|
| Design brief | Strictly follows Jainil's README structure and the team chat (front lidar, depth camera, ROS 2/Nav2 style, Pi 5) | What I consider best for this mission |
| Core method | **Dynamic Window Approach** (Fox et al. 1997) on a rolling log-odds costmap | **VFH+** direction selection (Ulrich & Borenstein 1998) + **Control Barrier Function** QP filter (Ames et al. 2019) |
| What it decides | Forward speed $v$ + yaw rate $\omega$ (unicycle: faces where it flies) | Holonomic body velocity $(v_x, v_y)$ (can sidestep); yaw is left to the planner |
| Obstacle representation | Rolling 8 × 8 m grid, 0.1 m cells, log-odds, **candidate → confirmed** | Raw scan points + 5 s point memory + tracked moving discs |
| Getting around things | Local A* sub-goal on the costmap, then DWA trajectory scoring | VFH+ free-sector search, target-corridor rule, side commitment |
| Safety guarantee | Admissibility check (no arc within $r_{inf}$; $v \le \sqrt{2ad}$) | **Formal:** $h_i = \|p_i\| - r_s \ge 0$ forward-invariant (CBF) + braking limit |
| Moving obstacles | ✗ Not modelled (treated as static cells that decay) | ✓ α-β tracking, predicted positions, CBF uses obstacle velocity |
| Above/below the lidar plane | **Depth camera** points in the height band → costmap | **Forward ToF beams** (±10°) + upward ToF → altitude band |
| Front-only lidar support | Native (default 180°): yaw-to-motion model | Memory + "blind-sector" speed caps + yaw hint |
| Time alignment | Pose buffer, scan placed with the pose **at capture time** | Motion compensation to the current pose |
| Frames | FLU, plus an FRD switch (mirror) | FLU |
| Config | **YAML** (`configs/oa_dwa.yaml`, team convention) | Python dataclass |
| Report to mapping/GCS | ✓ `report()`: confirmed obstacles + blocked-path events | Status + min clearance only |
| Failsafes | STALE, DEGRADED, BLOCKED, STUCK, EMERGENCY | STALE, BLOCKED, STUCK, EMERGENCY, SQUEEZE (narrow-passage mode) |
| Unit tests | 7 (time alignment, FRD mirror, candidate/confirmed, spikes, pillar, blocked/stale, timing) | 7 (QP vs brute force, CBF never breaches, VFH gap, tracker, failsafes, timing) |

## 2. Techniques used, by algorithm and pipeline stage
| Stage | Algo 1 | Algo 2 |
|---|---|---|
| Measurement filtering | Range/FOV limits; isolated-spike rejection (disagrees with both neighbours, which agree); scan quality score | Range/FOV limits; closest-point-per-1° reduction |
| 3D sensing | Depth points within $\pm(h_h + 0.30)$ m of the drone, floor rejected, voxel 0.1 m | Two pitched ToF beams; a hit counts if inside the height band and **not** seen by the lidar (not a wall) |
| Pose handling | 2 s pose buffer, linear interpolation (yaw on the circle) at the capture time | Re-express the latest scan in the current pose |
| Obstacle memory | Log-odds grid: hit +0.85, free-space −0.6, decay 0.5/s; confirmed $L \ge 1.2$ | Points in the odometry frame for 5 s (blind sectors) |
| Moving obstacles | - | Segmentation → greedy NN association → α-β filter ($\alpha = 0.5$, $\beta = 0.2$); moving if speed > 0.15 m/s **and** travelled > 0.5 m |
| Direction / plan | Local A* sub-goal when the goal segment is blocked; DWA window 7 × 11 + stop/straight; two-segment trajectories (turn 1 s, then straight), 3 s horizon | Polar histogram (max-magnitude, physical thresholds 1.22/1.84 m), hysteresis, cost $5\Delta_t + 1\Delta_h + 2\Delta_p$, 2 s side commitment |
| Speed | Admissibility $v \le \sqrt{2 a d_{coll}}$; velocity reward | Corridor clearance $v \le \sqrt{2 a (L - m)}$ |
| Final safety | Admissible set only | Exact 2D QP: $\min \|u - u_{nom}\|^2$ s.t. $n_i^\top u \le \kappa(h_i) + n_i^\top v_i$ |
| Scoring / cost | $G = 1.0\cdot(0.7\,\text{progress} + 0.3\,\text{heading}) + 0.3\,v/v_{max} - 0.6\,\text{obstacle} - 0.15\,\text{change}$, with obstacle $= e^{-(d - r_{inf})/0.2}$ | (direction cost above) |
| Stuck handling | Head-on rotate rule; BLOCKED → rotate-search + replan | Narrow-passage squeeze (margin 15 → 8 cm); EMERGENCY escape at closing speed via the CBF |

## 3. Test method
* **Simulator:** AirMouse simulator (`../airmouse_sim`): GPS-denied maze, noisy lidar with dropouts, drifting odometry, gusts, command latency, velocity-PID dynamics.
* **Role split:** a global planner (A* + pure pursuit at 0.8 m/s) knows **only the walls**. Every box, pillar, hanging beam and moving person (0.4-0.5 m/s) must be handled by the avoider. The planner replans when the avoider requests it or when there's no progress for 4 s (the same for all methods).
* **Hardware per algorithm:** Algo 1 gets the lidar + a simulated forward depth camera (16 × 12 rays, 70° × 55°, 4 m). Algo 2 gets the lidar + forward/up ToF. Each runs on the hardware its design assumes.
* **Reference:** `brake_repulse`, the simulator's existing reactive layer (brake + repulsion bubble).
* **Trials:** random start/goal 6-14 m apart. 30 per method per scenario (18 for narrow, where fewer valid start/goal pairs exist), **108 per method in total**, on the same seeds for every method. Brackets are **95% Wilson confidence intervals**.

## 4. Results

### 4.1 Per scenario
| Scenario | Method | Success | Collisions | Timeouts |
|---|---|---|---|---|
| **clutter** (16 obstacles, 3 people, 360°) | brake_repulse | 60% [42-75] | 23% [12-41] | 17% |
| | Algo 1 | 63% [46-78] | 7% [2-21] | 30% |
| | **Algo 2** | **83% [66-93]** | **3% [1-17]** | 13% |
| **3d** (low boxes + hanging beams) | brake_repulse | 57% [39-73] | 33% [19-51] | 10% |
| | Algo 1 | 37% [22-54] | 10% [3-26] | 53% |
| | **Algo 2** | **77% [59-88]** | **7% [2-21]** | 17% |
| **narrow** (0.9-1.1 m doors) | brake_repulse | 78% [55-91] | 17% [6-39] | 6% |
| | Algo 1 | 50% [29-71] | 6% [1-26] | 44% |
| | **Algo 2** | **83% [61-94]** | **6% [1-26]** | 11% |
| **front 180° lidar** | brake_repulse | 63% [46-78] | 20% [10-37] | 17% |
| | Algo 1 | 63% [46-78] | **10% [3-26]** | 27% |
| | **Algo 2** | **70% [52-83]** | 13% [5-30] | 17% |

### 4.2 Pooled over all 108 trials per method
| Metric | brake_repulse | **Algo 1** (DWA) | **Algo 2** (VFH+ + CBF) |
|---|---|---|---|
| **Success** | 63% [54-71] | 54% [44-63] | **78% [69-85]** |
| **Collisions** | 24% [17-33] | **8% [4-15]** | **7% [4-14]** |
| …of which moving obstacles | 5 | 8 | **4** |
| Timeouts (too cautious / stuck) | 13% | **38%** | 15% |
| Mean speed when successful | 0.57 m/s | 0.43 m/s | **0.61 m/s** |
| Median minimum clearance | 0.10 m | 0.20 m | **0.25 m** |
| Compute per update (desktop, 8 parallel jobs) | ~0 ms | 18 ms | **11 ms** |

## 5. Analysis
1. **Safety: both algorithms cut collisions by about 3×** against the existing reactive layer (24% → 7-8%). Their confidence intervals overlap, so **they are equally safe** at this sample size.
2. **Mission effectiveness: Algo 2 is clearly better.** Pooled success is 78% vs 54%; the intervals [69-85] and [44-63] don't overlap, so the difference is significant. Algo 1 fails mostly by **timing out** (38%): it's cautious (0.43 m/s), and in narrow and 3D scenes the DWA admissible set often shrinks to "stop or spin".
3. **Why Algo 1 is slower:**
   * DWA only commands forward speed + yaw, so it must turn its whole body before sidestepping. Algo 2 can sidestep directly (a multirotor is holonomic).
   * Its admissibility rule (no arc within $r_{inf}$ for the full 3 s horizon) is conservative in clutter.
   * Its costmap keeps "ghost" cells of moved obstacles until they decay or are cleared by a beam.
4. **Moving obstacles:** Algo 1 has no motion model (8 moving-obstacle hits). Algo 2's tracker + velocity-aware CBF gives the fewest (4). With a 180° front lidar Algo 2's advantage shrinks (13% vs 10% collisions, intervals overlap), because people approaching from the blind side can't be tracked. That's a hardware limit, and a reason to prefer a 360° lidar.
5. **3D obstacles:** Algo 1's depth camera sees the whole forward volume and gives few collisions (10%), but its timeouts are high (53%), because the depth points densify the costmap and DWA gets stuck. Algo 2's two cheap ToF beams reach a similar collision rate (7%) at 77% success. The depth camera (₹20K) **isn't needed** for good avoidance.
6. **Narrow doors:** Algo 2 (squeeze mode) is at least as good as the reference. Algo 1 often refuses 0.9 m doors: with $r_{inf} = 0.40$ m only a 10 cm-wide line through the door is admissible.
7. **Compute:** both are well under a 10 Hz lidar period on a desktop. Algo 1 costs more (trajectories × obstacles distance checks + local A*). Measure both on the Pi 5.

## 6. Where each one wins
| Algo 1 is better when… | Algo 2 is better when… |
|---|---|
| The team commits to the ROS 2 / Nav2 architecture (it maps 1:1 onto costmap + DWB) | Mission performance matters: success, speed, moving people |
| A depth camera is on board and dense 3D coverage is wanted | The hardware budget is tight (lidar + 3 ToF sensors) |
| Mapping/GCS want an obstacle and blocked-path **report** | A **formal** clearance guarantee is wanted for the Design Review |
| The FC can't do lateral velocity well (yaw-to-motion only) | The FC accepts body-frame $v_x, v_y$ (normal for multirotors) |
| Explainability: every cell has a confidence and a candidate/confirmed status | Narrow doors and clutter |

## 7. Recommendation
* **Fly Algo 2** as the primary avoider. It has the best success rate (78%), the joint-best collision rate (7%), the highest speed and clearance, and the lowest compute.
* **Keep Algo 1 as the documented baseline.** It satisfies the team's "baseline first" rule and the README/ROS 2 structure. Its `report()` (confirmed obstacles + blocked paths) is valuable on its own: **feed Algo 1's costmap report to mapping/GCS even while Algo 2 flies.**
* **Best combined system (next step):** Algo 1's Layers 1-3 (time-aligned, candidate/confirmed costmap) as the shared obstacle model, plus Algo 2's VFH+ + CBF as the command layer. That gives one obstacle map for mapping, tagging and avoidance, and the stronger controller.
* **Hardware:** a 360° lidar > 180° (fewer blind-side hits for both); multi-zone ToF > single ToF; the depth camera is optional for avoidance.

## 8. Threats to validity (be honest in the review)
* Simulation only. The people in the simulator don't dodge, and the sensor models are simplified.
* 18-30 trials per cell. Differences under ~15 points within one scenario are not significant (see the intervals).
* Algo 1's weights were tuned less than Algo 2's (about 1 day vs about 3 days of iteration), which may understate it.
* Each algorithm was tested with its *own* 3D sensor (depth camera vs ToF), so the 3D comparison mixes algorithm and sensor effects.
* Compute was measured on a desktop with 8 parallel jobs. The Pi 5 numbers still need measuring.
