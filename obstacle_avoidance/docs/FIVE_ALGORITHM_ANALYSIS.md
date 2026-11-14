# Obstacle Avoidance: Analysis of All 5 Algorithms

**NIDAR AirMouse 2026-27 · Automation · Abhyuday Saksena**
**Raw data:** `results/five/` (per-trial JSON). The 3D rows for D and L come from `3d_DL_rerun.json`, run after the final climb-limit fix.

---

## 1. The five algorithms and their baseline ideas

| # | Name | Sensors | Baseline method (reference) | Core idea in one line |
|---|---|---|---|---|
| **0** | **Brake + repulsion** (existing sim layer) | 2D lidar | Artificial potential field (Khatib 1986) + emergency brake | Obstacles push the drone away; brake if something is ahead |
| **1** | **Layered costmap + DWA** (`oa_dwa/`) | 2D lidar + depth camera | Dynamic Window Approach (Fox, Burgard, Thrun 1997), Nav2-style | Simulate every reachable (speed, turn) for 3 s and pick the best safe one |
| **2** | **VFH+ + CBF** (`oa/`) | 360° 2D lidar + 2 forward ToF beams + up ToF | VFH+ (Ulrich & Borenstein 1998) + Control Barrier Function (Ames et al. 2019) | Free direction closest to the goal, then a mathematical clearance guarantee |
| **D** | **Depth-camera VFH+ + CBF** (`oa/` + `oa/points3d.py`) | **Depth camera only** (D435 class, 87° × 58°, 0.3-6 m) | Depth → 2.5D columns → virtual scan (cf. ROS `depthimage_to_laserscan`) + the Algo 2 core | The camera sees 3D: avoid walls sideways, duck under beams, climb over boxes |
| **L** | **Lidar + multi-zone ToF VFH+ + CBF** (`oa/` + `oa/points3d.py`) | 360° 2D lidar + ring of **4 multi-zone ToF** (8×8 zones, 45° each, 3.5 m) | Same core, fusing the lidar with 3D ToF points | Full-circle horizontal view + 3D coverage around the drone |

### 1.1 How each baseline works
**0. Brake + repulsion**
* Forward speed is capped at $0.8\,(d_{front} - 0.4)$.
* Every point inside the bubble $r + 0.22$ adds a push away from it, scaled by $2.5 \cdot \max(\text{bubble} - d)$.
* No map, no prediction, no guarantee.

**1. DWA**
* **L1:** spike-filtered lidar + depth points in the height band.
* **L2:** each scan is placed with the pose at its capture time.
* **L3:** log-odds costmap (hit +0.85, free −0.6, decay 0.5/s, *candidate → confirmed*).
* **L4:**
  * local A* sub-goal when the goal is blocked;
  * dynamic window $v_c \pm a\Delta T$, $\omega_c \pm \alpha\Delta T$, sampled 7 × 11;
  * two-segment trajectories (turn 1 s, then straight), 3 s horizon;
  * reject any arc within $r_{inf}$;
  * score $G = (0.7\,\text{progress} + 0.3\,\text{heading}) + 0.3\,v - 0.6\,e^{-(d - r_{inf})/0.2} - 0.15\,\text{change}$.
* **L5:** failsafes + an obstacle / blocked-path report.

**2. VFH+ + CBF**
* Polar histogram (max magnitude $(1 - d/W)^2$, enlargement $\arcsin(r_s/d)$); blocked under 1.2 m, free beyond 1.8 m.
* Cost-minimising free sector, with side commitment.
* Braking speed $v \le \sqrt{2aL}$.
* CBF QP: $\min\|u - u_{nom}\|^2$ subject to $n_i^\top u \le \kappa(h_i) + n_i^\top v_i$, solved exactly in 2D.
* α-β tracker for moving people; 5 s memory; forward ToF beams for low and hanging obstacles.

**D and L. 3D front-end + the Algo 2 core**
* The 3D points are binned into 0.1 m vertical columns relative to the drone's band $B = \pm(h_h + c)$:
  * a column spanning $B$ → **wall** → VFH+/CBF;
  * free space observed beneath it (lowest point above the lowest height the sensor can see at that range) → **hanging** → duck under;
  * free space observed above, top ≤ 0.8 m climb → **low** → climb over.
* Altitude target: $z^* \in [\max z_{low}^{top} + h_h + c,\ \min z_{hang}^{bot} - h_h - c]$.
* Low or hanging columns stay CBF obstacles until the altitude actually clears them.
* **D:** wall columns become a virtual 360-beam scan feeding VFH+/CBF/tracker. The 87° FOV means yaw towards motion, 5 s memory, and only creeping (0.15 m/s) into directions behind.
* **L:** the real 360° lidar scan, plus ToF wall columns, plus the altitude band.

---

## 2. Test setup (identical for all five)
* **Simulator:** AirMouse simulator (`../airmouse_sim`). Noisy lidar with dropouts, drifting odometry, gusts, latency, velocity-PID dynamics.
* **Planner:** A* + pure pursuit at 0.8 m/s that knows **only the walls**. Each avoider must handle every box, pillar, hanging beam and walking person (0.4-0.5 m/s). The planner replans on request or after 4 s without progress.
* **Trials:** random start/goal 6-14 m apart, the same seeds for every algorithm. Scenarios:
  * **clutter:** 16 obstacles, 3 people;
  * **3d:** low boxes + 5 hanging beams, 2 people;
  * **narrow:** 0.9-1.1 m doors;
  * **front180:** the clutter scenario with a 180° front lidar (not run for D, whose FOV is fixed by its camera).
* **Statistics:** 95% Wilson confidence intervals in brackets.

---

## 3. Results

### 3.1 Per scenario: success / collisions / timeouts (%)
| Scenario (trials) | 0 Brake+repulse | 1 DWA | 2 VFH+CBF | D Depth | L Lidar+MZ-ToF |
|---|---|---|---|---|---|
| clutter (30) | 60 / 23 / 17 | 63 / 3 / 33 | **83 / 3 / 13** | 63 / 17 / 20 | 73 / 7 / 20 |
| 3d (30) | 57 / 33 / 10 | 37 / 17 / 47 | **77** / 7 / 17 | 53 / 13 / 33 | 73 / **0** / 27 |
| narrow (18) | 78 / 17 / 6 | 39 / 11 / 50 | **83** / 6 / 11 | 56 / 17 / 28 | **83 / 0** / 17 |
| front 180° (30) | 63 / 20 / 17 | 60 / 17 / 23 | **77 / 3** / 20 | - | 67 / 7 / 27 |

### 3.2 Pooled over the 3 scenarios all five ran (78 trials each)
| Metric | 0 Brake+repulse | 1 DWA | **2 VFH+CBF** | D Depth | **L Lidar+MZ-ToF** |
|---|---|---|---|---|---|
| **Success** | 63% [52-73] | 47% [37-58] | **81% [71-88]** | 58% [47-68] | 76% [65-84] |
| **Collisions** | 26% [17-36] | 10% [5-19] | 5% [2-12] | 15% [9-25] | **3% [1-9]** |
| …moving-obstacle hits | 3 | 7 | **0** | 8 | **0** |
| Timeouts | **12%** | 42% | 14% | 27% | 22% |
| Mean speed (successful runs) | 0.57 m/s | 0.43 | **0.61** | 0.55 | 0.49 |
| Median min clearance | 0.10 m | 0.20 | **0.25** | 0.20 | 0.23 |
| Compute per update (desktop, 8 parallel jobs) | ~0 ms | 24 ms | **10 ms** | 19 ms | 11 ms |

The 4-scenario pool (108 trials; D excluded) shows the same picture:
* Algo 2: 80% success [71-86], 4.6% collisions
* Algo L: 73% [64-81], 3.7%
* DWA: 51%, 12%
* Brake + repulsion: 63%, 24%

### 3.3 Ranking
| Rank | By safety (collisions) | By mission success | Overall |
|---|---|---|---|
| 1 | **L** 3% | **2** 81% | **2 ≈ L** (top tier) |
| 2 | **2** 5% | **L** 76% | |
| 3 | 1 DWA 10% | 0 63% | D |
| 4 | D 15% | D 58% | 1 DWA |
| 5 | 0 26% | 1 47% | 0 |

*Algo 2 and L are **statistically tied** on both success and collisions (their intervals overlap). Everything else is clearly below them.*

---

## 4. Analysis

**Why 2 and L lead**
* They share the VFH+ + CBF core: holonomic sidestepping, braking-distance speed, a formal clearance filter, and moving-obstacle prediction.
* They're the only two with **zero moving-obstacle hits** on the 3 common scenarios, because the 360° lidar sees people approaching from any side.

**L vs 2 (same core, different 3D sensing)**
* **L wins on 3D safety:** 0% collisions in 3d and narrow, against Algo 2's 7% and 6%. The multi-zone ToF ring sees the *whole* volume around the drone; Algo 2's two single-point beams can miss a beam or box that isn't dead ahead.
* **L is slower** (0.49 vs 0.61 m/s; timeouts 22% vs 14%). More obstacles are visible, and it climbs and ducks, both of which slow it down. This is a tuning trade-off, not a limit: it can be tuned faster (e.g. the 0.25× speed scale while changing altitude).

**D (depth camera as the main sensor)**
* It sees 3D well (3d: 53% / 13%), but its **87° field of view is the bottleneck**. Most of its collisions are **moving obstacles from the side or behind** (8 hits), which a forward camera can't see.
* It must constantly yaw towards its motion and creep into unseen directions, which explains the 27% timeouts.
* It works for static mazes, but a forward depth camera **alone** isn't sufficient for the safety level the rulebook asks for.

**1 (DWA)**
* Safe-ish (10% collisions), but the least effective (47%, 42% timeouts).
* The unicycle model can't sidestep. Its conservative admissibility check ("no arc within $r_{inf}$ for 3 s") refuses 0.9 m doors (narrow: 50% timeouts). It has no motion model.
* It's the correct ROS 2 / Nav2 *baseline*, and its costmap report is useful to mapping, but it isn't the best flyer.

**0 (brake + repulsion)**
* Fast and rarely stuck, but collides in a quarter of runs. It's the "before" picture for the Design Review: every proposed algorithm cuts collisions by 1.7-10×.

**Front-only (180°) lidar**
* Algo 2 handled it best (3% collisions); L 7%, DWA 17%, brake + repulsion 20%. Even so, a 360° lidar is preferable.

---

## 5. What each algorithm uses (technique matrix)
| Technique | 0 | 1 | 2 | D | L |
|---|---|---|---|---|---|
| Potential-field repulsion | ✓ | | | | |
| Log-odds costmap, candidate → confirmed | | ✓ | | | |
| Time-aligned pose buffer | | ✓ | | | |
| Local A* sub-goal | | ✓ | | | |
| DWA trajectory scoring | | ✓ | | | |
| VFH+ polar histogram + side commitment | | | ✓ | ✓ | ✓ |
| Braking-distance speed limit | ✓ (simple) | ✓ | ✓ | ✓ | ✓ |
| CBF safety filter (exact QP) | | | ✓ | ✓ | ✓ |
| Moving-obstacle α-β tracking + prediction | | | ✓ | ✓ | ✓ |
| Obstacle memory (5 s) | | ✓ (costmap) | ✓ | ✓ | ✓ |
| 2.5D column classification (wall / low / hanging) | | | | ✓ | ✓ |
| Climb-over / duck-under | | | ✓ (ToF beams) | ✓ | ✓ |
| Virtual scan from depth | | | | ✓ | |
| Narrow-passage squeeze mode | | | ✓ | ✓ | ✓ |
| Failsafes (stale / blocked / stuck / emergency) | | ✓ | ✓ | ✓ | ✓ |
| FRD/FLU switch, YAML config | | ✓ | | | |

---

## 6. Hardware view (approximate; verify prices and weights with the electronics team)
| | Sensors | Rough cost (₹) | Notes |
|---|---|---|---|
| 0 / 2 | 360° 2D lidar + 3 single-point ToF | lidar 8-25k + ToF ~0.5-1.5k each | Cheapest strong option |
| 1 | 2D lidar + depth camera | lidar + ~20k camera (team chat figure) | Most expensive; worst flyer |
| D | depth camera only | ~20k | No lidar; FOV-limited |
| **L** | 360° 2D lidar + 4 multi-zone ToF (VL53L5CX class) | lidar + ~1.5-3k per ToF | **Best safety per rupee**; tiny, low power |

---

## 7. Recommendation
1. **Fly Algo L** (360° lidar + 4 multi-zone ToF, VFH+ + CBF core) if the arena has low or hanging obstacles, or collisions are heavily penalised. It has the lowest collision rate (3%, 0 in 3D and narrow) and costs far less than a depth camera.
2. **Algo 2** is the equal alternative (best success, 81%) if the multi-zone ToF sensors aren't available. It needs only the lidar + 3 cheap ToF sensors.
3. **Algo D** is only worth it if the team is committed to the depth camera for survivor tagging anyway. Then **add the 360° lidar** and it effectively becomes L + camera.
4. **Keep Algo 1** as the documented baseline (team rule "baseline first") and as the **obstacle-report provider** for mapping/GCS.
5. Next tuning target for L: reduce timeouts (speed scale while changing altitude, squeeze timing) without losing its 0% 3D collisions.

## 8. Threats to validity
* Simulation only. The people don't dodge; sensor models are simplified (depth noise σ ≈ 1% · d², one ray per ToF zone).
* 18-30 trials per cell, so intra-scenario differences under about 15 points aren't significant.
* The algorithms received different amounts of tuning (Algo 2 the most).
* The 3D comparison mixes algorithm and sensor effects by design: each algorithm uses its own sensors.
* Compute times are desktop figures with 8 parallel jobs; measure on the Pi 5 / Jetson.
