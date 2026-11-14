# Obstacle Avoidance, Algo 2 (VFH+ + CBF): NIDAR AirMouse 2026-27

**Owner:** Abhyuday Saksena · **Status:** v1 baseline, tested in simulation · **Deadline:** 23 Oct 2026

Local obstacle avoidance for an indoor, GPS-denied multirotor. It takes the velocity the navigation stack *wants* (A* + pure pursuit) and returns the closest velocity that is **safe**, with a mathematical clearance guarantee and failsafes. Pure Python + NumPy, no solver libraries, about 2-10 ms per update on a desktop CPU (360° lidar; about 23 ms with a front-only lidar).

```
 SLAM / A* / pure pursuit (Mahima)          lidar scan, odometry, ToF
            │ u_des = (vx, vy) body frame          │
            ▼                                      ▼
   ┌───────────────────────── ObstacleAvoider ─────────────────────────┐
   │ scan → points → moving-obstacle tracker → obstacle memory          │
   │ → EMERGENCY check → VFH+ direction → braking-distance speed        │
   │ → CBF safety filter (hard guarantee) → FOV guard → failsafes       │
   │ → vertical guard (low boxes / hanging beams)                       │
   └───────────────┬───────────────────────────────┬────────────────────┘
                   │ u_safe, z_target, yaw hint     │ status, replan request
                   ▼                                ▼
           flight controller                 planner / GCS / mapping (Jainil)
```

---

## 1. Quick start
```bash
pip install numpy               # pygame only for the simulator benchmark
python tests/test_oa.py         # 7 unit tests incl. timing
python examples/basic_usage.py  # avoids a pillar, prints status each step
```
```python
from oa import ObstacleAvoider, AvoidanceConfig

avoider = ObstacleAvoider(AvoidanceConfig(drone_radius=0.25, v_max=1.0, lidar_fov_deg=360))

# every control cycle (e.g. 20 Hz):
out = avoider.update(
    t=now_s,                         # seconds
    pose=(x, y, yaw),                # SMOOTH odometry frame (FC / EKF), NOT the SLAM pose
    u_des=(vx, vy),                  # desired BODY-frame velocity from the planner
    scan=(angles, ranges) or None,   # only when a NEW scan arrived; np.inf = no return
    tof={"fwd_down": d1, "fwd_up": d2, "up": d3}, z=height, cruise_alt=1.0)   # optional
send_to_fc(out.vx, out.vy, out.z_target, out.yaw_rate)   # yaw_rate only set for front-only lidars
if out.replan: planner.replan()      # BLOCKED / STUCK / EMERGENCY
gcs.log(out.status, out.min_clearance)
```

> **Integration rule:** `pose` must be a frame that **never jumps** (raw odometry). Remembered obstacles are stored in it. Feeding a scan-matched SLAM pose that jumps by a few cm made the avoider see walls that weren't there (found and fixed in simulation, §7).

---

## 2. The algorithm (mathematical specification)

**Notation.** Body frame: x forward, y left. The drone is at the origin. Obstacle points $p_i \in \mathbb{R}^2$, distance $d_i = \|p_i\|$, bearing $\beta_i = \operatorname{atan2}(p_{i,y}, p_{i,x})$. Drone radius $r$, margin $m$, **safety radius $r_s = r + m$**. Commanded velocity $u \in \mathbb{R}^2$.

### 2.1 Pre-processing (`scan.py`)
1. Keep returns with $r_{min} \le \rho_k < r_{max}$ inside the lidar FOV: $|\theta_k| \le \tfrac12\text{FOV}$. Then $p_k = \rho_k(\cos\theta_k, \sin\theta_k)$.
2. **Motion compensation.** The latest scan was taken at pose $(x_0, y_0, \psi_0)$ and the drone is now at $(x_1, y_1, \psi_1)$, so every point is re-expressed: $p' = R(\psi_1)^\top\big(R(\psi_0)p + [x_0, y_0]^\top - [x_1, y_1]^\top\big)$.
3. **Obstacle memory.** Points are stored in the odometry frame (voxel 8 cm) for $T_{mem} = 5$ s within 3 m. Remembered points that fall **outside the current FOV** are added back. This is how a *front* lidar still avoids things beside and behind it.
4. **Reduction.** Keep the closest point per 1° of bearing. That's all avoidance needs, and it bounds the cost on a Raspberry Pi.

### 2.2 Moving-obstacle tracking (`tracker.py`)
* **Segmentation:** angle-ordered points split where $\|p_{k+1} - p_k\| > 0.3$ m.
* Only **compact** segments (extent ≤ 0.9 m, at least 3 points) are tracked: people and debris. Walls are excluded, because their visible centroid "moves" as more of the wall comes into view.
* **Association:** greedy nearest neighbour, gate 0.6 m, in the odometry frame.
* **α-β filter** for each track, with prediction $\hat c = c + v\,\Delta t$ and residual $e = z - \hat c$:
  $c \leftarrow \hat c + \alpha e$, $v \leftarrow v + \tfrac{\beta}{\Delta t} e$, using $\alpha = 0.5$, $\beta = 0.2$.
* A track counts as **moving** if it has at least 5 hits, $\|v\| > 0.15$ m/s **and** it has actually travelled at least 0.5 m since first seen (wall ends and door jambs show apparent motion as the viewpoint changes but never travel). Its points are removed from the static set and handled as a moving disc $(c, v, \rho)$.

### 2.3 Direction selection: VFH+ (`vfh.py`)
Polar histogram over $n = 72$ sectors ($5°$ each). Inside the window $W = 3$ m, each point has magnitude and enlargement angle

$$m_i = \left(1 - \frac{d_i}{W}\right)^2,\qquad \gamma_i = \arcsin\!\left(\min\!\left(1, \frac{r_s}{d_i}\right)\right)$$

(enlarging by $\gamma_i$ lets the drone be treated as a point). Sector value:

$$H_k = \max_i \{\, m_i : |\angle(\theta_k, \beta_i)| \le \gamma_i + \tfrac{\alpha_s}{2} \,\}$$

Taking the **max** (rather than the classic sum) makes the thresholds physical distances: $H_k > \tau \iff d < W(1 - \sqrt{\tau})$. With $\tau_{high} = 0.35$ a sector is blocked under 1.22 m, and with $\tau_{low} = 0.15$ it's free beyond 1.84 m. In between, the previous value is kept (hysteresis, no flicker).

**Target first.** With target bearing $\theta_t = \operatorname{atan2}(u_{des})$, the target is accepted directly if its corridor of half-width $r_s$ is free for the stopping distance:

$$L(\theta_t) \ \ge\ \frac{v^2}{2a_b} + v\,\tau_{lat} + m + 0.6$$

where $L(\theta)$ is the free length of that corridor (`corridor_clearance`). This stops far walls from causing needless detours, which was the dominant failure of plain VFH in testing.

**Otherwise** pick a free sector minimising
$$g(c) = \mu_t\,|\Delta(c, \theta_t)| + \mu_h\,|\Delta(c, 0)| + \mu_p\,|\Delta(c, c_{prev})|,\qquad (\mu_t, \mu_h, \mu_p) = (5, 1, 2)$$
subject to $|\Delta(c, \theta_t)| \le 135°$. If no sector qualifies, the status is **BLOCKED** and a replan is requested. **Side commitment:** once it detours left or right, it keeps that side for 2 s, which prevents the left/right oscillation seen in front of narrow gaps.

Moving obstacles also block the directions of their **predicted** positions $c + v\,k\,T_p/3$, $k = 1, 2, 3$, with $T_p = 1$ s.

### 2.4 Speed: braking distance
Along the chosen direction, $v = \min\big(\|u_{des}\|,\ v_{max},\ \sqrt{2 a_b \max(0, L - m)}\big)$, multiplied by $\max\big(0.4, \cos(\tfrac12|\Delta|)\big)$ on large detours.

### 2.5 Safety filter: Control Barrier Function (`cbf.py`)
For every obstacle $i$ (static points with $v_i = 0$ and $\rho_i = 0$, plus moving discs) define the barrier

$$h_i = \|p_i\| - (r_s + \rho_i) \qquad (h_i \ge 0 \Leftrightarrow \text{safe}).$$

Since $\dot p_i = v_i - u$, we get $\dot h_i = n_i^\top(v_i - u)$ with $n_i = p_i / \|p_i\|$. Requiring $\dot h_i \ge -\kappa(h_i)$ gives a **linear** constraint on $u$:

$$\boxed{\,n_i^\top u \ \le\ \kappa(h_i) + n_i^\top v_i\,}\qquad
\kappa(h) = \begin{cases}\min\!\big(\alpha h,\ \sqrt{2 a_b h}\big) & h \ge 0\\ \max\!\big(\alpha_{in} h,\ -v_{push}\big) & h < 0\end{cases}$$

with $\alpha = 2$, $\alpha_{in} = 1$ and $v_{push} = 0.4$ m/s (a gentle, capped push-out, so it doesn't overshoot into the opposite wall).

* $\kappa$ is a class-$\mathcal K$ function, so by Nagumo's theorem and standard CBF results, $h_i \ge 0$ is **forward invariant** (the drone never gets closer than $r_s$) for the velocity-controlled model, as long as the QP is feasible.
* The $\sqrt{2 a_b h}$ branch means the approach speed towards anything never exceeds the speed from which the vehicle can still brake to a stop within the remaining gap $h$.
* For $h < 0$ (already too close, e.g. a moving obstacle came at us), the right-hand side is negative, which **forces** motion away.
* **Latency:** $r_s$ is inflated by $\|u_{prev}\|\,\tau_{lat}$ (distance travelled before a new command acts).
* **Front lidar:** for every blind direction $e_\phi$ ($|\phi| > \tfrac12$FOV, every 15°), add $e_\phi^\top u \le v_{blind} = 0.15$ m/s. The drone may only creep into space it cannot see.
* **Speed limit:** a 16-gon, $e_k^\top u \le v_{max}\cos(\pi/16)$.

**QP:** $u^* = \arg\min_u \|u - u_{nom}\|^2$ subject to $Au \le b$. In 2D it is solved **exactly** without a solver: if $u_{nom}$ is feasible it's the answer; otherwise $u^*$ is the closest feasible point among the projections onto each line, $u_{nom} - (a_i^\top u_{nom} - b_i)a_i$, and the vertices of every pair of lines. That's $O(m^2)$ with $m \lesssim 100$, and it's checked against brute-force grid search in the tests. If the constraints are infeasible (boxed in), the farthest obstacles are relaxed first.

### 2.6 Vertical guard: obstacles the 2D lidar can't see (`vertical.py`)
Two single-point ToF sensors pitched $\pm\theta$ ($10°$) give a hit at height $z_{hit} = z \pm d\sin\theta$, horizontal distance $d\cos\theta$. If $|z_{hit} - z| < h_h + c$ and the lidar does **not** see anything at that distance (otherwise it's just a wall), the point is stored as a low box or a hanging beam. Near remembered points the altitude target is held inside

$$z^* \in \big[\max(z_{low}) + h_h + c,\ \min(z_{hang}) - h_h - c\big]$$

and forward speed is cut to 25% until the climb or duck is done. The upward ToF caps $z^*$ under the ceiling.

### 2.7 Failsafes and status (what the GCS and planner see)
| Status | Condition | Action |
|---|---|---|
| `OK` | target direction free | pass $u_{des}$ through (CBF-checked) |
| `SLOWED` | braking limit or CBF active | reduced speed, same direction |
| `AVOIDING` | VFH+ detour | detour direction |
| `+SQUEEZE` | slow progress for 1.5 s | **narrow-passage mode:** margin shrinks 15 → 8 cm at 0.3 m/s |
| `BLOCKED` | no free direction within ±135° | hover (CBF still pushes away from approaching objects), **replan** |
| `STUCK` | wanted > 0.2 m/s, got < 0.05 for 3 s | **replan** |
| `EMERGENCY` | obstacle within $r + 5$ cm | escape at max(0.3, closing speed + 0.3) through the CBF, **replan** |
| `STALE` | no scan for 0.35 s | **hover** (zero horizontal velocity) |
| `IDLE` | $\|u_{des}\| < 0.05$ | hold |

This covers the team's failsafe list: unreliable sensor (STALE), blocked path (BLOCKED/STUCK → replan), navigation failure (EMERGENCY).

---

## 3. Parameters (`oa/config.py`), the ones to tune on hardware
| Parameter | Default | Meaning / how to set |
|---|---|---|
| `drone_radius` | 0.25 m | **Measure** with prop guards (Navyansh) |
| `margin` | 0.15 m | Extra clearance. Lower = faster in clutter, riskier |
| `a_brake` | 1.5 m/s² | Deceleration the FC really achieves. **Measure in flight tests (Vaibhav)** |
| `latency_s` | 0.10 s | Lidar period + compute + FC response |
| `v_max` | 1.0 m/s | Indoor cruise limit |
| `lidar_fov_deg` | 360 | **Set to your lidar** (e.g. 180/270 for a front lidar) |
| `cbf_alpha` | 2.0 1/s | Higher = brakes later (more aggressive) |
| `tau_low/high` | 0.15/0.35 | Free beyond 1.84 m / blocked under 1.22 m |
| `memory_s` | 5 s | Longer helps front lidars; limited by odometry drift |
| `blind_speed` | 0.15 m/s | Max speed into unseen directions |

---

## 4. Hardware requirements (for the electronics team)
| Item | Requirement | Why (from the benchmarks below) |
|---|---|---|
| **2D lidar** | ≥ 10 Hz, ≥ 6 m range, **360° strongly preferred** | Front-only 180° works but collisions rise 3% → 10%, and moving obstacles from behind can't be seen |
| **Forward ToF ×2** (±10° pitch) + **upward ToF** | ≥ 3-4 m single-point | Low boxes and hanging beams invisible to the lidar |
| **Better: multi-zone ToF / extra lidar layers** | e.g. an 8×8 multizone ToF (ST VL53L5CX class, check datasheet) | Simulated 3-layer sensing cut 3D collisions **10% → 3.3%**, the biggest hardware win found |
| **Compute** | Raspberry Pi 4/5 or Jetson | **9 ms** per update on desktop (360°), 23 ms front-lidar; a Pi is a few times slower. Measure with `python tests/test_oa.py` on the Pi. Must stay under the lidar period (100 ms) |
| **FC interface** | body-frame velocity setpoints + altitude setpoint at ≥ 20 Hz | Same as PX4 Offboard / our own FC |
| **Odometry** | smooth local pose (flow + IMU EKF) at ≥ 20 Hz | Motion compensation and obstacle memory |

An RGB-D camera (₹20K) is **not required** for avoidance.

---

## 5. Results (AirMouse simulator, `bench/bench_avoidance.py`)
**Test setup.** A global planner (A* + pure pursuit) knows **only the walls**. The avoider must handle every box, pillar, hanging beam and moving person (walking at 0.4-0.5 m/s) using only lidar, ToF and odometry. Each trial is a random start/goal 6-14 m apart at 0.8 m/s, with replanning on request. **Success** = goal reached. 30 trials per row; small differences of about ±8% are within noise.

**Scenario `clutter`** (16 boxes/pillars, 3 moving people, 360° lidar), with layer ablation:

| Method | Success | Collisions | Timeouts | Min clearance | ms/update |
|---|---|---|---|---|---|
| no avoidance | 27% | 73% | 0% | 0.05 m | - |
| current sim baseline (brake + repulsion) | 60% | 23% | 17% | 0.10 m | - |
| CBF filter only | 37% | 7% | 57% | 0.15 m | 9.0 |
| VFH+ only | 83% | 3% | 13% | 0.20 m | 5.4 |
| **full (this repo)** | **83%** | **3%** | 13% | **0.25 m** | 9.8 |

**Other scenarios** (current baseline vs **full**; measured before the last three fixes in §7. Re-run the commands in §8 to refresh):

| Scenario | Baseline success / collisions | **Full success / collisions** |
|---|---|---|
| `3d`: low boxes + hanging beams | 57% / 33% | **80% / 10%** |
| `3d` + 3-layer sensing (hardware upgrade) | - | 73% / **3.3%** |
| `narrow`: 0.9-1.1 m doors | 78% / 17% | 78% / 17% (no gain yet, see §6) |
| `clutter`, **front 180° lidar** | 63% / 20% | 67% / **10%** (all 3 hits: people from behind) |

**Full mission** (exploration + survivors + exit, 16 maps, simulator baseline explorer vs explorer + this avoider via `sim_adapter/explorer_oa.py`):

| | Survivors | Crashes | Exit + land | Coverage |
|---|---|---|---|---|
| explorer (brake + repulsion) | 75% | 12.5% | 87.5% | 98% |
| explorer + this avoider, first try (r_safe 0.40 > planner clearance 0.30) | 54% | 12.5% | 62.5% | 78% |
| explorer + this avoider, **radius matched** (r_safe 0.33) | 72% | 12.5% | 75% | 99% |

**Honest reading:** at mission level the avoider is currently **at parity, not better**. The explorer's remaining crashes come from its own behaviour, and the extra caution costs time (2 missions hit the time limit). The isolated tests above, where avoidance is the only variable, show the real effect. Mission-level gains need the avoider tuned **together with Mahima's planner** (§6).

**What the ablation shows:** VFH+ provides the navigation (getting around things). The CBF provides the guarantee (largest minimum clearance, no static collisions in clutter). On its own the CBF is too conservative: it stops in front of obstacles instead of going around (57% timeouts). The two together are the design.

---

## 6. Limitations and next steps
0. **Integration rule (most important): planner clearance ≥ avoider safety radius `r_safe`.** If the planner routes closer to walls than `r_safe`, the avoider fights every path, flies slowly and runs out of time (measured: coverage 98% → 78%). Agree on one number with Mahima: e.g. A* inflation = `drone_radius + margin`.
1. **Narrow doors (0.9 m)** are no better than the baseline yet. Next: tune squeeze mode, add a dedicated "door alignment" behaviour that centres on the gap before entering.
2. **Moving obstacles from blind sectors** (front lidar) can't be avoided reactively, so a 360° lidar is recommended. Simulated people also don't dodge, unlike real ones.
3. **Local minima** (U-shaped dead ends) are left to the global planner by design: BLOCKED/STUCK → `replan`.
4. Single-point ToF misses obstacles that aren't dead ahead, so prefer multi-zone ToF.
5. The CBF guarantee assumes the FC tracks velocity commands within `a_brake` and `latency_s`. **Measure both in flight tests and set them.**
6. To do before hardware: ROS2 node wrapper (topics `/scan`, `/odom`, `/cmd_vel_des` → `/cmd_vel_safe`, `/oa_status`); profile on the actual Pi; props-off bench test with a real lidar.

---

## 7. Development log (evidence for the Design Review / NIDAR Diary)
| Problem found in simulation | Fix |
|---|---|
| Plain VFH (summed histogram) detoured from far walls: 75% timeouts | Max-histogram (physical distance thresholds) + target-corridor rule → 25% → 75% success |
| Left/right oscillation in front of narrow gaps | 2 s side commitment + narrow-passage (squeeze) mode |
| Emergency escape slower than an approaching person | Escape at closing speed + 0.3 m/s, routed through the CBF |
| Front lidar: escape/sidestep into unseen space (33% collisions) | 5 s memory + blind-sector speed constraints → 10% |
| 37 ms per update with long memory | Closest-point-per-degree reduction → 9-23 ms |
| Mission integration: false EMERGENCY near walls | Avoider must use smooth odometry, not the jumping SLAM pose |
| Walls *alongside* the drone stopped it (braking rule) | Corridor width = min(r_safe, current side clearance) |
| Door jambs / wall ends tracked as "moving people" | A track must actually travel 0.5 m to count as moving |
| Push-out from inside the margin overshot into the opposite wall | Gentler, capped push-out (α = 1, ≤ 0.4 m/s) |
| Mission slower than baseline (avoider radius > planner clearance) | Integration rule: planner clearance ≥ r_safe |

---

## 8. Repository layout
```
oa/config.py       all parameters             oa/avoider.py   pipeline + failsafes
oa/scan.py         points, memory, frames     oa/vfh.py       VFH+ direction selection
oa/tracker.py      moving obstacles           oa/cbf.py       CBF + exact 2D QP
oa/vertical.py     ToF vertical guard
tests/test_oa.py   unit tests (QP vs brute force, clearance guarantee, VFH, tracker, failsafes, timing)
examples/basic_usage.py
bench/bench_avoidance.py   simulator benchmark (needs ../airmouse_sim)
sim_adapter/explorer_oa.py full AirMouse mission using this avoider
results/           benchmark outputs (summary.txt + per-trial JSON)
```
Reproduce everything:
```bash
python bench/bench_avoidance.py --scenario clutter --trials 30
python bench/bench_avoidance.py --scenario 3d --methods brake_repulse,full
python bench/bench_avoidance.py --scenario clutter --fov 180 --methods brake_repulse,full
python bench/bench_avoidance.py --scenario 3d --methods full --sim-set "sensors.lidar_layers_z_m=[-0.15,0.0,0.15]"
```

**References:** Ulrich & Borenstein, *VFH+: Reliable obstacle avoidance for fast mobile robots*, ICRA 1998 · Ames et al., *Control Barrier Functions: Theory and Applications*, ECC 2019 · Kalata, *The tracking index: α-β filters* (IEEE TAES 1984).
