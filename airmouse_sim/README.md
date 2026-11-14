# AirMouse Simulator (NIDAR 2026-27)

A fast 2.5D test bench for GPS-denied indoor autonomy. Every run generates a new AirMouse-style maze: one entry/exit point, corridors, rooms, turns, clutter, hanging beams, moving obstacles, dark rooms and up to 6 survivors. You plug in your algorithm, and the simulator scores the run the way the mission is judged. It also has the failures you'll meet in the real hall: sensor dropouts, lost radio link, a draining battery and wind.

## Install and run (Windows or Ubuntu)
```
pip install numpy pygame
python run.py                                          # watch the baseline
python run.py --algo algorithms/my_algo.py --seed 42   # your algorithm on one map
python run.py --profile hard --profile faults          # stack profiles
python run.py --set drone.vel_pid.kp=2.0               # override any config value
python run.py --headless --runs 50 --jobs 8 --label v2 # batch -> leaderboard.csv
python run.py --leaderboard                            # compare all versions
python run.py --seed 42 --report reports/              # JSON + event log + PNG for the NIDAR Diary
python run.py --seed 42 --export-gazebo maze42.sdf     # same map for Gazebo / PX4 SITL
```
Viewer keys: `SPACE` pause, `N` single step, `+`/`-` speed, `R` new map, `L` lidar rays, **`K` operator kill switch**, `S` screenshot, `ESC` quit.

## What is simulated

### World (2.5D)
| Feature | Details |
|---|---|
| Maze | DFS corridors and dead ends, merged rooms, loops, doors of random width and position, one entry/exit. The drone starts **on the ground, outside**. |
| Floor obstacles | Boxes and pillars with random **heights** (0.4-1.6 m). A 2D lidar at 1 m can't see the low ones. |
| Hanging obstacles | Beams, pipes and trays from the ceiling with a random bottom height (0.7-1.6 m). |
| Moving obstacles | Walking-person cylinders, 1.8 m tall, that bounce around the maze. |
| Survivors | 2-6 lying people, some tucked beside obstacles (partial occlusion). |
| Dark rooms | Detector recall drops unless `drone_has_light`; optical flow degrades. |
| Featureless floor | Optical flow fails there, so odometry coasts and drifts. |
| Placement rule | Obstacles never block the maze at cruise altitude. |

### Drone and flight control
Velocity setpoints (like PX4 Offboard) go through **command latency → per-axis velocity PID (kp/ki/kd, D on measurement, anti-windup) → tilt limit → motor lag → drag + wind gusts** (an Ornstein-Uhlenbeck process plus steady wind). There's altitude control, a ceiling, landing, and landing on top of boxes. Tune `drone.vel_pid` and `drone.vz_pid` the way you'd tune `MPC_XY_VEL_*` in PX4.

### Sensors (what the algorithm gets; no ground truth)
| Sensor | Model |
|---|---|
| Lidar | Rate-limited scans, max/min range, noise, random dropouts (`inf`), sees only obstacles crossing its plane. `lidar_layers_z_m` turns it into a multi-layer lidar. |
| ToF | `down`, `up`, and two forward beams pitched ±`tof_fwd_pitch_deg` to catch low boxes and hanging beams. |
| Barometer | Noisy and drifting. |
| Optical flow + gyro | Body velocity with noise, `flow_quality`, invalid (`None`) over featureless floor, in the dark or when too low. Gyro bias makes the integrated odometry drift. |
| Camera detector | Pinhole camera pitched down, rate-limited. Outputs **bounding boxes + confidence**, recall vs distance curve (`detector_recall_curve`: put Mahima's measured numbers here), occlusion, dark-room penalty, false positives. With `camera_type: depth`, detections also carry metric depth. |

### Mission systems
| System | Model |
|---|---|
| Battery | Energy (Wh) drained by hover + speed² + climb power. Running out ends the mission with a forced landing. |
| Radio link | GCS at the take-off point. Link lost when out of range or through more than `max_walls` walls, or during a scheduled outage. The GCS view freezes while the link is down. |
| Faults | Scheduled per seed: lidar blackout, camera blackout, flow loss, link outage, gyro bias jump, fast battery drain (see `profiles/faults.json`). |
| Kill switch | `K` in the viewer, an operator emergency stop. |
| Event log | State changes, takeoff/landing, faults, link loss and regain, survivor tags, battery levels, collisions. Saved with `--report`. |

### Scoring (competition-style; weights in `config.json → scoring`)
+100 per survivor (full points within 0.5 m, half at 1 m), −30 per false report, −50 per collision, −200 crash, +150 for **exiting through the entry and landing**, a time bonus, a coverage bonus, and −300 if the battery runs out. **These weights are placeholders: set them from the rulebook's Final Mission marking scheme.**

## Profiles (`--profile name`, stackable)
| Profile | What it tests |
|---|---|
| `easy` | No clutter, faults or wind. For debugging. |
| `hard` | Narrow identical centred doors (perceptual aliasing), dead ends, more clutter and hanging beams, 2 moving obstacles, survivors against obstacles, more dark rooms and featureless floor. |
| `faults` | One of each sensor/comms fault plus more lidar dropouts and detector false positives. |
| `low_battery` | Small battery plus a fast-drain fault, to test your return-home budgeting. |
| `windy` | Strong gusts plus more latency and motor lag, for PID tuning. |
| `low_cost_hw` | Slower, shorter-range, noisier sensors. |
| `depth_camera` | RGB-D detections, onboard light and a 3-layer lidar. Shows what better hardware buys you. |

## Writing an algorithm
Copy `algorithms/template.py`. The full observation and command spec is in `algorithms/base.py`.
- `step(obs)` returns `{"vx", "vy", "vz", "yaw_rate", "state", "done"}`.
- `get_survivors()` gives positions in the start frame. These are scored.
- Optional: `get_map()` and `get_pose()` (sent to the GCS, pose error is scored), `debug_text()`.
- Helpers: `altitude_hold()`, and `bbox_ground_range()` (mono camera bbox → distance on a flat floor).

### The baseline (`algorithms/explorer.py`) to beat
- State machine: PREFLIGHT → TAKEOFF → ENTER → EXPLORE/LOOK → RETURN → EXIT → LAND → DONE, with failsafes HOLD (lidar lost), return-home on battery/time budget, and EMERGENCY_LAND.
- Localisation: dead reckoning, heading snap to the maze axes, and **correlative scan-to-map matching** (Hector-SLAM style; there's no loop closure yet, that's a good next step).
- Exploration: utility-based (unknown area + floor the camera hasn't seen − path cost) with goal hysteresis, A* paths, and LOOK turns towards unseen floor.
- Survivors: bbox → bearing + ground-plane or depth range, clustering, **multi-view least-squares triangulation**, and a consistency + confidence filter.
- Safety: lidar braking and a normalised repulsion bubble, plus forward ToF memory of low/hanging obstacles (climb over or duck under).

Baseline results (`python run.py --leaderboard` after running the profiles):

| Profile | Maps | Survivors found | Crashes | Exit + land | Coverage |
|---|---|---|---|---|---|
| default | 16 | 75% | 12.5% | 87.5% | 98% |
| faults | 8 | 76% | 0% | 100% | 98% |
| depth_camera | 8 | 81-89% | 12-25% | 75-88% | 97-99% |
| low_battery | 8 | 49% | 0% | 88% | 91% |
| windy | 8 | 62% | 38% | 62% | 85% |
| hard | 8 | 15% | 38% | 25% | 36% |

Where it's weak, so beat it there: collisions with **moving obstacles** and **wall corners** (the avoidance is purely reactive), **wind** (default PID not tuned for gusts), and **hard maps** (narrow identical doors). Most missed survivors are in dark rooms seen with a mono camera.
Small changes can ripple through a whole mission, so always compare versions on **16+ maps** (`--runs 16 --jobs 8`).

## ASSUMED values: replace with the rulebook and your datasheets
Arena size (20 × 15 m), corridor/cell size (2.5 m), door widths, entry width, ceiling (2.5 m), mission time (480 s), match tolerance (1 m), scoring weights, drone size/speed/PID, battery Wh and power, link range, and every sensor spec. The simulator is only as real as these numbers.

## Gazebo
`--export-gazebo` writes walls, boxes, pillars and hanging beams with their real heights, plus survivor stand-ins, as an SDF world. The header has the PX4 spawn command. Moving obstacles aren't exported.
