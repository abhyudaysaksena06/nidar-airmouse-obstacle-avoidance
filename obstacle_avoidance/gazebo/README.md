# OA-Stack in Gazebo

The drone flies through the benchmark maze in Gazebo. Gazebo's own sensors produce the lidar and ToF data, and OA-Stack flies the drone from them in real time. A live evaluation runs in the terminal.

## What you need (Ubuntu 22.04/24.04)
- Gazebo Harmonic (`gz sim`): https://gazebosim.org/docs/harmonic/install_ubuntu
- `sudo apt install python3-numpy python3-gz-transport13 python3-gz-msgs10`
- This folder (`obstacle_avoidance/`) next to `airmouse_sim/`, as in the zip

## Run
```bash
bash gazebo/run_gazebo.sh                       # mixed_clutter, 5 worlds (seeds 207..211)
bash gazebo/run_gazebo.sh guide_people 213 3    # scenario, first seed, number of worlds
```
For each world it:
1. builds the world;
2. opens Gazebo;
3. starts OA-Stack;
4. shows the live evaluation;
5. closes Gazebo and moves to the next world.

At the end it prints a results table, which is also saved in `results/gazebo_*.csv`.

In Gazebo, right-click the drone (`airmouse`) and choose **Follow** to keep the camera on it. The lidar and ToF rays are drawn live.

## What is simulated
| Real part | In Gazebo |
|---|---|
| RPLIDAR C1, 8 cm above base_link | `gpu_lidar`: 500 beams, 360°, 0.15–12 m, 10 Hz, σ 15 mm |
| VL53L5CX UP, +20°, at (0.10, 0, 0.045) | `gpu_lidar` 8×8, 45°×45°, 3 m, 15 Hz, re-ordered into the guide's zone order |
| VL53L5CX DOWN, −35°, at (0.15, 0, 0) | same, tilted down |
| EKF height, SLAM pose | Gazebo pose + slow drift (σ 4.7 cm height, 1.5 cm position) |
| ArduPilot velocity setpoints | `VelocityControl` (body-frame vx, vy, vz, yaw rate) |

**Limits of this setup:**
- **Kinematic drone:** there's no tilt or motor model, and gravity is off.
- **People:** moved every 0.1 s through Gazebo's `set_pose` service. If your Gazebo version refuses that call, they stay still and a warning is printed.
- **Judging:** success and collisions are judged against the true world geometry.

## Files
| File | What it does |
|---|---|
| `gz_world.py` | Writes the Gazebo world (maze, drone and sensors) for a scenario and seed |
| `oa_gazebo.py` | Gazebo ↔ OA-Stack bridge and live evaluation. `--selftest` runs the same loop without Gazebo |
| `run_gazebo.sh` | Checks the installs, then runs N worlds and prints the results table |

## Tuning
```bash
python3 gazebo/oa_gazebo.py --seed 207 --set margin=0.2 --set vfh_window=2.5
```
Any `StackConfig` parameter can be set this way.

## Lessons from porting to Gazebo (also apply to the real drone)
1. **Point the nose where the drone moves.** Both ToF sensors look forward. Sliding sideways while avoiding means flying blind into ducts. The bridge sets yaw toward the velocity command.
2. **Blind band at flying height.** An obstacle whose top is 0–8 cm below the lidar plane (for example a 1.0 m pillar at a 1.0 m cruise):
   - is missed by the lidar;
   - shows only its side to the DOWN ToF.

   `--set unconfirmed_top_margin=0.04` treats such obstacles as solid until the drone climbs. In the benchmark it raised timeouts from 17% to 30%, so it's off by default. Better options are to fly at a height that keeps clear of common obstacle heights, or to mount the DOWN ToF less steeply.
3. **Match the ToF angles exactly.** The guide's zone geometry puts the outer zone centres at ±22.5°. A sensor model that's off by 3° was enough to cause a crash.

## Self-test results (no Gazebo, same loop, 16 worlds)
- 13 reached the goal.
- 2 hit a pillar in the blind band (lesson 2).
- 1 timed out.

Real Gazebo results may differ, because its sensors are real ray-traced sensors.
