# Obstacle Avoidance: NIDAR AirMouse 2026-27 (Automation)

**Owner:** Abhyuday Saksena · **Deadline:** 23 Oct 2026

There are **five obstacle-avoidance configurations** behind one interface. They were compared on identical simulated maps, with **full analysis in [docs/FIVE_ALGORITHM_ANALYSIS.md](docs/FIVE_ALGORITHM_ANALYSIS.md)**.

| # | Algorithm | Sensors | Success | Collisions |
|---|---|---|---|---|
| 0 | Brake + repulsion (existing reference) | 2D lidar | 63% | 26% |
| 1 | Layered costmap + DWA ([docs](docs/ALGO1_README.md)): follows Jainil's README | lidar + depth camera | 47% | 10% |
| 2 | VFH+ + CBF ([docs](docs/ALGO2_README.md)) | 360° lidar + 3 ToF | **81%** | 5% |
| D | Depth camera as main sensor, VFH+ + CBF core | depth camera only | 58% | 15% |
| **L** | **360° lidar + 4 multi-zone ToF**, VFH+ + CBF core (**recommended**) | lidar + ToF ring | 76% | **3%** |

*Pooled over clutter / 3D / narrow, 78 trials each. Algo 2 and L are statistically tied as the top tier.* The earlier two-way study is in [docs/COMPARATIVE_STUDY.md](docs/COMPARATIVE_STUDY.md).

```
planner (A* + pure pursuit) ──u_des──▶ [ Algo 1 or Algo 2 ] ──safe command──▶ flight controller
lidar / odometry / depth or ToF ─────▶                     ──status, replan, report──▶ planner / mapping / GCS
```

## Quick start
```bash
pip install numpy pyyaml         # + pygame for the simulator benchmark
python tests/test_oa_dwa.py      # Algo 1: 7 tests
python tests/test_oa.py          # Algo 2 / D / L core: 9 tests
python examples/basic_usage.py   # Algo 2 demo
python bench/bench_avoidance.py --scenario clutter --methods brake_repulse,algo1,full --trials 30
```
```python
from oa_dwa import DWAAvoider, DWAConfig          # Algo 1
avoider = DWAAvoider(DWAConfig.from_yaml("configs/oa_dwa.yaml"))
# or
from oa import ObstacleAvoider, AvoidanceConfig  # Algo 2 / D / L
avoider = ObstacleAvoider(AvoidanceConfig())
# Algo D / L: also pass 3D points each frame:
#   avoider.update(..., points3d=xyz_body, points3d_vfov=(el_min, el_max), z=height)
# Algo D (no lidar): scan = oa.points3d.virtual_scan(avoider.cloud.split(xyz, z, vfov)[0])

out = avoider.update(t, odom_pose, u_des, scan=(angles, ranges))   # every control cycle
fc.send(out.vx, out.vy, out.yaw_rate)        # yaw_rate: always set by Algo 1; front-lidar only for Algo 2
if out.replan: planner.replan()
```

## Integration rules (both algorithms)
1. Give the avoider **smooth odometry**, never the SLAM pose (loop-closure jumps corrupt the obstacle memory).
2. **Planner clearance ≥ avoider safety radius** (A* inflation ≥ 0.40 m), or the avoider fights every path.
3. Measure the real **braking deceleration** and **command latency** in flight tests, and put them in the config.

## Layout
```
oa_dwa/  Algo 1: perception (L1), frames (L2), costmap (L3), subgoal + dwa (L4), avoider (L5)
oa/      Algo 2 / D / L core: scan, tracker, vfh, cbf, vertical, points3d (3D front-end), avoider
configs/oa_dwa.yaml           Algo 1 parameters (team YAML convention)
tests/   test_oa_dwa.py, test_oa.py
bench/bench_avoidance.py      simulator benchmark (needs ../airmouse_sim)
sim_adapter/explorer_oa.py    full AirMouse mission with Algo 2
results/five/                 5-algorithm benchmark data (results/final/ = earlier 3-way run)
docs/                         FIVE_ALGORITHM_ANALYSIS, ALGO1_README, ALGO2_README, COMPARATIVE_STUDY
```
