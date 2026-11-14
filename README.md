# NIDAR AirMouse 2026-27: Obstacle Avoidance (OA-Stack)

Obstacle avoidance for the AirMouse indoor drone. It uses only the team's budget sensor stack: RPLIDAR C1, two VL53L5CX 8x8 ToF sensors (UP +20°, DOWN −35°) and EKF height.

- `obstacle_avoidance/oa_stack/`: the algorithm. Stages A–H: lidar tilt correction, ToF zones to 3D points, hazard classification, hazard memory, altitude band, VFH+ with braking limit and CBF safety filter, failsafes.
- `obstacle_avoidance/oa/`: shared core (VFH+, CBF, α-β tracker).
- `obstacle_avoidance/bench/`: benchmark, timing and live viewer (`python bench/live_stack.py`).
- `obstacle_avoidance/gazebo/`: Gazebo Harmonic simulation with live evaluation (`bash obstacle_avoidance/gazebo/run_gazebo.sh`).
- `obstacle_avoidance/tests/`: unit tests.
- `airmouse_sim/`: Python simulator used by the benchmark.

Benchmark (270 runs, 3 scenarios): 80% success and 7% collisions, against 38% and 62% for the guide's baseline. Compute: about 1.6 ms per cycle.

Author: Abhyuday Saksena
