# NIDAR AirMouse 2026-27: Obstacle Avoidance (OA-Stack)

Obstacle avoidance for the AirMouse indoor drone. It uses only the team's budget sensor stack: RPLIDAR C1, two VL53L5CX 8x8 ToF sensors (UP +20°, DOWN −35°) and EKF height.

- `obstacle_avoidance/oa_stack/`: the algorithm. Stages A–H: lidar tilt correction, ToF zones to 3D points, hazard classification, hazard memory, altitude band, VFH+ with braking limit and CBF safety filter, failsafes.
- `obstacle_avoidance/oa/`: shared core (VFH+, CBF, α-β tracker).
- `obstacle_avoidance/bench/`: benchmark, timing and live viewer (`python bench/live_stack.py`).
- `obstacle_avoidance/gazebo/`: Gazebo Harmonic simulation with live evaluation (`bash obstacle_avoidance/gazebo/run_gazebo.sh`).
- `obstacle_avoidance/tests/`: unit tests.
- `airmouse_sim/`: Python simulator used by the benchmark.

Benchmark (270 runs, 3 scenarios): 80% success and 7% collisions, against 38% and 62% for the guide's baseline. Compute: about 1.6 ms per cycle.

## Controls: drone model, PX4 SITL, ROS 2 offboard and PID tuning

**Start here: [`docs/CONTROLS_GUIDE.md`](docs/CONTROLS_GUIDE.md)** (also as PDF). It takes you from an empty virtual machine to the drone flying in PX4.

- `ros2_ws/src/nidar_drone_description/`: the drone from the team's SolidWorks URDF (ROS 2), with RViz and Gazebo Harmonic launch files and all sensors.
- `ros2_ws/src/nidar_control/`: `offboard_control` (ROS 2 → PX4 missions: hover, square, step) and `pid_logger` (command vs actual, CSV, `/nidar/pid_debug`).
- `px4/`: PX4 model and airframe `gz_nidar_drone`, plus install scripts.
- `setup/`: install scripts 01–05 for Ubuntu 24.04 (ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16, XRCE agent, QGC).
- `tools/`: `cad_urdf_to_xacro.py` (re-import the CAD) and `pid_analyze.py` (step metrics, plots and tuning hints).
- `cad_export_original/`: the untouched SolidWorks export.

Author: Abhyuday Saksena
