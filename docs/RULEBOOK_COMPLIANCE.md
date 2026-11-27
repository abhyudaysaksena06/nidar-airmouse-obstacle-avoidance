# NIDAR Rulebook v2.2: compliance of the controls stack (PS2, NIDAR AirMouse)

This file checks every rule in the *NIDAR 26-27 Rulebook Ver 2.2* that applies to Problem Statement 2
(AirMouse) against this repository's controls work: the drone model, the simulation, the PX4 configuration
and the ROS 2 nodes.

**Status key:**

* ✅ = done and tested in simulation.
* 🔧 = done in software, still to be done on the real drone or CAD.
* ➡️ = belongs to another subsystem (mapping, detection, GCS); this repo provides the interface.

## General rules (section 8)

| Rule | Requirement | Status | How / where |
|---|---|---|---|
| 8.1 | Teams design and integrate their own solution | ✅ | Own model, PX4 airframe, ROS 2 nodes |
| 8.2 | No commercial ready-to-fly airframe; open-source software allowed | ✅ | Airframe from the team's own CAD; PX4 and ROS 2 are open source |
| 8.4 | No GSM/LTE/5G/public Wi-Fi/internet/cloud during the mission | ✅ | PX4 ↔ QGC over local UDP/telemetry radio, PX4 ↔ ROS over the local XRCE agent. Internet is only needed to **install** the software, never in flight. See the guide, section 7.6 |
| 8.5 | Only local communication; no cables or tethers to the drone in flight | ✅ / 🔧 | Simulation uses localhost only. Real drone: a telemetry radio or local Wi-Fi access point that you bring, never a tether |
| 8.6 | Jury may inspect hardware, software and configuration | ✅ | Everything is in this repo: airframe params `px4/airframes/4910_gz_nidar_drone`, node code, configs |

## PS2: NIDAR AirMouse (rules 8.20–8.31 and Annexure, sections 1–10)

| Rule | Requirement | Status | How / where |
|---|---|---|---|
| 8.20 / §1 | Autonomous drone in a GPS-denied indoor environment | ✅ | `arena_demo` mission flies with no operator input after start (tested end-to-end in PX4 SITL with GPS off) |
| **8.24 / §4** | **No GPS/GNSS-based navigation** | ✅ | PX4 airframe: `SENS_EN_GPSSIM 0`, `SYS_HAS_GPS 0`, `EKF2_GPS_CTRL 0`, magnetometer off (`SYS_HAS_MAG 0`, `EKF2_MAG_TYPE 5`, unreliable indoors). Position, height and heading come **only** from vision odometry (`EKF2_EV_CTRL 15`, `EKF2_HGT_REF 3`). The Gazebo model has **no GNSS sensor** |
| 8.24 | (how PX4 gets position) | ✅ / 🔧 | `nidar_control/ev_bridge` publishes `/fmu/in/vehicle_visual_odometry`. Sim: Gazebo pose stands in for SLAM. **Real drone: feed the SLAM pose (e.g. RPLIDAR SLAM) into the same node with `-p odom_topic:=<slam odometry>`** |
| 8.21 / §1 | Enter and exit through the **same** designated point | ✅ | Recall retraces the breadcrumbs of the flown path and leaves through the entry; `arena_demo` always ends this way |
| 8.22 / §3 | Arena ≤ 15 × 15 m, corridors ≥ 1 m wide, 8 ft clear height, 2 × 2 m rooms, netted roof | ✅ | `worlds/make_arena.py` makes a practice arena to exactly these numbers on a 1 m grid (the generator checks room sizes and that everything is reachable) |
| 8.23 / §9 | Take off inside the 2 ft × 2 ft launch area | ✅ | Spawn and launch area at the arena's entry. Drone footprint **with guards 0.48 × 0.45 m** fits the 0.61 × 0.61 m area |
| §9 | Drone sized to fly safely in 1 m corridors and 8 ft height | ✅ | 0.45 m wide → 0.27 m clearance each side when centred; flight altitude 1.2 m, fence ceiling 2.1 m (roof net at 2.44 m) |
| 8.29 / §9 | No contact with walls, ceiling or obstacles | ✅ / ➡️ | Waypoints on corridor centres, geofence ceiling, nose points along the path so the forward ToF looks ahead. Obstacle avoidance itself is OA-Stack in `obstacle_avoidance/` |
| **8.30 / §2** | **Propeller guards fully covering the propeller area** | 🔧 | **The CAD has no guards.** Placeholders (r = 85 mm ring, 20 g each) are in the model so size and weight are realistic. **CAD team: design real guards** |
| §2 | All-up weight ≤ 10 kg | ✅ | Model 1.50 kg with guards (no battery in the CAD yet; a 4S battery keeps it far below 10 kg) |
| **8.31 / §10** | **Emergency stop** | ✅ | `ros2 topic pub --once /nidar/emergency_stop std_msgs/msg/Empty` (force disarm). In QGC also the *Emergency Stop / Kill* action |
| 8.31 / §10 | **Mission abort** | ✅ | `/nidar/abort` → lands where it is. QGC *Land* also works |
| 8.31 / §10 | **Communication-loss recovery** (loss of command and control link) | ✅ | When the GCS link drops (PX4 `gcs_connection_lost_counter` rises), the node recalls: it retraces its path out of the entry and lands on the launch area. If the companion computer or node itself dies, PX4 lands after 1 s (`COM_OBL_RC_ACT 4`, `COM_OF_LOSS_T 1`) |
| 8.31 / §10 | **Low-battery fail-safe** | ✅ | Node: battery ≤ 30 % → recall. PX4 backstop: ≤ 12 % lands (`COM_LOW_BAT_ACT 2`). Simulated battery drains over 30 min |
| §10 | **Geofence breach** fail-safe | ✅ | Arena box (15 × 15 m + launch area, 2.1 m ceiling): setpoints are clamped inside it, and if the drone is ever outside it, it recalls. (PX4's own geofence needs GPS, so it can't be used.) |
| §10 | **Emergency recall** | ✅ | `/nidar/recall` → back along the flown path, out of the entry, land on the launch area |
| §4 | Max flight time 30 min | ✅ | Node recalls at 25 min (`mission_time_limit_s 1500`), leaving 5 min to exit. Fast-completion bonus is ≤ 15 min |
| §4 / 8.28 | No manual input after start (only start and safety abort or stop) | ✅ | One command starts the mission; operator inputs are only stop, abort and recall. **Do not change QGC parameters or modes during a scored run (−50 per instance)** |
| §4 | No FPV goggles or extra viewing devices; supervise only through the GCS | ➡️ | Procedure: all viewing on the one GCS screen |
| 8.25–8.27, §5, §6 | 2D map, survivor detection, grid tagging, live camera on the GCS | ➡️ | Not controls work. Hooks provided: camera `/camera/image_raw`, lidar `/scan`, ToF and ultrasonic topics in simulation; `/nidar/mission_status` (JSON: state, progress, position from launch, battery, fail-safe reason, inside-fence) for the GCS display |
| §8 | Max 2 people for setup, 1 operator | ➡️ | Procedure. The 4-terminal start can be scripted for the 5-minute setup |
| Penalty: landing outside the launch zone | Land in the designated zone | ✅ | Recall ends over the launch area (vision position) and lands there. **Tested: landed 0.02 m from the centre** of the 0.61 m area |
| §9 / Penalty: obstacle contact | No contact | ✅ | `tools/arena_check.py` on the `arena_demo` flight: **closest guard edge to a wall +0.087 m**, highest point 1.34 m (net 2.44 m). Corridor flying rules are in the guide, section 7.4 |

## What the other teams must provide for full compliance

1. **SLAM / localisation**: a pose source to feed `ev_bridge` (rule 8.24). Without it the real drone has no position.
2. **CAD**: propeller guards (8.30), arms, battery, correct ESC positions (see the guide, section 10).
3. **Perception and GCS**: survivor detection, 1 m grid tagging, the 2D map, the live camera feed on **one** GCS
   screen, and showing `/nidar/mission_status`.
4. **Exploration**: `arena_demo` follows a known example route. In competition the layout is secret, so the
   OA-Stack / exploration planner must produce the waypoints. It can send them through the same offboard
   interface, and the fail-safes in this node still apply.
