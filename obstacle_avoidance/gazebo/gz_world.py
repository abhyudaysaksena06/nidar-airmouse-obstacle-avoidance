"""Gazebo (Harmonic, gz sim) world for a live OA-Stack run.

    python3 gazebo/gz_world.py --scenario mixed_clutter --seed 207 --out /tmp/airmouse.sdf

The maze is the same seeded world the benchmark uses. The drone ("airmouse") carries the zip's sensor
stack as real Gazebo sensors:
  * RPLIDAR C1  : gpu_lidar, 500 beams / 360 deg, 0.15-12 m, 10 Hz, sigma 15 mm, 8 cm above base_link
  * VL53L5CX UP : gpu_lidar 8x8, 45 deg x 45 deg, 3 m, 15 Hz, at (0.10, 0, 0.045) tilted +20 deg
  * VL53L5CX DN : gpu_lidar 8x8, 45 deg x 45 deg, 3 m, 15 Hz, at (0.15, 0, 0)     tilted -35 deg
It is flown by velocity commands (gz VelocityControl, gravity off): a kinematic multirotor, so
there is no tilt. People are separate models that oa_gazebo.py moves every 0.1 s.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "bench"))
sys.path.insert(0, os.path.join(HERE, ".."))

import bench_stack as B  # noqa: E402
from oa_stack import StackConfig  # noqa: E402

WORLD = "airmouse"
COL = {"wall": (0.72, 0.74, 0.78), "fence": (0.72, 0.74, 0.78), "box": (0.91, 0.54, 0.23),
       "hanging": (0.38, 0.45, 0.85), "pillar": (0.55, 0.35, 0.17), "person": (0.85, 0.28, 0.55)}
TOF_HALF = math.radians(45.0 / 2)              # guide zone_directions(): outer zone centres at +-22.5 deg


def mat(c, a=1.0):
    s = f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a}"
    return f"<material><ambient>{s}</ambient><diffuse>{s}</diffuse><specular>0.1 0.1 0.1 1</specular></material>"


def static_box(name, cx, cy, cz, sx, sy, sz, col):
    g = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    return (f"<model name='{name}'><static>true</static><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>"
            f"<link name='l'><collision name='c'>{g}</collision><visual name='v'>{g}{mat(col)}</visual></link></model>")


def cylinder_model(name, x, y, r, h, col, static=True):
    g = f"<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>"
    phys = "" if static else "<gravity>false</gravity><inertial><mass>80</mass></inertial>"
    return (f"<model name='{name}'><static>{'true' if static else 'false'}</static><pose>{x:.3f} {y:.3f} {h / 2:.3f} 0 0 0</pose>"
            f"<link name='l'>{phys}<collision name='c'>{g}</collision><visual name='v'>{g}{mat(col)}</visual></link></model>")


def lidar(name, topic, xyz, pitch_up_deg, hs, hmin, hmax, vs, vmin, vmax, rmin, rmax, hz, noise, visual):
    # SDF pitch is right-handed about y: +pitch tilts the sensor DOWN, so "tilted up" = negative pitch
    return f"""
      <sensor name='{name}' type='gpu_lidar'>
        <pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 {-math.radians(pitch_up_deg):.5f} 0</pose>
        <topic>{topic}</topic><update_rate>{hz}</update_rate><always_on>1</always_on><visualize>{str(visual).lower()}</visualize>
        <lidar>
          <scan>
            <horizontal><samples>{hs}</samples><resolution>1</resolution><min_angle>{hmin:.6f}</min_angle><max_angle>{hmax:.6f}</max_angle></horizontal>
            <vertical><samples>{vs}</samples><resolution>1</resolution><min_angle>{vmin:.6f}</min_angle><max_angle>{vmax:.6f}</max_angle></vertical>
          </scan>
          <range><min>{rmin}</min><max>{rmax}</max><resolution>0.001</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise>
        </lidar>
      </sensor>"""


def drone(x, y, z, yaw, cfg: StackConfig):
    body = (0.18, 0.18, 0.06)
    props = "".join(
        f"<visual name='prop{i}'><pose>{0.13 * sx} {0.13 * sy} 0.035 0 0 0</pose>"
        f"<geometry><cylinder><radius>0.11</radius><length>0.01</length></cylinder></geometry>{mat((0.15, 0.15, 0.15), 0.6)}</visual>"
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))))
    sensors = (
        lidar("c1", "/airmouse/lidar", cfg.lidar_xyz, 0.0, 500, -math.pi, math.pi - 2 * math.pi / 500, 1, 0, 0,
              cfg.lidar_min_range, cfg.lidar_max_range, 10, 0.015, True)
        + lidar("tof_up", "/airmouse/tof_up", cfg.up_xyz, cfg.up_pitch_deg, 8, -TOF_HALF, TOF_HALF, 8, -TOF_HALF, TOF_HALF,
                0.02, cfg.tof_max_range, 15, 0.008, True)
        + lidar("tof_down", "/airmouse/tof_down", cfg.down_xyz, cfg.down_pitch_deg, 8, -TOF_HALF, TOF_HALF, 8, -TOF_HALF, TOF_HALF,
                0.02, cfg.tof_max_range, 15, 0.008, True))
    return f"""
    <model name='airmouse'>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name='base_link'>
        <gravity>false</gravity>
        <inertial><mass>1.2</mass><inertia><ixx>0.012</ixx><iyy>0.012</iyy><izz>0.02</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name='c'><geometry><cylinder><radius>{cfg.drone_radius:.3f}</radius><length>0.08</length></cylinder></geometry></collision>
        <visual name='body'><geometry><box><size>{body[0]} {body[1]} {body[2]}</size></box></geometry>{mat((0.12, 0.12, 0.14))}</visual>
        <visual name='nose'><pose>0.11 0 0 0 0 0</pose><geometry><box><size>0.05 0.04 0.03</size></box></geometry>{mat((0.9, 0.2, 0.2))}</visual>
        <visual name='guard'><geometry><cylinder><radius>{cfg.drone_radius:.3f}</radius><length>0.02</length></cylinder></geometry>{mat((0.2, 0.6, 0.9), 0.25)}</visual>
        {props}
        {sensors}
      </link>
      <plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>
    </model>"""


def build(scenario, seed):
    cfg, tof_range = B.build_cfg(scenario)
    tr = B.make_trial(cfg, seed)
    if tr is None:
        raise SystemExit(f"seed {seed}: no valid start/goal, try another seed")
    world, s, g, path = tr
    scfg = StackConfig(drone_radius=cfg["drone"]["radius_m"], tof_max_range=tof_range)
    x0, y0, x1, y1 = world.arena
    m = 0.3
    parts = []
    for i, r in enumerate(world.rects):
        if r.x1 < x0 - m or r.x0 > x1 + m or r.y1 < y0 - m or r.y0 > y1 + m:
            continue
        ax0, ay0, ax1, ay1 = max(r.x0, x0 - m), max(r.y0, y0 - m), min(r.x1, x1 + m), min(r.y1, y1 + m)
        if min(ax1 - ax0, ay1 - ay0, r.z1 - r.z0) <= 0:
            continue
        parts.append(static_box(f"{r.kind}_{i}", (ax0 + ax1) / 2, (ay0 + ay1) / 2, (r.z0 + r.z1) / 2,
                                ax1 - ax0, ay1 - ay0, r.z1 - r.z0, COL.get(r.kind, COL["wall"])))
    for i, c in enumerate(world.cylinders):
        parts.append(cylinder_model(f"pillar_{i}", c.x, c.y, c.r, c.z1, COL["pillar"]))
    for i, p in enumerate(world.movers):
        parts.append(cylinder_model(f"person_{i}", p.x, p.y, p.r, p.h, COL["person"], static=False))
    yaw = math.atan2(path[min(5, len(path) - 1)][1] - s[1], path[min(5, len(path) - 1)][0] - s[0])
    goal = (f"<model name='goal'><static>true</static><pose>{g[0]:.3f} {g[1]:.3f} 0.01 0 0 0</pose><link name='l'>"
            f"<visual name='v'><geometry><cylinder><radius>0.5</radius><length>0.02</length></cylinder></geometry>"
            f"{mat((0.12, 0.44, 0.82), 0.6)}</visual></link></model>")
    start = goal.replace("'goal'", "'start'").replace(f"{g[0]:.3f} {g[1]:.3f}", f"{s[0]:.3f} {s[1]:.3f}") \
        .replace(mat((0.12, 0.44, 0.82), 0.6), mat((0.16, 0.62, 0.25), 0.6))
    fx, fy = (x0 + x1) / 2, (y0 + y1) / 2
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{WORLD}">
    <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.82 0.86 0.92 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>{fx:.1f} {fy:.1f} 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction></light>
    <model name="floor"><static>true</static><pose>{fx:.2f} {fy:.2f} 0 0 0 0</pose><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{x1 - x0 + 2:.1f} {y1 - y0 + 2:.1f}</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{x1 - x0 + 2:.1f} {y1 - y0 + 2:.1f}</size></plane></geometry>{mat((0.92, 0.91, 0.88))}</visual>
    </link></model>
    {start}
    {goal}
    {chr(10).join('    ' + p for p in parts)}
    {drone(s[0], s[1], cfg["drone"]["cruise_alt_m"], yaw, scfg)}
  </world>
</sdf>
"""
    return sdf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="mixed_clutter", choices=list(B.SCENARIOS))
    ap.add_argument("--seed", type=int, default=207)
    ap.add_argument("--out", default="airmouse.sdf")
    a = ap.parse_args()
    with open(a.out, "w", newline="\n") as f:
        f.write(build(a.scenario, a.seed))
    print(f"world {a.scenario} seed {a.seed} -> {a.out}")


if __name__ == "__main__":
    main()
