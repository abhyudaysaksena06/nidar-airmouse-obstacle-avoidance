"""Export a generated world to a Gazebo (gz Harmonic) SDF file."""
import math


def _box(name, x, y, z, sx, sy, sz, rgba, yaw=0.0):
    col = " ".join(str(c) for c in rgba)
    geo = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
    return f"""    <model name="{name}"><static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="c"><geometry>{geo}</geometry></collision>
        <visual name="v"><geometry>{geo}</geometry>
          <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>
      </link></model>
"""


def _cyl(name, x, y, r, h, rgba):
    col = " ".join(str(c) for c in rgba)
    geo = f"<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>"
    return f"""    <model name="{name}"><static>true</static>
      <pose>{x:.3f} {y:.3f} {h / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry>{geo}</geometry></collision>
        <visual name="v"><geometry>{geo}</geometry>
          <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>
      </link></model>
"""


def export_sdf(world, path):
    h = world.cfg["arena"]["wall_height_m"]
    out = []
    for k, r in enumerate(world.rects):
        z0, z1 = r.z0, (h if r.kind == "fence" else r.z1)
        rgba = {"wall": (0.7, 0.7, 0.7, 1), "fence": (0.4, 0.4, 0.4, 1), "box": (0.8, 0.45, 0.2, 1),
                "hanging": (0.35, 0.45, 0.8, 1)}[r.kind]
        out.append(_box(f"{r.kind}_{k}", (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2, (z0 + z1) / 2,
                        r.x1 - r.x0, r.y1 - r.y0, z1 - z0, rgba))
    for k, c in enumerate(world.cylinders):
        out.append(_cyl(f"pillar_{k}", c.x, c.y, c.r, c.z1, (0.8, 0.45, 0.2, 1)))
    for s in world.survivors:  # lying person stand-in: 1.7 x 0.45 x 0.25 m
        out.append(_box(f"survivor_{s.id}", s.x, s.y, 0.125, 1.7, 0.45, 0.25, (0.9, 0.2, 0.2, 1), s.yaw))
    sx, sy, syaw = world.start
    sdf = f"""<?xml version="1.0"?>
<!-- AirMouse random maze, seed {world.seed}
     Drone start (outside, facing entry): x={sx:.2f} y={sy:.2f} yaw={syaw:.3f}
     PX4: PX4_GZ_WORLD=<this file name without .sdf> PX4_GZ_MODEL_POSE="{sx:.2f},{sy:.2f},0.2,0,0,{syaw:.3f}" make px4_sitl gz_x500_flow
     Moving obstacles are not exported (static world). -->
<sdf version="1.9">
  <world name="airmouse_{world.seed}">
    <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{world.width + 10:.0f} {world.height + 10:.0f}</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{world.width + 10:.0f} {world.height + 10:.0f}</size></plane></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
    </link></model>
{''.join(out)}  </world>
</sdf>
"""
    with open(path, "w") as f:
        f.write(sdf)
    return path
