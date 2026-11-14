"""Export one benchmark world (and the OA-Stack flight through it) as a Gazebo SDF world.

    python bench/export_gazebo_world.py --scenario mixed_clutter --seed 207 --out results/stack/airmouse_world.sdf

Then on Ubuntu with Gazebo (Harmonic / gz sim):   gz sim airmouse_world.sdf
Everything is static (no physics needed for a screenshot). The flight path is a line of small
spheres coloured by height (blue = low, yellow = high), start = green, goal = blue.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_world_3d import fly  # noqa: E402

COL = {"wall": (0.75, 0.77, 0.80, 1), "fence": (0.75, 0.77, 0.80, 1), "box": (0.91, 0.54, 0.23, 1),
       "hanging": (0.48, 0.36, 0.77, 1), "pillar": (0.55, 0.35, 0.17, 1), "person": (0.83, 0.23, 0.23, 1)}


def material(c):
    s = " ".join(f"{v:.3f}" for v in c)
    return f"<material><ambient>{s}</ambient><diffuse>{s}</diffuse><specular>0.1 0.1 0.1 1</specular></material>"


def geom(shape):
    return f"<geometry>{shape}</geometry>"


def static_model(name, x, y, z, shape, color, collide=True):
    g = geom(shape)
    col = f"<collision name='c'>{g}</collision>" if collide else ""
    return (f"<model name='{name}'><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
            f"<link name='l'>{col}<visual name='v'>{g}{material(color)}</visual></link></model>")


def viridis_like(t):
    """Dark blue (low) -> teal -> yellow (high)."""
    t = min(max(t, 0.0), 1.0)
    a, b, c = (0.23, 0.32, 0.55), (0.13, 0.57, 0.55), (0.99, 0.91, 0.14)
    lo, hi, u = (a, b, t * 2) if t < 0.5 else (b, c, t * 2 - 1)
    return tuple(p + (q - p) * u for p, q in zip(lo, hi)) + (1,)


def build(world, s, g, traj):
    x0, y0, x1, y1 = world.arena
    m = 0.3
    parts = []
    for i, r in enumerate(world.rects):
        if r.x1 < x0 - m or r.x0 > x1 + m or r.y1 < y0 - m or r.y0 > y1 + m:
            continue
        ax0, ay0, ax1, ay1 = max(r.x0, x0 - m), max(r.y0, y0 - m), min(r.x1, x1 + m), min(r.y1, y1 + m)
        sx, sy, sz = ax1 - ax0, ay1 - ay0, r.z1 - r.z0
        if min(sx, sy, sz) <= 0:
            continue
        parts.append(static_model(f"{r.kind}_{i}", (ax0 + ax1) / 2, (ay0 + ay1) / 2, (r.z0 + r.z1) / 2,
                                  f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>", COL.get(r.kind, COL["wall"])))
    for i, c in enumerate(world.cylinders):
        parts.append(static_model(f"pillar_{i}", c.x, c.y, c.z1 / 2,
                                  f"<cylinder><radius>{c.r:.3f}</radius><length>{c.z1:.3f}</length></cylinder>",
                                  COL["pillar"]))
    for i, p in enumerate(world.movers):
        parts.append(static_model(f"person_{i}", p.x, p.y, p.h / 2,
                                  f"<cylinder><radius>{p.r:.3f}</radius><length>{p.h:.3f}</length></cylinder>",
                                  COL["person"]))
    # flight path: one static model, one visual per sample (no collisions)
    vis, last = [], None
    for k, (x, y, z) in enumerate(traj):
        if last is not None and (x - last[0]) ** 2 + (y - last[1]) ** 2 < 0.12 ** 2:
            continue
        last = (x, y)
        c = viridis_like((z - 0.8) / 1.2)
        vis.append(f"<visual name='p{k}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
                   f"{geom('<sphere><radius>0.05</radius></sphere>')}{material(c)}</visual>")
    parts.append("<model name='flight_path'><static>true</static><link name='l'>" + "".join(vis) + "</link></model>")
    parts.append(static_model("start", s[0], s[1], traj[0, 2], "<sphere><radius>0.15</radius></sphere>",
                              (0.16, 0.62, 0.25, 1), collide=False))
    parts.append(static_model("goal", g[0], g[1], traj[-1, 2], "<sphere><radius>0.15</radius></sphere>",
                              (0.12, 0.44, 0.82, 1), collide=False))
    fx, fy = (x0 + x1) / 2, (y0 + y1) / 2
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="airmouse_benchmark">
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.85 0.88 0.92 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>{fx:.1f} {fy:.1f} 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction>
    </light>
    <model name="floor"><static>true</static><pose>{fx:.2f} {fy:.2f} 0 0 0 0</pose><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{x1 - x0 + 2:.1f} {y1 - y0 + 2:.1f}</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{x1 - x0 + 2:.1f} {y1 - y0 + 2:.1f}</size></plane></geometry>
      {material((0.92, 0.91, 0.88, 1))}</visual></link></model>
    {chr(10).join('    ' + p for p in parts)}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="mixed_clutter")
    ap.add_argument("--seed", type=int, default=207)
    ap.add_argument("--method", default="stack")
    ap.add_argument("--out", default="results/stack/airmouse_world.sdf")
    a = ap.parse_args()
    world, s, g, traj, result, t = fly(a.scenario, a.seed, a.method)
    with open(a.out, "w") as f:
        f.write(build(world, s, g, traj))
    print(f"{a.scenario} seed {a.seed}: {result} in {t:.0f} s -> {a.out}")


if __name__ == "__main__":
    main()
