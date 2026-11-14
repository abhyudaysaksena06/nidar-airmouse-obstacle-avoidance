"""3D picture of one benchmark world with the OA-Stack flight drawn in.

    python bench/render_world_3d.py --scenario mixed_clutter --seed 205 --out results/stack/world_3d.png

Walls are drawn see-through so the obstacles inside the maze stay visible:
low boxes (orange), ducts hanging from above (purple), pillars (brown), people (red),
drone path coloured by height, start (green) and goal (blue).
"""
import argparse
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_stack as B  # noqa: E402
from oa_stack import StackConfig  # noqa: E402
from sim.engine import Simulation  # noqa: E402


def box_faces(x0, y0, z0, x1, y1, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return [[v[i] for i in q] for q in f]


def prism(x, y, r, z0, z1, n=16):
    th = np.linspace(0, 2 * math.pi, n + 1)
    ring = [(x + r * math.cos(t), y + r * math.sin(t)) for t in th]
    faces = [[(*ring[i], z0), (*ring[i + 1], z0), (*ring[i + 1], z1), (*ring[i], z1)] for i in range(n)]
    return faces + [[(px, py, z1) for px, py in ring[:-1]]]


def fly(scenario, seed, method):
    cfg, tof_range = B.build_cfg(scenario)
    tr = B.make_trial(cfg, seed)
    if tr is None:
        raise SystemExit("no valid start/goal for this seed, try another")
    world, s, g, path = tr
    scfg = StackConfig(drone_radius=cfg["drone"]["radius_m"], tof_max_range=tof_range)
    pilot = B.StackPilot(method, list(path), g, scfg, tof_range, seed)
    sim = Simulation(world, pilot, seed=seed)
    pilot.sim = sim
    d = sim.drone
    d.x, d.y, d.z, d.landed = s[0], s[1], cfg["drone"]["cruise_alt_m"], False
    d.yaw = math.atan2(path[min(5, len(path) - 1)][1] - s[1], path[min(5, len(path) - 1)][0] - s[0])
    limit = 25.0 + B.path_length(path) / 0.8 * 2.5
    traj, result = [], "timeout"
    while sim.t < limit:
        sim.step()
        traj.append((d.x, d.y, d.z))
        if sim.crashed:
            result = "collision"
            break
        if math.hypot(d.x - g[0], d.y - g[1]) < 0.5:
            result = "success"
            break
    return world, s, g, np.array(traj), result, sim.t


def render(world, s, g, traj, title, out):
    fig = plt.figure(figsize=(15, 9.5), dpi=130)
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    x0, y0, x1, y1 = world.arena
    m = 0.3
    floor = Poly3DCollection([[(x0 - m, y0 - m, 0), (x1 + m, y0 - m, 0), (x1 + m, y1 + m, 0), (x0 - m, y1 + m, 0)]],
                             facecolor="#ece9e2", edgecolor="none", zorder=1)
    ax.add_collection3d(floor)
    # every solid in ONE collection so matplotlib depth-sorts the faces together
    faces, fc, ec = [], [], []
    style = {"wall": (0.60, 0.64, 0.69, 0.22), "fence": (0.60, 0.64, 0.69, 0.22),
             "box": (0.91, 0.54, 0.23, 0.95), "hanging": (0.48, 0.36, 0.77, 0.95)}

    def add(fs, col, edge):
        faces.extend(fs); fc.extend([col] * len(fs)); ec.extend([edge] * len(fs))
    for r in world.rects:
        if r.x1 < x0 - m or r.x0 > x1 + m or r.y1 < y0 - m or r.y0 > y1 + m:
            continue
        col = style.get(r.kind, style["wall"])
        add(box_faces(max(r.x0, x0 - m), max(r.y0, y0 - m), r.z0, min(r.x1, x1 + m), min(r.y1, y1 + m), r.z1),
            col, (0.35, 0.38, 0.42, 0.5) if r.kind in ("wall", "fence") else (0.2, 0.2, 0.2, 0.8))
    for c in world.cylinders:
        add(prism(c.x, c.y, c.r, 0.0, c.z1), (0.55, 0.35, 0.17, 0.95), (0.3, 0.2, 0.1, 0.4))
    for mv in world.movers:
        add(prism(mv.x, mv.y, mv.r, 0.0, mv.h), (0.83, 0.23, 0.23, 0.95), (0.4, 0.1, 0.1, 0.4))
    ax.add_collection3d(Poly3DCollection(faces, facecolors=fc, edgecolors=ec, linewidths=0.3, zorder=2))
    # flight drawn on top, plus its shadow on the floor
    z = traj[:, 2]
    ax.plot(traj[:, 0], traj[:, 1], 0 * z + 0.01, color="#444", linewidth=1.0, alpha=0.6, zorder=3)
    for i in range(0, len(traj) - 1, 2):
        ax.plot(traj[i:i + 3, 0], traj[i:i + 3, 1], traj[i:i + 3, 2],
                color=plt.cm.viridis((z[i] - 0.8) / 1.2), linewidth=3.0, zorder=4)
    ax.scatter([s[0]], [s[1]], [z[0]], s=90, color="#2a9d3f", depthshade=False, zorder=5)
    ax.scatter([g[0]], [g[1]], [z[-1]], s=90, color="#1f6fd1", depthshade=False, zorder=5)
    ax.set_xlim(x0 - m, x1 + m); ax.set_ylim(y0 - m, y1 + m); ax.set_zlim(0, 2.5)
    ax.set_box_aspect((x1 - x0, y1 - y0, 2.5 * 1.5))
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.view_init(elev=42, azim=-62)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=style["wall"][:3], alpha=0.4, label="walls 2.4 m (see-through)"),
                       Patch(color=style["box"][:3], label="low boxes / obstacles"),
                       Patch(color=style["hanging"][:3], label="ducts (hanging)"),
                       Patch(color=(0.55, 0.35, 0.17), label="pillars"), Patch(color=(0.83, 0.23, 0.23), label="people"),
                       Line2D([], [], color=plt.cm.viridis(0.3), lw=3, label="OA-Stack flight (colour = height)"),
                       Line2D([], [], color="#444", lw=1, label="flight shadow on floor"),
                       Line2D([], [], marker="o", color="#2a9d3f", ls="", label="start"),
                       Line2D([], [], marker="o", color="#1f6fd1", ls="", label="goal")],
              loc="upper left", fontsize=9, framealpha=0.9)
    ax.set_title(title, fontsize=13)
    fig.colorbar(plt.cm.ScalarMappable(norm=plt.Normalize(0.8, 2.0), cmap="viridis"), ax=ax,
                 shrink=0.45, pad=0.08, label="flight height (m)")
    fig.subplots_adjust(left=0, right=0.97, top=0.95, bottom=0.02)
    fig.savefig(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="mixed_clutter")
    ap.add_argument("--seed", type=int, default=205)
    ap.add_argument("--method", default="stack")
    ap.add_argument("--out", default="results/stack/world_3d.png")
    a = ap.parse_args()
    world, s, g, traj, result, t = fly(a.scenario, a.seed, a.method)
    title = (f"AirMouse benchmark world '{a.scenario}' (seed {a.seed}) - {a.method}: {result} "
             f"in {t:.0f} s, height {traj[:, 2].min():.2f}-{traj[:, 2].max():.2f} m")
    render(world, s, g, traj, title, a.out)
    print(title, "->", a.out)


if __name__ == "__main__":
    main()
