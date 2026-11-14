"""Offline preview of a Gazebo SDF world written by export_gazebo_world.py (no Gazebo needed).

    python bench/preview_sdf.py results/stack/airmouse_world.sdf --out results/stack/sdf_view.png

Reads the boxes, cylinders and spheres (pose, size, diffuse colour) straight from the .sdf and draws
them with a perspective camera, a 1 m floor grid and simple sun shading. It shows what the file
contains; it is not a Gazebo screenshot.
"""
import argparse
import math
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

SUN = np.array([-0.4, 0.3, -0.9]) / np.linalg.norm([-0.4, 0.3, -0.9])


def nums(text):
    return [float(v) for v in text.split()]


def box(c, size):
    (x, y, z), (sx, sy, sz) = c, size
    x0, x1, y0, y1, z0, z1 = x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z - sz / 2, z + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    q = [((4, 5, 6, 7), (0, 0, 1)), ((0, 1, 5, 4), (0, -1, 0)), ((1, 2, 6, 5), (1, 0, 0)),
         ((2, 3, 7, 6), (0, 1, 0)), ((3, 0, 4, 7), (-1, 0, 0))]          # floor-side face never visible
    return [([v[i] for i in f], n) for f, n in q]


def cylinder(c, r, h, n=20):
    x, y, z = c
    th = np.linspace(0, 2 * math.pi, n + 1)
    out = []
    for i in range(n):
        a, b = th[i], th[i + 1]
        m = (a + b) / 2
        out.append(([(x + r * math.cos(a), y + r * math.sin(a), z - h / 2), (x + r * math.cos(b), y + r * math.sin(b), z - h / 2),
                     (x + r * math.cos(b), y + r * math.sin(b), z + h / 2), (x + r * math.cos(a), y + r * math.sin(a), z + h / 2)],
                    (math.cos(m), math.sin(m), 0)))
    out.append(([(x + r * math.cos(t), y + r * math.sin(t), z + h / 2) for t in th[:-1]], (0, 0, 1)))
    return out


def shade(rgb, normal):
    lam = max(0.0, -float(np.dot(SUN, normal)))
    k = 0.55 + 0.45 * lam
    return tuple(min(1.0, c * k) for c in rgb)


def load(path):
    world = ET.parse(path).getroot().find("world")
    faces, colors, spheres, floor = [], [], [], None
    for model in world.findall("model"):
        mp = nums(model.findtext("pose", "0 0 0 0 0 0"))[:3]
        for vis in model.iter("visual"):
            vp = nums(vis.findtext("pose", "0 0 0 0 0 0"))[:3]
            c = (mp[0] + vp[0], mp[1] + vp[1], mp[2] + vp[2])
            rgb = tuple(nums(vis.findtext("material/diffuse", "0.7 0.7 0.7 1"))[:3])
            g = vis.find("geometry")
            if g.find("plane") is not None:
                sx, sy = nums(g.findtext("plane/size"))
                floor = (c, sx, sy, rgb)
            elif g.find("box") is not None:
                for f, n in box(c, nums(g.findtext("box/size"))):
                    faces.append(f); colors.append(shade(rgb, n))
            elif g.find("cylinder") is not None:
                for f, n in cylinder(c, float(g.findtext("cylinder/radius")), float(g.findtext("cylinder/length"))):
                    faces.append(f); colors.append(shade(rgb, n))
            elif g.find("sphere") is not None:
                spheres.append((c, float(g.findtext("sphere/radius")), rgb))
    bg = tuple(nums(world.findtext("scene/background", "0.85 0.88 0.92 1"))[:3])
    return faces, colors, spheres, floor, bg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("--out", default="results/stack/sdf_view.png")
    ap.add_argument("--elev", type=float, default=55)
    ap.add_argument("--azim", type=float, default=-60)
    a = ap.parse_args()
    faces, colors, spheres, floor, bg = load(a.sdf)
    fig = plt.figure(figsize=(16, 9), dpi=140, facecolor=bg)
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False, facecolor=bg)
    ax.set_proj_type("persp", focal_length=0.45)
    (fx, fy, _), sx, sy, frgb = floor
    x0, x1, y0, y1 = fx - sx / 2, fx + sx / 2, fy - sy / 2, fy + sy / 2
    ax.add_collection3d(Poly3DCollection([[(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)]],
                                         facecolor=frgb, edgecolor="none", zorder=1))
    for gx in np.arange(math.ceil(x0), x1 + 1e-6):
        ax.plot([gx, gx], [y0, y1], [0.001, 0.001], color=(0.7, 0.7, 0.7), lw=0.4, zorder=1.5)
    for gy in np.arange(math.ceil(y0), y1 + 1e-6):
        ax.plot([x0, x1], [gy, gy], [0.001, 0.001], color=(0.7, 0.7, 0.7), lw=0.4, zorder=1.5)
    ax.add_collection3d(Poly3DCollection(faces, facecolors=colors, edgecolors=(0.25, 0.25, 0.25, 0.35),
                                         linewidths=0.25, zorder=2))
    big = [sp for sp in spheres if sp[1] > 0.1]
    small = [sp for sp in spheres if sp[1] <= 0.1]
    if small:
        p = np.array([sp[0] for sp in small])
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color=(0.3, 0.3, 0.3), lw=0.6, alpha=0.5, zorder=3)
        ax.scatter(p[:, 0], p[:, 1], p[:, 2], s=28, c=[sp[2] for sp in small], depthshade=False, zorder=4,
                   edgecolors="none")
    for c, r, rgb in big:
        ax.scatter(*c, s=220, color=rgb, depthshade=False, zorder=5, edgecolors="white", linewidths=1.2)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(0, 2.6)
    ax.set_box_aspect((x1 - x0, y1 - y0, 2.6 * 1.3), zoom=1.12)
    ax.set_axis_off()
    ax.view_init(elev=a.elev, azim=a.azim)
    fig.subplots_adjust(0, 0, 1, 1)
    fig.text(0.012, 0.015, f"Preview of {a.sdf.replace(chr(92), '/').split('/')[-1]} (rendered offline from the SDF file, "
             f"not a Gazebo screenshot)", fontsize=9, color=(0.35, 0.35, 0.35))
    fig.savefig(a.out, facecolor=bg)
    print(f"{len(faces)} faces, {len(spheres)} spheres -> {a.out}")


if __name__ == "__main__":
    main()
