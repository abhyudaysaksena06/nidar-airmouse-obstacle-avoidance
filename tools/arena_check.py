#!/usr/bin/env python3
"""Check a flown arena mission against the rulebook: wall clearance, ceiling and landing spot.

    python3 tools/arena_check.py ~/nidar_logs/pid_<date>_<time>.csv
    python3 tools/arena_check.py <csv> --half-width 0.225 --plot ~/nidar_logs/track.png

Uses the CSV that pid_logger writes during every offboard mission, and the arena layout
from ros2_ws/src/nidar_drone_description/worlds/make_arena.py. It reports:
  * closest approach of the drone's guard edge to any wall (must stay > 0: PS2 rule 8.29,
    "safe mission completion" score and the obstacle-contact penalty)
  * highest altitude vs the 2.44 m roof net
  * where it landed vs the 2 ft x 2 ft launch area (penalty if outside)
PASS/FAIL is printed for each.
"""
import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros2_ws", "src", "nidar_drone_description", "worlds"))
import make_arena as A  # noqa: E402

LAUNCH_X, LAUNCH_Y = 7.5, -0.5      # launch-area centre in arena coordinates


def dist_to_segment(px, py, s):
    x0, y0, x1, y1 = s
    dx, dy = x1 - x0, y1 - y0
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--half-width", type=float, default=0.225, help="drone half-width incl. guards [m]")
    ap.add_argument("--plot", help="save a top-view PNG of the flown track")
    a = ap.parse_args()

    grid = A.parse(A.LAYOUT)
    entry, rooms = A.check(grid)
    walls = A.walls(grid, entry)
    rows = [r for r in csv.DictReader(open(a.csv)) if r["x"] and r["y"] and r["z"]]
    n0, e0, d0 = float(rows[0]["x"]), float(rows[0]["y"]), float(rows[0]["z"])
    track, worst, max_alt = [], (1e9, None), 0.0
    for r in rows:
        n, e, alt = float(r["x"]) - n0, float(r["y"]) - e0, d0 - float(r["z"])
        X, Y = LAUNCH_X + e, LAUNCH_Y + n            # PX4 NED from launch -> arena x (east), y (north)
        track.append((X, Y, alt))
        max_alt = max(max_alt, alt)
        if alt > 0.3:                                  # in flight
            d = min(dist_to_segment(X, Y, s) for s in walls) - A.WALL_T / 2
            if d < worst[0]:
                worst = (d, (X, Y, alt))
    clearance = worst[0] - a.half_width
    lx, ly, _ = track[-1]
    off = max(abs(lx - LAUNCH_X), abs(ly - LAUNCH_Y))
    ok1, ok2, ok3 = clearance > 0.05, max_alt < A.CEILING - 0.3, off <= A.LAUNCH / 2
    print(f"samples: {len(rows)}")
    print(f"[{'PASS' if ok1 else 'FAIL'}] closest guard edge to a wall: {clearance:+.3f} m "
          f"at arena ({worst[1][0]:.2f}, {worst[1][1]:.2f}), alt {worst[1][2]:.2f} m   (need > +0.05 m)")
    print(f"[{'PASS' if ok2 else 'FAIL'}] highest altitude {max_alt:.2f} m   (roof net at {A.CEILING} m)")
    print(f"[{'PASS' if ok3 else 'FAIL'}] landed {lx - LAUNCH_X:+.2f} m east, {ly - LAUNCH_Y:+.2f} m north "
          f"of the launch-area centre   (centre must be within +-{A.LAUNCH / 2:.3f} m)")
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7.4))
        for (c, r), v in grid.items():
            if A.region(v) is None:
                ax.add_patch(plt.Rectangle((c, r), 1, 1, color="#4a4a4a"))
        for x0, y0, x1, y1 in walls:
            ax.plot([x0, x1], [y0, y1], color="#c92a2a", lw=2)
        ax.add_patch(plt.Rectangle((LAUNCH_X - A.LAUNCH / 2, LAUNCH_Y - A.LAUNCH / 2), A.LAUNCH, A.LAUNCH,
                                   color="#fab005"))
        xs, ys = [t[0] for t in track], [t[1] for t in track]
        ax.plot(xs, ys, color="#1c7ed6", lw=1.2, label="flown track")
        ax.plot(*worst[1][:2], "x", color="#e8590c", ms=10, label=f"closest to wall ({clearance:+.2f} m)")
        ax.set_xlim(-0.5, 15.5)
        ax.set_ylim(-1.5, 15.5)
        ax.set_aspect("equal")
        ax.grid(color="#adb5bd", lw=0.4)
        ax.set_xticks(range(16))
        ax.set_yticks(range(16))
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title("arena_demo: flown track (PX4, no GPS)", fontsize=10)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print("saved", a.plot)
    sys.exit(0 if (ok1 and ok2 and ok3) else 1)


if __name__ == "__main__":
    main()
