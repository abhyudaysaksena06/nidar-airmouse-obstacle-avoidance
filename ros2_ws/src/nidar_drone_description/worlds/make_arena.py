#!/usr/bin/env python3
"""Generate a NIDAR AirMouse practice arena (Rulebook v2.2, PS2, Annexure 5).

    python3 make_arena.py                 # writes nidar_arena.sdf + nidar_arena_top.png
    python3 make_arena.py --layout my.txt # your own layout (same text format)

Rules modelled (PS2 sections 3 and 9):
  * arena <= 15 m x 15 m on a 1 m x 1 m grid, netted roof
  * corridors >= 1 m clear width, clear height 8 ft (2.44 m)
  * rooms 2 m x 2 m
  * ONE entry/exit point, with the 2 ft x 2 ft launch area just outside it
  * up to 6 survivors (dummies) in rooms
The real layout is secret until the event: this is one plausible example. Edit
LAYOUT (or pass --layout) to practise on others.

World frame (Gazebo, ENU): x = east, y = north, origin = SW corner of the arena,
arena covers x 0..15, y 0..15. Cell (c, r) is the 1 m square x c..c+1, y r..r+1.
"""
import argparse
import os
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
CEILING = 2.44           # 8 ft
WALL_T = 0.05            # wall thickness (on the grid lines)
LAUNCH = 0.61            # 2 ft

# Top row of text = north (y = 14). '#' = solid, '.' = corridor,
# a letter = a 2x2 room (same letter = same room). 'E' = entry cell (south edge).
LAYOUT = """
###aa####bb####
###aa####bb####
##...........##
##.#####.###.##
cc.#####.###.dd
cc.......###.dd
##.#####.###.##
##.#####.......
ee.#####.###ff.
ee.......###ff.
##.#####.###.##
##.#####.###.##
##...........##
#######.#######
#######E#######
"""
# Door of each room: (room cell, neighbouring corridor cell), both (col, row)
DOORS = {"a": ((3, 13), (3, 12)), "b": ((9, 13), (9, 12)), "c": ((1, 9), (2, 9)),
         "d": ((13, 9), (12, 9)), "e": ((1, 5), (2, 5)), "f": ((12, 6), (12, 7))}
SURVIVOR_ROOMS = ["a", "b", "c", "d", "e", "f"]


def parse(text):
    rows = [r for r in text.strip("\n").splitlines()]
    assert len(rows) == 15 and all(len(r) == 15 for r in rows), "layout must be 15 x 15"
    grid = {}
    for i, row in enumerate(rows):
        r = 14 - i
        for c, ch in enumerate(row):
            grid[(c, r)] = ch
    return grid


def region(ch):
    if ch == "#":
        return None
    if ch in ".E":
        return "corridor"
    return ch


def check(grid):
    """Every open cell must be reachable from the entry, and rooms must be 2x2."""
    entry = [k for k, v in grid.items() if v == "E"]
    assert len(entry) == 1 and entry[0][1] == 0, "exactly one entry 'E' on the south edge"
    rooms = {}
    for k, v in grid.items():
        if region(v) not in (None, "corridor"):
            rooms.setdefault(v, []).append(k)
    for name, cells in rooms.items():
        cs, rs = {c for c, _ in cells}, {r for _, r in cells}
        assert len(cells) == 4 and len(cs) == 2 and len(rs) == 2, f"room {name} must be 2 m x 2 m"
        assert name in DOORS, f"room {name} has no door"
    seen, q = {entry[0]}, deque([entry[0]])
    while q:
        c, r = q.popleft()
        for n in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if n in grid and n not in seen and not wall_between(grid, (c, r), n):
                seen.add(n)
                q.append(n)
    open_cells = {k for k, v in grid.items() if region(v)}
    missing = open_cells - seen
    assert not missing, f"cells not reachable from the entry: {sorted(missing)}"
    return entry[0], rooms


def wall_between(grid, a, b):
    ra, rb = region(grid.get(a, "#")), region(grid.get(b, "#"))
    if ra is None or rb is None:
        return True
    if ra == rb:
        return False
    for room, (rc, cc) in DOORS.items():
        if {a, b} == {rc, cc}:
            return False
    return True


def walls(grid, entry):
    """Wall segments on grid lines: list of (x0, y0, x1, y1)."""
    segs = []
    for c in range(16):          # vertical lines x = c
        for r in range(15):
            left, right = (c - 1, r), (c, r)
            lo, ro = region(grid.get(left, "#")), region(grid.get(right, "#"))
            if (lo or ro) and wall_between(grid, left, right):
                segs.append((c, r, c, r + 1))
    for r in range(16):          # horizontal lines y = r
        for c in range(15):
            below, above = (c, r - 1), (c, r)
            if (c, r) == entry and r == 0:
                continue             # the entry opening in the south wall
            bo, ao = region(grid.get(below, "#")), region(grid.get(above, "#"))
            if (bo or ao) and wall_between(grid, below, above):
                segs.append((c, r, c + 1, r))
    return merge(segs)


def merge(segs):
    """Join collinear touching unit segments into long walls (fewer models)."""
    out, segs = [], sorted(set(segs))
    used = set()
    for s in segs:
        if s in used:
            continue
        x0, y0, x1, y1 = s
        used.add(s)
        while True:
            nxt = (x1, y1, x1 + (x1 - x0 > 0), y1 + (y1 - y0 > 0)) if False else (
                (x1, y1, x1 + 1, y1) if y0 == y1 else (x1, y1, x1, y1 + 1))
            if nxt in segs and nxt not in used:
                used.add(nxt)
                x1, y1 = nxt[2], nxt[3]
            else:
                break
        out.append((x0, y0, x1, y1))
    return out


def box(name, x, y, z, sx, sy, sz, rgba, collide=True, static=True, yaw=0.0):
    col = (f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
           if collide else "")
    return (f'<model name="{name}"><static>{str(static).lower()}</static><pose>{x} {y} {z} 0 0 {yaw}</pose>'
            f'<link name="l">{col}<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>\n')


def survivor(name, x, y):
    """Lying dummy: body cylinder + head sphere, warm colour (camera-visible)."""
    return f'''<model name="{name}"><static>true</static><pose>{x} {y} 0.12 0 1.5708 0.6</pose>
  <link name="l">
    <collision name="c"><geometry><cylinder><radius>0.15</radius><length>1.1</length></cylinder></geometry></collision>
    <visual name="body"><geometry><cylinder><radius>0.15</radius><length>1.1</length></cylinder></geometry>
      <material><ambient>0.85 0.35 0.2 1</ambient><diffuse>0.85 0.35 0.2 1</diffuse></material></visual>
    <visual name="head"><pose>0 0 0.68 0 0 0</pose><geometry><sphere><radius>0.12</radius></sphere></geometry>
      <material><ambient>0.95 0.75 0.6 1</ambient><diffuse>0.95 0.75 0.6 1</diffuse></material></visual>
  </link></model>
'''


def world(grid, entry, rooms, name="nidar_arena"):
    segs = walls(grid, entry)
    parts = []
    for i, (x0, y0, x1, y1) in enumerate(segs):
        horiz = y0 == y1
        length = (x1 - x0 if horiz else y1 - y0) + WALL_T
        parts.append(box(f"wall_{i}", (x0 + x1) / 2, (y0 + y1) / 2, CEILING / 2,
                         length if horiz else WALL_T, WALL_T if horiz else length, CEILING,
                         "0.72 0.70 0.66 1"))
    # netted roof: a thin see-through sheet the drone must not touch
    parts.append(box("roof_net", 7.5, 7.5, CEILING + 0.01, 15.0, 15.0, 0.02, "0.2 0.2 0.2 0.25"))
    # 1 m reference grid painted on the floor (what the judges score against)
    for k in range(16):
        parts.append(box(f"grid_x{k}", k, 7.5, 0.001, 0.02, 15.0, 0.002, "0.25 0.25 0.25 1", collide=False))
        parts.append(box(f"grid_y{k}", 7.5, k, 0.001, 15.0, 0.02, 0.002, "0.25 0.25 0.25 1", collide=False))
    # 2 ft x 2 ft launch / landing area just outside the entry
    lx, ly = entry[0] + 0.5, -0.5
    parts.append(box("launch_area", lx, ly, 0.002, LAUNCH, LAUNCH, 0.004, "0.95 0.8 0.1 1", collide=False))
    for i, rn in enumerate(SURVIVOR_ROOMS):
        cells = rooms.get(rn)
        if not cells:
            continue
        cx = sum(c for c, _ in cells) / 4 + 0.5
        cy = sum(r for _, r in cells) / 4 + 0.5
        parts.append(survivor(f"survivor_{i + 1}", cx, cy))
    return f'''<?xml version="1.0"?>
<!-- GENERATED by make_arena.py: NIDAR AirMouse practice arena (Rulebook v2.2, PS2).
     Arena x 0..15 (east), y 0..15 (north); entry/exit at cell ({entry[0]}, 0);
     2 ft x 2 ft launch area centred at ({lx}, {ly}). -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>30.3564</latitude_deg><longitude_deg>76.3647</longitude_deg><elevation>250</elevation></spherical_coordinates>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.75 0.8 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>7.5 7.5 20 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.55 0.55 0.52 1</ambient><diffuse>0.55 0.55 0.52 1</diffuse></material></visual></link></model>
{"".join(parts)}  </world>
</sdf>
'''


def top_png(grid, entry, rooms, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7.6))
    for (c, r), v in grid.items():
        if region(v) is None:
            ax.add_patch(plt.Rectangle((c, r), 1, 1, color="#4a4a4a"))
        elif region(v) != "corridor":
            ax.add_patch(plt.Rectangle((c, r), 1, 1, color="#ffe8cc"))
    for x0, y0, x1, y1 in walls(grid, entry):
        ax.plot([x0, x1], [y0, y1], color="#c92a2a", lw=2.5)
    for i, rn in enumerate(SURVIVOR_ROOMS):
        cells = rooms.get(rn)
        if cells:
            cx = sum(c for c, _ in cells) / 4 + 0.5
            cy = sum(r for _, r in cells) / 4 + 0.5
            ax.plot(cx, cy, "o", ms=11, color="#e8590c")
            ax.annotate(f"S{i + 1}", (cx, cy), ha="center", va="center", fontsize=7, color="white")
    lx = entry[0] + 0.5
    ax.add_patch(plt.Rectangle((lx - LAUNCH / 2, -0.5 - LAUNCH / 2), LAUNCH, LAUNCH, color="#fab005"))
    ax.annotate("launch 2x2 ft\nentry / exit", (lx, -0.5), xytext=(lx + 1.2, -1.2), fontsize=8,
                arrowprops=dict(arrowstyle="->"))
    ax.set_xticks(range(16))
    ax.set_yticks(range(16))
    ax.grid(color="#adb5bd", lw=0.5)
    ax.set_xlim(-0.5, 15.5)
    ax.set_ylim(-1.8, 15.5)
    ax.set_aspect("equal")
    ax.set_title("NIDAR AirMouse practice arena (15 m x 15 m, 1 m grid)\n"
                 "red = walls (2.44 m), cream = 2x2 m rooms, S = survivor", fontsize=10)
    ax.set_xlabel("x / east [m]")
    ax.set_ylabel("y / north [m]")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", help="text file with a 15x15 layout")
    ap.add_argument("--out", default=os.path.join(HERE, "nidar_arena.sdf"))
    ap.add_argument("--png", default=os.path.join(HERE, "nidar_arena_top.png"))
    a = ap.parse_args()
    grid = parse(open(a.layout).read() if a.layout else LAYOUT)
    entry, rooms = check(grid)
    with open(a.out, "w") as fh:
        fh.write(world(grid, entry, rooms))
    print("wrote", a.out, f"({len(walls(grid, entry))} walls, {len(rooms)} rooms, entry cell {entry})")
    try:
        top_png(grid, entry, rooms, a.png)
        print("wrote", a.png)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
