"""Random AirMouse-style maze generator (2.5D).

World frame: x right, y up, z up, metres. The arena (maze) sits inside an
outside margin so the drone starts OUTSIDE and must enter through the
single designated entry/exit opening, as in the rulebook.

Every obstacle cell stores a height interval [zlo, zhi]: walls span floor to
ceiling, boxes stand on the floor, hanging beams/pipes come down from the
ceiling. A 2D lidar only sees what crosses its scan plane.
"""
import math
import random
from collections import deque
from dataclasses import dataclass, field

import numpy as np

RES = 0.05  # occupancy grid resolution (m)


@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float
    kind: str = "wall"   # wall | fence | box | hanging
    z0: float = 0.0
    z1: float = 2.5


@dataclass
class Cylinder:
    x: float
    y: float
    r: float
    z1: float = 1.5


@dataclass
class Survivor:
    id: int
    x: float
    y: float
    yaw: float  # body orientation of a lying person


@dataclass
class MovingObstacle:
    x: float
    y: float
    r: float
    vx: float
    vy: float
    h: float = 1.8  # a walking person / rolling debris


@dataclass
class World:
    cfg: dict
    seed: int
    width: float          # full world incl. margin
    height: float
    ceiling: float
    arena: tuple          # (x0, y0, x1, y1) of the maze
    rects: list = field(default_factory=list)
    cylinders: list = field(default_factory=list)
    movers: list = field(default_factory=list)
    survivors: list = field(default_factory=list)
    dark_zones: list = field(default_factory=list)      # [(x0,y0,x1,y1)] unlit rooms
    texture_zones: list = field(default_factory=list)   # [(x0,y0,x1,y1)] featureless floor
    start: tuple = (0.0, 0.0, 0.0)  # x, y, yaw (outside, facing entry)
    entry: tuple = (0.0, 0.0)       # centre of the entry opening
    zlo: np.ndarray = None          # per-cell obstacle bottom (inf = free)
    zhi: np.ndarray = None          # per-cell obstacle top
    walls: np.ndarray = None        # walls/fence only (for radio link)
    _occ_cache: dict = field(default_factory=dict)

    def __post_init__(self):
        self.grid = np.isfinite(self.zlo)  # obstacles at any height (footprint)

    def occ_at(self, z, half=0.0):
        """Cells blocked somewhere in the height band [z-half, z+half]."""
        key = (round(z, 2), round(half, 2))
        g = self._occ_cache.get(key)
        if g is None:
            g = (self.zlo <= z + half) & (self.zhi >= z - half)
            if len(self._occ_cache) > 200:
                self._occ_cache.clear()
            self._occ_cache[key] = g
        return g

    def surface_below(self, x, y, z):
        """Height of the floor or obstacle top directly under (x, y)."""
        iy, ix = int(y / RES), int(x / RES)
        if 0 <= iy < self.zlo.shape[0] and 0 <= ix < self.zlo.shape[1]:
            if self.zlo[iy, ix] <= 0.01 and self.zhi[iy, ix] <= z:
                return float(self.zhi[iy, ix])
        return 0.0

    def surface_above(self, x, y, z):
        iy, ix = int(y / RES), int(x / RES)
        if 0 <= iy < self.zlo.shape[0] and 0 <= ix < self.zlo.shape[1]:
            lo = self.zlo[iy, ix]
            if np.isfinite(lo) and lo >= z:
                return float(lo)
        return self.ceiling

    @staticmethod
    def _in(zones, x, y):
        return any(a <= x <= c and b <= y <= d for a, b, c, d in zones)

    def is_dark(self, x, y):
        return self._in(self.dark_zones, x, y)

    def is_textureless(self, x, y):
        return self._in(self.texture_zones, x, y)

    def step_movers(self, dt):
        g = self.grid
        for m in self.movers:
            nx, ny = m.x + m.vx * dt, m.y + m.vy * dt
            if _circle_hits(g, nx, m.y, m.r):
                m.vx = -m.vx
                nx = m.x
            if _circle_hits(g, m.x, ny, m.r):
                m.vy = -m.vy
                ny = m.y
            m.x, m.y = nx, ny


# ---------------------------------------------------------------------------
def _circle_hits(grid, x, y, r):
    i0, i1 = int((y - r) / RES), int((y + r) / RES)
    j0, j1 = int((x - r) / RES), int((x + r) / RES)
    if i0 < 0 or j0 < 0 or i1 >= grid.shape[0] or j1 >= grid.shape[1]:
        return True
    sub = grid[i0:i1 + 1, j0:j1 + 1]
    if not sub.any():
        return False
    ii, jj = np.nonzero(sub)
    cy, cx = (ii + i0 + 0.5) * RES, (jj + j0 + 0.5) * RES
    return bool(((cx - x) ** 2 + (cy - y) ** 2 <= (r + RES * 0.71) ** 2).any())


def _stamp(zlo, zhi, rects, cylinders):
    """Write obstacles into the height-interval grids (union per cell)."""
    for r in rects:
        i0, i1 = max(int(r.y0 / RES), 0), int(math.ceil(r.y1 / RES))
        j0, j1 = max(int(r.x0 / RES), 0), int(math.ceil(r.x1 / RES))
        zlo[i0:i1, j0:j1] = np.minimum(zlo[i0:i1, j0:j1], r.z0)
        zhi[i0:i1, j0:j1] = np.maximum(zhi[i0:i1, j0:j1], r.z1)
    for c in cylinders:
        i0, i1 = int((c.y - c.r) / RES), int((c.y + c.r) / RES) + 1
        j0, j1 = int((c.x - c.r) / RES), int((c.x + c.r) / RES) + 1
        ys = (np.arange(i0, i1) + 0.5) * RES
        xs = (np.arange(j0, j1) + 0.5) * RES
        yy, xx = np.meshgrid(ys, xs, indexing="ij")
        m = (xx - c.x) ** 2 + (yy - c.y) ** 2 <= c.r ** 2
        zlo[i0:i1, j0:j1] = np.where(m, np.minimum(zlo[i0:i1, j0:j1], 0.0), zlo[i0:i1, j0:j1])
        zhi[i0:i1, j0:j1] = np.where(m, np.maximum(zhi[i0:i1, j0:j1], c.z1), zhi[i0:i1, j0:j1])


def inflate(grid, radius_cells):
    """Dilate a boolean grid by a disk of radius_cells."""
    out = grid.copy()
    r = int(math.ceil(radius_cells))
    H, W = grid.shape
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > radius_cells * radius_cells or (di == 0 and dj == 0):
                continue
            src = grid[max(0, -di):H - max(0, di), max(0, -dj):W - max(0, dj)]
            out[max(0, di):H - max(0, -di), max(0, dj):W - max(0, -dj)] |= src
    return out


def _reachable(blocked_fine, start_xy, drone_r):
    """BFS on a 0.1 m grid, obstacles inflated by the drone radius."""
    g = blocked_fine[: blocked_fine.shape[0] // 2 * 2, : blocked_fine.shape[1] // 2 * 2]
    coarse = g.reshape(g.shape[0] // 2, 2, g.shape[1] // 2, 2).any(axis=(1, 3))
    blocked = inflate(coarse, (drone_r + 0.05) / (RES * 2))
    H, W = blocked.shape
    si, sj = int(start_xy[1] / (RES * 2)), int(start_xy[0] / (RES * 2))
    seen = np.zeros_like(blocked)
    if blocked[si, sj]:
        return seen
    q = deque([(si, sj)])
    seen[si, sj] = True
    while q:
        i, j = q.popleft()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < H and 0 <= b < W and not seen[a, b] and not blocked[a, b]:
                seen[a, b] = True
                q.append((a, b))
    return seen  # 0.1 m resolution


# ---------------------------------------------------------------------------
def generate_world(cfg, seed=None):
    if seed is None:
        seed = random.randrange(1_000_000)
    rng = random.Random(seed)
    A, O, S, D = cfg["arena"], cfg["obstacles"], cfg["survivors"], cfg["drone"]

    C = A["cell_m"]
    nx, ny = max(2, int(A["width_m"] // C)), max(2, int(A["height_m"] // C))
    m = A["outside_margin_m"]
    ax0, ay0 = m, m
    ax1, ay1 = m + nx * C, m + ny * C
    W, H = ax1 + m, ay1 + m
    t = A["wall_thickness_m"]
    ceil_h = A["wall_height_m"]

    # wall state: 1 solid, 0 door (partial opening), 2 fully open, 3 entry
    vw = [[1] * (nx + 1) for _ in range(ny)]   # vertical line x=i*C, row j
    hw = [[1] * nx for _ in range(ny + 1)]     # horizontal line y=j*C, col i

    # 1) perfect maze by randomized DFS -> corridors, turns, dead ends
    seen = [[False] * nx for _ in range(ny)]
    stack = [(rng.randrange(nx), rng.randrange(ny))]
    seen[stack[0][1]][stack[0][0]] = True
    while stack:
        i, j = stack[-1]
        nbrs = [(a, b) for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1))
                if 0 <= a < nx and 0 <= b < ny and not seen[b][a]]
        if not nbrs:
            stack.pop()
            continue
        a, b = rng.choice(nbrs)
        if a != i:
            vw[j][max(a, i)] = 0
        else:
            hw[max(b, j)][i] = 0
        seen[b][a] = True
        stack.append((a, b))

    # 2) merge blocks of cells into rooms
    rooms = []
    for _ in range(A["rooms"]):
        rw = min(rng.randint(A["room_min_cells"], A["room_max_cells"]), nx)
        rh = min(rng.randint(A["room_min_cells"], A["room_max_cells"]), ny)
        i0, j0 = rng.randint(0, nx - rw), rng.randint(0, ny - rh)
        rooms.append((i0, j0, rw, rh))
        for j in range(j0, j0 + rh):
            for i in range(i0 + 1, i0 + rw):
                vw[j][i] = 2
        for j in range(j0 + 1, j0 + rh):
            for i in range(i0, i0 + rw):
                hw[j][i] = 2

    # 3) extra doors -> loops
    for j in range(ny):
        for i in range(1, nx):
            if vw[j][i] == 1 and rng.random() < A["loop_prob"]:
                vw[j][i] = 0
    for j in range(1, ny):
        for i in range(nx):
            if hw[j][i] == 1 and rng.random() < A["loop_prob"]:
                hw[j][i] = 0

    # 4) entry/exit opening on the outer boundary
    side = A["entry_side"]
    if side == "random":
        side = rng.choice(["south", "north", "west", "east"])
    if side in ("south", "north"):
        hw[0 if side == "south" else ny][rng.randrange(nx)] = 3
    else:
        vw[rng.randrange(ny)][0 if side == "west" else nx] = 3

    rects = []
    uniform_w = rng.uniform(A["door_min_m"], A["door_max_m"])

    def seg(x0, y0, x1, y1, state, length):
        """Wall segment along a cell edge, with an opening for doors."""
        if state == 2:
            return None
        horizontal = y0 == y1
        if state == 1:
            w, off = 0.0, 0.0
        elif state == 3:
            w = A["entry_width_m"]
            off = (length - w) / 2
        else:
            w = uniform_w if A.get("door_uniform") else \
                rng.uniform(A["door_min_m"], min(A["door_max_m"], length - 0.4))
            w = min(w, length - 0.4)
            off = (length - w) / 2 if A.get("door_centered") else rng.uniform(0.2, length - 0.2 - w)
        pieces = [(0.0, length)] if w == 0 else [(0.0, off), (off + w, length)]
        for p0, p1 in pieces:
            if p1 - p0 < 0.05:
                continue
            if horizontal:
                rects.append(Rect(x0 + p0 - t / 2, y0 - t / 2, x0 + p1 + t / 2, y0 + t / 2, "wall", 0, ceil_h))
            else:
                rects.append(Rect(x0 - t / 2, y0 + p0 - t / 2, x0 + t / 2, y0 + p1 + t / 2, "wall", 0, ceil_h))
        return off + w / 2 if w else None

    entry = None
    for j in range(ny):
        for i in range(nx + 1):
            c = seg(ax0 + i * C, ay0 + j * C, ax0 + i * C, ay0 + (j + 1) * C, vw[j][i], C)
            if vw[j][i] == 3:
                entry = (ax0 + i * C, ay0 + j * C + c)
    for j in range(ny + 1):
        for i in range(nx):
            c = seg(ax0 + i * C, ay0 + j * C, ax0 + (i + 1) * C, ay0 + j * C, hw[j][i], C)
            if hw[j][i] == 3:
                entry = (ax0 + i * C + c, ay0 + j * C)

    f = 0.1  # venue fence around the outside margin
    rects += [Rect(0, 0, W, f, "fence", 0, ceil_h), Rect(0, H - f, W, H, "fence", 0, ceil_h),
              Rect(0, 0, f, H, "fence", 0, ceil_h), Rect(W - f, 0, W, H, "fence", 0, ceil_h)]

    so = D["start_outside_m"]
    ex, ey = entry
    start = {"south": (ex, ey - so, math.pi / 2), "north": (ex, ey + so, -math.pi / 2),
             "west": (ex - so, ey, 0.0), "east": (ex + so, ey, math.pi)}[side]

    shape = (int(math.ceil(H / RES)), int(math.ceil(W / RES)))
    zlo = np.full(shape, np.inf)
    zhi = np.full(shape, -np.inf)
    _stamp(zlo, zhi, rects, [])
    walls = np.isfinite(zlo)

    cruise = D["cruise_alt_m"]
    band = 0.25  # the drone must be able to pass at cruise altitude +-band

    def blocked_at_cruise(zl, zh):
        return (zl <= cruise + band) & (zh >= cruise - band)

    def cell_counts(reach):
        counts = np.zeros((ny, nx))
        r = RES * 2
        for j in range(ny):
            for i in range(nx):
                a0, a1 = int((ay0 + j * C) / r), int((ay0 + (j + 1) * C) / r)
                b0, b1 = int((ax0 + i * C) / r), int((ax0 + (i + 1) * C) / r)
                counts[j, i] = reach[a0:a1, b0:b1].sum()
        return counts

    base = cell_counts(_reachable(walls, start[:2], D["radius_m"]))

    def try_place(new_r, new_c):
        nonlocal zlo, zhi
        zl2, zh2 = zlo.copy(), zhi.copy()
        _stamp(zl2, zh2, new_r, new_c)
        fp_new = np.isfinite(zl2).sum() - np.isfinite(zlo).sum()
        if fp_new < 4:  # overlapped a wall entirely
            return False
        counts = cell_counts(_reachable(blocked_at_cruise(zl2, zh2), start[:2], D["radius_m"]))
        if np.any((base > 0) & (counts < 0.5 * base)):
            return False
        zlo, zhi = zl2, zh2
        return True

    def rand_pt(pad=0.4):
        return rng.uniform(ax0 + pad, ax1 - pad), rng.uniform(ay0 + pad, ay1 - pad)

    # 5) static floor obstacles (boxes, pillars) with random heights
    cylinders, obstacles = [], []
    placed = tries = 0
    while placed < O["static_count"] and tries < O["static_count"] * 30:
        tries += 1
        x, y = rand_pt()
        if math.hypot(x - ex, y - ey) < O["keep_clear_of_entry_m"]:
            continue
        h = rng.uniform(*O["box_height_m"])
        if rng.random() < 0.5:
            sx, sy = rng.uniform(*O["box_size_m"]), rng.uniform(*O["box_size_m"])
            nr, nc = [Rect(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2, "box", 0.0, h)], []
        else:
            nr, nc = [], [Cylinder(x, y, rng.uniform(*O["cylinder_radius_m"]), h)]
        if try_place(nr, nc):
            rects += nr
            cylinders += nc
            obstacles.append((x, y))
            placed += 1

    # 6) hanging obstacles (beams, pipes, cable trays) from the ceiling
    placed = tries = 0
    while placed < O["hanging_count"] and tries < O["hanging_count"] * 30:
        tries += 1
        x, y = rand_pt()
        if math.hypot(x - ex, y - ey) < O["keep_clear_of_entry_m"]:
            continue
        L, T = rng.uniform(*O["hanging_length_m"]), rng.uniform(*O["hanging_thickness_m"])
        sx, sy = (L, T) if rng.random() < 0.5 else (T, L)
        bottom = rng.uniform(*O["hanging_bottom_m"])
        nr = [Rect(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2, "hanging", bottom, ceil_h)]
        if try_place(nr, []):
            rects += nr
            obstacles.append((x, y))
            placed += 1

    reach = _reachable(blocked_at_cruise(zlo, zhi), start[:2], D["radius_m"])
    floor_occ = np.isfinite(zlo) & (zlo <= 0.01)

    def reachable_pt(x, y):
        return bool(reach[int(y / (RES * 2)), int(x / (RES * 2))])

    def floor_free(x, y, r=0.35):
        i0, i1 = int((y - r) / RES), int((y + r) / RES) + 1
        j0, j1 = int((x - r) / RES), int((x + r) / RES) + 1
        return not floor_occ[i0:i1, j0:j1].any()

    def sample(cond, n_try=3000, near=None):
        for _ in range(n_try):
            if near:
                a, d = rng.uniform(-math.pi, math.pi), rng.uniform(0.5, 0.9)
                x, y = near[0] + d * math.cos(a), near[1] + d * math.sin(a)
                if not (ax0 + 0.3 < x < ax1 - 0.3 and ay0 + 0.3 < y < ay1 - 0.3):
                    continue
            else:
                x, y = rand_pt(0.3)
            if reachable_pt(x, y) and floor_free(x, y) and cond(x, y):
                return x, y
        return None

    # 7) survivors (up to six, rulebook); some tucked next to obstacles
    n_surv = rng.randint(S["min"], S["max"])
    survivors = []
    for k in range(n_surv):
        def ok(x, y):
            return math.hypot(x - ex, y - ey) > S["min_dist_from_entry_m"] and \
                all(math.hypot(x - s.x, y - s.y) > S["min_separation_m"] for s in survivors)
        near = rng.choice(obstacles) if obstacles and rng.random() < S["near_obstacle_prob"] else None
        p = sample(ok, near=near) or sample(ok)
        if p:
            survivors.append(Survivor(k + 1, p[0], p[1], rng.uniform(-math.pi, math.pi)))

    # 8) moving obstacles for avoidance testing
    movers = []
    for _ in range(O["moving_count"]):
        p = sample(lambda x, y: math.hypot(x - ex, y - ey) > 4.0)
        if p:
            a = rng.uniform(-math.pi, math.pi)
            sp = O["moving_speed_mps"]
            movers.append(MovingObstacle(p[0], p[1], O["moving_radius_m"],
                                         sp * math.cos(a), sp * math.sin(a), O["moving_height_m"]))

    # 9) environment: unlit rooms and featureless floor patches
    cells = [(i, j) for i in range(nx) for j in range(ny)]
    rng.shuffle(cells)
    n_dark = int(round(A["dark_cell_frac"] * len(cells)))
    dark = [(ax0 + i * C, ay0 + j * C, ax0 + (i + 1) * C, ay0 + (j + 1) * C) for i, j in cells[:n_dark]]
    tex = []
    for _ in range(A["textureless_zones"]):
        x, y = rand_pt(1.0)
        sx, sy = rng.uniform(1.0, 2.5), rng.uniform(1.0, 2.5)
        tex.append((x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2))

    return World(cfg=cfg, seed=seed, width=W, height=H, ceiling=ceil_h, arena=(ax0, ay0, ax1, ay1),
                 rects=rects, cylinders=cylinders, movers=movers, survivors=survivors,
                 dark_zones=dark, texture_zones=tex, start=start, entry=entry,
                 zlo=zlo, zhi=zhi, walls=walls)
