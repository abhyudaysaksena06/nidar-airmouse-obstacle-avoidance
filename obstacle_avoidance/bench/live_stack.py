"""Watch OA-Stack fly, live (pygame window).

    python bench/live_stack.py                                   # mixed_clutter, seeds 207, 208, ...
    python bench/live_stack.py --scenario guide_people --seed 213
    python bench/live_stack.py --method guide                    # watch the guide's baseline instead

Left: the maze from above - walls, low boxes (orange), ducts (blue outline), pillars, people (pink),
lidar rays, hazards OA-Stack currently remembers (orange = low, blue = duct, red = in-band),
the planner's path, and the flight trail coloured by height.
Right: status, flags, height (actual vs OA-Stack's target), speed, clearance, compute time.

Keys: SPACE pause   +/- speed   N next world   L lidar rays   S screenshot   ESC quit
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_stack as B  # noqa: E402
from oa_stack import StackConfig  # noqa: E402
from sim.engine import Simulation  # noqa: E402

WIN_W, WIN_H, PANEL = 1440, 860, 440
BG, FLOOR, WALL = (232, 230, 225), (250, 249, 246), (55, 58, 64)
BOX, DUCT, PILLAR, PERSON = (230, 140, 60), (70, 110, 200), (140, 95, 50), (215, 70, 150)
OK, WARN, BAD, GREY = (40, 150, 80), (210, 140, 20), (200, 50, 50), (110, 110, 110)
HAZ = {0: DUCT, 1: BOX, 2: BAD}


def height_colour(z):
    t = float(np.clip((z - 0.8) / 1.2, 0, 1))
    return (int(60 + 190 * t), int(80 + 150 * t), int(170 - 140 * t))


class Run:
    """One trial, stepped by the viewer. Same setup as bench_stack.run_trial."""

    def __init__(self, scenario, seed, method):
        cfg, tof_range = B.build_cfg(scenario)
        tr = None
        while tr is None:
            tr = B.make_trial(cfg, seed)
            if tr is None:
                seed += 1
        self.world, self.s, self.g, path = tr
        self.scenario, self.seed, self.method = scenario, seed, method
        self.radius = cfg["drone"]["radius_m"]
        self.cruise = cfg["drone"]["cruise_alt_m"]
        scfg = StackConfig(drone_radius=self.radius, tof_max_range=tof_range)
        self.pilot = B.StackPilot(method, list(path), self.g, scfg, tof_range, seed)
        self.sim = Simulation(self.world, self.pilot, seed=seed)
        self.pilot.sim = self.sim
        d = self.sim.drone
        d.x, d.y, d.z, d.landed = self.s[0], self.s[1], self.cruise, False
        d.yaw = math.atan2(path[min(5, len(path) - 1)][1] - self.s[1], path[min(5, len(path) - 1)][0] - self.s[0])
        self.limit = 25.0 + B.path_length(path) / 0.8 * 2.5
        self.last = None
        orig = self.pilot.alg.update

        def record(*a, **k):
            self.last = orig(*a, **k)
            return self.last
        self.pilot.alg.update = record
        self.trail, self.alts, self.targets = [], [], []
        self.min_clear, self.result = math.inf, None

    def step(self):
        if self.result:
            return
        sim, d = self.sim, self.sim.drone
        sim.step()
        self.trail.append((d.x, d.y, d.z))
        self.alts.append(d.z)
        zt = getattr(self.last, "z_target", None) if self.last is not None and not isinstance(self.last, dict) \
            else (self.last or {}).get("z_target")
        self.targets.append(zt if zt is not None else self.cruise)
        self.min_clear = min(self.min_clear, float(np.min(sim.last_ranges)) - self.radius)
        if sim.crashed:
            who = "person" if "moving" in sim.end_reason else B.crash_kind(self.world, d.x, d.y)
            self.result = "COLLISION: " + who
        elif math.hypot(d.x - self.g[0], d.y - self.g[1]) < 0.5:
            self.result = "SUCCESS"
        elif sim.t >= self.limit:
            self.result = "TIMEOUT"


class Viewer:
    def __init__(self, args):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("OA-Stack live simulation")
        self.font = pygame.font.SysFont("consolas", 15)
        self.big = pygame.font.SysFont("consolas", 19, bold=True)
        self.clock = pygame.time.Clock()
        self.args, self.speed, self.paused, self.rays = args, 1, False, True
        self.seed = args.seed
        self.new_run()

    # --- setup ------------------------------------------------------------------------
    def new_run(self):
        self.run = Run(self.args.scenario, self.seed, self.args.method)
        self.seed = self.run.seed + 1
        self.done_at = None
        x0, y0, x1, y1 = self.run.world.arena
        m = 0.4
        self.view = (x0 - m, y0 - m, x1 + m, y1 + m)
        vw, vh = WIN_W - PANEL - 20, WIN_H - 20
        self.k = min(vw / (self.view[2] - self.view[0]), vh / (self.view[3] - self.view[1]))
        self.static = self.draw_static()

    def px(self, x, y):
        return (int(10 + (x - self.view[0]) * self.k), int(10 + (self.view[3] - y) * self.k))

    def rect(self, surf, col, x0, y0, x1, y1, width=0):
        a, b = self.px(x0, y1), self.px(x1, y0)
        pygame.draw.rect(surf, col, (a[0], a[1], max(1, b[0] - a[0]), max(1, b[1] - a[1])), width)

    def draw_static(self):
        w = self.run.world
        surf = pygame.Surface((WIN_W - PANEL, WIN_H))
        surf.fill(BG)
        self.rect(surf, FLOOR, *self.view)
        for gx in range(int(math.ceil(self.view[0])), int(self.view[2]) + 1):
            pygame.draw.line(surf, (236, 234, 228), self.px(gx, self.view[1]), self.px(gx, self.view[3]))
        for gy in range(int(math.ceil(self.view[1])), int(self.view[3]) + 1):
            pygame.draw.line(surf, (236, 234, 228), self.px(self.view[0], gy), self.px(self.view[2], gy))
        for r in w.rects:
            if r.x1 < self.view[0] or r.x0 > self.view[2] or r.y1 < self.view[1] or r.y0 > self.view[3]:
                continue
            if r.kind == "box":
                self.rect(surf, BOX, r.x0, r.y0, r.x1, r.y1)
            elif r.kind != "hanging":
                self.rect(surf, WALL, r.x0, r.y0, r.x1, r.y1)
        for r in w.rects:
            if r.kind == "hanging":
                self.rect(surf, DUCT, r.x0, r.y0, r.x1, r.y1, 3)
        for c in w.cylinders:
            pygame.draw.circle(surf, PILLAR, self.px(c.x, c.y), max(3, int(c.r * self.k)))
        for p, col in ((self.run.s, OK), (self.run.g, (40, 110, 220))):
            pygame.draw.circle(surf, col, self.px(*p), 9)
            pygame.draw.circle(surf, (255, 255, 255), self.px(*p), 9, 2)
        return surf

    # --- drawing ----------------------------------------------------------------------
    def text(self, t, pos, col=(30, 30, 30), big=False):
        self.screen.blit((self.big if big else self.font).render(t, True, col), pos)

    def draw(self):
        run, scr = self.run, self.screen
        sim, d = run.sim, run.sim.drone
        scr.blit(self.static, (0, 0))
        path = run.pilot.path
        if len(path) > 1:
            pygame.draw.lines(scr, (175, 175, 190), False, [self.px(*p) for p in path], 1)
        if self.rays:
            c = self.px(d.x, d.y)
            for a, r in zip(sim.lidar_angles[::5], sim.last_ranges[::5]):
                r = min(r, 6.0)
                pygame.draw.line(scr, (255, 200, 200), c, self.px(d.x + r * math.cos(a + d.yaw), d.y + r * math.sin(a + d.yaw)))
        for m in run.world.movers:
            pygame.draw.circle(scr, PERSON, self.px(m.x, m.y), max(4, int(m.r * self.k)))
        haz = getattr(run.pilot.alg, "hazards", None)
        if haz is not None and len(haz.items):
            for x, y, _, _, kind in haz.items:
                pygame.draw.circle(scr, HAZ[int(kind)], self.px(x, y), 3)
        for i in range(1, len(run.trail), 2):
            a, b = run.trail[i - 1], run.trail[i]
            pygame.draw.line(scr, height_colour(b[2]), self.px(a[0], a[1]), self.px(b[0], b[1]), 4)
        c = self.px(d.x, d.y)
        pygame.draw.circle(scr, (20, 20, 20), c, max(5, int(run.radius * self.k)), 2)
        pygame.draw.line(scr, (20, 20, 20), c, self.px(d.x + 0.45 * math.cos(d.yaw), d.y + 0.45 * math.sin(d.yaw)), 3)
        v = math.hypot(d.vx, d.vy)
        if v > 0.05:
            pygame.draw.line(scr, OK, c, self.px(d.x + d.vx, d.y + d.vy), 3)
        self.panel()
        pygame.display.flip()

    def panel(self):
        run, scr = self.run, self.screen
        sim, d, out = run.sim, run.sim.drone, run.last
        x0 = WIN_W - PANEL
        pygame.draw.rect(scr, (250, 250, 250), (x0, 0, PANEL, WIN_H))
        name = "OA-Stack" if run.method == "stack" else run.method
        self.text(f"{name} - live", (x0 + 14, 12), big=True)
        self.text(f"{run.scenario}  seed {run.seed}   x{self.speed}" + ("  PAUSED" if self.paused else ""), (x0 + 14, 38), GREY)
        y = 66
        if run.result:
            col = OK if run.result == "SUCCESS" else WARN if run.result == "TIMEOUT" else BAD
            self.text(run.result, (x0 + 14, y), col, True)
        else:
            self.text("flying...", (x0 + 14, y), (40, 110, 220), True)
        y += 34
        status, flags, vert, ms = "-", [], "-", 0.0
        if out is not None and not isinstance(out, dict):
            status, flags = out.status, out.flags
            vert, ms = out.info.get("vertical", "-"), out.info.get("compute_ms", 0.0)
        scol = OK if status in ("OK", "CLEAR") else WARN if status in ("AVOIDING", "SLOWED", "IDLE") else GREY if status == "-" else BAD
        rows = [("time", f"{sim.t:5.1f} s / limit {run.limit:.0f} s"),
                ("status", status),
                ("height mode", vert),
                ("flags", " ".join(flags) or "none"),
                ("speed", f"{math.hypot(d.vx, d.vy):.2f} m/s"),
                ("height", f"{d.z:.2f} m"),
                ("closest wall", f"{max(0.0, float(np.min(sim.last_ranges)) - run.radius):.2f} m  (min {max(0.0, run.min_clear):.2f})"),
                ("hazards in memory", str(len(getattr(getattr(run.pilot.alg, 'hazards', None), 'items', [])))),
                ("compute", f"{ms:.1f} ms per cycle")]
        for k, val in rows:
            self.text(f"{k:<18}", (x0 + 14, y), GREY)
            self.text(val, (x0 + 180, y), scol if k == "status" else (30, 30, 30))
            y += 22
        # height plot: actual (colour) vs OA-Stack target (dashed grey), last 20 s
        y += 12
        self.text("height: actual (blue) vs target (grey)", (x0 + 14, y), GREY)
        y += 22
        gx, gy, gw, gh = x0 + 14, y, PANEL - 28, 170
        pygame.draw.rect(scr, (255, 255, 255), (gx, gy, gw, gh))
        pygame.draw.rect(scr, (200, 200, 200), (gx, gy, gw, gh), 1)
        zmax = 2.4

        def yz(z):
            return gy + gh - int(z / zmax * gh)
        for zz in (0.5, 1.0, 1.5, 2.0):
            pygame.draw.line(scr, (236, 236, 236), (gx, yz(zz)), (gx + gw, yz(zz)))
            self.text(f"{zz:.1f}", (gx + 2, yz(zz) - 16), (170, 170, 170))
        n = int(20.0 / sim.dt)
        alts, tg = run.alts[-n:], run.targets[-n:]
        if len(alts) > 1:
            xs = [gx + int(i * gw / n) for i in range(len(alts))]
            for i in range(0, len(tg) - 1, 4):
                pygame.draw.line(scr, (150, 150, 150), (xs[i], yz(tg[i])), (xs[i + 1], yz(tg[i + 1])), 2)
            pygame.draw.lines(scr, (40, 110, 220), False, [(xs[i], yz(a)) for i, a in enumerate(alts)], 2)
        y = gy + gh + 22
        legend = [(WALL, WALL, "wall (2.4 m)"), (BOX, BOX, "low box - fly over"), (None, DUCT, "duct - fly under"),
                  (PILLAR, PILLAR, "pillar"), (PERSON, PERSON, "person (moving)"),
                  ((255, 200, 200), (255, 200, 200), "lidar rays"),
                  (DUCT, DUCT, "remembered duct"), (BOX, BOX, "remembered low obstacle"),
                  (OK, OK, "velocity command")]
        for fill, edge, label in legend:
            if fill is None:
                pygame.draw.rect(scr, edge, (x0 + 14, y + 2, 16, 12), 3)
            elif label.startswith("remembered"):
                pygame.draw.circle(scr, edge, (x0 + 22, y + 8), 4)
            elif label in ("lidar rays", "velocity command"):
                pygame.draw.line(scr, edge, (x0 + 14, y + 8), (x0 + 30, y + 8), 3)
            else:
                pygame.draw.rect(scr, fill, (x0 + 14, y + 2, 16, 12))
            self.text(label, (x0 + 40, y), (60, 60, 60))
            y += 19
        self.text("trail colour = height (blue low, yellow high)", (x0 + 14, y + 4), GREY)
        self.text("SPACE pause  +/- speed  N next world", (x0 + 14, WIN_H - 40), GREY)
        self.text("L lidar rays  S screenshot  ESC quit", (x0 + 14, WIN_H - 22), GREY)

    # --- main loop --------------------------------------------------------------------
    def loop(self, max_seconds=None, shot=None):
        t_start = time.time()
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                    pygame.quit()
                    return
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.speed = min(16, self.speed * 2)
                    elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.speed = max(1, self.speed // 2)
                    elif e.key == pygame.K_n:
                        self.new_run()
                    elif e.key == pygame.K_l:
                        self.rays = not self.rays
                    elif e.key == pygame.K_s:
                        fn = f"live_{self.run.scenario}_seed{self.run.seed}_t{int(self.run.sim.t)}.png"
                        pygame.image.save(self.screen, fn)
                        print("saved", fn)
            if not self.paused:
                # real time: one frame = 1/30 s of simulated time at x1
                steps = max(1, int(round(self.speed / 30.0 / self.run.sim.dt)))
                for _ in range(steps):
                    self.run.step()
                if self.run.result and self.done_at is None:
                    self.done_at = time.time()
                    print(f"{self.run.scenario} seed {self.run.seed}: {self.run.result} at {self.run.sim.t:.1f} s")
                if self.done_at and time.time() - self.done_at > 3.0:
                    self.new_run()                      # next world after a 3 s pause
            self.draw()
            if shot and max_seconds and time.time() - t_start > max_seconds:
                pygame.image.save(self.screen, shot)
                pygame.quit()
                return
            self.clock.tick(30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="mixed_clutter", choices=list(B.SCENARIOS))
    ap.add_argument("--seed", type=int, default=207)
    ap.add_argument("--method", default="stack")
    ap.add_argument("--speed", type=int, default=1, help="1 = real time")
    ap.add_argument("--shot", default=None, help="save a screenshot after --seconds and quit (for testing)")
    ap.add_argument("--seconds", type=float, default=None)
    a = ap.parse_args()
    v = Viewer(a)
    v.speed = a.speed
    v.loop(a.seconds, a.shot)


if __name__ == "__main__":
    main()
