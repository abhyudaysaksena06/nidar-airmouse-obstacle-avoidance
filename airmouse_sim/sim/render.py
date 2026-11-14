"""Pygame viewer.

Left: ground truth (only the simulator sees this).
Right: the GROUND STATION view - only what came over the radio link.

Keys: SPACE pause, N step, +/- speed, R new map, L lidar rays,
      K operator KILL switch, S screenshot, ESC quit.
"""
import math

import numpy as np
import pygame

WIN_W, WIN_H, PANEL = 1400, 820, 470
BG, WALL, FENCE = (245, 244, 240), (40, 40, 45), (150, 150, 150)
LOW, HANG = (214, 120, 60), (90, 120, 200)
SURV, FOUND, REP, TRUE_T, EST_T = (220, 50, 50), (40, 160, 90), (120, 60, 200), (40, 110, 220), (170, 90, 220)
OK, BAD, WARN = (40, 150, 80), (200, 50, 50), (210, 140, 20)


class Viewer:
    def __init__(self, sim):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("AirMouse simulator")
        self.font = pygame.font.SysFont("consolas", 14)
        self.big = pygame.font.SysFont("consolas", 17, bold=True)
        self.clock = pygame.time.Clock()
        self.speed, self.paused, self.show_rays = 2, False, True
        self.set_sim(sim)

    def set_sim(self, sim):
        self.sim = sim
        w = sim.w
        vw, vh = WIN_W - PANEL - 20, WIN_H - 120
        self.s = min(vw / w.width, vh / w.height)
        self.ox, self.oy = 10, 10
        self.static = self._draw_static()
        self.map_surf, self.map_xf, self._map_t = None, None, None

    def px(self, x, y):
        return int(self.ox + x * self.s), int(self.oy + (self.sim.w.height - y) * self.s)

    def _rect(self, surf, col, x0, y0, x1, y1, width=0):
        a, b = self.px(x0, y1), self.px(x1, y0)
        pygame.draw.rect(surf, col, (a[0], a[1], max(1, b[0] - a[0]), max(1, b[1] - a[1])), width)

    def _draw_static(self):
        w = self.sim.w
        surf = pygame.Surface((WIN_W - PANEL, WIN_H))
        surf.fill(BG)
        self._rect(surf, (255, 255, 255), *w.arena)
        for z in w.dark_zones:
            self._rect(surf, (222, 222, 232), *z)
        for z in w.texture_zones:
            self._rect(surf, (238, 232, 210), *z)
            self._rect(surf, (200, 190, 150), *z, 1)
        for r in w.rects:
            if r.kind == "hanging":
                continue
            col = {"wall": WALL, "fence": FENCE, "box": LOW}[r.kind]
            self._rect(surf, col, r.x0, r.y0, r.x1, r.y1)
            if r.kind == "box":
                surf.blit(self.font.render(f"{r.z1:.1f}", True, (120, 60, 20)), self.px(r.x1, r.y1))
        for c in w.cylinders:
            pygame.draw.circle(surf, LOW, self.px(c.x, c.y), max(2, int(c.r * self.s)))
            surf.blit(self.font.render(f"{c.z1:.1f}", True, (120, 60, 20)), self.px(c.x + c.r, c.y + c.r))
        for r in w.rects:
            if r.kind == "hanging":
                self._rect(surf, HANG, r.x0, r.y0, r.x1, r.y1, 2)
                surf.blit(self.font.render(f"^{r.z0:.1f}", True, HANG), self.px(r.x1, r.y1))
        ex, ey = self.px(*w.entry)
        pygame.draw.circle(surf, OK, (ex, ey), 6, 2)
        sx, sy = self.px(*w.start[:2])
        pygame.draw.circle(surf, OK, (sx, sy), int(w.cfg["mission"]["exit_radius_m"] * self.s), 1)
        surf.blit(self.font.render("ENTRY/EXIT", True, OK), (sx + 8, sy + 8))
        lg = ["orange = box (top m)", "blue outline = hanging (bottom m)",
              "grey-blue = dark room   beige = featureless floor"]
        for i, t in enumerate(lg):
            surf.blit(self.font.render(t, True, (80, 80, 80)), (12, WIN_H - 58 + i * 17))
        return surf

    def _person(self, surf, s, col):
        c, d = math.cos(s.yaw), math.sin(s.yaw)
        a, b = self.px(s.x - 0.8 * c, s.y - 0.8 * d), self.px(s.x + 0.8 * c, s.y + 0.8 * d)
        pygame.draw.line(surf, col, a, b, max(3, int(0.35 * self.s)))
        pygame.draw.circle(surf, col, b, max(3, int(0.18 * self.s)))

    # ------------------------------------------------------------- GCS map
    def _update_map_surface(self):
        m = self.sim.gcs["map"]
        if not m:
            return
        P = m["grid"]
        img = np.full(P.shape + (3,), 160, dtype=np.uint8)
        img[P < 0.35] = (255, 255, 255)
        img[P > 0.65] = (30, 30, 30)
        surf = pygame.surfarray.make_surface(img[::-1].transpose(1, 0, 2))
        bw, bh = PANEL - 20, 300
        k = min(bw / surf.get_width(), bh / surf.get_height())
        self.map_surf = pygame.transform.scale(surf, (int(surf.get_width() * k), int(surf.get_height() * k)))
        self.map_xf = (m["origin"], m["res"], P.shape, k)

    def _map_px(self, x, y, x0, y0):
        (ox, oy), res, shape, k = self.map_xf
        return int(x0 + (x - ox) / res * k), int(y0 + (shape[0] - (y - oy) / res) * k)

    def _text(self, t, pos, col=(30, 30, 30), big=False):
        self.screen.blit((self.big if big else self.font).render(t, True, col), pos)

    def draw(self):
        sim, w, d, scr = self.sim, self.sim.w, self.sim.drone, self.screen
        scr.blit(self.static, (0, 0))
        reps, matched = sim.match()
        found = {sid for sid, _ in matched}
        for s in w.survivors:
            self._person(scr, s, FOUND if s.id in found else SURV)
        for m in w.movers:
            pygame.draw.circle(scr, (200, 60, 160), self.px(m.x, m.y), int(m.r * self.s))
        if len(sim.true_trail) > 1:
            pygame.draw.lines(scr, TRUE_T, False, [self.px(*p) for p in sim.true_trail], 2)
        if len(sim.est_trail) > 1:
            pygame.draw.lines(scr, EST_T, False, [self.px(*sim.start_to_world(*p)) for p in sim.est_trail], 1)
        if self.show_rays:
            c = self.px(d.x, d.y)
            mr = sim.cs["lidar_max_range_m"]
            for a, r in zip(sim.lidar_angles[::6], sim.last_ranges[::6]):
                r = min(r, mr)
                e = self.px(d.x + r * math.cos(a + d.yaw), d.y + r * math.sin(a + d.yaw))
                pygame.draw.line(scr, (255, 190, 190), c, e, 1)
        fov, rng = math.radians(sim.cs["camera_hfov_deg"]), 3.5
        cam_ok = not sim.faults.active(sim.t, "camera_blackout")
        pts = [self.px(d.x, d.y)] + [self.px(d.x + rng * math.cos(d.yaw + a), d.y + rng * math.sin(d.yaw + a))
                                     for a in np.linspace(-fov / 2, fov / 2, 8)]
        pygame.draw.polygon(scr, (250, 200, 60) if cam_ok else BAD, pts, 1)
        for x, y in reps:
            p = self.px(x, y)
            pygame.draw.line(scr, REP, (p[0] - 6, p[1] - 6), (p[0] + 6, p[1] + 6), 3)
            pygame.draw.line(scr, REP, (p[0] - 6, p[1] + 6), (p[0] + 6, p[1] - 6), 3)
        c = self.px(d.x, d.y)
        pygame.draw.circle(scr, (20, 20, 20), c, max(4, int(sim.cd["radius_m"] * self.s)), 2)
        pygame.draw.line(scr, (20, 20, 20), c, self.px(d.x + 0.5 * math.cos(d.yaw), d.y + 0.5 * math.sin(d.yaw)), 2)
        self._altitude_strip()
        self._panel()
        pygame.display.flip()

    def _altitude_strip(self):
        sim, scr = self.sim, self.screen
        x0, y0, wd, ht = 440, WIN_H - 100, WIN_W - PANEL - 450, 90
        pygame.draw.rect(scr, (250, 250, 250), (x0, y0, wd, ht))
        pygame.draw.rect(scr, (180, 180, 180), (x0, y0, wd, ht), 1)
        ceil = sim.w.ceiling
        alts = sim.alt_trail[-wd:]
        if len(alts) > 1:
            pts = [(x0 + i, y0 + ht - int(a / ceil * ht)) for i, a in enumerate(alts)]
            pygame.draw.lines(scr, TRUE_T, False, pts, 2)
        cr = sim.cd["cruise_alt_m"]
        pygame.draw.line(scr, (200, 200, 200), (x0, y0 + ht - int(cr / ceil * ht)), (x0 + wd, y0 + ht - int(cr / ceil * ht)))
        t = sim.tof
        self._text(f"altitude z={sim.drone.z:.2f} m  tof down={t.get('down', 0):.2f} up={t.get('up', 0):.2f} "
                   f"fwd_dn={t.get('fwd_down', 0):.1f} fwd_up={t.get('fwd_up', 0):.1f}", (x0 + 4, y0 + 2), (60, 60, 60))

    def _panel(self):
        sim, scr = self.sim, self.screen
        x0 = WIN_W - PANEL
        pygame.draw.rect(scr, (250, 250, 250), (x0, 0, PANEL, WIN_H))
        g = sim.gcs
        link_col = OK if sim.link_up else BAD
        self._text("GROUND STATION", (x0 + 10, 8), big=True)
        self._text("LINK UP" if sim.link_up else "LINK LOST - data frozen", (x0 + 200, 10), link_col, True)
        if g["t"] is not None and g["t"] != self._map_t:
            self._update_map_surface()
            self._map_t = g["t"]
        my = 34
        if self.map_surf:
            scr.blit(self.map_surf, (x0 + 10, my))
            if g["pose"]:
                pygame.draw.circle(scr, TRUE_T, self._map_px(g["pose"][0], g["pose"][1], x0 + 10, my), 4)
            for k, (sx, sy) in enumerate(g["survivors"]):
                p = self._map_px(sx, sy, x0 + 10, my)
                pygame.draw.circle(scr, REP, p, 6, 2)
                self._text(str(k + 1), (p[0] + 6, p[1] - 8), REP)
        y = 345
        b = g["battery"]
        pygame.draw.rect(scr, (220, 220, 220), (x0 + 10, y, 200, 12))
        pygame.draw.rect(scr, OK if b > 0.3 else WARN if b > 0.15 else BAD, (x0 + 10, y, int(200 * max(0, b)), 12))
        self._text(f"battery {100 * b:.0f}%   state {g['state']}", (x0 + 220, y - 2))
        y += 20
        for k, (sx, sy) in enumerate(g["survivors"][:6]):
            self._text(f"survivor #{k + 1}: ({sx:5.1f}, {sy:5.1f}) m", (x0 + 10 + (k % 2) * 225, y + (k // 2) * 16), REP)
        y += 52
        h = sim.faults
        flags = [("lidar", not h.active(sim.t, "lidar_blackout")), ("camera", not h.active(sim.t, "camera_blackout")),
                 ("flow", sim.flow_quality >= 0.3), ("link", sim.link_up)]
        for i, (n, ok) in enumerate(flags):
            pygame.draw.circle(scr, OK if ok else BAD, (x0 + 18 + i * 90, y + 7), 6)
            self._text(n, (x0 + 28 + i * 90, y))
        y += 22
        sc = sim.score()
        lines = [
            f"seed {sim.w.seed}  t={sc['time_s']:.0f}/{sim.cm['time_limit_s']:.0f}s  x{self.speed}"
            + ("  PAUSED" if self.paused else "") + f"   SCORE {sc['score']}",
            f"found {sc['survivors_found']}/{sc['survivors_total']}  false {sc['false_reports']}  "
            f"loc err {sc['mean_loc_error_m']}  coll {sc['collisions']}",
            f"coverage {sc['coverage_pct']}%  pose err {sc['pose_error_m']} m  raw odom drift {sc['odom_drift_m']} m",
            f"status: {sc['end_reason']}",
            (sim.algo.debug_text() or "")[:62],
        ]
        for i, t in enumerate(lines):
            self._text(t, (x0 + 10, y + i * 17))
        y += len(lines) * 17 + 6
        self._text("EVENT LOG", (x0 + 10, y), big=True)
        y += 20
        for tt, msg in sim.events[-14:]:
            col = BAD if ("COLLISION" in msg or "LOST" in msg or "FAULT start" in msg or "END" in msg) else (60, 60, 60)
            self._text(f"{tt:6.1f}s {msg}"[:62], (x0 + 10, y), col)
            y += 16
        self._text("SPACE pause N step +/- speed R new map L rays", (x0 + 10, WIN_H - 36), (110, 110, 110))
        self._text("K KILL switch  S screenshot  ESC quit", (x0 + 10, WIN_H - 20), (110, 110, 110))

    def loop(self, new_sim):
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                    pygame.quit()
                    return self.sim.score()
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif e.key == pygame.K_n and self.paused:
                        self.sim.step()
                    elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.speed = min(32, self.speed * 2)
                    elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.speed = max(1, self.speed // 2)
                    elif e.key == pygame.K_r:
                        self.set_sim(new_sim(None))
                    elif e.key == pygame.K_l:
                        self.show_rays = not self.show_rays
                    elif e.key == pygame.K_k:
                        self.sim.kill()
                    elif e.key == pygame.K_s:
                        self.save(f"screenshot_seed{self.sim.w.seed}_t{int(self.sim.t)}.png")
            if not self.paused:
                for _ in range(self.speed):
                    self.sim.step()
            self.draw()
            self.clock.tick(30)

    def save(self, fn):
        pygame.image.save(self.screen, fn)
        print("saved", fn)
