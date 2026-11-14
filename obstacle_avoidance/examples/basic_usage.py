"""Minimal integration example: what the navigation loop calls every cycle.

    python examples/basic_usage.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from oa import AvoidanceConfig, ObstacleAvoider  # noqa: E402

cfg = AvoidanceConfig(drone_radius=0.25, v_max=1.0, lidar_fov_deg=360)
avoider = ObstacleAvoider(cfg)

# fake world: a pillar 1.5 m straight ahead of the drone's path
pillar, pillar_r = np.array([2.5, 0.0]), 0.3
angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)

x, y, yaw, t, dt = 0.0, 0.0, 0.0, 0.0, 0.1
for k in range(60):
    # 1) a scan arrives (here: ray-cast the pillar)
    rel = pillar - [x, y]
    ranges = np.full(360, np.inf)
    for i, a in enumerate(angles):
        d = np.array([math.cos(a + yaw), math.sin(a + yaw)])
        b = rel @ d
        disc = b * b - (rel @ rel - pillar_r ** 2)
        if disc >= 0 and b > 0:
            ranges[i] = b - math.sqrt(disc)
    # 2) the planner / pure pursuit wants to fly straight ahead at 0.8 m/s
    u_des = (0.8, 0.0)
    # 3) the avoider turns it into a safe command
    out = avoider.update(t, (x, y, yaw), u_des, scan=(angles, ranges))
    # 4) send out.vx, out.vy to the flight controller (here: integrate)
    x += out.vx * dt
    y += out.vy * dt
    t += dt
    clear = np.hypot(*(pillar - [x, y])) - pillar_r - cfg.drone_radius
    if k % 5 == 0:
        print(f"t={t:4.1f}s  pos=({x:4.2f},{y:5.2f})  cmd=({out.vx:4.2f},{out.vy:5.2f})  "
              f"status={out.status:<9} clearance={clear:4.2f} m  replan={out.replan}")
