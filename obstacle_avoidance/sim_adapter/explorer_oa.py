"""Full AirMouse mission (airmouse_sim baseline explorer) with its reactive
brake/repulsion layer replaced by this obstacle-avoidance stack.

    cd airmouse_sim
    python run.py --algo ../obstacle_avoidance/sim_adapter/explorer_oa.py
    python run.py --algo ../obstacle_avoidance/sim_adapter/explorer_oa.py --headless --runs 16 --jobs 8 --label oa
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # obstacle_avoidance/ -> package `oa`

from algorithms.explorer import Algorithm as Explorer  # noqa: E402  (airmouse_sim on sys.path via run.py)
from oa import AvoidanceConfig, ObstacleAvoider  # noqa: E402

NO_AVOID_STATES = ("PREFLIGHT", "TAKEOFF", "LAND", "EMERGENCY_LAND", "DONE")


class Algorithm(Explorer):
    def reset(self, info):
        super().reset(info)
        # The explorer's planner keeps paths ~0.30 m from walls (radius + 0.05).
        # RULE: avoider safety radius <= planner clearance, otherwise the avoider
        # fights every path. So margin 0.08 -> r_safe = 0.33 m.
        self.oa = ObstacleAvoider(AvoidanceConfig(
            drone_radius=info["drone_radius"], margin=0.08, min_margin=0.05,
            v_max=info["max_speed"], lidar_max_range=info["lidar_max_range"]))
        self._t = 0.0
        self.oa_status = ""

    def step(self, obs):
        self._t = obs["t"]
        o = obs["odom"]
        # the avoider needs a SMOOTH local frame: raw odometry, not the
        # scan-matched SLAM pose (its corrections would misplace remembered obstacles)
        self._odom = (o["x"], o["y"], o["yaw"])
        return super().step(obs)

    def _out(self, vx, vy, vz, w, lid):
        if lid is None or self.state in NO_AVOID_STATES:
            return super()._out(vx, vy, vz, w, None)
        scan = (lid["angles"], lid["ranges"]) if lid["new"] else None
        out = self.oa.update(self._t, self._odom, (vx, vy), scan=scan)
        self.oa_status = out.status
        if out.replan and self.state in ("EXPLORE", "RETURN"):
            self.last_plan = -99.0  # ask the planner for a new path now
        return super()._out(out.vx, out.vy, vz, w, None)

    def debug_text(self):
        return f"{super().debug_text()} OA={self.oa_status}"
