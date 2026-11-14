"""Copy this file to start your own algorithm:
    python run.py --algo algorithms/my_algo.py
"""
import math

import numpy as np

from algorithms.base import BaseAlgorithm


class Algorithm(BaseAlgorithm):
    def reset(self, info):
        super().reset(info)
        self.survivors = []

    def step(self, obs):
        # 1) take off and hold cruise altitude
        vz = self.altitude_hold(obs, self.info["cruise_alt"])
        if obs["tof"]["down"] < 0.6 * self.info["cruise_alt"]:
            return {"vx": 0.0, "vy": 0.0, "vz": vz, "yaw_rate": 0.0, "state": "TAKEOFF"}

        # 2) lidar lost -> hover in place (failsafe)
        lid = obs["lidar"]
        if lid is None:
            return {"vx": 0.0, "vy": 0.0, "vz": vz, "yaw_rate": 0.0, "state": "HOLD"}

        # 3) your logic here: localisation, mapping, planning, avoidance ...
        ranges, angles = lid["ranges"], lid["angles"]
        front = ranges[np.abs(angles) < math.radians(20)].min()
        if front < 1.0:
            return {"vx": 0.0, "vy": 0.0, "vz": vz, "yaw_rate": 1.0, "state": "AVOID"}
        return {"vx": 0.5, "vy": 0.0, "vz": vz, "yaw_rate": 0.0, "state": "EXPLORE"}

    def get_survivors(self):
        return self.survivors
