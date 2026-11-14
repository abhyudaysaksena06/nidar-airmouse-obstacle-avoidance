"""Random fault schedule for one mission (reproducible from the seed)."""
import random


class Faults:
    TYPES = ("lidar_blackout", "camera_blackout", "flow_loss", "link_outage")

    def __init__(self, cfg, time_limit, seed):
        rng = random.Random(seed * 7919 + 17)
        self.events = []  # (t0, t1, type, value)
        for name in self.TYPES:
            f = cfg[name]
            for _ in range(f["count"]):
                t0 = rng.uniform(15.0, time_limit * 0.8)
                self.events.append((t0, t0 + rng.uniform(*f["duration_s"]), name, None))
        g = cfg["gyro_bias_jump"]
        for _ in range(g["count"]):
            t0 = rng.uniform(15.0, time_limit * 0.8)
            self.events.append((t0, 1e9, "gyro_bias_jump", rng.choice([-1, 1]) * g["magnitude_rps"]))
        b = cfg["battery_fast_drain"]
        for _ in range(b["count"]):
            t0 = rng.uniform(15.0, time_limit * 0.6)
            self.events.append((t0, 1e9, "battery_fast_drain", b["factor"]))
        self.events.sort()

    def active(self, t, name):
        return any(e[2] == name and e[0] <= t < e[1] for e in self.events)

    def value(self, t, name, default):
        v = default
        for t0, t1, n, val in self.events:
            if n == name and t0 <= t < t1:
                v = val
        return v

    def transitions(self, t_prev, t):
        """Fault start/end messages between two times (for the event log)."""
        out = []
        for t0, t1, n, _ in self.events:
            if t_prev < t0 <= t:
                out.append(f"FAULT start: {n}")
            if t_prev < t1 <= t and t1 < 1e8:
                out.append(f"FAULT end: {n}")
        return out
