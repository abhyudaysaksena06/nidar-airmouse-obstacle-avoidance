"""Multirotor translational dynamics, as seen from an Offboard velocity setpoint.

setpoint --(command latency)--> velocity PID (per axis) --> tilt/thrust limit
--> motor lag --> acceleration + drag towards wind --> velocity --> position

Tune vel_pid / vz_pid in config.json exactly like MPC_XY_VEL_P/I/D and
MPC_Z_VEL_P/I in PX4, and watch overshoot and settling in the viewer.
"""
import math
from collections import deque


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PID:
    def __init__(self, kp, ki, kd, i_max):
        self.kp, self.ki, self.kd, self.i_max = kp, ki, kd, i_max
        self.i = 0.0

    def step(self, err, d_meas, dt):
        self.i = max(-self.i_max, min(self.i_max, self.i + err * dt))
        return self.kp * err + self.ki * self.i - self.kd * d_meas  # D on measurement

    def reset(self):
        self.i = 0.0


class Drone:
    G = 9.81

    def __init__(self, x, y, yaw, cfg, wind_cfg, dt, rng):
        self.c, self.wc, self.rng = cfg, wind_cfg, rng
        self.x, self.y, self.z, self.yaw = x, y, 0.0, yaw
        self.vx = self.vy = self.vz = self.wz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.wind = [0.0, 0.0]
        p, q = cfg["vel_pid"], cfg["vz_pid"]
        self.pid = [PID(p["kp"], p["ki"], p["kd"], p["i_max"]),
                    PID(p["kp"], p["ki"], p["kd"], p["i_max"]),
                    PID(q["kp"], q["ki"], q["kd"], q["i_max"])]
        n = max(0, int(round(cfg["cmd_latency_s"] / dt)))
        self.queue = deque([(0.0, 0.0, 0.0, 0.0)] * n)
        self.landed = True

    def body_vel(self):
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        return cy * self.vx + sy * self.vy, -sy * self.vx + cy * self.vy

    def step(self, cmd, dt, floor_z, ceil_z):
        """cmd = (vx_body, vy_body, vz, yaw_rate). Returns 'ceiling' if hit."""
        c = self.c
        self.queue.append(cmd)
        cvx, cvy, cvz, cwz = self.queue.popleft()

        # gusts: Ornstein-Uhlenbeck process + steady wind
        tau, sd = self.wc["gust_time_const_s"], self.wc["gust_std_mps"]
        for k in range(2):
            self.wind[k] += (-self.wind[k] / tau) * dt + sd * math.sqrt(2 * dt / tau) * self.rng.gauss(0, 1)
        wx = self.wind[0] + self.wc["steady_mps"][0]
        wy = self.wind[1] + self.wc["steady_mps"][1]

        if self.landed and cvz <= 0.05:
            self.vx = self.vy = self.vz = self.wz = 0.0
            for p in self.pid:
                p.reset()
            self.z = floor_z
            return None
        self.landed = False

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        dvx, dvy = cy * cvx - sy * cvy, sy * cvx + cy * cvy
        sp = math.hypot(dvx, dvy)
        if sp > c["max_speed_mps"]:
            dvx, dvy = dvx * c["max_speed_mps"] / sp, dvy * c["max_speed_mps"] / sp
        dvz = max(-c["max_vz_mps"], min(c["max_vz_mps"], cvz))

        ax_c = self.pid[0].step(dvx - self.vx, self.ax, dt)
        ay_c = self.pid[1].step(dvy - self.vy, self.ay, dt)
        az_c = self.pid[2].step(dvz - self.vz, self.az, dt)
        a_lim = self.G * math.tan(math.radians(c["max_tilt_deg"]))
        a = math.hypot(ax_c, ay_c)
        if a > a_lim:
            ax_c, ay_c = ax_c * a_lim / a, ay_c * a_lim / a
        az_c = max(-c["max_vert_accel_mps2"], min(c["max_vert_accel_mps2"], az_c))

        k = min(1.0, dt / c["motor_time_const_s"])  # motor / attitude lag
        self.ax += (ax_c - self.ax) * k
        self.ay += (ay_c - self.ay) * k
        self.az += (az_c - self.az) * k

        drag = c["drag_coeff"]
        self.vx += (self.ax + drag * (wx - self.vx)) * dt
        self.vy += (self.ay + drag * (wy - self.vy)) * dt
        self.vz += (self.az - drag * self.vz) * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        cwz = max(-c["max_yaw_rate_rps"], min(c["max_yaw_rate_rps"], cwz))
        self.wz += (cwz - self.wz) * min(1.0, dt / c["yaw_time_const_s"])
        self.yaw = wrap(self.yaw + self.wz * dt)

        hit = None
        if self.z + c["half_height_m"] > ceil_z:
            self.z, self.vz = ceil_z - c["half_height_m"], 0.0
            hit = "ceiling"
        if self.z <= floor_z:
            self.z, self.vz = floor_z, 0.0
            if cvz <= 0.05:
                self.landed = True
        return hit

    def power_w(self, bc):
        v2 = self.vx ** 2 + self.vy ** 2
        if self.landed:
            return 5.0  # electronics only
        return bc["hover_power_w"] + bc["speed_power_w_per_mps2"] * v2 + \
            bc["climb_power_w_per_mps"] * max(0.0, self.vz)
