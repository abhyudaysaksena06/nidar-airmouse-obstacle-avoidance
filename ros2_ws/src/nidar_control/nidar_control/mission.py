"""Mission planning for the offboard node, kept free of ROS so it can be unit-tested.

All positions are PX4 local NED: x = north, y = east, z = DOWN (so 2 m up is z = -2).
A mission is a list of Waypoints. The node moves a "carrot" setpoint toward the
current waypoint at `speed` m/s, so the ROS parameter `speed` really sets how fast
the drone flies. (PX4's own limits, e.g. MPC_XY_VEL_MAX set in QGroundControl,
still cap it.)
"""
import math
from dataclasses import dataclass


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0
    hold: float = 0.0       # seconds to wait after reaching it
    jump: bool = False      # True = step the setpoint instantly (for PID step tests)
    label: str = ""


def build_mission(kind, altitude, side, step_size, hover_time):
    """Missions are relative to the launch area. hover/square/step fly over the open
    floor SOUTH of the launch area (negative north), outside the arena walls;
    arena_demo flies INTO the practice arena through the entry/exit."""
    z = -abs(altitude)
    if kind == "hover":
        return [Waypoint(0, 0, z, hold=hover_time, label="hover")]
    if kind == "square":
        s = side
        return [Waypoint(0, 0, z, hold=2, label="climb"),
                Waypoint(-s, 0, z, hold=1, label="corner 1 (south)"),
                Waypoint(-s, s, z, hold=1, label="corner 2 (south-east)"),
                Waypoint(0, s, z, hold=1, label="corner 3 (east)"),
                Waypoint(0, 0, z, hold=2, label="back home")]
    if kind == "step":
        d = step_size
        dz = d / 2          # keep below the 2.44 m ceiling
        return [Waypoint(-1.5, 0, z, hold=5, label="settle"),
                Waypoint(-1.5 - d, 0, z, hold=6, jump=True, label=f"step x -{d} m"),
                Waypoint(-1.5, 0, z, hold=6, jump=True, label="step x back"),
                Waypoint(-1.5, d, z, hold=6, jump=True, label=f"step y +{d} m"),
                Waypoint(-1.5, 0, z, hold=6, jump=True, label="step y back"),
                Waypoint(-1.5, 0, z - dz, hold=6, jump=True, label=f"step z up {dz} m"),
                Waypoint(-1.5, 0, z, hold=6, jump=True, label="step z back"),
                Waypoint(-1.5, 0, z, yaw=math.radians(45), hold=6, jump=True, label="step yaw +45 deg"),
                Waypoint(-1.5, 0, z, yaw=0.0, hold=6, jump=True, label="step yaw back"),
                Waypoint(0, 0, z, hold=2, label="back over launch area")]
    if kind == "arena_demo":
        # Practice arena (worlds/make_arena.py): launch area at arena (7.5, -0.5) facing
        # north. Corridor centres: entry x 7.5; row y 2.5; column x 8.5; row y 12.5.
        # north = arena_y + 0.5, east = arena_x - 7.5
        return [Waypoint(0.0, 0.0, z, hold=2, label="climb on launch area"),
                Waypoint(1.5, 0.0, z, hold=0, label="through the entry"),
                Waypoint(3.0, 0.0, z, hold=0, label="first junction"),
                Waypoint(3.0, 1.0, z, hold=0, label="into the central corridor"),
                Waypoint(13.0, 1.0, z, hold=0, label="north end of central corridor"),
                Waypoint(13.0, 2.0, z, hold=0, label="room S2 doorway"),
                Waypoint(14.0, 2.0, z, hold=3, label="inside room S2 (survivor)")]
    raise ValueError(f"unknown mission '{kind}' (use hover, square, step or arena_demo)")


class Carrot:
    """Setpoint that slides toward a target at a fixed speed."""

    def __init__(self, x, y, z, yaw=0.0):
        self.p = [x, y, z]
        self.yaw = yaw
        self.v = [0.0, 0.0, 0.0]
        self.cur_speed = 0.0

    def step(self, wp, speed, dt, face_travel=False, decel=0.0):
        """Move toward wp at `speed`. face_travel=True turns the nose toward the direction
        of travel (the forward ToF/lidar then look where the drone is going).
        decel > 0 [m/s^2]: ramp the speed UP and DOWN with this limit: accelerate gently from
        each corner and brake into the next (speed <= sqrt(2*decel*dist)). Without the ramp-up
        the drone falls behind the setpoint, PX4 then demands extra speed to catch up (measured:
        1.1 m/s for a 0.8 m/s leg) and the momentum overshoots the next corner into a wall."""
        if wp.jump:
            self.p = [wp.x, wp.y, wp.z]
            self.v = [0.0, 0.0, 0.0]
            self.cur_speed = 0.0
            self.yaw = wp.yaw
            return
        d = [wp.x - self.p[0], wp.y - self.p[1], wp.z - self.p[2]]
        dist = math.sqrt(sum(c * c for c in d))
        if decel > 0:
            speed = min(speed, self.cur_speed + decel * dt, math.sqrt(2 * decel * dist))
        self.cur_speed = speed
        move = speed * dt
        if dist <= move or dist < 1e-6:
            self.p = [wp.x, wp.y, wp.z]
            self.v = [0.0, 0.0, 0.0]
            self.cur_speed = 0.0
        else:
            self.p = [self.p[i] + d[i] / dist * move for i in range(3)]
            self.v = [d[i] / dist * speed for i in range(3)]
        if face_travel:
            if math.hypot(d[0], d[1]) > 0.05:
                self.yaw = math.atan2(d[1], d[0])     # NED: atan2(east, north)
        else:
            self.yaw = wp.yaw

    def at(self, wp):
        return (abs(self.p[0] - wp.x) < 1e-6 and abs(self.p[1] - wp.y) < 1e-6
                and abs(self.p[2] - wp.z) < 1e-6)
