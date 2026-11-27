"""NIDAR offboard mission node: ROS 2 -> PX4, with the NIDAR Rulebook v2.2 (PS2) fail-safes.

    ros2 launch nidar_control offboard.launch.py mission:=arena_demo
    ros2 run nidar_control offboard_control --ros-args -p mission:=square -p speed:=0.8

Missions:  hover | square | step | arena_demo (enter the practice arena, visit a room, exit, land)

Fail-safes (PS2 rule 8.31 + section 10) - each one is logged and shown on /nidar/mission_status:
    emergency stop   ros2 topic pub --once /nidar/emergency_stop std_msgs/msg/Empty   -> motors OFF now
    mission abort    ros2 topic pub --once /nidar/abort std_msgs/msg/Empty            -> land where it is
    emergency recall ros2 topic pub --once /nidar/recall std_msgs/msg/Empty           -> fly back the way
                     it came, out of the entry, land on the launch area
    low battery      battery remaining <= low_battery                                 -> recall
    comm loss        QGroundControl link lost (PX4 gcs_connection_lost_counter rises) -> recall
    geofence         drone outside the arena box                                      -> recall
                     (setpoints are also clamped inside the box, so it never aims outside)
    time limit       flight time >= mission_time_limit_s (rule: 30 min max)           -> recall
    node crash       PX4 itself lands after 1 s without setpoints (COM_OBL_RC_ACT=4)

Why recall instead of PX4 "Return": PX4 Return needs GPS (banned, rule 8.24) and flies a
straight line, i.e. into walls. Recall retraces the corners of the commanded route already
flown (corridor centre-lines, known to be free) backwards and leaves through the entry/exit
(rule 8.21).

Frames: PX4 local NED, origin = launch area. x = north, y = east, z = DOWN.
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import (BatteryStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from std_msgs.msg import Empty, String

from nidar_control.mission import Carrot, Waypoint, build_mission
from nidar_control.px4_io import PX4_QOS, now_us, versioned

RATE_HZ = 20.0
ARENA_FENCE = dict(north_min=-1.0, north_max=15.5, east_min=-7.5, east_max=7.5, alt_max=2.1, margin=0.3)
TEST_FENCE = dict(north_min=-5.0, north_max=0.3, east_min=-4.0, east_max=4.0, alt_max=2.1, margin=0.3)
REACHED_M = 0.3
FORCE_DISARM = 21196.0   # MAVLink magic number: disarm even in the air (emergency stop)


class OffboardControl(Node):
    def __init__(self):
        super().__init__("nidar_offboard")
        p = self.declare_parameter
        self.kind = p("mission", "square").value
        self.alt = float(p("altitude", 1.2).value)
        self.speed = float(p("speed", 0.8).value)
        self.side = float(p("side", 2.0).value)
        self.step_size = float(p("step_size", 1.0).value)
        self.hover_time = float(p("hover_time", 10.0).value)
        self.land = bool(p("land", True).value)
        # ---- fail-safe settings (defaults = rulebook practice arena) ----
        self.recall_speed = float(p("recall_speed", 0.6).value)
        self.low_battery = float(p("low_battery", 0.30).value)
        self.time_limit = float(p("mission_time_limit_s", 1500.0).value)   # 25 of the 30 min
        # a corner is "reached" only within this distance: the drone settles before it
        # turns, so it does not cut corners in 1 m corridors (measured: 0.3 m cut a corner)
        self.corner_tol = float(p("corner_tolerance", 0.15).value)
        self.corner_speed = float(p("corner_max_speed", 0.1).value)    # m/s, "stopped" at a corner
        self.corner_decel = float(p("corner_decel", 0.5).value)        # m/s^2 braking into corners
        # Geofence box in metres from the launch point (north, east, altitude).
        #   arena_demo : the arena (15 x 15 m, starting 0.5 m north of the launch area,
        #                centred on it east-west) - leaving it is a rulebook penalty.
        #   test missions (hover/square/step): the open floor SOUTH of the launch area,
        #                outside the arena walls.
        # Ceiling: 2.44 m (8 ft) clear height -> 2.1 m max altitude.
        box = ARENA_FENCE if self.kind == "arena_demo" else TEST_FENCE
        self.fence = {k: float(p(f"fence_{k}", v).value) for k, v in box.items()}

        self.mission = build_mission(self.kind, self.alt, self.side, self.step_size, self.hover_time)

        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.pub_status = self.create_publisher(String, "/nidar/mission_status", 10)
        for t in versioned("/fmu/out/vehicle_status"):
            self.create_subscription(VehicleStatus, t, self._on_status, PX4_QOS)
        for t in versioned("/fmu/out/vehicle_local_position"):
            self.create_subscription(VehicleLocalPosition, t, self._on_pos, PX4_QOS)
        for t in versioned("/fmu/out/battery_status"):
            self.create_subscription(BatteryStatus, t, self._on_battery, PX4_QOS)
        self.create_subscription(Empty, "/nidar/emergency_stop", lambda _: self._estop(), 10)
        self.create_subscription(Empty, "/nidar/abort", lambda _: self._abort("operator abort"), 10)
        self.create_subscription(Empty, "/nidar/recall", lambda _: self._recall("operator emergency recall"), 10)

        self.status = self.pos = self.battery = None
        self.state = "WAIT_PX4"
        self.reason = ""
        self.ticks = 0
        self.wp_i = 0
        self.hold_left = None
        self.carrot = None
        self.last_cmd_tick = -100
        self.gcs_lost_count = None
        self.crumbs = []
        self.t_takeoff = None
        self.route = None            # list of waypoints being flown (mission or recall)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"mission={self.kind} altitude={self.alt} m speed={self.speed} m/s "
            f"({len(self.mission)} waypoints). Fail-safes armed. Waiting for PX4 ...")

    # ---------------- callbacks ----------------
    def _on_status(self, msg):
        self.status = msg

    def _on_pos(self, msg):
        self.pos = msg

    def _on_battery(self, msg):
        self.battery = msg

    # ---------------- PX4 helpers ----------------
    def _command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = now_us(self)
        self.pub_cmd.publish(m)

    def _stream(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.timestamp = now_us(self)
        self.pub_mode.publish(m)
        sp = TrajectorySetpoint()
        sp.position = [float(v) for v in self.carrot.p]
        if self._corridor():
            # Corridors: POSITION-ONLY setpoints. With velocity feed-forward PX4's lagging
            # velocity loop carried 0.95 m/s into a corner whose setpoint had already
            # stopped (0.39 m overshoot toward a wall, measured). Position-only = PX4
            # approaches every point without overshoot.
            sp.velocity = [math.nan] * 3
        else:
            sp.velocity = [float(v) for v in self.carrot.v]   # feed-forward (open area, PID tests)
        sp.acceleration = [math.nan] * 3
        sp.yaw = float(self.carrot.yaw)
        sp.timestamp = now_us(self)
        self.pub_sp.publish(sp)

    def _corridor(self):
        """True inside the arena (arena_demo) and when recalling: careful corridor flying."""
        return self.kind == "arena_demo" or self.state == "RECALL"

    def _armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _say(self, txt):
        self.get_logger().info(f"[{self.state}] {txt}")

    def _set(self, state, txt=""):
        self.state = state
        self._say(txt)

    # ---------------- geofence ----------------
    def _rel(self, x, y, z):
        """NED position -> metres from the launch point (north, east, altitude up)."""
        return x - self.home[0], y - self.home[1], self.home[2] - z

    def _inside_fence(self, n, e, alt, margin=0.0):
        f = self.fence
        return (f["north_min"] + margin <= n <= f["north_max"] - margin and
                f["east_min"] + margin <= e <= f["east_max"] - margin and alt <= f["alt_max"] - margin)

    def _clamp(self, w):
        """Keep every target inside the fence (minus margin)."""
        f, m = self.fence, self.fence["margin"]
        n, e, alt = self._rel(w.x, w.y, w.z)
        n = min(max(n, f["north_min"] + m), f["north_max"] - m)
        e = min(max(e, f["east_min"] + m), f["east_max"] - m)
        alt = min(alt, f["alt_max"] - m)
        w.x, w.y, w.z = self.home[0] + n, self.home[1] + e, self.home[2] - alt
        return w

    # ---------------- fail-safe actions ----------------
    def _estop(self):
        self.reason = "EMERGENCY STOP"
        self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, FORCE_DISARM)
        self._set("ESTOP", "EMERGENCY STOP: motors cut (force disarm)")

    def _abort(self, why):
        if self.state in ("DONE", "ESTOP", "WAIT_PX4"):
            return
        self.reason = why
        self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._set("LANDING", f"MISSION ABORT ({why}): landing where it is")

    def _recall(self, why):
        if self.state == "RECALL":
            return
        if self.state not in ("MISSION", "HOLD"):
            self._say(f"recall requested ({why}) but not flying a mission - ignored")
            return
        self.reason = why
        # Retrace the COMMANDED route (corridor centre-lines, corner by corner) backwards,
        # at cruise height - not the drone's actual track, which contains its tracking
        # error. The route ends over the launch area, then the drone lands.
        cruise_z = self.home[2] - self.alt_cruise()
        route = [Waypoint(c[0], c[1], cruise_z, self.carrot.yaw, 0, False, "corner (retrace)")
                 for c in reversed(self.crumbs)]
        route.append(Waypoint(self.home[0], self.home[1], cruise_z, self.carrot.yaw,
                              1.0, False, "above launch area"))
        self.route = [self._clamp(w) for w in route]
        self.wp_i = 0
        self.hold_left = None
        what = "EXIT" if why.startswith("mission complete") else "EMERGENCY RECALL"
        self._set("RECALL", f"{what} ({why}): retracing {len(self.crumbs)} corners of the flown route "
                            f"out of the entry/exit, then landing on the launch area")

    def alt_cruise(self):
        return min(self.alt, self.fence["alt_max"] - self.fence["margin"])

    def _check_failsafes(self):
        """Automatic triggers while flying. Returns True if one fired."""
        n, e, alt = self._rel(self.pos.x, self.pos.y, self.pos.z)
        if not self._inside_fence(n, e, alt):
            self._recall(f"geofence breach at N {n:.2f} E {e:.2f} alt {alt:.2f} m")
            return True
        b = self.battery
        if b is not None and 0 <= b.remaining <= self.low_battery:
            self._recall(f"low battery {b.remaining * 100:.0f} %")
            return True
        if self.status is not None:
            c = self.status.gcs_connection_lost_counter
            if self.gcs_lost_count is None:
                self.gcs_lost_count = c
            elif c > self.gcs_lost_count:
                self.gcs_lost_count = c
                self._recall("ground-station link lost")
                return True
        if self.t_takeoff is not None and self._elapsed() >= self.time_limit:
            self._recall(f"mission time limit {self.time_limit / 60:.0f} min")
            return True
        return False

    def _elapsed(self):
        return (self.get_clock().now().nanoseconds - self.t_takeoff) / 1e9 if self.t_takeoff else 0.0

    # ---------------- state machine (20 Hz) ----------------
    def _tick(self):
        self.ticks += 1
        dt = 1.0 / RATE_HZ
        if self.state == "WAIT_PX4":
            if self.status is None or self.pos is None:
                if self.ticks % 60 == 0:
                    self._say("no PX4 data yet - are MicroXRCEAgent, PX4 and ev_bridge running?")
                return
            if not (self.pos.xy_valid and self.pos.z_valid):
                if self.ticks % 60 == 0:
                    self._say("PX4 is up, waiting for a valid position (vision odometry from ev_bridge) ...")
                return
            self.home = (self.pos.x, self.pos.y, self.pos.z)
            self.carrot = Carrot(self.pos.x, self.pos.y, self.pos.z, self.pos.heading)
            self.mission = [self._clamp(self._shift(w)) for w in self.mission]
            for w in self.mission:
                w.yaw = self.pos.heading + w.yaw          # yaw relative to start heading
            self.route = self.mission
            self.crumbs = []            # corners of the commanded route already flown
            self._set("STREAM", "PX4 ready. Streaming setpoints before switching to Offboard.")
            self.stream_ticks = 0
            return

        # Stream setpoints ONLY while we are in charge. After LAND (or an abort) PX4's
        # own controller must be the only writer of trajectory_setpoint: if we kept
        # streaming, PX4 would alternate between "descend" and our "hold altitude",
        # bounce on the ground and never detect the landing.
        if self.state in ("STREAM", "ARMING", "MISSION", "HOLD", "RECALL"):
            self._stream()

        if self.state == "STREAM":
            self.stream_ticks += 1
            if self.stream_ticks >= RATE_HZ:   # PX4 needs >1 s of setpoints first
                self._set("ARMING", "switching to Offboard and arming")
            return

        if self.state == "ARMING":
            if self._armed() and self._offboard():
                self.t_takeoff = self.get_clock().now().nanoseconds
                self._set("MISSION", f"armed in Offboard. Flying '{self.kind}'.")
                self._announce()
                return
            if self.ticks - self.last_cmd_tick >= RATE_HZ:   # retry once a second
                self.last_cmd_tick = self.ticks
                self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)  # 6 = Offboard
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            return

        if self.state in ("MISSION", "RECALL", "HOLD"):
            if not self._offboard():
                self._set("ABORTED", "PX4 left Offboard mode (QGC/RC took over or a PX4 fail-safe). "
                          "Stopping; PX4 is in control now.")
                return
            if self.state != "RECALL" and self._check_failsafes():
                return
            if self.state == "HOLD":
                return
            self._fly_route(dt, self.speed if self.state == "MISSION" else self.recall_speed)
            return

        if self.state == "LANDING":
            if not self._armed():
                self._set("DONE", "landed and disarmed. Press Ctrl+C to exit.")
            return

        if self.state == "ESTOP" and not self._armed() and not getattr(self, "_estop_done", False):
            self._estop_done = True
            self._say("motors off and disarmed")

    def _fly_route(self, dt, speed):
        wp = self.route[self.wp_i]
        corridor = self._corridor()
        # In corridors: nose along the path (forward ToF looks ahead), brake smoothly into
        # every corner, and only turn once the drone is ON the corner and nearly STOPPED -
        # otherwise momentum carries it into the wall after the turn (seen in testing).
        self.carrot.step(wp, speed, dt, face_travel=corridor, decel=self.corner_decel if corridor else 0.0)
        err = math.dist((self.pos.x, self.pos.y, self.pos.z), (wp.x, wp.y, wp.z))
        if corridor:
            reached = self.carrot.at(wp) and err < self.corner_tol and \
                math.hypot(self.pos.vx, self.pos.vy) < self.corner_speed
        else:
            reached = self.carrot.at(wp) and (wp.jump or err < REACHED_M)
        if self.hold_left is None and reached:
            # normal waypoints: start holding once the drone is there.
            # step-test waypoints (jump): start the clock at the step itself.
            self.hold_left = wp.hold
            if self.state == "MISSION" and (not self.crumbs or self.crumbs[-1] != (wp.x, wp.y, wp.z)):
                self.crumbs.append((wp.x, wp.y, wp.z))     # remember the corner for recall
            self._say(f"at '{wp.label}' (error {err:.2f} m), holding {wp.hold:.0f} s")
        elif self.hold_left is not None:
            self.hold_left -= dt
        if self.ticks % int(RATE_HZ * 2) == 0:
            n, e, alt = self._rel(self.pos.x, self.pos.y, self.pos.z)
            bat = f" | battery {self.battery.remaining * 100:.0f} %" if self.battery else ""
            self._say(f"pos N {n:6.2f} E {e:6.2f} alt {alt:5.2f} m | "
                      f"speed {math.hypot(self.pos.vx, self.pos.vy):4.2f} m/s{bat}")
        if self.hold_left is not None and self.hold_left <= 0:
            self.hold_left = None
            self.wp_i += 1
            if self.wp_i >= len(self.route):
                self._route_done()
            else:
                self._announce()

    def _route_done(self):
        if self.state == "RECALL":
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._set("LANDING", "recall complete: over the launch area, landing")
        elif self.kind == "arena_demo":
            self._recall("mission complete - exiting the arena")
        elif self.land:
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._set("LANDING", "mission complete, landing")
        else:
            self.wp_i = len(self.route) - 1
            self._set("HOLD", "mission complete, holding last point")

    def _shift(self, w):
        """Mission is written relative to take-off point; make it absolute."""
        w.x += self.home[0]
        w.y += self.home[1]
        w.z += self.home[2]
        return w

    def _announce(self):
        w = self.route[self.wp_i]
        n, e, alt = self._rel(w.x, w.y, w.z)
        self._say(f"-> waypoint {self.wp_i + 1}/{len(self.route)} '{w.label}' "
                  f"N {n:.1f} E {e:.1f} alt {alt:.1f} m")

    # ---------------- status for the ground station ----------------
    def _publish_status(self):
        s = {"state": self.state, "mission": self.kind, "failsafe_reason": self.reason,
             "elapsed_s": round(self._elapsed(), 1), "time_limit_s": self.time_limit}
        if self.route:
            s["waypoint"] = f"{min(self.wp_i + 1, len(self.route))}/{len(self.route)}"
            s["progress_pct"] = round(100.0 * self.wp_i / max(1, len(self.route)), 1) \
                if self.state in ("MISSION", "RECALL") else (100.0 if self.state == "DONE" else 0.0)
        if self.pos is not None and self.carrot is not None:
            n, e, alt = self._rel(self.pos.x, self.pos.y, self.pos.z)
            s["position_from_launch_m"] = {"north": round(n, 2), "east": round(e, 2), "alt": round(alt, 2)}
            s["inside_geofence"] = self._inside_fence(n, e, alt)
        if self.battery is not None:
            s["battery_pct"] = round(self.battery.remaining * 100, 1)
        self.pub_status.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C or launch shutdown
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
