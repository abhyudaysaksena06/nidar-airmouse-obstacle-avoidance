"""NIDAR PID logger: prints COMMANDED vs ACTUAL for every control loop and saves CSV.

    ros2 run nidar_control pid_logger
    ros2 run nidar_control pid_logger --ros-args -p print_period:=0.2 -p log_dir:=~/nidar_logs

What it compares (PX4 cascade, outer -> inner loop)
    position  : trajectory_setpoint / vehicle_local_position_setpoint  vs  vehicle_local_position
    velocity  : same topics, vx vy vz
    attitude  : vehicle_attitude_setpoint (q_d)  vs  vehicle_attitude (q)
    body rates: vehicle_rates_setpoint           vs  vehicle_angular_velocity
The attitude/rate SETPOINTS only reach ROS after px4/add_pid_topics.sh (rebuild PX4).
Without that patch the logger still works for position/velocity and actual attitude.

Outputs
    terminal : a table every `print_period` seconds
    CSV      : <log_dir>/pid_<date>_<time>.csv  (one row per `sample_period`)
               -> analyse with:  python3 tools/pid_analyze.py <csv>
    ROS topic: /nidar/pid_debug (std_msgs/String, JSON) - the same row, for any
               other pipeline (e.g. the team's ground-station link) to consume.
"""
import csv
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import (TrajectorySetpoint, VehicleAngularVelocity, VehicleAttitude,
                          VehicleAttitudeSetpoint, VehicleLocalPosition,
                          VehicleLocalPositionSetpoint, VehicleRatesSetpoint)
from rclpy.node import Node
from std_msgs.msg import String

from nidar_control.px4_io import PX4_QOS, quat_to_euler, versioned, wrap_pi

# column order of CSV + JSON: (name, group)
COLUMNS = [
    ("x", "pos"), ("y", "pos"), ("z", "pos"),
    ("vx", "vel"), ("vy", "vel"), ("vz", "vel"),
    ("roll", "att"), ("pitch", "att"), ("yaw", "att"),
    ("rollrate", "rate"), ("pitchrate", "rate"), ("yawrate", "rate"),
]
DEG = {"roll", "pitch", "yaw", "rollrate", "pitchrate", "yawrate"}


class PidLogger(Node):
    def __init__(self):
        super().__init__("nidar_pid_logger")
        self.print_period = float(self.declare_parameter("print_period", 0.5).value)
        self.sample_period = float(self.declare_parameter("sample_period", 0.02).value)
        log_dir = os.path.expanduser(self.declare_parameter("log_dir", "~/nidar_logs").value)
        os.makedirs(log_dir, exist_ok=True)

        self.sp = {}    # name -> commanded value (SI units, radians)
        self.act = {}   # name -> actual value
        self.seen = set()

        def sub(msg_type, topic, cb):
            for t in versioned(topic):
                self.create_subscription(msg_type, t, cb, PX4_QOS)

        sub(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._on_traj_sp)
        sub(VehicleLocalPositionSetpoint, "/fmu/out/vehicle_local_position_setpoint", self._on_lpos_sp)
        sub(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_lpos)
        sub(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att)
        sub(VehicleAttitudeSetpoint, "/fmu/out/vehicle_attitude_setpoint", self._on_att_sp)
        sub(VehicleRatesSetpoint, "/fmu/out/vehicle_rates_setpoint", self._on_rates_sp)
        sub(VehicleAngularVelocity, "/fmu/out/vehicle_angular_velocity", self._on_rates)

        self.pub = self.create_publisher(String, "/nidar/pid_debug", 10)
        self.path = os.path.join(log_dir, time.strftime("pid_%Y%m%d_%H%M%S.csv"))
        self.fh = open(self.path, "w", newline="")
        self.csv = csv.writer(self.fh)
        header = ["t"]
        for n, _ in COLUMNS:
            header += [f"{n}_sp", f"{n}"]
        self.csv.writerow(header)
        self.t0 = time.time()
        self.create_timer(self.sample_period, self._sample)
        self.create_timer(self.print_period, self._print)
        self.get_logger().info(f"logging to {self.path}")

    # ---------- callbacks (all PX4 frames: NED / FRD, radians) ----------
    def _mark(self, what):
        if what not in self.seen:
            self.seen.add(what)
            self.get_logger().info(f"receiving {what}")

    def _on_traj_sp(self, m):
        self._mark("trajectory_setpoint (offboard command)")
        for i, n in enumerate(("x", "y", "z")):
            if not math.isnan(m.position[i]):
                self.sp[n] = m.position[i]
        for i, n in enumerate(("vx", "vy", "vz")):
            if not math.isnan(m.velocity[i]):
                self.sp[n] = m.velocity[i]
        if not math.isnan(m.yaw):
            self.sp["yaw"] = m.yaw

    def _on_lpos_sp(self, m):
        # PX4's own position-controller setpoint: preferred when available
        self._mark("vehicle_local_position_setpoint")
        for n in ("x", "y", "z", "vx", "vy", "vz"):
            v = getattr(m, n)
            if not math.isnan(v):
                self.sp[n] = v

    def _on_lpos(self, m):
        self._mark("vehicle_local_position")
        for n in ("x", "y", "z", "vx", "vy", "vz"):
            self.act[n] = getattr(m, n)

    def _on_att(self, m):
        self._mark("vehicle_attitude")
        self.act["roll"], self.act["pitch"], self.act["yaw"] = quat_to_euler(m.q)

    def _on_att_sp(self, m):
        self._mark("vehicle_attitude_setpoint")
        self.sp["roll"], self.sp["pitch"], self.sp["yaw"] = quat_to_euler(m.q_d)

    def _on_rates_sp(self, m):
        self._mark("vehicle_rates_setpoint")
        self.sp["rollrate"], self.sp["pitchrate"], self.sp["yawrate"] = m.roll, m.pitch, m.yaw

    def _on_rates(self, m):
        self._mark("vehicle_angular_velocity")
        self.act["rollrate"], self.act["pitchrate"], self.act["yawrate"] = m.xyz

    # ---------- output ----------
    def _row(self):
        row = {"t": round(time.time() - self.t0, 3)}
        for n, _ in COLUMNS:
            # PX4 fields are numpy float32 -> plain float so JSON/CSV accept them
            s, a = self.sp.get(n), self.act.get(n)
            row[f"{n}_sp"] = None if s is None else float(s)
            row[n] = None if a is None else float(a)
        return row

    def _sample(self):
        if not self.act:
            return
        row = self._row()
        self.csv.writerow([row["t"]] + [("" if row[k] is None else f"{row[k]:.5f}")
                                        for n, _ in COLUMNS for k in (f"{n}_sp", n)])
        self.pub.publish(String(data=json.dumps(row)))

    def _print(self):
        if not self.act:
            self.get_logger().info("waiting for PX4 topics (is MicroXRCEAgent running?)")
            return
        self.fh.flush()
        lines = [f"t={time.time() - self.t0:7.1f}s   {'command':>9} {'actual':>9} {'error':>9}"]
        for n, g in COLUMNS:
            a, s = self.act.get(n), self.sp.get(n)
            if a is None:
                continue
            k, unit = (math.degrees(1), "deg" + ("/s" if "rate" in n else "")) if n in DEG \
                else (1.0, "m/s" if g == "vel" else "m")
            if s is None:
                lines.append(f"  {n:9s} {'-':>9} {a * k:9.3f} {'-':>9}  {unit}")
            else:
                e = wrap_pi(s - a) if n == "yaw" else s - a
                lines.append(f"  {n:9s} {s * k:9.3f} {a * k:9.3f} {e * k:9.3f}  {unit}")
        print("\n".join(lines) + "\n", flush=True)

    def destroy_node(self):
        self.fh.close()
        self.get_logger().info(f"saved {self.path}")
        super().destroy_node()


def main():
    rclpy.init()
    node = PidLogger()
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
