"""Vision-odometry bridge: ROS nav_msgs/Odometry (ENU / FLU) -> PX4 VehicleOdometry (NED / FRD).

Rulebook v2.2 rule 8.24: no GPS/GNSS. PX4 therefore estimates position ONLY from
external vision odometry on /fmu/in/vehicle_visual_odometry.

    SIMULATION : Gazebo ground truth (/odom) -> this node -> PX4      (stand-in for SLAM)
    REAL DRONE : SLAM pose (e.g. lidar SLAM) -> this node -> PX4      (same node, same topic)

    ros2 run nidar_control ev_bridge                                  # sim defaults
    ros2 run nidar_control ev_bridge --ros-args -p odom_topic:=/slam/odom -p body_velocity:=false

Parameters
    odom_topic      input nav_msgs/Odometry                    (default /odom)
    rate_hz         max rate sent to PX4                        (default 30)
    body_velocity   true if the input twist is in the body frame (Gazebo, most SLAM) (default true)
    pos_noise_m     extra Gaussian noise added to position, to mimic SLAM in sim (default 0)
    pos_std_m / vel_std / yaw_std_rad   reported 1-sigma accuracy (PX4 weights the data by it)
"""
import math
import random

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nidar_control.px4_io import PX4_QOS, now_us

# ENU (x east, y north, z up) -> NED (x north, y east, z down)
R_NED_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
# body FLU (ROS) -> body FRD (PX4)
R_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    """Rotation matrix -> quaternion [w, x, y, z] (w >= 0)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def enu_flu_to_ned_frd(p_enu, q_enu_flu, v, body_velocity=True):
    """Pure conversion (unit-tested): returns position NED, quaternion FRD->NED, velocity."""
    p_ned = R_NED_ENU @ np.asarray(p_enu, float)
    R = R_NED_ENU @ quat_to_rot(*q_enu_flu) @ R_FLU_FRD
    q = rot_to_quat(R)
    v = np.asarray(v, float)
    v_out = R_FLU_FRD @ v if body_velocity else R_NED_ENU @ v
    return p_ned, q, v_out


class EvBridge(Node):
    def __init__(self):
        super().__init__("nidar_ev_bridge")
        p = self.declare_parameter
        topic = p("odom_topic", "/odom").value
        self.period_us = int(1e6 / float(p("rate_hz", 30.0).value))
        self.body_vel = bool(p("body_velocity", True).value)
        self.noise = float(p("pos_noise_m", 0.0).value)
        self.pos_var = float(p("pos_std_m", 0.02).value) ** 2
        self.vel_var = float(p("vel_std", 0.05).value) ** 2
        self.yaw_var = float(p("yaw_std_rad", 0.02).value) ** 2
        self.pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", PX4_QOS)
        self.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)
        self.origin = None
        self.last_us = 0
        self.count = 0
        self.get_logger().info(f"{topic} -> /fmu/in/vehicle_visual_odometry (no GPS: rule 8.24)")
        self.create_timer(5.0, self._report)

    def _on_odom(self, m):
        t = now_us(self)
        if t - self.last_us < self.period_us:
            return
        self.last_us = t
        pp = m.pose.pose.position
        p_enu = np.array([pp.x, pp.y, pp.z])
        if self.origin is None:
            # local frame origin = where the drone starts (the launch area)
            self.origin = p_enu.copy()
            self.get_logger().info(f"origin set at ENU ({pp.x:.2f}, {pp.y:.2f}, {pp.z:.2f})")
        p_enu = p_enu - self.origin
        if self.noise > 0:
            p_enu = p_enu + np.array([random.gauss(0, self.noise) for _ in range(3)])
        q = m.pose.pose.orientation
        tw = m.twist.twist.linear
        p_ned, q_ned, v = enu_flu_to_ned_frd(p_enu, (q.w, q.x, q.y, q.z), (tw.x, tw.y, tw.z),
                                             self.body_vel)
        o = VehicleOdometry()
        o.timestamp = t
        o.timestamp_sample = t
        o.pose_frame = VehicleOdometry.POSE_FRAME_NED
        o.position = [float(x) for x in p_ned]
        o.q = [float(x) for x in q_ned]
        o.velocity_frame = (VehicleOdometry.VELOCITY_FRAME_BODY_FRD if self.body_vel
                            else VehicleOdometry.VELOCITY_FRAME_NED)
        o.velocity = [float(x) for x in v]
        o.angular_velocity = [math.nan] * 3
        o.position_variance = [self.pos_var] * 3
        o.orientation_variance = [self.yaw_var] * 3
        o.velocity_variance = [self.vel_var] * 3
        o.reset_counter = 0
        o.quality = 100
        self.pub.publish(o)
        self.count += 1

    def _report(self):
        if self.count == 0:
            self.get_logger().warn("no odometry received yet - is the Gazebo/SLAM odometry topic up?")
        self.count = 0


def main():
    rclpy.init()
    node = EvBridge()
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
