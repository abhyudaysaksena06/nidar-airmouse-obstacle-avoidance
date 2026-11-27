"""Small helpers shared by the NIDAR nodes: PX4 QoS, topic names, maths."""
import math

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

# PX4's uXRCE-DDS bridge publishes best-effort; this profile talks to it both ways.
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def versioned(name):
    """PX4 >= 1.16 adds _v1, _v2 ... to topics whose message has a version > 0
    (e.g. /fmu/out/vehicle_status_v1). Subscribing to every candidate means the
    same code works on PX4 1.14, 1.15 and 1.16+."""
    return [name] + [f"{name}_v{i}" for i in range(1, 4)]


def now_us(node):
    return int(node.get_clock().now().nanoseconds / 1000)


def quat_to_euler(q):
    """PX4 quaternion [w, x, y, z] -> (roll, pitch, yaw) in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
