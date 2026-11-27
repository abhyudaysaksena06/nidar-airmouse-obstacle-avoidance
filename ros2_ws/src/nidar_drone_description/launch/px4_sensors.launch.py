"""REQUIRED with PX4 SITL (make px4_sitl gz_nidar_drone):
  * ev_bridge: Gazebo pose -> PX4 vision odometry. NO GPS is used (rulebook v2.2 rule
    8.24), so without this PX4 has no position and will not arm in Position/Offboard.
    On the real drone the SLAM pose takes this place.
  * lidar / ToF / ultrasonic / camera data into ROS 2, shown in RViz.

    ros2 launch nidar_drone_description px4_sensors.launch.py            # + RViz
    ros2 launch nidar_drone_description px4_sensors.launch.py rviz:=false
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from description import robot_description  # noqa: E402


def generate_launch_description():
    share = get_package_share_directory("nidar_drone_description")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "gz_bridge_px4.yaml"),
                          "use_sim_time": True}]),
        Node(package="nidar_control", executable="ev_bridge", output="screen",
             parameters=[{"odom_topic": "/odom"}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description(px4=True), "use_sim_time": True}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[{"use_sim_time": True}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "px4_sensors.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
