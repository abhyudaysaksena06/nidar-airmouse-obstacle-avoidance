"""STEP 1 - look at the drone in RViz (no physics).

    ros2 launch nidar_drone_description display.launch.py
    ros2 launch nidar_drone_description display.launch.py gui:=false

A slider window (joint_state_publisher_gui) lets you turn the 4 propellers.
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from description import robot_description  # noqa: E402


def generate_launch_description():
    share = get_package_share_directory("nidar_drone_description")
    gui = LaunchConfiguration("gui")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="show the propeller slider window"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description()}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "display.rviz")]),
    ])
