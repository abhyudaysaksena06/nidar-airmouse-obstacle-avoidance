"""Offboard mission + PID logger together.

    ros2 launch nidar_control offboard.launch.py                     # arena_demo (config/offboard.yaml)
    ros2 launch nidar_control offboard.launch.py mission:=step speed:=0.5
    ros2 launch nidar_control offboard.launch.py logger:=false
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("nidar_control"), "config", "offboard.yaml")
    args = ["mission", "altitude", "speed", "side", "step_size", "hover_time",
            "recall_speed", "low_battery", "mission_time_limit_s"]
    # defaults come from config/offboard.yaml; command-line arguments override them
    with open(cfg) as fh:
        yaml_params = yaml.safe_load(fh)["nidar_offboard"]["ros__parameters"]
    defaults = {a: str(yaml_params[a]) for a in args}
    overrides = {a: LaunchConfiguration(a) for a in args}
    return LaunchDescription(
        [DeclareLaunchArgument(a, default_value=defaults[a]) for a in args] + [
            DeclareLaunchArgument("logger", default_value="true"),
            Node(package="nidar_control", executable="offboard_control", output="screen",
                 parameters=[overrides, {"land": bool(yaml_params.get("land", True))}]),
            Node(package="nidar_control", executable="pid_logger", output="screen",
                 condition=IfCondition(LaunchConfiguration("logger"))),
        ])
