"""STEP 2 - the same URDF in Gazebo Harmonic, in the NIDAR practice arena (rulebook v2.2),
with physics, sensors and ROS 2 topics.

    ros2 launch nidar_drone_description gazebo.launch.py            # Gazebo + RViz
    ros2 launch nidar_drone_description gazebo.launch.py rviz:=false
    ros2 launch nidar_drone_description gazebo.launch.py headless:=true   # no Gazebo window

Then fly it without PX4 (Gazebo's own velocity controller):
    ros2 topic pub --once /enable std_msgs/msg/Bool "{data: true}"
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}"
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from description import robot_description  # noqa: E402


def generate_launch_description():
    share = get_package_share_directory("nidar_drone_description")
    world = os.path.join(share, "worlds", "nidar_arena.sdf")
    headless = LaunchConfiguration("headless")
    gz_args = PythonExpression([
        "'-r -v 3 ", world, "' + (' -s --headless-rendering' if '", headless,
        "' == 'true' else '')"])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        # default spawn = centre of the 2 ft x 2 ft launch area outside the entry, facing in (north)
        DeclareLaunchArgument("x", default_value="7.5"),
        DeclareLaunchArgument("y", default_value="-0.5"),
        DeclareLaunchArgument("z", default_value="0.2", description="spawn height [m]"),
        DeclareLaunchArgument("yaw", default_value="1.5708"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description(), "use_sim_time": True}]),

        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "nidar_drone",
                        "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                        "-z", LaunchConfiguration("z"), "-Y", LaunchConfiguration("yaw")]),

        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "gz_bridge.yaml"),
                          "use_sim_time": True}]),

        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "gazebo.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
