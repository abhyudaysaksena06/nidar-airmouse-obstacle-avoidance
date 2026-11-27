#!/usr/bin/env bash
# Checks that everything from steps 1-5 is installed. Safe to run any time.
. "$(dirname "$0")/00_common.sh"
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
pass=0; bad=0
chk() { if eval "$2" >/dev/null 2>&1; then printf '  \033[32m[OK]\033[0m   %s\n' "$1"; pass=$((pass+1)); else printf '  \033[31m[MISSING]\033[0m %s  -> %s\n' "$1" "$3"; bad=$((bad+1)); fi; }
echo "NIDAR setup check"
chk "ROS 2 $ROS_DISTRO"            "[ -f /opt/ros/$ROS_DISTRO/setup.bash ]"           "run setup/01"
chk "Gazebo (gz sim)"             "gz sim --versions"                                 "run setup/01"
chk "xacro + check_urdf"          "command -v xacro && command -v check_urdf"         "run setup/01"
chk "ros_gz bridge"               "ros2 pkg prefix ros_gz_bridge"                     "run setup/01"
chk "PX4 downloaded"              "[ -d $PX4_DIR/.git ]"                              "run setup/02"
chk "PX4 built"                   "[ -x $PX4_DIR/build/px4_sitl_default/bin/px4 ]"    "run setup/02"
chk "NIDAR model in PX4"          "[ -f $PX4_DIR/Tools/simulation/gz/models/nidar_drone/model.sdf ]" "run px4/install_into_px4.sh"
chk "NIDAR arena in PX4"          "[ -f $PX4_DIR/Tools/simulation/gz/worlds/nidar_arena.sdf ]" "run px4/install_into_px4.sh"
chk "MicroXRCEAgent"              "command -v MicroXRCEAgent"                         "run setup/03"
chk "QGroundControl"              "[ -x $HOME/QGroundControl.AppImage ]"              "run setup/04"
chk "workspace built"             "ros2 pkg prefix nidar_drone_description && ros2 pkg prefix nidar_control && ros2 pkg prefix px4_msgs" "run setup/05 in a new terminal"
echo "$pass OK, $bad missing"
