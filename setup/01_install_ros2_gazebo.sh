#!/usr/bin/env bash
# STEP 1: ROS 2 Jazzy + Gazebo Harmonic + tools.  ~15 min, ~4 GB download.
set -eo pipefail
. "$(dirname "$0")/00_common.sh"

say "Updating Ubuntu"
sudo apt-get update && sudo apt-get -y upgrade

say "Base tools"
sudo apt-get install -y curl git wget gnupg lsb-release software-properties-common \
  build-essential cmake python3-pip python3-venv locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

say "Adding the ROS 2 apt repository"
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update

say "Installing ROS 2 $ROS_DISTRO desktop + Gazebo Harmonic (ros_gz) + robot tools"
sudo apt-get install -y \
  ros-$ROS_DISTRO-desktop \
  ros-$ROS_DISTRO-ros-gz \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-dev-tools python3-colcon-common-extensions liburdfdom-tools

say "rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

say "Auto-load ROS in every new terminal"
LINE="source /opt/ros/$ROS_DISTRO/setup.bash"
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc

source /opt/ros/$ROS_DISTRO/setup.bash
command -v gz >/dev/null && ok "Gazebo: $(gz sim --versions 2>/dev/null | head -1)"
ok "ROS 2 $ROS_DISTRO installed. CLOSE this terminal and open a new one, then run 02."
