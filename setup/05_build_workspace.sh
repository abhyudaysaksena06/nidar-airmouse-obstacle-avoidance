#!/usr/bin/env bash
# STEP 5: build our ROS 2 packages (run again after every code change that is not Python/yaml).
set -eo pipefail
. "$(dirname "$0")/00_common.sh"
source /opt/ros/$ROS_DISTRO/setup.bash
cd "$WS_DIR"
say "Building $WS_DIR (px4_msgs takes ~5 min the first time)"
colcon build --symlink-install
LINE="source $WS_DIR/install/setup.bash"
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
ok "Workspace built and added to ~/.bashrc. Open a NEW terminal, then run check_setup.sh."
