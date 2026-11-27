#!/usr/bin/env bash
# STEP 3: the PX4 <-> ROS 2 link: Micro XRCE-DDS Agent + px4_msgs.  ~15 min.
set -eo pipefail
. "$(dirname "$0")/00_common.sh"
source /opt/ros/$ROS_DISTRO/setup.bash

if ! command -v MicroXRCEAgent >/dev/null; then
  say "Building Micro XRCE-DDS Agent $XRCE_AGENT_VERSION"
  rm -rf /tmp/Micro-XRCE-DDS-Agent
  git clone -b "$XRCE_AGENT_VERSION" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent
  mkdir -p /tmp/Micro-XRCE-DDS-Agent/build && cd /tmp/Micro-XRCE-DDS-Agent/build
  cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig /usr/local/lib/
fi
ok "MicroXRCEAgent: $(command -v MicroXRCEAgent)"

say "px4_msgs ($PX4_MSGS_BRANCH) into the workspace"
if [ ! -d "$WS_DIR/src/px4_msgs" ]; then
  git clone -b "$PX4_MSGS_BRANCH" https://github.com/PX4/px4_msgs.git "$WS_DIR/src/px4_msgs"
fi
ok "px4_msgs ready. Run 04."
