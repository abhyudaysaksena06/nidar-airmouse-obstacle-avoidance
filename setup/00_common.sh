# Shared settings for the setup scripts (sourced, not run).
ROS_DISTRO=jazzy
PX4_VERSION=v1.16.0
PX4_MSGS_BRANCH=release/1.16
XRCE_AGENT_VERSION=v2.4.3
PX4_DIR="$HOME/PX4-Autopilot"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"
say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m    OK: %s\033[0m\n' "$*"; }
fail() { printf '\033[1;31m    FAILED: %s\033[0m\n' "$*"; exit 1; }
[ "$(id -u)" -eq 0 ] && fail "run as your normal user, not with sudo (the script asks for sudo itself)"
. /etc/os-release
[ "$VERSION_ID" = "24.04" ] || echo "WARNING: these scripts are made for Ubuntu 24.04, you have $PRETTY_NAME"
