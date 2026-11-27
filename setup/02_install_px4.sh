#!/usr/bin/env bash
# STEP 2: PX4-Autopilot (SITL) + our drone model.  ~30-60 min (download + compile).
set -eo pipefail
. "$(dirname "$0")/00_common.sh"

if [ ! -d "$PX4_DIR/.git" ]; then
  say "Downloading PX4 $PX4_VERSION (about 2 GB, full history needed for its version check)"
  git clone --recursive --branch "$PX4_VERSION" https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
  ok "PX4 already downloaded at $PX4_DIR"
fi
cd "$PX4_DIR"

say "PX4 Python packages"
# ROS installs Ubuntu's numpy, which pip cannot uninstall -> PX4's own script fails.
# Installing a pip numpy alongside first avoids that error.
python3 -m pip install --break-system-packages --ignore-installed "numpy<2"
python3 -m pip install --break-system-packages -r Tools/setup/requirements.txt

say "PX4 system dependencies (official script, simulation only)"
bash Tools/setup/ubuntu.sh --no-nuttx

say "Adding the NIDAR drone model and airframe to PX4"
bash "$REPO_DIR/px4/install_into_px4.sh" "$PX4_DIR"
bash "$REPO_DIR/px4/add_pid_topics.sh" "$PX4_DIR"

say "Compiling PX4 SITL (first time takes 10-30 min)"
make px4_sitl
ok "PX4 built. REBOOT the VM now (the PX4 script changed groups/permissions), then run 03."
