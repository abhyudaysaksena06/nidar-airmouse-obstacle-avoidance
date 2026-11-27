#!/usr/bin/env bash
# STEP 4: QGroundControl (ground station).  ~5 min.
set -eo pipefail
. "$(dirname "$0")/00_common.sh"

say "QGroundControl prerequisites"
sudo usermod -a -G dialout "$USER"
sudo apt-get remove -y modemmanager || true
sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
  libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev

QGC="$HOME/QGroundControl.AppImage"
if [ ! -f "$QGC" ]; then
  say "Downloading QGroundControl"
  wget -O "$QGC" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage \
    || wget -O "$QGC" https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage
fi
chmod +x "$QGC"
ok "QGroundControl at $QGC  (start it with: ~/QGroundControl.AppImage). Log out/in once for the dialout group."
