#!/usr/bin/env bash
# Copy the NIDAR drone model, airframe and practice arena into a PX4-Autopilot checkout.
# Usage: bash px4/install_into_px4.sh ~/PX4-Autopilot
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
[ -d "$PX4/Tools/simulation/gz/models" ] || { echo "Not a PX4-Autopilot folder: $PX4"; exit 1; }
rm -rf "$PX4/Tools/simulation/gz/models/nidar_drone"
cp -r "$HERE/models/nidar_drone" "$PX4/Tools/simulation/gz/models/"
mkdir -p "$PX4/Tools/simulation/gz/models/nidar_drone/meshes"
cp "$HERE"/../ros2_ws/src/nidar_drone_description/meshes/*.STL "$PX4/Tools/simulation/gz/models/nidar_drone/meshes/"
# NIDAR AirMouse practice arena (rulebook v2.2) as a PX4 world
cp "$HERE"/../ros2_ws/src/nidar_drone_description/worlds/nidar_arena.sdf "$PX4/Tools/simulation/gz/worlds/"
AF="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
cp "$HERE/airframes/4910_gz_nidar_drone" "$AF/"
if ! grep -q "4910_gz_nidar_drone" "$AF/CMakeLists.txt"; then
  # register the airframe right after the x500 entry
  sed -i '0,/4001_gz_x500/s//4001_gz_x500\n\t4910_gz_nidar_drone/' "$AF/CMakeLists.txt"
fi
grep -q "4910_gz_nidar_drone" "$AF/CMakeLists.txt" && echo "Installed. Now run:  cd $PX4 && make px4_sitl gz_nidar_drone"
