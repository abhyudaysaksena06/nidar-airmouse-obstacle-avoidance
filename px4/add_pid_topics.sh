#!/usr/bin/env bash
# Make PX4 publish its controller SETPOINTS to ROS 2 as well (by default it only
# publishes the actual attitude/position). Needed for the live PID logger.
# Usage: bash px4/add_pid_topics.sh ~/PX4-Autopilot   (then rebuild PX4)
set -euo pipefail
PX4="${1:-$HOME/PX4-Autopilot}"
F="$PX4/src/modules/uxrce_dds_client/dds_topics.yaml"
[ -f "$F" ] || { echo "not found: $F"; exit 1; }
add() {  # $1 topic, $2 type
  grep -qE "^\s*- topic: /fmu/out/$1$" "$F" && return
  sed -i "0,/^subscriptions:/s//  - topic: \/fmu\/out\/$1\n    type: px4_msgs::msg::$2\n\nsubscriptions:/" "$F"
  echo "added /fmu/out/$1"
}
add vehicle_attitude_setpoint      VehicleAttitudeSetpoint
add vehicle_rates_setpoint         VehicleRatesSetpoint
add vehicle_angular_velocity       VehicleAngularVelocity
add vehicle_local_position_setpoint VehicleLocalPositionSetpoint
echo "Done. Rebuild PX4: cd $PX4 && make px4_sitl"
