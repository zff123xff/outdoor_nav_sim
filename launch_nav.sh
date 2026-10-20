#!/usr/bin/env bash
set -eo pipefail
sim_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$sim_dir/ros_env.sh"
exec ros2 launch px4_gps_imu go2_px4_nav.launch.py "$@"
