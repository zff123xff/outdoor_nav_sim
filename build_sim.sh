#!/usr/bin/env bash
set -eo pipefail
sim_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -r "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash" ]]; then
    source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
else
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
source "${OUTDOOR_NAV_WS:-$(dirname "$sim_dir")/outdoor_nav}/install/local_setup.bash"
cd "$sim_dir"
CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" colcon build \
  --base-paths src --packages-select mid360_simulation champ_msgs champ champ_base twist_mux south_lake_sim go2w_sim px4_gps_imu --executor sequential --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "$@"
source "$sim_dir/install/local_setup.bash"
ctest --test-dir build/go2w_sim --output-on-failure
ctest --test-dir build/px4_gps_imu --output-on-failure
