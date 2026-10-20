#!/usr/bin/env bash
# Source this file to connect this shell to the local Go2-W simulation.
_ros_cli_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_ros_cli_nav="${OUTDOOR_NAV_WS:-$(dirname "$_ros_cli_ws")/outdoor_nav}"
_ros_cli_base="${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
if [[ ! -r "$_ros_cli_base" ]]; then
    _ros_cli_base="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
for _ros_cli_setup in "$_ros_cli_base" "$_ros_cli_nav/install/local_setup.bash" "$_ros_cli_ws/install/local_setup.bash"; do
    if [[ ! -r "$_ros_cli_setup" ]]; then
        echo "Missing ROS setup file: $_ros_cli_setup" >&2
        return 1 2>/dev/null || exit 1
    fi
done
if [[ ! "${ROS_DOMAIN_ID:-30}" =~ ^[0-9]+$ ]] || (( 10#${ROS_DOMAIN_ID:-30} > 232 )); then
    echo 'ROS_DOMAIN_ID must be an integer between 0 and 232.' >&2
    return 1 2>/dev/null || exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_ros_cli_ws/config/fastdds_udp.xml"
    export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
fi
source "$_ros_cli_base" || return 1
source "$_ros_cli_nav/install/local_setup.bash" || return 1
source "$_ros_cli_ws/install/local_setup.bash" || return 1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/south_lake_ros_cli}"
mkdir -p "$ROS_LOG_DIR"
unset _ros_cli_ws _ros_cli_nav _ros_cli_setup _ros_cli_base
