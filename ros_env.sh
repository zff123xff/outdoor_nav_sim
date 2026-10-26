#!/usr/bin/env bash
# Shared environment for simulation, commands and direct ROS CLI use.
# Navigation installation is optional unless OUTDOOR_NAV_OPTIONAL=0 is set.
_cmd_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_cmd_nav="${OUTDOOR_NAV_WS:-$(dirname "$_cmd_root")/outdoor_nav}"
_cmd_base="${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
[[ -r "$_cmd_base" ]] || _cmd_base="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ ! "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || (( 10#$ROS_DOMAIN_ID > 232 )); then
 echo 'ROS_DOMAIN_ID must be an integer between 0 and 232.' >&2
 return 1
fi
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
 export FASTRTPS_DEFAULT_PROFILES_FILE="$_cmd_root/simulation/config/fastdds_udp.xml"
 export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
fi
source "$_cmd_base" || return 1
if [[ -n "${HIL_INSTALL_DIR:-}" ]]; then
 _cmd_install="$HIL_INSTALL_DIR"
elif [[ -r "$_cmd_nav/install_nx/local_setup.bash" ]]; then
 _cmd_install="$_cmd_nav/install_nx"
else
 _cmd_install="$_cmd_nav/install"
fi
if [[ -r "$_cmd_install/local_setup.bash" ]]; then
 source "$_cmd_install/local_setup.bash" || return 1
elif [[ "${OUTDOOR_NAV_OPTIONAL:-1}" != 1 ]]; then
 echo "Missing navigation workspace: $_cmd_install" >&2
 return 1
fi
if [[ -z "${HIL_INSTALL_DIR:-}" && -r "$_cmd_root/install/local_setup.bash" ]]; then
 source "$_cmd_root/install/local_setup.bash" || return 1
fi
if [[ "${OUTDOOR_NAV_SYSTEM_DEPS:-0}" != 1 && "$(uname -m)" == x86_64 ]]; then
 if [[ -d "$_cmd_root/deps/twist_mux/opt/ros/humble/share/ament_index" ]]; then
  export AMENT_PREFIX_PATH="$_cmd_root/deps/twist_mux/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
 fi
 if [[ -z "${HIL_INSTALL_DIR:-}" && -d "$_cmd_root/deps/mqtt/usr" ]]; then
  export LD_LIBRARY_PATH="$_cmd_root/deps/mqtt/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$_cmd_root/deps/mqtt/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export CMAKE_PREFIX_PATH="$_cmd_root/deps/mqtt/usr:${CMAKE_PREFIX_PATH:-}"
 fi
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/outdoor_nav_sim_ros}"
mkdir -p "$ROS_LOG_DIR"
unset _cmd_root _cmd_nav _cmd_base _cmd_install
