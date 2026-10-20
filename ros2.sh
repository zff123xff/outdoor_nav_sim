#!/usr/bin/env bash
# Run a ROS 2 CLI command with the project's validated simulation transport.
set -eo pipefail
_ros_cli_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$_ros_cli_script_dir/ros_env.sh"
exec ros2 "$@"
