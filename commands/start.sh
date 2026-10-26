#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:---help}"
[[ $# == 0 ]] || shift
case "$mode" in
 -h|--help) echo 'Usage: bash commands/start.sh [tasks|mqtt|mqtt-gateway|broker] [options]'; exit 0;;
esac
OUTDOOR_NAV_OPTIONAL=0 source "$here/../ros_env.sh"
case "$mode" in
 tasks) exec python3 "$here/scripts/run_execute_task.py" --serve-only "$@";;
 mqtt) exec python3 "$here/scripts/run_mqtt_stack.py" "$@";;
 mqtt-gateway)
  exec ros2 launch outdoor_nav_mqtt_sim mqtt.launch.py robot_namespace:=robot_1 \
   mqtt_config:="$here/config/mqtt/client.yaml" nav_params:="$here/../simulation/config/nav.yaml" "$@";;
 broker)
  if command -v mosquitto >/dev/null; then
   exec mosquitto -c "$here/config/mqtt/broker.conf" "$@"
  fi
  exec "$here/../deps/mqtt/usr/sbin/mosquitto" -c "$here/config/mqtt/broker.conf" "$@";;
 *) echo "Unknown stack: $mode" >&2; exit 2;;
esac
