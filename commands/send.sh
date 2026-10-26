#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:---help}"
[[ $# == 0 ]] || shift
case "$mode" in
 -h|--help)
  echo 'Usage: bash commands/send.sh [gp_goal|uav/target_gps|waypoints|task|mqtt|forward|velocity|yolo_detect] [options]'
  echo 'GPS: --file FILE or --latitude LAT --longitude LON --altitude ALT; --namespace robot_1'
  echo 'Waypoints/task: --file FILE --namespace robot_1; --dry-run validates without sending.'
  echo 'Velocity: --vx MPS --vy MPS --wz RADPS --duration SEC; ends with zero velocity.'
  echo 'Detection: yolo_detect publishes target_detected=true; --clear sends false, without resuming navigation.'
  exit 0;;
esac
OUTDOOR_NAV_OPTIONAL=0 source "$here/../ros_env.sh"
case "$mode" in
 gp_goal|gps) exec python3 "$here/scripts/publish_gps.py" --topic gp_goal "$@";;
 uav/target_gps|uav) exec python3 "$here/scripts/publish_gps.py" --topic uav/target_gps "$@";;
 waypoints|way_points) exec python3 "$here/scripts/run_execute_task.py" --mode multi-goal "$@";;
 task) exec python3 "$here/scripts/run_execute_task.py" "$@";;
 mqtt) exec python3 "$here/scripts/publish_mqtt_task.py" "$@";;
 velocity) exec python3 "$here/scripts/publish_velocity.py" "$@";;
 yolo_detect) exec python3 "$here/scripts/publish_yolo_detect.py" "$@";;
 forward) exec python3 "$here/scripts/navigate_forward.py" "$@";;
 *) echo "Unknown command: $mode" >&2; exit 2;;
esac
