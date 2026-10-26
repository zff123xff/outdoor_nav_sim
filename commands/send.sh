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
# Standard-message senders can address navigation on another machine. Load a
# local overlay when present, but do not require the navigation implementation.
source "$here/../ros_env.sh"
case "$mode" in
 waypoints|way_points|task)
  if ! python3 -c 'from execute_task_server.srv import ExecuteTask; from msg_set_msgs.msg import MultiGoal'; then
   echo '任务发送需要本机可导入 execute_task_server/ExecuteTask 和 msg_set_msgs/MultiGoal。' >&2
   echo '请在当前容器编译并 source 相应 Humble 接口包，或在已安装任务包的 NX 上运行该命令。' >&2
   echo '仅发送 GPS 单目标可用：bash commands/send.sh gp_goal。' >&2
   exit 1
  fi;;
esac
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
