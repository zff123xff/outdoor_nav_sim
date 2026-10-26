#!/usr/bin/env bash
# Install relocatable MQTT dependencies locally and build the hardware protocol entry point.
set -eo pipefail
sim_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cache="$sim_dir/deps/mqtt/debs"
if [[ "$(uname -m)" == x86_64 ]]; then
  mkdir -p "$cache"
  cd "$cache"
  apt download libpaho-mqttpp-dev libpaho-mqttpp3-1 libpaho-mqtt-dev libpaho-mqtt1.3 \
    mosquitto libmosquitto1 libcjson1 libdlt2 libwebsockets16 libwrap0 libev4 \
    python3-pymap3d python3-paho-mqtt python3-shapely libgeos-c1v5 libgeos3.10.2
  for archive in "$cache"/*.deb; do
    dpkg-deb -x "$archive" "$sim_dir/deps/mqtt"
  done
else
  echo "ARM64 uses native system Paho/MQTT/Python dependencies; no x86 libraries will be loaded."
fi
source "$sim_dir/ros_env.sh"
nav_ws="${OUTDOOR_NAV_WS:-$(dirname "$sim_dir")/outdoor_nav}"
python3 -c 'from msg_set_msgs.msg import MultiTask; from execute_task_server.srv import ExecuteTask; import multi_task_bridge.node' || {
  echo 'Build outdoor_nav task/message packages first; this installer does not modify that workspace.' >&2
  exit 1
}
cd "$sim_dir"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths simulation/src commands/src --packages-select outdoor_nav_mqtt_sim \
  --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOUTDOOR_NAV_SOURCE="$nav_ws" -DBUILD_TESTING=OFF
