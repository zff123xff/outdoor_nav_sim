#!/usr/bin/env bash
# Sensor / actuator half of the outdoor_nav HIL integration.
set -eo pipefail
sim_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_dir/ros_env.sh"
interface=zny
for arg in "$@"; do
    case "$arg" in interface:=*) interface="${arg#interface:=}";; esac
done
if [[ "$interface" == zny ]]; then
    set -- "spawn_config:=$sim_dir/simulation/config/zny_spawn.yaml" \
        "sensor_config:=$sim_dir/simulation/config/zny_sensors.yaml" "$@"
fi
exec ros2 launch simulation_bringup go2_px4_sim.launch.py \
    navigation:=false interface:=zny \
    params_file:="$sim_dir/simulation/config/nav.yaml" "$@"
