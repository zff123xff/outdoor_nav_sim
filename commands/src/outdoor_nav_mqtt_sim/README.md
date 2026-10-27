# outdoor_nav_mqtt_sim

Simulation-side entry point for the hardware command-and-control (MQTT) stack.
It links the real hardware protocol implementation to the task pipeline that
lives in this repository, so a simulation run consumes the same task payloads
as the field robot.

## What it builds

| Target | Source | Purpose |
|---|---|---|
| `mqtt_gateway` | `src/mqtt_gateway.cpp` + `swarms_backend/src/mqttnode/mqttnode.cpp` | MQTT ↔ ROS protocol node |
| `task_bridge.py` | `scripts/task_bridge.py` | `multi_task_bridge` parser with its mock FCU status timer disabled |

`scripts/task_bridge.py` reuses `multi_task_bridge.node.MultiTaskBridge` from
`commands/src/multi_task_bridge/` and only removes the simulated FCU status
timer, so the task parser stays byte-identical to the hardware one.

## Build

The package has no dependency on a workspace outside this repository, so the
plain workspace build works:

```bash
colcon build --base-paths commands/src --packages-select outdoor_nav_mqtt_sim \
  --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`bash commands/setup_mqtt.sh` does the same for the whole MQTT stack
(`msg_set_msgs`, `execute_task_server`, `multi_goal_to_gp_goal`,
`multi_task_bridge` and this package) and prepares the local Paho/Mosquitto
dependencies under `deps/mqtt` on x86_64.

Native dependencies: Paho MQTT C++ (`PahoMqttCpp`), `jsoncpp` and `yaml-cpp`
for the C++ node; `paho-mqtt`, `pymap3d` and `shapely` for the Python side.
The Docker image installs these already (see `docker/Dockerfile`).

## Run

```bash
ros2 launch outdoor_nav_mqtt_sim mqtt.launch.py \
    mqtt_config:=commands/config/mqtt/client.yaml \
    nav_params:=simulation/config/nav.yaml
```

`robot_namespace` defaults to `robot_1` and must match the navigation side.
`start_pipeline:=false` reuses an already running task pipeline instead of
starting a second ExecuteTask service and waypoint queue.

## Vendored backend subset

`swarms_backend/` holds the MQTT protocol implementation, copied from the
adjacent `outdoor_nav` workspace (`src/external_pkg/swarms_backend`) so that
this package builds without it. Only the subset the gateway actually compiles
is kept — the mavlink headers, the serial/MAVLink launcher and the other
backend nodes are not part of this build and are not copied.

| Path | Role |
|---|---|
| `swarms_backend/src/mqttnode/mqttnode.cpp` | protocol implementation |
| `swarms_backend/include/mqttnode/mqttnode.hpp` | node class and topic wiring |
| `swarms_backend/include/mqttnode/mqtt_utils.h` | payload ↔ message conversion |
| `swarms_backend/include/mqttnode/mqtt_topic.h` | MQTT topic constants |
| `swarms_backend/include/mqttnode/mqtt_response.h` | response codes |
| `swarms_backend/include/mqttnode/uav_ctl_state.h` | control state enum |
| `swarms_backend/include/mavlink_ros/uav_state.h` | UAV state structs |

The copy keeps the hardware behaviour intact, including the MQTT5
`ResponseTopic`/`CorrelationData` ACK that `commands/send.sh mqtt` matches
against. Only `msg_set_msgs` is looked up at runtime through
`ament_index_cpp`, so the gateway needs no files from `swarms_backend` at
execution time.

All seven files are byte-identical to the upstream copies; upstream source is
not patched here. One upstream log line therefore stays visible: the gateway
looks for a `system_version.txt` a few directories above the executable and
logs `Can not find system_version.txt ...` when it is absent. That file
describes the hardware platform version and has no source in this repository,
so the lookup simply misses and the gateway continues to serve the protocol.

### Provenance and license

Upstream project: `https://git.cnaeit.com/XJSF/software_group/amg0523062/swarms_backend`
(the project README in that tree records the address). The upstream package
manifest declares `TODO: License declaration` and ships no license file, so
these files stay an internal-source copy with no upstream license grant; the
`Apache-2.0` in `package.xml` covers this package's own `mqtt_gateway.cpp`,
`scripts/` and `launch/` code only.

## License

Apache-2.0 (this package's own code; see the vendored subset note above)
