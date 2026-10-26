# 指令输入样例说明

本目录包含 8 份数据文件，供发送入口、隔离接口测试和人工核对使用。文件放在这里不会自动下发；只有运行发送命令才会读取。以下命令均在 `outdoor_nav_sim` 根目录执行。完整运行前提和接口说明见 [commands/README.md](../README.md)。

## 每个文件的用途

| 文件 | 用来验证什么 | 谁读取它 | 正常或预期结果 |
| --- | --- | --- | --- |
| [gps_goal.yaml](gps_goal.yaml) | 一个 GPS 目标能否进入导航；GPS 和 UAV 两个入口使用相同格式 | `send.sh gp_goal`、`send.sh uav/target_gps` 默认读取 | 发出一条 NavSatFix，经导航输入桥转换为地图目标；生成器默认目标约为绝对 map (1,0)，手改后以文件实际坐标为准 |
| [multi_goal.yaml](multi_goal.yaml) | 直接发送 GPS 航点列表，验证航点队列 | `send.sh waypoints` 默认读取 | 发出一个含两点的 MultiGoal，队列按到达情况依次输出 GPS 目标 |
| [execute_task.yaml](execute_task.yaml) | 通过任务服务提交航线，验证 ExecuteTask → MultiGoal | `send.sh task` 默认读取 | 服务接受 goto_waypoints 请求，生成两点 GPS 航点任务；success 表示接受，不表示到达 |
| [mqtt_task_stream.json](mqtt_task_stream.json) | 按实际指控协议，从 MQTT 一直传递到任务服务和航点队列 | `send.sh mqtt` 默认读取 | 匹配本次请求的 MQTT ACK，并经 MultiTask、ExecuteTask 形成航点任务；ACK 不表示到达 |
| [negative/wrong_agent.yaml](negative/wrong_agent.yaml) | 任务发给错误机器人时能否拒绝 | 隔离接口测试；手动指定给 `send.sh task` | agent_id=wrong_robot，与默认 robot_1 不符，发送脚本在提交前拒绝 |
| [negative/unsupported_task.yaml](negative/unsupported_task.yaml) | 接收服务能否拒绝不支持的任务类型 | 隔离接口测试；手动指定给 `send.sh task` | capability_id=unsupported_test，原 ExecuteTask 服务返回 error，不生成 MultiGoal |
| [negative/local_multi_negative.yaml](negative/local_multi_negative.yaml) | GPS 航点队列能否拒绝本地坐标输入 | 隔离接口测试；手动指定给 `send.sh waypoints` | is_gps_aid=false，队列输出警告并跳过，不生成 GPS 目标 |
| [test_info.json](test_info.json) | 人工核对生成参数、参考原点及 ENU/GPS 坐标是否对应 | 人工查看与示例维护，不作为发送输入 | 记录 namespace、reference、targets_enu、targets_gps；不能传给 send.sh 当任务发送 |

多航点、ExecuteTask 和 MQTT 三份正常输入表达同一组附近航点，但分别测试不同的接入链路，不能互换文件格式。两个航点是绝对 map (1,0) → (2,0)，不是当前机器人前方；相对当前位置使用 `send.sh forward --distance 1`，该命令不读取本目录。

## 正常文件的关键字段

| 文件 | 修改目标时看哪些字段 |
| --- | --- |
| gps_goal.yaml | latitude、longitude、altitude，单位为度、度、海拔米。发送器读取这三个字段；文件中的 header/status 仅展示 NavSatFix 结构，实际发送时由脚本生成 |
| multi_goal.yaml | multi_goal_points 下的 x_or_lat/y_or_lon/z_or_alt 表示纬度/经度/海拔；is_gps_aid=true 表示 GPS 输入。当前 GPS 链路不完整传递 yaw/vel |
| execute_task.yaml | request_json 是一整个 JSON 字符串；agent_id 匹配 --namespace，capability_id 为 goto_waypoints，params.waypoint_frame=gps，params.waypoints 中使用 lat/lon/alt |
| mqtt_task_stream.json | data.multiTask 内两层 dronesId 都要包含 client.yaml 的 vehicle_id；taskStream 中 taskType=7，arrivePath 的 x_lat/y_lon/z_alt 为 GPS 航点，taskId 为任务标识 |
| test_info.json | reference 是生成时的 GPS 原点，targets_enu 与 targets_gps 一一对应；只改此文件不会改变任何发送目标 |

GPS 和 UAV 默认共用 gps_goal.yaml，修改它会同时改变两个入口的默认目标。需要不同目标时复制文件，发送时分别指定 `--file`。

## 使用方式

```bash
# 核对正常输入，不发送任务
bash commands/send.sh gp_goal --file commands/examples/gps_goal.yaml --dry-run
bash commands/send.sh waypoints --file commands/examples/multi_goal.yaml --dry-run
bash commands/send.sh task --file commands/examples/execute_task.yaml --dry-run
# 仅检查 MQTT JSON 能否解析；不连接 broker
python3 -m json.tool commands/examples/mqtt_task_stream.json
```

正常输入准备好、仿真和导航就绪后，按需选择发送入口；MQTT 使用 `bash commands/send.sh mqtt --file commands/examples/mqtt_task_stream.json`，前置栈启动步骤见主 README。MQTT 发送器没有 dry-run 参数。

异常样例通过 `tests/integration/command_interfaces.py` 在空闲的隔离 ROS 域中验证。`--dry-run` 仅做发送侧检查：wrong_agent 会被它拒绝，但 unsupported_task 的能力拒绝发生在实际 ExecuteTask 服务中，local_multi_negative 的拒绝发生在实际 GPS 队列中。后二者能被解析不代表接收端支持。

## 生成与维护

8 份数据文件由 `commands/scripts/generate_examples.py` 生成，本 README 人工维护。修改 GPS 原点、机器人命名空间或 MQTT vehicle_id 后，按对应配置重新生成：

```bash
source ros_env.sh
python3 commands/scripts/generate_examples.py --namespace robot_1 --vehicle-id 1
# 生成到独立目录，便于核对后再使用
python3 commands/scripts/generate_examples.py --namespace robot_a --vehicle-id 2 --data-dir /tmp/robot_a_examples
```

生成器只写文件，不发送目标，不修改共享导航参数。默认原点读取 `simulation/config/nav.yaml`，也可用 `--params-file` 指定。手改默认数据会在再次生成时被覆盖；需要长期保存的自定义任务放在独立路径，通过 `--file` 使用。重新生成后仍需确保导航、仿真和队列使用同一 GPS 原点。

直接速度 `send.sh velocity` 和检测停车 `send.sh yolo_detect` 也不读取本目录，分别通过速度/时长参数、true/false 检测状态运行，详见 [主 README](../README.md#手动速度与-yolo-检测停车)。

坐标说明以生成器默认数据为基准；若手动修改了目标，实际运动目标以 YAML/JSON 文件为准，test_info.json 不会自动随手改同步。
