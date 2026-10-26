# 指令与任务接入交接

本目录只负责目标/任务下发、ExecuteTask 服务与 MQTT 接入，不启动 Gazebo、Nav2 或 RViz。以下命令均在 `outdoor_nav_sim` 根目录执行。仿真与导航的启动流程见 [仿真说明](../simulation/README.md)。

## 目录与调用链

| 路径 | 用途 |
| --- | --- |
| send.sh | GPS、UAV、航点、ExecuteTask、MQTT 和相对前方目标的统一发送入口 |
| start.sh | 启动或管理任务接收服务、MQTT 网关、本地 broker |
| setup_mqtt.sh | 按需准备依赖并构建可选 MQTT 网关 |
| scripts/ | 指令发送、任务服务复用、broker 进程管理和示例生成 |
| config/mqtt/ | 日常联调和隔离接口测试两组 broker/client 配置 |
| examples/ | 附近目标、任务输入、异常输入和原点对照信息 |
| src/outdoor_nav_mqtt_sim/ | 复用实际指控协议的薄网关 ROS 包，不包含导航算法 |

详细图见 [指令接入到运动执行链路](../simulation/README.md#指令接入到运动执行的详细链路)。

## 运行前准备

先构建相邻 `outdoor_nav` 的消息、导航和任务包，至少包括 `msg_set_msgs`、`execute_task_server`、`multi_goal_to_gp_goal`、`multi_task_bridge`、`nav2_input_bridge`。脚本读取其安装空间，不修改导航项目。默认使用同级目录，可用 `OUTDOOR_NAV_WS` 指定位置；`HIL_INSTALL_DIR` 可指定导航安装空间。

默认命名空间为 `robot_1`。ROS_DOMAIN_ID、命名空间与共享 `simulation/config/nav.yaml` 的 GPS 原点应与导航一致。已有 ROS_DOMAIN_ID、RMW 和 DDS 配置会保留；项目未配置时默认域为 30。默认 ZNY 使用系统时间；旧 Classic 接法发送工具加 `--use-sim-time`，MQTT 启动加 `use_sim_time:=true`。

```bash
# 环境、导航各一个终端
bash simulation/start.sh
bash ../outdoor_nav/launch_hil.sh params_file:="$PWD/simulation/config/nav.yaml"
# 先检查输入，不发送目标
bash commands/send.sh gp_goal --dry-run
bash commands/send.sh waypoints --dry-run
bash commands/send.sh task --dry-run
```

确认 EKF ready=1、导航 active 和新鲜 TF 后再发送实际目标。通信排查见 [ROS CLI](../ROS_CLI.md)。

## 发送入口与数据格式

### 每条指令读取哪个输入文件

下表路径均相对 `outdoor_nav_sim` 根目录。不传 `--file` 时读取对应默认文件；指定 `--file` 后使用指定文件。

| 指令 | 默认任务输入文件 | 指定其它文件 |
| --- | --- | --- |
| `bash commands/send.sh gp_goal`（别名 `gps`） | [commands/examples/gps_goal.yaml](examples/gps_goal.yaml) | `--file /path/gps.yaml` |
| `bash commands/send.sh uav/target_gps`（别名 `uav`） | [commands/examples/gps_goal.yaml](examples/gps_goal.yaml)，与 gp_goal 共用 | `--file /path/uav_gps.yaml` |
| `bash commands/send.sh waypoints`（别名 `way_points`） | [commands/examples/multi_goal.yaml](examples/multi_goal.yaml) | `--file /path/waypoints.yaml` |
| `bash commands/send.sh task` | [commands/examples/execute_task.yaml](examples/execute_task.yaml) | `--file /path/task.yaml` |
| `bash commands/send.sh mqtt` | [commands/examples/mqtt_task_stream.json](examples/mqtt_task_stream.json) | `--file /path/task_stream.json` |
| `bash commands/send.sh forward --distance 1` | 不读取任务文件；根据当前 TF 和朝向计算前方 1 米目标 | 使用 `--distance`，不支持 `--file`；省略距离时默认 5 米 |
| `bash commands/send.sh velocity --vx 0.3 --duration 3` | 不读取任务文件，速度与时长来自命令参数 | `--vx`、`--vy`、`--wz`、`--duration` |
| `bash commands/send.sh yolo_detect` | 不读取任务文件，默认发布 Bool true | `--clear` 发布 false，不恢复导航 |

GPS/UAV 也可直接传入 `--latitude`、`--longitude`、`--altitude` 三个参数，此时不读取样例文件；这三个参数必须一起提供，不能与 `--file` 同用。修改共用 gps_goal.yaml 会同时影响 GPS 和 UAV 两个入口的默认目标；需要不同目标时分别指定文件。

任务输入与运行配置分开：

| 使用入口 | 默认运行配置 | 覆盖方式 |
| --- | --- | --- |
| `send.sh waypoints`、`send.sh task`、`start.sh tasks` | [simulation/config/nav.yaml](../simulation/config/nav.yaml)，读取 GPS 参考原点 | `--params-file /path/nav.yaml` |
| `send.sh mqtt` | [commands/config/mqtt/client.yaml](config/mqtt/client.yaml)，读取 broker、认证和 vehicle_id | `--config /path/client.yaml` |
| `start.sh mqtt`、`start.sh mqtt-gateway` | 同一 client.yaml 和 simulation/config/nav.yaml | `mqtt_config:=/path/client.yaml nav_params:=/path/nav.yaml` |
| `start.sh broker` | [commands/config/mqtt/broker.conf](config/mqtt/broker.conf) | 修改此配置文件；指定其它配置时直接运行 `mosquitto -c /path/broker.conf` |

`start.sh` 各模式只启动接收链路，不读取任务输入文件、不自动发送 examples 中的任务。`examples/negative/` 仅为显式选择的异常测试输入，`examples/test_info.json` 仅记录坐标对照，均不是默认发送文件。

### 消息接口与发送方式

| 命令 | ROS 接口或协议 | 发送行为 |
| --- | --- | --- |
| send.sh gp_goal | /robot_1/gp_goal，NavSatFix | 等待订阅匹配后发布一次并退出 |
| send.sh uav/target_gps | /robot_1/uav/target_gps，NavSatFix | 同上 |
| send.sh waypoints | /robot_1/waypoint_goals，MultiGoal | 发布一次；进程保活任务链路 |
| send.sh task | /agent/robot_1/execute_task，ExecuteTask | 调用一次；进程保活任务链路 |
| send.sh mqtt | MQTT5 taskStream 请求 | 等待协议 ACK 后退出 |
| send.sh forward | /robot_1/goal_pose，PoseStamped | 检查就绪状态，按当前朝向计算并发送一次 |
| send.sh velocity | /robot_1/cmd_vel_remote，Twist | 按频率发送指定时长；结束/中断发零速度 |
| send.sh yolo_detect | /robot_1/target_detected，Bool | true 请求停止规划和取消跟踪，--clear 仅发布 false |

```bash
bash commands/send.sh gp_goal --file commands/examples/gps_goal.yaml
bash commands/send.sh uav/target_gps --latitude 40.9445353 --longitude 113.1728309 --altitude 1298.7345711
bash commands/send.sh waypoints --file commands/examples/multi_goal.yaml
bash commands/send.sh task --file commands/examples/execute_task.yaml
bash commands/send.sh forward --distance 1
# 仅启动任务服务/航点队列，不发送任务
bash commands/start.sh tasks
```

每条发送命令单独按需运行。GPS/UAV、waypoints、task、forward、velocity、yolo_detect 使用 `--namespace robot_a` 切换机器人；ExecuteTask 文件内的 agent_id 必须同步。`way_points` 只是脚本别名，实际话题为 `/{ns}/waypoint_goals`。

GPS 单目标字段是 latitude/longitude/altitude，单位为度、度、海拔米。GPS 多航点必须设置 `is_gps_aid=true`，`multi_goal_points[].x_or_lat/y_or_lon/z_or_alt` 为纬度、经度、海拔。当前队列不支持本地坐标多航点，GPS 链路不完整传递 yaw/vel。

ExecuteTask YAML 的 `request_json` 是 JSON 字符串：

```json
{"mission_id":"example","agent_id":"robot_1","capability_id":"goto_waypoints","params":{"waypoint_frame":"gps","waypoints":[{"lat":40.9445353,"lon":113.1728309,"alt":1298.7345711}]}}
```

GPS、waypoints、task、velocity 和 yolo_detect 支持 `--dry-run`；它检查可解析性及发送侧约束，不替代接收服务的能力校验。例如不支持的 capability 由原 ExecuteTask 服务拒绝。MQTT 和 forward 没有 dry-run 参数。

任务工具会复用已有服务和队列，缺失时自行启动，并核对已有队列/导航的 GPS 原点。它们追加航点，不覆盖旧队列。保持任务终端运行；Ctrl+C 只关闭本工具创建的子进程，不关闭复用的服务，也不取消已发布目标。发送成功、服务 success、MQTT ACK 均不等于到达；需观察导航动作及最终位置。

## 手动速度与 YOLO 检测停车

两项命令直接使用参数，不读取 examples 中的文件，均支持 `--dry-run` 和 `--namespace`。对应实际实现是 `scripts/publish_velocity.py`、`scripts/publish_yolo_detect.py`，不新增 shell 转发脚本。

```bash
# 本体系：前进 0.3 m/s，持续 3 秒（会发送运动指令）
bash commands/send.sh velocity --vx 0.3 --duration 3
# 左移和原地转向按需单独使用，单位分别是 m/s 与 rad/s
bash commands/send.sh velocity --vy 0.2 --duration 2
bash commands/send.sh velocity --wz 0.3 --duration 2
# 先核对数值，不发送
bash commands/send.sh velocity --vx 0.3 --duration 3 --dry-run

# 模拟识别到目标：通过原 target_detected 接口取消当前导航，并终止本工具的速度发送
bash commands/send.sh yolo_detect
# 发送检测 false；不自动恢复导航，也不清空任务队列
bash commands/send.sh yolo_detect --clear
```

vx 正向前、vy 正向左、wz 正向逆时针。默认速度均为 0，持续 1 秒，频率 20 Hz；范围为 |vx|≤1.5 m/s、|vy|≤0.6 m/s、|wz|≤1 rad/s，频率允许 10～100 Hz。发送前等待速度输入订阅，使用墙钟计时；Ctrl+C 会发送一段零速度后退出。手动开始时发送 stop_planner 取消已有路径。

速度只进入现有 twist_mux 的 cmd_vel_remote 输入（优先级 20），导航输入 cmd_vel_nav 为 10；最终 cmd_vel 仍只有仲裁器发布。脚本发布成功不代表机器人实际移动或到达，需要观察实际状态。应先按仿真说明确认通信、定位及导航就绪。

`send.sh yolo_detect` 保留命令名称，实际发布 `/{ns}/target_detected`（std_msgs/Bool），使用导航原有接口。true 通知输入桥发布 stop_planner 并请求取消 FollowPath；本项目正在运行的速度发送器也会收到事件，归零后退出。false 被导航原回调忽略，不恢复任务。

导航侧没有新增 yolo_detect 订阅或 twist_mux 检测锁；原有导航和手动速度优先级保持不变。该信号不清空航点队列、不持续禁止新的目标或其他速度来源。发送成功表示事件已发送，不代表已确认机器人停稳。

相关修改已撤回并重新构建 nav2_input_bridge、robot_launch。已有导航进程需要重启。当前隔离验证结果见 tests/results/manual_commands_target_detected.json；旧 manual_commands.json 是此前速度锁方案的历史结果。

## MQTT 安装与启动

```bash
# 首次需要 MQTT 时执行；日常启动不重复安装
bash commands/setup_mqtt.sh
# 一个终端运行栈，另一个终端发送
bash commands/start.sh mqtt
bash commands/send.sh mqtt --file commands/examples/mqtt_task_stream.json
```

x86_64 安装脚本下载并解包 Paho、Mosquitto 和 Python 依赖至 `deps/mqtt`，不安装系统服务。ARM64 使用原生系统依赖，该脚本不安装 ARM 系统库。网关构建需要相邻导航项目的 MQTT 后端源码，以及其已安装的消息/任务模块；Python 侧使用 paho-mqtt、pymap3d、shapely，C++ 侧使用 Paho、jsoncpp、yaml-cpp 等依赖。

`start.sh mqtt` 对默认 `127.0.0.1:1883` 自动启动或复用 broker；外部地址由配置指定，不启动本地 broker。可单独用 `start.sh broker` 或 `start.sh mqtt-gateway`。MQTT 栈默认同时启动 ExecuteTask 服务和队列；已有 `start.sh tasks` 时应加 `start_pipeline:=false`，避免重复节点。

```bash
# 使用外部 broker 配置；启动和发送使用同一文件
bash commands/start.sh mqtt mqtt_config:=/path/client.yaml start_pipeline:=false
bash commands/send.sh mqtt --config /path/client.yaml --file /path/task.json
```

| 配置 | 默认用途 |
| --- | --- |
| config/mqtt/client.yaml + broker.conf | 日常本地开发，127.0.0.1:1883 |
| config/mqtt/test_client.yaml + broker_test.conf | 隔离接口测试，127.0.0.1:18884 |

客户端配置包含 address、mqtt_username、mqtt_passwd、vehicle_id。凭证放在独立本地配置，避免写入版本库。ROS 命名空间和 vehicle_id 是不同标识：agent_id 匹配命名空间；JSON 中两层 dronesId 均须包含配置的 vehicle_id（0～127）。

请求主题为 `/mqtt/request/task_deploy/nx/task_stream/{vehicle_id}`，响应主题为 `/mqtt/response/task_deploy/nx/task_stream/{vehicle_id}`。发送器使用 MQTT5 ResponseTopic/CorrelationData 匹配本次 ACK。`--legacy` 改发 `/mqtt/web/task_stream/{vehicle_id}`，此路径没有应用层 ACK。

样例中 `taskType=7` 表示航线任务，`arrivePath[].x_lat/y_lon/z_alt` 表示纬度、经度、海拔；lowHeight/highHeight 为协议高度字段。网关复用原协议实现，任务解析器保留 float32 GPS 的精度限制。`task_bridge.py` 关闭原解析器的模拟 FCU 状态定时器，不伪造 MAVROS 连接状态。

## 示例维护与异常测试

本目录 8 份数据文件的逐项用途、读取入口、关键字段和预期结果见 [examples/README.md](examples/README.md)。

生成器默认单目标对应绝对 map (1,0)，两点任务对应 (1,0) → (2,0)，不随机器人当前位置变化。相对当前朝向使用 forward。`examples/test_info.json` 记录生成原点及 ENU/GPS 对照，不能作为任务文件发送。

```bash
source ros_env.sh
# 仅生成文件，不发送指令；原点改变后重新生成，并重启仿真/导航/任务队列
python3 commands/scripts/generate_examples.py --namespace robot_1
# 其他机器人生成到独立目录；MQTT vehicle_id 也要对应
python3 commands/scripts/generate_examples.py --namespace robot_a --vehicle-id 2 --data-dir /tmp/robot_a_examples
```

`examples/negative/` 包含 wrong_agent、unsupported_task、local_multi_negative 三份异常输入。它们由隔离接口测试读取，分别验证发送前拒绝、任务服务拒绝和 GPS 队列拒绝。旧绝对 Pose 模板与两份远距离手改任务已移出项目，原文暂存于 `/tmp/outdoor_nav_commands_audit_20261021/source/commands/examples/`。

## 验证与排查

```bash
source ros_env.sh
# 必须指定空闲的隔离域；测试会先检查域是否为空、18884 是否空闲
python3 tests/integration/command_interfaces.py --domain-id 103 --output /tmp/command_interfaces.json
```

测试不启动机器狗、不发布里程计，覆盖 GPS、UAV、多航点、ExecuteTask、MQTT ACK 到 MultiGoal，以及三项异常拒绝。它不验证实际运动到达；forward 的就绪条件与真实运动应在独立仿真环境验证。

| 现象 | 检查方向 |
| --- | --- |
| 找不到消息类型/模块 | 导航任务包是否编译、根目录 ros_env.sh 是否加载正确安装空间 |
| GPS 发送等待超时 | 域与命名空间、目标订阅者、Classic 时钟是否匹配 |
| 原点不一致而拒绝发送 | 两侧 params_file 与已有队列参数；按同一配置重启 |
| MQTT connection refused | broker 是否运行、启动和发送是否用了同一地址 |
| 有 ACK 但没有任务 | vehicle_id/dronesId、MultiTask 解析器和 ExecuteTask 链路 |
| 同名节点或重复队列 | 已有任务管线时使用 start_pipeline:=false |

本次核查：30 个原文件全部检查，保留 26 个、移除 4 个；新增本 README 为交接入口。可选 MQTT 包构建、Shell/Python 检查、4 个 dry-run 入口及 8 份示例再生成比对通过。ROS 域 103 的五条正常链路和三项异常测试通过，未收到模拟 FCU 状态。结果见 [接口验证记录](../tests/results/commands_cleanup_interfaces.json)，逐文件理由见 [FILE_AUDIT.md](FILE_AUDIT.md)。

公共环境已合并到项目根目录 `ros_env.sh`；send/start 会加载它并要求导航安装空间存在，直接 ROS CLI 先 `source ros_env.sh`。commands 不再单独维护 env.sh，也不包装导航启动。

接口回归输入在 /tmp 临时生成，不覆盖手改 examples；GPS 发布器等待实际 nav2_input_bridge 订阅完成匹配，避免只匹配到调试订阅时漏发。新版速度/停车及原有指令链路验证记录分别见 [手动控制](../tests/results/manual_commands.json) 和 [指令回归](../tests/results/commands_with_manual_regression.json)。
