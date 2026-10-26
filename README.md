# Go2-W 仿真与导航联调

项目按两部分组织：`simulation/` 运行仿真与传感器数据处理，`commands/` 下发目标、任务并接入 MQTT。导航算法仍由相邻 `outdoor_nav` 提供。

```text
outdoor_nav_sim/
├── simulation/
│   ├── start.sh                  # 纯环境：模型、传感器、PX4 ECL、TF、执行速度
│   ├── config/nav.yaml           # 仿真、导航、任务队列共用的参数与 GPS 原点
│   ├── config/fastdds_udp.xml
│   └── src/
│       ├── sim_bringup/          # 环境启动与仿真定位传感器生成
│       ├── sim_robot/            # Go2-W 模型、关节控制与运动执行
│       ├── sim_world/            # 场景、地图、环境资源及接口辅助节点
│       ├── sim_lidar/            # MID360 点云仿真
│       ├── sim_localization/     # PX4 ECL 定位融合
│       └── third_party/champ/    # 上游 CHAMP 步态控制库
├── commands/
│   ├── send.sh                   # GPS / UAV / 多航点 / ExecuteTask / MQTT
│   ├── start.sh                  # 任务服务、MQTT 网关、本地 broker
│   ├── config/mqtt/              # 指控连接参数
│   ├── examples/                 # 附近目标、任务输入与异常测试样例
│   ├── scripts/                  # 指令发布与任务接入
│   └── src/outdoor_nav_mqtt_sim/  # 复用原项目 MQTT 协议的薄网关
├── ROS_CLI.md                    # 项目公共 ROS 通信排查
├── tests/                        # 集成测试与验证结果
├── build_sim.sh                  # 构建仿真主体并运行测试
└── ros_env.sh                    # 公共 ROS 环境
```

## 启动

在本目录运行 `bash build_sim.sh` 构建仿真主体；导航端另需编译 outdoor_nav。MQTT 为可选功能，需要时运行 `bash commands/setup_mqtt.sh`。

仿真电脑：

```bash
ROS_DOMAIN_ID=30 bash simulation/start.sh
# 无窗口：追加 gui:=false rviz:=false
```

导航电脑或 NX：

```bash
cd ../outdoor_nav
ROS_DOMAIN_ID=30 bash deploy/nx/run.sh hil
# 本机也可在仿真项目运行 bash ../outdoor_nav/launch_hil.sh params_file:="$PWD/simulation/config/nav.yaml"
```

默认命名空间为 `robot_1`，对外采用北京 ZNY 接口；内部仍是 Gazebo Classic / CHAMP 混合轮腿、农田场景、PX4 ECL 定位和 5 米 MID360 点云。导航默认使用系统时间，导航负责公共定位与雷达 TF。两端使用同一域、命名空间及 `simulation/config/nav.yaml`。默认 GPS 原点为 `(40.9445353, 113.172819, 1298.7345711102703)`，支持现有 GPS 多航点队列。

环境启动不下达任务。等待 EKF `ready=1`、导航 active 后，在第三个终端选择一种指令：

```bash
bash commands/send.sh gp_goal
bash commands/send.sh uav/target_gps
bash commands/send.sh waypoints
bash commands/send.sh task
```

上面每条命令均会发送目标或任务，按需单独运行。详细参数、MQTT 指控、坐标约定见 [指令接入](commands/README.md)；跨机/NX 部署见 [HIL 接入](simulation/README.md#导航对接接口)。

`waypoints` 对应真实 ROS 话题 `/{ns}/waypoint_goals`，不是 `/way_points`。提交成功表示已接收，不表示已到达。修改 GPS 原点后需同步重启仿真、导航和任务队列。



运行调用链、逐文件清理依据及结果见 [仿真流程与清理记录](simulation/README.md)

当前链路图见 [仿真、导航与指令关系图](simulation/README.md#当前-zny-模式整体链路)，指令细分见 [任务到运动执行链路](simulation/README.md#指令接入到运动执行的详细链路)。
