# sim_ws 工作约定

这是 Gazebo Classic 11 仿真工作空间。模型、传感器、仿真启动和控制变更留在 sim_ws；不修改相邻 outdoor_nav。导航算法从 outdoor_nav/install 读取。项目已移除 Fortress 入口、插件和构建依赖。

项目分为 simulation/（环境、传感器、PX4 数据处理）和 commands/（指令、任务和 MQTT 接入）。默认 profile:=robotdog_nav、locomotion_mode:=hybrid，使用 agriculture.world、robotdog_spawn.yaml、robotdog_sensors.yaml；共享参数位于 simulation/config/nav.yaml，使用非零 GPS 原点支持航点队列，雷达量程 5 m。profile:=current 使用 depot.world、原 sim_spawn.yaml/go2w_nav2.yaml 和固定腿轮式控制，仍是 Classic。两个终端保持相同域、命名空间、profile；切换模式或 GPS 原点需重启环境和导航。ZNY 的外参、采样率与限速见 simulation/config/zny_*.yaml；该外参也由 launch 固定核对。

环境入口 bash simulation/start.sh，导航直接使用 outdoor_nav/launch_hil.sh（NX 使用 outdoor_nav/deploy/nx/run.sh hil）；默认 ZNY 接口（导航发布公共 TF、公共传感器系统时间），interface:=classic 保留旧接法。指令统一使用 bash commands/send.sh，任务接入使用 bash commands/start.sh。修改导航 YAML 只需重启导航。Classic 先暂停物理，配置控制器后排队同时激活控制器并解除暂停。默认 CHAMP 唯一控制腿部 effort 轨迹，四轮由 wheel_velocity_controller 驱动；不要同时启动 stand_initializer 或 diff_drive_controller。定位保留 PX4 ECL，不使用真值作导航定位。

外接导航时环境不运行 twist_mux；速度链路为导航侧标准 twist_mux → cmd_vel → 仿真混合控制器。ZNY 模型 TF 仅发到 simulation/tf 和 simulation/tf_static；导航负责公共 TF，仿真不重复发布。旧内置导航入口仍保留 cmd_vel_remote 优先链路。不能增加竞争仲裁器。非零遥控通过 remote_stop_bridge 取消旧导航；混合控制器有 0.5 秒墙钟输入超时。Classic 使用 RViz 目标和 ROS 遥控，没有原引擎的自定义 GUI 点击和键盘面板。

先读 ROS_CLI.md。ROS CLI 和 Python ROS 工具先 source ros_env.sh，然后直接运行 ros2 或 python3。保留已有 ROS_DOMAIN_ID、RMW 和 DDS；项目默认 30、命名空间 robot_1 只用于未覆盖的启动配置，连接现有机器人要确认其实际配置。检查网络权限、发现和实际消息接收。权限拒绝必须报告实际错误。

仅在用户授权运动时操作现有机器人。实现验证可启动独立通信域及独立 Classic master 的测试仿真，确认域内无现有机器人；不要对其他实例发布测试速度。运动前确认 EKF ready=1、导航 active 和新鲜 TF。“前方 d 米”按当前姿态计算目标。观察本次任务的状态、实际位置和误差，发布成功不等于到达。禁止伪造定位或关闭碰撞检查。

包职责：`px4_gps_imu` 仅负责传感器消息到定位估计，不加入 Gazebo 或导航启动依赖；`simulation_bringup` 负责仿真传感器插件和环境编排；导航启动源码放在 `simulation/src/sim_bringup/launch/`。定位上游 ECL/Matrix 与融合测试不能作为无用资源删除。

源码目录统一为 `simulation/src/sim_{bringup,robot,world,lidar,localization}`，第三方 CHAMP 放在 `simulation/src/third_party/champ`。ROS 包名保持不变；目录与包名对照见 `simulation/README.md`，不要假定包名等于源码目录名。

逐文件清理依据见 `simulation/README.md` 和 `simulation/FILE_AUDIT.tsv`；新资源必须有运行、测试或来源/许可用途，模型资源应沿 SDF/DAE/material 引用核验。环境直接启动 `go2_px4_sim.launch.py`，不再经过单纯转发的 go2w 包装 launch。

文档按职责就近维护：commands/README.md 为指令与 MQTT 交接，simulation/README.md 为环境流程和对接，包级实现记录合入对应 README。已有说明应整合，不新增重复 docs 副本。
