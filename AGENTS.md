# sim_ws 工作约定

这是 Gazebo Classic 11 仿真工作空间。模型、传感器、仿真启动和控制变更留在 sim_ws；不修改相邻 outdoor_nav。导航算法从 outdoor_nav/install 读取。项目已移除 Fortress 入口、插件和构建依赖。

默认 profile:=robotdog_nav、locomotion_mode:=hybrid，使用 agriculture.world、robotdog_spawn.yaml、robotdog_sensors.yaml 和 robotdog_nav2.yaml。原点为 (0,0,0)，雷达量程 5 m。profile:=current 使用 depot.world、原 sim_spawn.yaml/go2w_nav2.yaml 和固定腿轮式控制，仍是 Classic。两个终端保持相同域、命名空间、profile；切换模式或 GPS 原点需重启环境和导航。

环境入口 bash launch_sim.sh，导航入口 bash launch_nav.sh。修改导航 YAML 只需重启导航。Classic 先暂停物理，配置控制器后排队同时激活控制器并解除暂停。默认 CHAMP 唯一控制腿部 effort 轨迹，四轮由 wheel_velocity_controller 驱动；不要同时启动 stand_initializer 或 diff_drive_controller。定位保留 PX4 ECL，不使用真值作导航定位。

速度链路为 cmd_vel_nav / cmd_vel_remote → 标准 twist_mux → cmd_vel → 混合控制器。不能增加竞争仲裁器。非零遥控通过 remote_stop_bridge 取消旧导航；混合控制器有 0.5 秒墙钟输入超时。Classic 使用 RViz 目标和 ROS 遥控，没有原引擎的自定义 GUI 点击和键盘面板。

先读 ROS_CLI.md。优先 bash ros2.sh；Python ROS 工具先 source ros_env.sh。保留已有 ROS_DOMAIN_ID、RMW 和 DDS；项目默认 30、命名空间 robot 只用于未覆盖的启动配置，连接现有机器人要确认其实际配置。检查网络权限、发现和实际消息接收。权限拒绝必须报告实际错误。

仅在用户授权运动时操作现有机器人。实现验证可启动独立通信域及独立 Classic master 的测试仿真，确认域内无现有机器人；不要对其他实例发布测试速度。运动前确认 EKF ready=1、导航 active 和新鲜 TF。“前方 d 米”按当前姿态计算目标。观察本次任务的状态、实际位置和误差，发布成功不等于到达。禁止伪造定位或关闭碰撞检查。
