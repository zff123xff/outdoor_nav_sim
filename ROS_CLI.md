# Classic 仿真 ROS CLI

```bash
cd /mnt/d/shixi/south_lake_institution/sim_ws
bash ros2.sh topic list --no-daemon --spin-time 5
timeout 15 bash ros2.sh topic echo /clock rosgraph_msgs/msg/Clock --once
timeout 15 bash ros2.sh topic echo /robot/ekf/status std_msgs/msg/String --once
bash ros2.sh lifecycle get /robot/controller_server
bash ros2.sh lifecycle get /robot/straight_planner
```

先确认 socket 网络权限、实际通信域和收到真实消息。仅发现话题不代表收到数据。ros_env.sh 加载 ROS、outdoor_nav/install、sim_ws/install；保留已有 RMW、DDS 和 ROS_DOMAIN_ID，未设置时才使用本项目的 domain 30 和 Fast DDS UDP 配置。自定义域、命名空间、profile 时，两个启动入口和 CLI 都需匹配。

```bash
ROS_DOMAIN_ID=31 bash ros2.sh topic list --no-daemon --spin-time 5
source ros_env.sh
ros2 control list_controllers
```

Classic 启动时暂停物理仿真，完成模型生成和控制器配置后，排队激活全部控制器并解除暂停。等 EKF ready=1、导航 active 和实时 map→base_link TF 有效后再控制。混合模式应看到 joint_state_broadcaster、leg_trajectory_controller、wheel_velocity_controller 三个 active 控制器；不会同时运行固定腿控制器或 diff_drive_controller。

用户授权运动后，先读取实时位姿：

```bash
timeout 15 bash ros2.sh topic echo /robot/odom nav_msgs/msg/Odometry --once --field pose.pose
timeout 15 bash ros2.sh run tf2_ros tf2_echo robot/map robot/base_link
```

“前方 d 米”必须按当前 yaw 计算：目标 (x+d*cos(yaw), y+d*sin(yaw))，不得假定当前位置或朝向。已有 scripts/navigate_forward.py 会检查就绪并计算目标；只有获得运动授权时才执行。发布成功不代表到达，要观察当前 action 状态、实际位置与目标误差，不能使用历史任务成功状态。

默认在 RViz 使用 2D Goal Pose。手动速度入口为 /robot/cmd_vel_remote，twist_mux 保持遥控优先；非零手动命令会通过 stop_planner 取消旧导航。取消也可使用现有 /robot/target_detected 接口：

```bash
bash ros2.sh topic pub --once /robot/target_detected std_msgs/msg/Bool '{data: true}'
```

不要发布虚假定位 TF，不要关闭碰撞检查。simulation/truth/odom 仅用于实际运动验证，不作为定位输入。
