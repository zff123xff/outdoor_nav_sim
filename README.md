# Go2-W Gazebo Classic 仿真

默认使用 Gazebo Classic 11、CHAMP 混合轮腿控制、MID360 扫描插件和 PX4 ECL 融合定位。仿真入口已移除其他引擎的启动选项、插件源码及构建依赖。outdoor_nav 仅作为已编译导航包的来源，本次未修改。

```bash
cd /path/to/sim_ws
bash build_sim.sh
# 终端 1：Classic、机器人、传感器和 PX4
DISPLAY=:0 bash launch_sim.sh
# 终端 2：导航和 RViz
DISPLAY=:0 bash launch_nav.sh
```

两个终端必须使用相同的 ROS_DOMAIN_ID、profile 和 robot_namespace。未显式设置通信域时，本项目使用 30；已有 RMW 和 DDS 配置保持不变。ROS 与工作空间通过 ros_env.sh 加载，导航工作空间可用 OUTDOOR_NAV_WS 指定。

从 GitHub 获取此工作空间：

```bash
git clone https://github.com/zff123xff/outdoor_nav_sim.git sim_ws
cd sim_ws
# 默认为相邻 ../outdoor_nav；如果导航工作空间在其他位置，各终端先设置：
export OUTDOOR_NAV_WS=/path/to/outdoor_nav
bash build_sim.sh
```

需要预先构建 `outdoor_nav`，提供 `install/local_setup.bash` 及导航依赖。仓库包含仿真源码、模型、配置、测试和核验报告；`build/`、`install/`、`log/` 在本地生成，不随源码上传。第三方源码保留在 `src/` 中，许可证随对应目录提供。

默认 `profile:=robotdog_nav`：agriculture.world 农田场景，GPS 参考原点 (0,0,0)，初始 XY/yaw 为 0，MID360 量程 0.1～5 m、10 Hz 发布，IMU 为 200 Hz，GPS 为 30 Hz。默认 `locomotion_mode:=hybrid`，腿部由 CHAMP + effort 轨迹控制器驱动，四轮由速度控制器驱动。

```bash
# 关闭 Classic GUI（CPU 射线传感器无需 GPU 显示）
bash launch_sim.sh gui:=false rviz:=false
# 空场对比
bash launch_sim.sh world:=empty.world gui:=false
# 固定腿 + 轮式驱动
bash launch_sim.sh locomotion_mode:=wheels
# 采用原 depot 配置与原导航参数，仍运行 Gazebo Classic
bash launch_sim.sh profile:=current
bash launch_nav.sh profile:=current
```

`profile:=current` 保留原 sim_spawn.yaml 的出生位置和底盘限速；其中包含原先设置的较高速度值。默认参考模式的底盘上限是 0.3 m/s、0.5 rad/s。

配置文件：

- `src/south_lake_sim/config/robotdog_spawn.yaml`：出生位置和底盘速度；支持 spawn_x/y/z/yaw 临时覆盖。
- `src/south_lake_sim/config/robotdog_sensors.yaml`：雷达安装、扫描范围和轮胎摩擦。
- `src/south_lake_sim/config/robotdog_nav2.yaml`：GPS 原点和导航参数。
- `src/go2w_sim/config/hybrid_champ.yaml`：参考 CHAMP 步态，nominal_height=0.225 m。
- `src/go2w_sim/config/hybrid_drive.yaml`：轮径、轮距、速度限制和指令超时。
- `src/go2w_sim/config/classic_hybrid_controllers.yaml`：12 个腿关节的 effort PID 和 4 个轮关节的速度控制器。

修改导航参数只需重启 launch_nav.sh；修改模型、场景、控制器、传感器或 GPS 原点后需要重启环境，GPS 原点变更还需重启导航。YAML 和模型采用 symlink-install，修改已有文件通常无需重编译；新增文件和 C++ 源码变更需要构建。

在 RViz 使用 2D Goal Pose 设置目标。Classic 界面不包含原引擎的自定义点击导航和键盘面板。手动控制通过 `/robot/cmd_vel_remote` 接入标准 twist_mux；收到非零遥控命令会通过 stop_planner 取消旧导航任务。导航输出为 cmd_vel_nav，mux 唯一输出为 cmd_vel。混合控制器把前进速度分配给轮子，把横移和转向交给 CHAMP，同时让轮子参与差速转向；指令中断约 0.5 秒后输出零速度。

在另一个终端使用项目自带的键盘工具（W/S 前后、A/D 转向、Q/E 横移、空格停止）：

```bash
source ros_env.sh
ros2 run go2w_sim classic_keyboard.py --ros-args -r __ns:=/robot
```

定位使用当前 PX4 ECL（GPS、IMU、磁力计、气压计）；参考项目自己的 gps_imu_fusion 不是 PX4，也不是 EKF。本项目未切换定位算法。Gazebo 真值只发布到 simulation/truth/odom 用于验证，不输入导航。

构建与运行需要 ROS 2 Humble、Gazebo Classic 11、gazebo_ros、gazebo_ros2_control、ros2_controllers 以及相邻 outdoor_nav/install。完整参数对比见 [ROBOTDOG_ALIGNMENT.md](ROBOTDOG_ALIGNMENT.md)，通信与就绪检查见 [ROS_CLI.md](ROS_CLI.md)。
