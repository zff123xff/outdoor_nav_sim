# 仿真环境启动与传感器生成

`launch/` 编排 Gazebo、模型、控制器、雷达与 PX4 定位节点；`src/px4_classic_sensors.cpp` 根据物理状态生成带噪声 GPS、地速、磁场、气压及仅供验证的真值。原生 Gazebo 插件提供 IMU。

统一入口：`bash simulation/start.sh`。底层入口：`ros2 launch simulation_bringup go2_px4_sim.launch.py`。

定位融合仍由独立的 `px4_gps_imu` 包承担。本包使用它导出的 PX4 地理数学库，不复制上游代码。`test/` 验证启动配置与 HIL/ZNY 接口契约。导航统一在 outdoor_nav 项目启动，本包不再启动导航节点。


## ZNY 接口实现与验证

启动方式与接口契约统一见 [仿真说明](../../README.md#导航对接接口)。以下代码路径以项目根目录为基准。

日期：2026-10-21。导航侧已提供独立 ZNY 入口，本次将仿真默认入口与它对齐。
Gazebo 引擎仍是 Classic，`interface:=zny/classic` 区分的是对接协议模式。

### 对齐内容

| 项目 | 现在的 ZNY 模式 |
| --- | --- |
| odom | /robot_1/mavros/odom；Odometry；BEST_EFFORT；ENU 位姿、本体系 twist；系统时间戳 |
| 点云 | /robot_1/livox/lidar；PointCloud2；BEST_EFFORT；xyz FLOAT32，offset 0/4/8，point_step=12 |
| 速度输入 | /robot_1/cmd_vel；Twist；RELIABLE；本体系 vx/vy/wz |
| 雷达安装 | 相对 base_link 为 (0.18,0,0.53)m，前倾 18° |
| 执行端限幅 | vx ±1.5 m/s、vy ±0.6 m/s、wz ±1.0 rad/s |
| 超时 | 0.5 秒墙钟无新指令，腿轮输出均归零 |
| 公共 TF | 由仿真生成；仿真模型 TF 只发到 /robot_1/simulation/tf 和 tf_static |
| 仿真内部 | Gazebo、CHAMP、控制器、GPS/IMU 与 PX4 ECL 仍使用仿真时间 |

原始 EKF odom 位于 simulation/ekf_odom，保持原传感器时间。
对外 odom/点云在输出真实新估计或新扫描时使用系统时间，导航无需订阅 /clock。
Gazebo 的 /clock 仍供内部节点使用。真值 simulation/truth/odom 仅用于验证，不输入导航。

odom twist 继续遵循 ROS Odometry 的 child_frame_id 约定；北京文档中将 twist 归为世界系的描述没有照搬。
点云坐标始终位于 livox_frame，导航公共固定 TF 与真实模型安装一致。

### 实现位置

- `simulation/start.sh`：默认 interface:=zny，选择 ZNY 出生/速度配置与传感器配置。
- `simulation/config/zny_spawn.yaml`、`zny_sensors.yaml`：约定限速和安装位姿；雷达配置中的 z=0.09417389 是相对 go2w_body，叠加 body 的 0.43582611 后为 base_link 上方 0.53 米。
- `simulation/src/sim_bringup/launch/go2_px4_sim.launch.py`：ZNY 固定核对外参、限速和节点职责；隐藏模型公共 TF，关闭仿真导航、mux 与遥控取消桥；公共定位 TF 由导航发布。
- `simulation/src/sim_localization/src/px4_ekf_node.cpp`：新增 ZNY 输出 QoS/系统时间戳；保留原内部估计数据。ZNY 输出频率上限为仿真时间 100 Hz，满足墙钟 ≥30 Hz 需以实测为准。
- `simulation/src/sim_robot/urdf/go2w.xacro`：向雷达插件传递模式，分离内部触发频率与对外扫描目标频率。
- `simulation/src/sim_lidar/{include,src}`：ZNY 发布 xyz 三字段点云和 BEST_EFFORT；以单调墙钟约 10 Hz 计算新射线扫描。
- `commands/scripts/{publish_gps,navigate_forward,run_execute_task}.py`：指令和任务默认系统时间，--use-sim-time 供旧模式使用。
- `commands/src/outdoor_nav_mqtt_sim/launch/mqtt.launch.py`：MQTT 任务链默认系统时间。
- `simulation/config/nav.yaml`：导航角速度上限对齐 1.0 rad/s。

首次实测中仿真实时因子约 0.76，原来的 10 Hz 仿真时钟扫描仅得到 7.6 Hz 墙钟输出。
因此 ZNY 将内部传感器触发提高至 100 Hz 仿真时间，在插件内按单调墙钟约 10 Hz 执行新扫描；每条消息来自本次新扫描，不重发旧点云凑频率。

### 兼容旧模式

ZNY 与旧 Classic 的启动和时钟选项统一见 [仿真说明](../../README.md#原-classic-接法)。ZNY 需要 hybrid 模式支持横移；两种模式不可交叉连接。

### 验证

运行 `bash build_sim.sh` 后，PX4 融合、两种模式的 launch 契约、腿轮限速/超时以及生命周期服务回归均通过。
额外运行 `tests/integration/command_interfaces.py --domain-id 94`，五条指令链路通过：gp_goal、uav/target_gps、waypoints、ExecuteTask、MQTT。

以下为 TF 迁移前的历史测试记录。独立 ROS 域 97、Gazebo master http://127.0.0.1:11370：

- 导航启动前：odom 墙钟约 73.3 Hz，点云约 9.39 Hz；公共 TF 发布者为零。
- 导航启动后：odom 约 72.9 Hz、点云约 9.38 Hz；公共 TF 只来自导航的三个节点。
- 雷达外参、xyz 布局、BEST_EFFORT、帧名与系统时间戳验证通过。
- `run.sh hil` 两个导航节点 active；根据当前朝向下发前方 1 米目标，动作返回成功，估计平面误差约 0.297 米（配置容差 0.30 米），真值位移约 0.700 米，随后速度归零。

结果：[纯传感器检查](../../../tests/results/zny_sensor_check.json)、[导航闭环](../../../tests/results/zny_joint_navigation.json)、[指令链路](../../../tests/results/command_interfaces.json)。
测试脚本为 `tests/integration/zny_live_check.py`，默认只读；只有显式 --move 才下达目标。
这是开发机软件仿真闭环，尚未实测 NX 跨机网络或实际机器狗。

## 启动入口整理

`launch/` 仅保留 `go2_px4_sim.launch.py`，负责 Gazebo、机器人、传感器、定位融合及 RViz。
已移除 `go2_px4_nav.launch.py`、`navigation.launch.py`、`rviz_config.py`；导航统一由相邻导航项目 `launch_hil.sh` 启动，NX 使用 `deploy/nx/run.sh hil`。

```bash
bash simulation/start.sh                 # 默认打开 Gazebo GUI 和 RViz
bash simulation/start.sh rviz:=false     # 关闭 RViz
bash simulation/start.sh gui:=false rviz:=false  # 无界面运行
```

RViz 使用当前机器人命名空间重写后的显示配置。`visualization_tf.py` 仅在 RViz 启用时运行，将公共定位 TF 与私有模型 TF 合并到 `/<namespace>/visualization/tf{,_static}`，将模型关节时间转换为显示用系统时间；不发布公共 `/tf`，不参与定位或导航。默认固定坐标为 `<namespace>/map`，导航启动后 map 定位链连通，才可完整显示模型、点云与导航图层；仅看仿真模型时可切换为 `<namespace>/base_link`。

导航职责已固定在 outdoor_nav，仿真不再提供导航归属开关，也不启动 twist_mux；原始 EKF odom 固定发布至 simulation/ekf_odom。

## 当前 TF 和原点职责

按最终要求恢复导航原有职责：导航发布 map→odom、odom→base_link 和 base_link→livox_frame。仿真 ZNY 不重复发布；robot_state_publisher 的私有身体/腿轮 TF 和 visualization_tf 显示合并链路保留。

导航输入桥与原始版本相比仅保留高度修正：TF 的 z 使用收到的 odom.pose.pose.position.z，不再强制置零，x/y 和姿态继续使用 odom。移除 publish_tf、use_odom_height 和 accept_external_origin 新增开关，原点仅使用 YAML reference_latitude/longitude/altitude，不订阅 mavros/set_gp_origin。需要重启仿真与导航两侧。

simulation_tf_boundary.json 是之前 TF 迁移到仿真时的历史结果，不代表当前配置；当前恢复验证见 tests/results/navigation_tf_restored.json。
