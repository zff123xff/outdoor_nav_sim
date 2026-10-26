# PX4 定位融合

本包只负责传感器数据处理与定位估计，不负责生成仿真数据或启动仿真环境。

- `src/px4_ekf_node.cpp`：接收 GPS、IMU、磁场和气压，发布 odom、mavros/odom 与 ekf/status。
- `include/px4_gps_imu/fusion.hpp`：传感器输入转换与 PX4 ECL 融合封装。
- `vendor/ecl/`、`vendor/matrix/`：定位必需的上游估计器和矩阵库，版本和许可证见 SOURCE.md。
- `test/fusion_test.cpp`：融合回归测试。


PX4 ECL 算法与 ENU/NED 转换保持不变，真值不输入估计器。导出的 px4_ecl 库也供仿真传感器复用地理投影和地磁计算；本包不依赖 Gazebo 或导航栈。

## 包职责整理记录

px4_gps_imu 只接收定位传感器消息，做 PX4 ECL 融合并发布估计结果。

| 原位置 | 整理后 |
| --- | --- |
| px4_gps_imu/launch/ 的仿真启动 | simulation_bringup/launch/ |
| px4_gps_imu/src/px4_classic_sensors.cpp 与 backends/classic/ | simulation_bringup/src/ 和 CMakeLists.txt |
| px4_gps_imu/launch/go2_px4_nav.launch.py | simulation/src/sim_bringup/launch/ |
| 启动配置和 HIL 契约测试 | simulation_bringup/test/ |
| launch 下的 Python 缓存 | 删除 |

保留 src/px4_ekf_node.cpp、include/、vendor/ecl、vendor/matrix 和 test/fusion_test.cpp；它们分别承担定位节点、融合封装、上游估计器/矩阵运算与融合回归测试，不是无用目录。SOURCE.md 和许可证保留。

px4_gps_imu 移除 Gazebo、控制器、模型和导航栈依赖，导出已有 PX4 静态数学库供传感器插件复用，不复制上游代码。构建脚本、统一启动脚本和集成测试引用同步改为 simulation_bringup。统一入口保持 `bash simulation/start.sh`；旧的直接命令需改为 `ros2 launch simulation_bringup go2_px4_sim.launch.py`。

本次只整理职责与构建安装关系，传感器生成、PX4 融合、ZNY 消息和控制逻辑保持原实现。

验证：8 个包构建成功；定位融合、启动配置、HIL 契约、混合控制命令和生命周期故障测试共 5 项通过。在独立 ROS 域 98 / Gazebo master 11371 中启动无界面仿真，确认 EKF ready=1，定位约 73.6 Hz、点云约 9.4 Hz、ZNY 字段与 QoS 通过；未下发运动指令，测试实例已停止。实测记录见 `../../../tests/results/px4_package_cleanup.json`。
