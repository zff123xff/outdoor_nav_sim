# PX4 ECL + Gazebo Classic

使用未修改的 PX4 ECL 和 Matrix 固定提交，见 SOURCE.md。Classic ModelPlugin 生成 GPS 位置、地速、磁场、气压和验证真值，原生 gazebo_ros_imu_sensor 提供 IMU；px4_ekf_node 融合传感器并发布 odom、mavros/odom 和 ekf/status。真值不输入定位器或导航。

入口 go2w_px4_sim.launch.py，导航入口 go2_px4_nav.launch.py。两者共享 profile 和参数文件，GPS 原点变更需同时重启。默认使用 robotdog_nav 配置，保留原 PX4 EKF 的 ENU/NED 转换及磁航向，不套用参考 gps_imu_fusion 的 yaw_offset。

仅构建 Gazebo Classic 后端 libpx4_classic_sensors.so，不构建或依赖其他仿真引擎。ROS 消息由原生插件直接发布，不使用跨引擎消息桥。
