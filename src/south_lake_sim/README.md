# Classic 仿真资源与导航配置

包含 agriculture / depot / empty 的 Classic world、模型网格、导航 YAML、RViz 配置和遥控取消桥。环境与导航入口位于 px4_gps_imu/launch，工作空间启动说明见 ../../README.md。

remote_stop_bridge 只监听 cmd_vel_remote 并取消旧导航，不发布速度；速度仲裁只由标准 twist_mux 执行。Gazebo Classic 不加载旧引擎的自定义 GUI 或传感器桥。
