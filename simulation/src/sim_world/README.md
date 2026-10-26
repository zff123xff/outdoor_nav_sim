# Classic 仿真资源与导航配置


remote_stop_bridge 只监听 cmd_vel_remote 并取消旧导航，不发布速度；速度仲裁只由标准 twist_mux 执行。Gazebo Classic 不加载旧引擎的自定义 GUI 或传感器桥。
