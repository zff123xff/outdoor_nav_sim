# MID360 点云仿真

本包编译 Gazebo Classic 射线插件 `libmid360_plugin.so`，从 `scan_mode/mid360.csv` 读取扫描角度并发布点云。`include/` 和 `src/` 为插件实现，`LICENSE` 保留上游许可。

传感器模型和外参由 `../sim_robot/urdf/go2w.xacro` 定义，外壳使用机器人包的 mid360.STL；本包不再包含重复外壳或不存在的 xacro。统一入口是工作空间 `simulation/start.sh`。ZNY 输出 xyz FLOAT32、BEST_EFFORT 和系统时间戳；兼容 Classic 模式保留旧字段。
