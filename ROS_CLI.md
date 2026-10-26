# ROS 通信与就绪检查

在项目根目录先 `source ros_env.sh`，再直接使用 `ros2` 或 Python ROS 工具。默认域 30，保留已有 domain、RMW 和 DDS 配置。跨机两侧需一致。

```bash
source ros_env.sh
ros2 topic list --no-daemon --spin-time 5
timeout 12 ros2 topic echo /robot_1/mavros/odom nav_msgs/msg/Odometry --once --field header --qos-reliability best_effort
timeout 12 ros2 topic echo /robot_1/livox/lidar sensor_msgs/msg/PointCloud2 --once --field header --qos-reliability best_effort
ros2 lifecycle get /robot_1/controller_server
ros2 lifecycle get /robot_1/straight_planner
timeout 6 ros2 run tf2_ros tf2_echo robot_1/map robot_1/base_link
```

检查 EKF 状态中的 ready=1，以及新鲜定位、点云、持续更新的 TF；发现话题不等于收到消息。查看 `topic info /robot_1/cmd_vel -v` 应只有一个仲裁发布者。

ZNY 对外传感器使用系统时间，导航不需要 /clock；Gazebo 内部仍使用仿真时间。定位保留 PX4 ECL 高度，不能伪造定位或将高度强制归零。起伏地形仍使用地图高度过滤，并不等同通用地面分割。生命周期启动失败有明确截止时间，需排查实际响应与 TF。

命名空间改为其他名字时同步替换以上命令。控制前读取新鲜位姿；前方距离必须按当前朝向换算。停止终端不会保证取消既有目标，须通过真实取消链路观察停止。
