# Classic 混合轮腿控制

来源为参考项目 go2w_control/hybrid_motion_controller.py 的控制分工和 go2w_config 的 CHAMP 配置。当前使用 ROS 2 原生 joint_trajectory_controller（effort）和 velocity_controllers。腿部 PID 保留 p=100、i=0.2、d=1；四轮顺序为 FL、FR、RL、RR。

CHAMP 从 go2w_body 计算运动学，使用参考 0.225 m 高度和 0.04 m 摆动。轮径为 0.086 m，轮距为 0.3802 m，保留当前模型实际尺寸。驱动限制输入和最大轮速，在需要限幅时同比缩放轮子和腿部转向；失去 cmd_vel 约 0.5 秒后两个输出通道都归零。

Classic 物理启动前先配置全部控制器，再排队激活并解除暂停。CHAMP 或混合驱动进程退出时关闭本次仿真。没有加入新的速度仲裁器，twist_mux 仍是唯一仲裁器。

这不是强化学习策略或复杂地形稳定控制。需要在不同速度、坡度和障碍场景下继续验证，不能由短时烟雾测试推断实机性能。
