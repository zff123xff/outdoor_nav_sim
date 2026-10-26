# Go2-W Classic 控制

默认由 CHAMP 生成 12 个腿关节的轨迹，通过 Classic effort PID 执行；混合控制器消费 mux 的 cmd_vel，向 CHAMP 转发 (0, vy, wz)，向四轮发送差速 (vx, wz)。坐标、轮径和轮距取当前官方模型。

CHAMP 配置见 config/hybrid_champ.yaml，驱动和失联参数见 config/hybrid_drive.yaml，effort 控制器见 config/classic_hybrid_controllers.yaml。locomotion_mode:=wheels 使用 controllers.yaml 与固定腿初始化器，不能和混合控制并行启动。

启动和验证通过 工作空间 simulation/start.sh、build_sim.sh。实际运动烟雾测试为 test/hybrid_sim_smoke.py，需指定空闲独立 ROS 域；脚本创建自己的 Classic master 和仿真，检查真实定位、TF、控制器，再执行短距离前进、转向、横移和超时停车。
