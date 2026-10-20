# robotdog_nav 对齐记录

参考：`/mnt/d/shixi/robotdog_nav_0714/robotdog_nav`。当前项目使用 Gazebo Classic 11，已移除 Fortress 的源码、入口和构建依赖；不卸载系统软件包。源码和旧生成物的迁移备份保存在 /tmp，均不参与当前构建。

| 项目 | 参考项目 | 当前实现 |
|---|---|---|
| 仿真引擎 | Gazebo Classic | Gazebo Classic 11 |
| 场景 | agriculture.world | 同场景网格、纹理、比例和碰撞 |
| 控制分工 | CHAMP 腿部 + 轮子差速 | 相同；保留 cmd_vel_remote/nav → twist_mux → cmd_vel 链路 |
| 腿部伺服 | 12 个 effort PID | 12 个 effort PID，参考增益 p=100、i=0.2、d=1 |
| 轮速 | 四轮速度控制 | 四轮速度控制，顺序 FL/FR/RL/RR |
| 步态 | 高度 0.225 m、摆动 0.04 m、支撑周期 0.25 s | 相同 |
| 轮径/轮距 | 代码固定 0.085 / 0.3 m | 当前实际模型的 0.086 / 0.3802 m |
| 雷达安装（机身坐标） | xyz=(0.25,0,0.13)，pitch=0.4 rad | 相同；模型和 TF 使用同一 URDF |
| 雷达 | MID360 CSV、200000 点/s、10 Hz、0.1～5 m | 接入现有 Classic MID360 CSV 插件，修正命名空间 frame_id |
| 轮胎摩擦 | 两方向均 0.6 | 相同 |
| GPS 原点/位置噪声/频率 | (0,0,0) / 0.0002 m / 30 Hz | 相同；保留本项目 PX4 所需地速、磁场和气压数据 |
| 定位算法 | GPS 转 ENU + IMU 朝向组合，速度填零 | 保留 PX4 ECL，不输入 Gazebo 真值 |

Classic 传感器直接发布 ROS 消息，不使用跨引擎消息桥。当前 PX4 传感器插件提供 GPS、地速、磁力计、气压计；原生 IMU 使用 gazebo_ros_imu_sensor。仍保留 /robot/mavros/odom、/robot/odom、/robot/livox/lidar、/robot/livox/imu 和现有导航接口。

CHAMP 从 go2w_body 计算腿部几何，避免把 base_link→go2w_body 的固定高度计入髋关节长度。base_link 是固定于机身的名义地面参考，机器人蹲到 0.225 m 机身高度后，其 z 可以低于地面；这不表示机身穿地。传感器外参和真值都使用实际模型坐标。

初始高度按当前模型 root 坐标计算，不直接复制参考项目的 z=0.75。默认 `spawn_z:=auto` 为 0.03417389 m。GPS 的位置仍测量 base_link，未模拟参考天线的机身安装点；IMU 仍采用本机原生 imu_link，未改成参考雷达 IMU。GPS 速度噪声保持 0.03 m/s。

参考目录缺少 cpr_agriculture 网格依赖，已补齐 Clearpath 上游固定提交 509afbeec942c6aad376a5a76a24330f1c56dc17 的资源和 BSD 许可证，见 models/cpr_agriculture/SOURCE.md。农田场景移除保存状态，保留原比例、光照、重力和零 GPS 原点；全部碰撞几何保留。depot 的原引擎风扇电机插件已移除，碰撞和外观保留。

`profile:=current` 仍使用原 depot 出生、限速、导航参数，默认 locomotion_mode=wheels，但引擎同样为 Classic。系统没有旧引擎切换参数。切换 profile 或 locomotion_mode 需要重启环境和导航。

构建测试包含混合分配、限幅、指令超时、PX4 融合和 Classic 两种控制模式配置检查。实际运行验证结果见下方记录。

## 2026-10-20 Classic 验证

- 构建仅包含 Classic 后端，3 项 CTest（混合驱动单元组、PX4 融合、两种 Classic 模式/导航原点配置）通过。混合驱动单元组包含 4 个单元用例。
- 独立域 228、独立 Classic master 中启动空场，收到真实消息，确认 EKF ready=1、两个导航 lifecycle active、新鲜 TF 和三个实际控制器 active 后才下发测试速度。
- 前进 0.12 m/s × 2 s，实际约 0.235 m；转向 0.15 rad/s × 2 s，实际约 0.406 rad。横移步态确实执行，侧向位移约 0.065 m，但伴随约 0.230 m 前后漂移，不能视为精确横移控制。
- 三次停止发布输入后均触发超时停车，末端实际速度小于 0.005 m/s。转向和横移阶段检测到腿部关节轨迹变化，测试没有关闭碰撞或伪造定位。
- 独立域 229 的默认 agriculture.world 静止测试通过，5 秒仿真时间内水平漂移约 0.026 m，末端速度 0.0045 m/s。
- 默认场景实测 IMU 约 200 Hz、GPS 约 30 Hz、点云约 10 Hz，frame_id 为 hybrid_test/livox_frame，收到有效回波。PX4 和导航均就绪。
- 测试运行后已停止全部检查实例。GUI 未做视觉验证；本机 X 服务报告客户端数量上限，测试使用无 GUI 的 CPU 射线。Classic 用户日志需要可写的 ~/.gazebo，本次测试已获授权在沙箱外运行。
- 结果日志：/tmp/go2w_classic_empty/result.json、/tmp/go2w_classic_agriculture/result.json；构建日志：/tmp/go2w_classic_final_build.log。这些短测不代表复杂地形或实机性能。
