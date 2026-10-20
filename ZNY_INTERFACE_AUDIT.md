# 北京 ZNY 仿真端对接核验

核验日期：2026-10-20。结论：**部分接口兼容，但目前不能按文档完整实现全部功能，不能直接验收为接口一致。**

对照文件：`/mnt/d/shixi/北京ZNY仿真端对接/北京ZNY仿真端对接.md`、`图片和附件/go2w仿真接口说明(2).md` 及其截图。核验对象是当前 sim_ws 源码、outdoor_nav 源码以及当前实际安装运行的程序。本次添加核验脚本和证据，未调整业务实现。

## 核验方式与边界

- 在独立 ROS 域 230、独立 Gazebo Classic master 中启动空世界、`robot_1`、默认 `robotdog_nav` 配置和真实 PX4 ECL/导航。确认 EKF ready、导航 active、新鲜 TF 后，接收真实消息并执行有限距离导航及停车。没有向已有机器人发指令，没有注入定位或关闭碰撞检查。
- 在独立域 231 中运行已安装的任务服务和转换器，以测试消息检验任务转换、队列和零原点；该域没有仿真机器人，合成里程计仅是任务转换单元的输入。
- 在独立域 232 中按文档 HIL 命令启动，关闭实体驱动/MAVROS/MID360；记录实际启动错误。
- 点云使用安装环境的 tf2_sensor_msgs C++ 转换实现验证，分别输入当前字段布局和缺少 y/z 的坏数据。
- 未验证外部 UE 仿真端、文档 Docker 镜像、实体机器人、MQTT 后端或完整 YOLO 检测器；没有原始 bag 可供回放。空世界成功不等于复杂场景避障通过。

## 接口对照

| 项目 | 文档约定 | 当前结果 | 判断 |
|---|---|---|---|
| 命名空间 | 例子混用 robot_1、robot_a，旧链路使用无前缀话题 | 默认 robot；可设置 robot_namespace:=robot_1/robot_a | 可配置对齐；旧命令不能原样混用 |
| 速度输入 | /robot_1/cmd_vel，Twist，车体 vx/vy/wz，约 20 Hz | 同名同类型、RELIABLE；混合控制器实际订阅 | 基本兼容；遥控应输入 cmd_vel_remote，由唯一 twist_mux 仲裁 |
| 速度上限 | vx 1.5、vy 0.6、wz 1.0 | 默认 vx 0.3、vy 0.25、wz 0.5 | 不一致，文档速度示例会限幅 |
| 输入超时 | 0.5 秒无更新停车 | 混合控制器使用 0.5 秒墙钟 watchdog；已有物理运动核验见 ROBOTDOG_ALIGNMENT.md | 已实现；单次发布不会持续行驶到目标 |
| 定位输出 | /robot_1/mavros/odom，Odometry，ENU，odom→base_link；频率章节有 10/≥30 Hz 两种说法 | 实测 50 Hz，robot_1/odom→robot_1/base_link，RELIABLE | 类型、帧、频率满足主要约定；TF 高度另有问题 |
| 点云输出 | /robot_1/livox/lidar，PointCloud2，约 10 Hz，BEST_EFFORT，xyz float32，stride 12/16 | 实测 10 Hz，livox_frame；xyz 偏移 0/4/8，intensity 偏移12，timestamp float64 偏移16，point_step=24；RELIABLE | 标准 ROS 消费正常，硬编码 stride 的外部端需适配 |
| QoS | 传感器 BEST_EFFORT | 当前发布 RELIABLE，实测 BEST_EFFORT 观察节点及 costmap 均收到数据 | 发布端更强可靠性可匹配该订阅；不是本次失联原因 |
| 雷达外参 | base_link 下 (0.18,0,0.53)，俯仰18° | 实测 TF (0.25,0,0.56582611)，俯仰0.4 rad≈22.92° | 与 robotdog_nav 配置对齐，但与此 HIL 文档不一致；勿再叠加文档静态 TF |
| 设置原点 | 向 /robot_a/mavros/set_gp_origin 发布 GeoPointStamped | 仿真对应话题订阅数0；原点从启动配置读取 | 未实现该动态接口 |
| GPS 单点 | /robot_a/gp_goal | 使用当前零原点换算 1 m 目标后成功 | 基础功能通过，需统一经纬度原点 |
| 本地多点 | /waypoint_goals，MultiGoal，is_gps_aid=false；execute_task JSON | 默认仿真不启动任务链；单独启动任务链后服务接收成功，但转换器跳过本地任务 | 功能不通，服务 success 不代表执行 |
| 纯转向 | 相同位置、目标 yaw 改变，vel=0 | 相同 XY、yaw 增加0.5 rad，未产生新 action，非零导航命令数0 | 未实现文档效果 |
| 目标检测停车 | YOLO 结果触发停车 | /robot_1/target_detected Bool 可取消正在执行的导航并停车 | 停车输入有效；检测器及全队列停止未完成 |

文档本身混有两套架构：旧的 `gps_imu_fusion → /world_odom → waypoint_nav`，以及带命名空间的 `/mavros/odom → Nav2`。当前项目采用后者，并保留 PX4 ECL 融合。当前环境没有安装旧 `gps_imu_fusion` / `waypoint_nav`，不能把两套命令拼在一起执行。文档另有点云 world/sensor 帧及 odom twist 世界系/车体系的表述冲突：当前点云是雷达本地坐标，Odometry twist 使用 child_frame_id 车体系，符合 ROS 消息定义。

## 已确认的功能问题

### 1. 任务被接收后不执行，且默认没有任务服务

当前仿真/导航默认入口没有启动 task_pipeline。即使另行启动，文档本地任务请求返回 `status=success`、`Task accepted and executing`，MultiGoal 也发出了，但 `is_gps_aid=false` 被 multi_goal_to_gp_goal 明确跳过，最终 gp_goal 发布数为0。

根因：`outdoor_nav/src/external_pkg/multi_goal_to_gp_goal/multi_goal_to_gp_goal/multi_goal_to_gp_goal.py:142`。该转换器只传递经纬高，丢弃 yaw、vel、ctl_mode，因此这些字段有消息定义也不等于实际控制支持。旧根话题 `/waypoint_goals` 没有对应订阅者；当前任务链使用命名空间内的话题。

### 2. 原点不统一，零原点被任务转换器当成未配置

默认 robotdog_nav 仿真和导航使用 (0,0,0)。独立 task_pipeline 实测读取 (40.9445353,113.172819,1298.7345711102703)。文档发布 set_gp_origin 无订阅者，无法消除差异。

进一步给转换器显式设置 (0,0,0)，GPS 任务仍没有发布目标，因为它把纬经度同时为0判定为“未配置”，见转换器 `:215–229`。这会造成等待不执行；若任务转换与导航各使用不同原点，还会出现目标距离和到达判断错误。

### 3. 停止当前导航没有停止多点队列

真实仿真中，对正在运动的第二个 GPS 目标发送 target_detected=true，action 状态变为 CANCELED，实测末速度约0.00267 m/s，当前停车链路有效。

但任务转换器没有相应停止/清空逻辑。独立消息测试中，发出停止信号后再提供满足第一个点到达条件的里程计，第二个 GPS 点仍被发布。因此多点任务有再次发目标的风险；不能据单点停车成功认定整项任务已经停止。

### 4. 原地转向和精确侧移不能按文档保证

straight_planner 在位置距离小于容差且无活动 action 时直接完成，不检查目标朝向，见 `outdoor_nav/src/navigation2/nav2_straight_planner/src/straight_planner.cpp:320`。物理仿真中的相同 XY 纯 yaw 目标实际未触发运动。任务转换器还会丢弃 yaw。

当前 RPP 路径控制主要输出 vx/wz，不输出导航 vy；“目标在左侧”不能等价为“保持姿态纯侧移”。混合控制器的手动 vy 输入已移植，但既有物理测试中 0.06 m/s×3 s 的侧移约0.065 m，并伴随约0.23 m 纵向漂移，见 ROBOTDOG_ALIGNMENT.md。它不满足文档理想积分模型的精确侧移假设。

### 5. TF 高度与里程计不一致，存在地面障碍误判

实测 Odometry z=-0.07741 m，同一 base_link 的导航 TF z=0。`outdoor_nav/src/navigation2/nav2_input_bridge/src/input_bridge_node.cpp:122–126` 将 z 强制置0。当前 Classic 机身存在姿态/高度变化，不能直接把 odom→base_link 投影到零高度后继续用于三维雷达外参。

在 empty.world，3341个有效点按当前 TF 转至 map 后，高度约0.144–0.188 m，全部落入当前 obstacle layer 的0.05–1.0 m范围。costmap 确实订阅了点云；240×240格地图中有18800个致命格（统计时刻的观测）。这说明空场景也存在大量不合理障碍标记。高度不一致是明确问题，但本轮没有隔离修复后重测，不能把所有高度偏差及致命格数量全部归因于这一行代码。

本轮1 m GPS目标仍完成，不能据此推断必然发生碰撞或所有目标失败；该问题需优先修复并进行地面分割/绕障验收。没有关闭碰撞检查。

### 6. 文档 HIL 启动命令直接失败

执行文档命令：

```bash
ros2 launch robot_launch outdoor_all_go2w.launch.py \
  robot_namespace:=robot_a use_go2w_driver:=false use_mavros:=false \
  use_mid360:=false use_auto_origin:=false
```

实际错误为 `package 'livox_ros_driver2' not found`。原因是 go2w_nav2.launch.py 在启动描述构建时无条件查询该包路径，即使雷达节点条件为false也会先报错，见 `outdoor_nav/src/robot_launch/launch/go2w_nav2.launch.py:119`。当前项目自己的 Classic 导航入口可以启动，但不能将其成功视为文档原命令通过。也不要把两套启动入口同时启动造成重复 twist_mux/导航节点。

## 文档中提到的问题是否复现

- **Field y does not exist：当前仿真点云未复现。** 实际 x/y/z 类型及偏移齐全，row_step、data长度一致；24字节布局在当前 tf2_sensor_msgs 中成功转换。故 timestamp 扩展本身不会触发该异常。缺少 y/z 的外部点云测试确实复现同名异常，接收端仍需入口校验。外部程序如果固定每点12/16字节读取，会有另一类解析错误。
- **启动后无错误/13节点：当前不能复现文档该验收结论。** 精确 HIL 命令先因缺包失败；项目 Classic 入口运行的是另一套节点组合。节点数量本身也不能证明任务执行正确。
- **回放原始 bag 直接导航：现有资料不能证明。** 截图中的 bag 仅见 gps/data、livox/imu、livox/lidar 等，缺少当前 PX4 链所需 GPS velocity、mag、baro，GPS 名称也不同。旧 fusion/waypoint 程序又未安装。应分别制定“录制完整 PX4 输入”或“录制导航 odom/cloud/TF/clock”的回放方案，并避免与现场 /clock 冲突。
- **无雷达导航、复杂避障、完整 YOLO：尚未验收。** 当前仅证明点云确实被 costmap 消费、GPS空场景单目标可完成、Bool 停车入口有效。文档中的未来功能描述不能作为已实现功能。

## 通过项及实测数值

1. 原生 Classic/PX4 真实数据：odom≈50 Hz、GPS≈30 Hz、IMU=200 Hz、cloud≈10 Hz；实际收到消息，非仅发现话题。
2. 当前原点下发送前方约1 m GPS目标：最终位置误差0.29847 m，FollowPath 成功，符合当前0.3 m位置容差。实际最大导航 vx≈0.258 m/s，vy=0。中途出现的多个旧 action 状态6伴随周期更新路径，不能单独据此判为碰撞失败。
3. 活动导航期间 target_detected 停车：action取消、末速度约0.00267 m/s。
4. 当前24字节点云字段布局通过标准转换；坏数据用例复现文档异常。

## 修复优先顺序

1. 统一仿真、导航和任务链原点；修正零原点有效性判断，提供明确的原点设置/读取契约。
2. 打通本地任务执行和任务状态回报；保证停止能取消并清空/暂停队列，避免“已接受但永不执行”。
3. 修正三维定位/TF高度关系，重新核验平地障碍标记及真实避障。
4. 补齐 HIL 无硬件依赖的启动流程；保留唯一速度仲裁器。
5. 明确 yaw、vel、ctl_mode 的执行语义，补纯转向及侧移验收；按物理模型调参后再承诺文档运动效果。
6. 形成单一版本接口规范，固定命名空间、雷达外参、字段 stride 解析规则、速度上限、定位频率和回放方案。

## 可重复运行的证据

- [Classic 真实消息与运动测试](tests/zny_classic_contract.py)，[完整结果](reports/zny_interface/classic_result.json)。
- [任务转换与队列测试](tests/zny_task_contract.py)，[完整结果](reports/zny_interface/task_result.json)。
- [点云转换结果](reports/zny_interface/cloud_transform_result.txt)，同目录保留 C++ 用例和 CMakeLists.txt。
- [文档原命令启动错误](reports/zny_interface/document_launch_error.txt)。

ROS Python 测试运行前 source ROS helper 与 sim_ws/ros_env.sh；仅选择确认无现有机器人的独立域。Classic 测试会创建自己的仿真并运动，约需数分钟，输出路径使用 /tmp。例如：

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source sim_ws/ros_env.sh
python3 sim_ws/tests/zny_classic_contract.py --domain-id 230 --output /tmp/zny_classic_audit
python3 sim_ws/tests/zny_task_contract.py --domain-id 231 --output /tmp/zny_task_audit
```
