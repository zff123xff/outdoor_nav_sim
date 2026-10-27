# ros2 版本的msg_set

#### 公用仓库，存放各类常用消息或自定义消息数据类型，建议关联上作为算法仓库的submodule。

#### 修改msg_set后请在readme中增加修改内容、修改人，并打上对应Tag

## 更新日志

## 2023.12.21

```xml
AlgoID:
  MOVE_POSITION       : 11
  ET_TRACK            : 12
  SWARM_FORMATION     : 13
  ATTACK_GOAL         : 14
  BUTTER_FLY          : 15
  SEARCH              : 16
  SWARM_EXPLORATION   : 17
  SINGLE_EXPLORATION  : 18
  ALGO_DROP           : 19
  AIR_DROP            : 20
  ALGO_RETURN         : 21
  GATHER              : 22
  ET_SEARCH           : 23
  ALGO_AVOID          : 24
  ALGO_TRACK          : 25
  ALGO_RELAY          : 26
  ALGO_RISEMOVE       : 27
  ALGO_POINT_ATTACK   : 28
  ALGO_CRUISE         : 29
  NIGHT_MOVE          : 30
  TRANS_FORMATION     : 31
```

## 2024.2.21

新增各模块所需的自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.0（汤佳境）</b></summary>
<ul>
<li>新增MultiTasks.msg
<li>新增SearchAttack.msg
<li>新增SingleTask.msg
<li>新增TakeoffGathering.msg
</li></ul>
</details>

## 2024.2.23

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.1（lmj）</b></summary>
<ul>
<li>修复SingleTask.msg中 takeoff_point的类型为mavros_msgs/Waypoint，与文档统一
<li>为SearchAttack.msg添加一个新的类型，int32[] drones_id

</li></ul>
</details>

<details open>
<summary><b style="font-size: 24px;">v1.0.2（吴天皓）</b></summary>
<ul>
<li>更新了MultiTask.msg;SingleTask.msg和AreaSample.msg的消息类型
</li></ul>
</details>

## 2024.2.26

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.3（lmj）</b></summary>
<ul>
<li>为SearchAttack.msg添加注释
</li></ul>
</details>

## 2024.2.27

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.3（lmj）</b></summary>
<ul>
<li>新增msg/DroneOdometry.msg
<li>更新消息：MultiGoal.msg, SearchAttack.msg, SingleTasks.msg
</li></ul>
</details>

## 2024.2.27

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.4（wjh）</b></summary>
<ul>
<li>新增：GatherGoal.msg,ExceptionCase.msg
<li>更新消息：TakeoffGathering.msg
</li></ul>
</details>

## 2024.03.01

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.5（tq）</b></summary>
<ul>
<li>新增：DropReturnMission.msg,
         DropMission.msg,
         ReturnMission.msg.
<li>更新消息：无
</li></ul>
</details>

## 2024.03.01

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.5（wax）</b></summary>
<ul>
<li>新增：SearchMission.msg
         SwarmMission.msg
<li>更新消息：无
</li></ul>
</details>

## 2024.3.5

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.6（lmj）</b></summary>
<ul>
<li>新增JumpTask.msg
</li></ul>
</details>

## 2024.3.5

修改自定义消息

<details open>
<summary><b style="font-size: 24px;">v1.0.7（dym）</b></summary>
<ul>
<li>ObjectBox.msg和ObjectDetection.msg
</li></ul>
</details>

## 2024.03.06

<details open>
<summary><b style="font-size: 24px;">v1.0.8（tjj）</b></summary>
<ul>
<li>在algo_id.yaml新增了ALGO_DROP、ALGO_RETURN、STEREO_SEARCH、SURROUND_SEARCH、ET_SEARCH编号</li>
<li>根据新增消息修改了映射文件</li>
</ul>
</details>

## 2024.03.07

<details open>
<summary><b style="font-size: 24px;">v1.0.9（wth）</b></summary>
<ul>
<li>修改：SwarmMission.msg为SfMission.msg,新增任务高度
</ul>
</details>

## 2024.03.19

<details open>
<summary><b style="font-size: 24px;">v1.0.9（wth）</b></summary>
<ul>
<li>修改：ObjectDetection.msg添加header和x,y,z
</ul>
</details>

## 2024.03.19

<details open>
<summary><b style="font-size: 24px;">v1.0.9（lmj）</b></summary>
<ul>
<li>修改：msg_set_msgs/msg/DroneOdometry.msg,添加状态：TASK_SUCCESS=128</li>
</ul>
</details>

## 2024.03.20

<details open>
<summary><b style="font-size: 24px;">v1.0.10（lmj）</b></summary>
<ul>
<li>修改：msg_set_msgs/msg/multitask.msg,添加状态：TASK_SUCCESS=128int16 edited_task  #被编辑的任务</li>
<li>修改：msg_set_msgs/msg/singleatask.msg,添加状态：int16 task_id  #任务编号</li>
</ul>
</details>

## 2024.04.08

<details open>
<summary><b style="font-size: 24px;">v1.0.11（tjj）</b></summary>
<ul>
<li>新增消息SwarmMincoTraj.msg，用于swarm_formation机间轨迹传输</li>
<li>新增消息SwarmMincoTrajInt.msg，用于swarm_formation机间轨迹传输</li>
<li>新增消息PolyTraj.msg，用于swarm_formation轨迹执行</li>
</ul>
</details>

## 2024.4.25

<details open>
<summary><b style="font-size: 24px;">v1.0.12（罗欣）</b></summary>
<ul>
<li>将config文件install到路径中</li>
</ul>
</details>

## 2024.5.7

<details open>
<summary><b style="font-size: 24px;">v1.0.13(lmj)</b></summary>
<ul>
<li>新增消息CoverJump.msg，用于cover任务的跳转任务点</li>
<li>新增消息CoverParam.msg，用于设置对应运动的参数</li>
<li>新增消息CoverTask.msg，用于配置整个cover任务</li>
</ul>
</details>

## 2024.5.7

<details open>
<summary><b style="font-size: 24px;">v1.0.13(wax)</b></summary>
<ul>
<li>新增消息Cover.msg，用于task_manage发送给task_shield的消息</li>
<li>修改消息sfMission.msg，用于任务模板发送给sf行为的消息</li>
</ul>
</details>

## 2024.5.8

<details open>
<summary><b style="font-size: 24px;">v1.0.13(lmj)</b></summary>
<ul>
<li>修改消息CoverIndex.msg，新增group_id</li>
<li>修改消息CoverIndex.msg，删除param</li>
</ul>
</details>

## 2024.5.13

删除uwb相关消息，现在依赖 [nlink_parser_ros2_interfaces](https://git.cnaeit.com/XJSF/product_group/amg0523062/uwb_lib/-/tree/ros2/nlink_parser_ros2/nlink_parser_ros2_interfaces?ref_type=heads)消息包

<details open>
<summary><b style="font-size: 24px;">v1.0.13(tjj)</b></summary>
<ul>
<li>删除消息LinktrackNode2.msg</li>
<li>删除消息LinktrackNodeframe2.msg</li>
</ul>
</details>

## 2024.5.16

<details open>
<summary><b style="font-size: 24px;">v1.0.13(jfq)</b></summary>
<ul>
<li>新增消息Bbox.msg</li>
<li>修改消息Bboxes.msg</li>
</ul>
</details>

## 2024.5.20

<details open>
<summary><b style="font-size: 24px;">v1.0.13(lmj)</b></summary>
<ul>
<li>GlobalParam.msg</li>
</ul>
</details>

## 2024.7.9

<details open>
<summary><b style="font-size: 24px;">v1.0.14(lmj)</b></summary>
<ul>
<li>新增消息RealyTask.msg</li>
</ul>
</details>

## 2024.7.10

<details open>
<summary><b style="font-size: 24px;">v1.0.14(lmj)</b></summary>
<ul>
<li>修改消息：CoverParam.msg,添加变量: height</li>
</ul>
</details>

## 2024.7.19

<details open>
<summary><b style="font-size: 24px;">v1.0.15(lmj)</b></summary>
<ul>
<li>修改消息：DroneOdometry.msg,添加变量: task_id,删除变量: 目标位置</li>
</ul>
</details>

## 2024.8.29

<details open>
<summary><b style="font-size: 24px;">v1.0.16(lmj)</b></summary>
<ul>
<li>修改消息：msg/SearchAttack.msg,添加变量: attack_flag</li>
</ul>
</details>

## 2024.9.2

<details open>
<summary><b style="font-size: 24px;">v1.0.16(lmj)</b></summary>
<ul>
<li>修改消息：config/topic_key.yaml,添加变量: track_img_topic</li>
</ul>
</details>

## 2024.9.2

<details open>
<summary><b style="font-size: 24px;">v1.0.16(lmj)</b></summary>
<ul>
<li>修改消息：config/topic_key.yaml,添加变量: track_img_topic</li>
</ul>
</details>

## 2024.9.2

<details open>
<summary><b style="font-size: 24px;">v1.0.17(wth)</b></summary>
<ul>
<li>新增消息：TargetGps,添加变量: target_pose_topic</li>
</ul>
</details>

## 2024.10.21

<details open>
<summary><b style="font-size: 24px;">v1.0.18(tjj)</b></summary>
<ul>
<li>新增定点所需的消息和话题</li>
<li>修改targetsGPS消息类型，新增header和world的x,y,z</li>
</ul>
</details>

## 2024.10.31

<details open>
<summary><b style="font-size: 24px;">v1.0.19(wth)</b></summary>
<ul>
<li>新增目标位置的像素信息</li>
</ul>
</details>

## 2024.11.28

<details open>
<summary><b style="font-size: 24px;">v1.0.20(tjj)</b></summary>
<ul>
<li>新增探索所需的消息和话题</li>
</ul>
</details>

## 2025.01.13

<details open>
<summary><b style="font-size: 24px;">v1.0.21(lmj)</b></summary>
<ul>
<li>修改单任务参数，SingleTask.msg， </li>
<li>修改sa任务参数，SearchAttack.msg， </li>
</ul>
</details>

## 2025.01.14

<details open>
<summary><b style="font-size: 24px;">v1.0.21(lmj)</b></summary>
<ul>
<li>增加单任务参数，SingleTask.msg， </li>
<li>增加sa任务参数，SearchAttack.msg， </li>
</ul>
</details>


## 2025.01.16

<details open>
<summary><b style="font-size: 24px;">v1.0.22(lmj)</b></summary>
<ul>
<li>修改单任务id类型，SingleTask.msg， </li>
<li>修改多任务id类型，SearchAttack.msg， </li>
<li>修改单任务id类型，JumpTask.msg </li>
</ul>
</details>

## 2025.02.24

<details open>
<summary><b style="font-size: 24px;">v1.0.23(lmj)</b></summary>
<ul>
<li>增加新的dropreturn的drones_id字段，DropReturnMission.msg， </li>
</ul>
</details>

## 2025.02.25

<details open>
<summary><b style="font-size: 24px;">v1.0.24(lmj)</b></summary>
<ul>
<li>增加新的msg CruiseInput.msg </li>
</ul>
</details>

## 2025.03.06

<details open>
<summary><b style="font-size: 24px;">v1.0.25(tjj)</b></summary>
<ul>
<li>增加新的msg AntiJamTask.msg </li>
<li> 修改msg AntiJamTask.msg 2025.03.13 </li>
</ul>
</details>

## 2025.03.06

<details open>
<summary><b style="font-size: 24px;">v1.0.26(lmj)</b></summary>
<ul>
<li>增加新的msg AdjustTasks.msg </li>
<li>增加类型drones_id SingleTask.msg </li>
</ul>
</details>

## 2025.03.17

<details open>
<summary><b style="font-size: 24px;">v1.0.27(tjj)</b></summary>
<ul>
<li> 增加msg FeatureInfo.msg </li>
<li> 修改msg MultiGoal.msg </li>
</ul>
</details>

## 2025.04.08
<details open>
<summary><b style="font-size: 24px;">v1.0.27(lmj)</b></summary>
<ul>
<li> 修改msg AntiJamTask.msg ，增加arrive_path</li>
</ul>
</details>

## 2025.05.15
<details open>
<summary><b style="font-size: 24px;">v1.0.28(tjj)</b></summary>
<ul>
<li> 修改编队相关消息</li>
</ul>
</details>

## 2025.06.10
<details open>
<summary><b style="font-size: 24px;">v1.0.29(tjj)</b></summary>
<ul>
<li> 添加编队相关消息</li>
</ul>
</details>
