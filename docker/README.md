# Docker 环境

宿主机不安装 ROS 或 Gazebo，仿真环境全部跑在容器里。宿主机只需要 Docker、显卡驱动和 X11 桌面。

| 宿主机需要 | 宿主机不需要 |
| --- | --- |
| Linux Docker Engine、当前用户的 Docker 访问权限 | ROS 2、Gazebo、colcon |
| 显卡驱动（无 NVIDIA 卡时用软件渲染） | 导航工作区 `outdoor_nav` |
| X11 桌面（无桌面时用无窗口模式） | 编译产物，旧机器的 `install` |

镜像是 `docker.io/osrf/ros:humble-desktop-full-jammy`：Ubuntu 22.04 + ROS 2 Humble + Gazebo **Classic 11**，不含 Gazebo Sim/Fortress。镜像构建时校验 `gazebo --version`，不是 11 直接失败。

## 构建镜像

在**普通用户**的桌面终端执行：

```bash
cd /path/to/outdoor_nav_sim
./docker/build.sh
```

镜像名默认 `outdoor_nav_humble`，可用 `DOCKER_IMAGE` 覆盖；额外参数直接追加，如 `./docker/build.sh --no-cache`。

不要用 `sudo ./docker/build.sh`：镜像内的用户 `ros` 会变成 root，容器就写不了挂载目录。换过宿主机用户后必须重新构建。

## 进入容器并启动仿真

```bash
ROS_DOMAIN_ID=10 ./docker/run.sh
```

容器内：

```bash
cd /workspace
bash build_sim.sh                                   # 编译 8 个仿真包并跑测试
bash simulation/start.sh                            # 启动农田场景
bash simulation/start.sh gui:=false rviz:=false     # 无桌面时
```

`build_sim.sh` 的测试用独立的 domain 81，跑之前确认这个域没有实际机器人在用。

仓库挂载到 `/workspace`，源码改动立刻可见。`build`、`install`、`log` 三个产物目录映射到宿主机 `.docker-workspace/`，退出容器后仍在；换 ROS 版本或架构时用 `DOCKER_ARTIFACT_DIR=/other/path` 另开一份。

容器默认通信域是 **10**；不用容器、直接在宿主机跑 `simulation/start.sh` 时，本项目默认域是 **30**。两端和 NX 必须用同一个域。

容器内已有 Gazebo、控制器、MQTT 原生库，不需要再跑 `deps/mqtt` 那套解包安装。镜像里有 nav2 的控制器、代价地图和 twist_mux 等基础包，但**没有完整导航栈**（nav2_bringup、规划器、地图服务），也没有 `outdoor_nav` 的自定义导航包（nav2_input_bridge、nav2_straight_planner）。导航算法属于相邻 `outdoor_nav`，在 NX 上跑，仿真容器只做被测端。

## 图形界面

仿真入口默认打开 gzclient 和 RViz。要单独用 GUI 工具，另开一个宿主机终端用同一域进第二个容器：

```bash
ROS_DOMAIN_ID=10 ./docker/run.sh
# 容器内
gzclient
rqt
rviz2
```

第二个容器不要再启动一套 gzserver。`gazebo` 命令会自己起一个 server，只在没有 gzserver 运行时用。

没有桌面或不想开窗口：

```bash
DOCKER_GUI=0 ./docker/run.sh bash -c 'bash simulation/start.sh gui:=false rviz:=false'
```

## GPU

默认自动探测：`nvidia-smi` 存在且 `docker run --gpus all` 实际可用才启用 GPU，否则退回普通渲染。

```bash
DOCKER_GPU=0 LIBGL_ALWAYS_SOFTWARE=1 ./docker/run.sh   # 强制软件渲染
DOCKER_GPU=1 ./docker/run.sh                            # 强制 NVIDIA，探测失败即报错
```

NVIDIA Container Toolkit 要由宿主机管理员装好，脚本改不了。软件渲染的仿真实时率取决于宿主机性能。

## 与 NX 联调

仿真电脑和 NX 必须是同一个 ROS 域、同一个机器人命名空间、同一份 GPS 原点。NX 用自己已编译的 ARM64 `outdoor_nav`，不要把宿主机或容器的 x86 产物搬过去。

```bash
# 仿真电脑
ROS_DOMAIN_ID=10 ./docker/run.sh
# 容器内
bash simulation/start.sh

# NX
cd /path/to/outdoor_nav
ROS_DOMAIN_ID=10 bash deploy/nx/run.sh hil
```

等 EKF `ready=1`、导航 active、TF 新鲜之后再发目标。容器内确认实际收到数据：

```bash
source ros_env.sh
ros2 topic list --no-daemon --spin-time 5
timeout 12 ros2 topic echo /robot_1/mavros/odom nav_msgs/msg/Odometry --once --field header
timeout 12 ros2 topic echo /robot_1/livox/lidar sensor_msgs/msg/PointCloud2 --once --field header
ros2 lifecycle get /robot_1/controller_server
timeout 6 ros2 run tf2_ros tf2_echo robot_1/map robot_1/base_link
```

看到 topic 不等于收到消息。多网卡、VPN、防火墙、AP 禁组播都要按真实网络排查，确保 DDS UDP 发现流量放行；**不要**设 `ROS_LOCALHOST_ONLY=1`。两台机器用系统时间，需 NTP/chrony 同步。

自定义 DDS 配置放在挂载目录里，或只读挂进容器：

```bash
DOCKER_DDS_DIR="$HOME/dds" \
FASTRTPS_DEFAULT_PROFILES_FILE=/dds/fastdds.xml \
FASTDDS_DEFAULT_PROFILES_FILE=/dds/fastdds.xml \
ROS_DOMAIN_ID=10 ./docker/run.sh
```

仿真与导航的话题契约见 [导航对接接口](../simulation/README.md#导航对接接口)。

## 环境变量

| 变量 | 默认 | 作用 |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `10` | 仿真容器通信域，需与导航端一致 |
| `DOCKER_IMAGE` | `outdoor_nav_humble` | 镜像名，build/run 共用 |
| `DOCKER_ARTIFACT_DIR` | `.docker-workspace` | 编译产物目录 |
| `DOCKER_GAZEBO_MASTER_URI` | `http://127.0.0.1:11346` | 容器 Classic master，避开宿主机 11345 |
| `DOCKER_GUI` | `auto` | `auto`/`0`/`1`，是否启用 X11 |
| `DOCKER_GPU` | `auto` | `auto`/`0`/`1`，是否启用 NVIDIA GPU |
| `DOCKER_DDS_DIR` | 未设置 | 自定义 DDS 配置的只读挂载点 |

已存在的 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`ROS_LOCALHOST_ONLY`、`ROS_DISCOVERY_SERVER`、`CYCLONEDDS_URI`、`FASTRTPS_DEFAULT_PROFILES_FILE` 会原样透传；仓库内的绝对路径自动换算成 `/workspace` 路径。

## 常见问题

**容器里写不了 `.docker-workspace`**（`PermissionError: log/build_...`）

`docker` 实际是 rootless Podman 时，宿主机用户会被映射成容器 root，普通用户 `ros` 看见的挂载目录属于 `0:0`。`run.sh` 已自动加 `--userns=keep-id`；若仍失败，先在宿主机同步最新脚本、退出旧容器再重进，不要对整个仓库 `chmod 777`。

**两个 gzserver 抢 master**

容器 Classic 用 11346，宿主机 Classic 用 11345。端口被占时在所有相关终端一致设置 `DOCKER_GAZEBO_MASTER_URI=http://127.0.0.1:11347`，两个 gzserver 不要共用端口。

**想全量重编译**

`build`、`install`、`log` 是挂载点，容器内不能 `rm -rf` 掉。先退出所有用这些产物的容器，再在**宿主机**执行：

```bash
rm -rf .docker-workspace/build .docker-workspace/install .docker-workspace/log
ROS_DOMAIN_ID=10 ./docker/run.sh
```

**新增了 ROS 依赖**

镜像里的依赖是构建时按 `package.xml` 装的。容器内重跑：

```bash
cd /workspace
source ros_env.sh
rosdep update --rosdistro humble
sudo apt-get update
rosdep install --from-paths simulation/src commands/src --ignore-src -r -y \
  --rosdistro humble \
  --skip-keys "$(tr '\n' ' ' < /usr/local/share/outdoor-nav/rosdep-skip-keys.txt)"
```

skip 列表里的 `nav2_input_bridge`、`nav2_straight_planner`、`msg_set_msgs`、`multi_task_bridge`、`execute_task_server`、`multi_goal_to_gp_goal` 由相邻 `outdoor_nav` 提供，镜像里没有，跳过只是避免 rosdep 找不到规则。

## 已知限制

- 拉取基础镜像、APT 和 rosdep 需要网络；Docker 权限、NVIDIA Toolkit、X11 认证和跨机防火墙都得在宿主机解决。
- 目前在 Ubuntu 22.04 + Podman 兼容 CLI 上验证过；**尚未**在 Ubuntu 20.04 Docker Engine、真实 GUI/NVIDIA 渲染、以及与 NX 跨机 DDS 上实机验收。
