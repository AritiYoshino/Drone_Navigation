# 本机环境与参考无人机启动流程

本机为 Jetson Xavier NX，Ubuntu 18.04.6 / L4T 32.5.1、ROS Melodic、CUDA 10.2。迁入源码保留在 `workspaces/`；适配后的编译副本位于 `runtime/workspaces/`，依赖安装到 `runtime/local/` 和 `runtime/sysroot/`。这些运行目录是当前入口的一部分。

`drone-nav` 的命令和组件流程对齐 `amov@10.53.71.32` 上 `/home/amov/Drone_Navigation_git/scripts/drone_nav.sh`。参考机实际使用 `/home/amov/` 下的独立 VINS、RealSense、Fast-Drone 工作区，包括 `/home/amov/Fast-Drone-250-master`；仓库内 `workspaces/` 是归档来源。本机继续使用自己的编译副本和依赖路径。

安装终端入口后，新终端加载本项目环境；原 `.bashrc` 备份在 `runtime/backups/`，旧 `vins_ws` / `yolov8_ws` 的自动 source 已由本项目替代。也可以直接用 `bash /home/z/Drone_Navigation_git/scripts/drone_nav.sh` 代替下文 `drone-nav`。

## 检查与设备配置

```bash
drone-nav check
drone-nav doctor
drone-nav doctor --hardware
```

`check` 按独立环境检查配置文件、ROS 包、编译产物和 `ldd` 动态库依赖，不连接硬件。`doctor` 调用本机现有 `scripts/doctor.sh`，检查软件环境及 USB/串口设备；`--hardware` 等价于 `--require-hardware`，要求硬件设备存在。它不测量在线 ROS 话题频率，也不评估定位质量。未插入飞控、相机时普通诊断显示 `PENDING` 属于预期；加 `--json` 可输出报告，本次检查报告保存在 `runtime/logs/drone-nav-doctor.json`。

默认飞控端口是 `/dev/ttyACM0:57600`，相机序列号为空时自动选择。默认值在 `config/drone.env`；导航入口支持可选的 `config/local.env`，例如：

```bash
DRONE_FCU_URL=/dev/ttyACM0:57600
DRONE_CAMERA_SERIAL=""
```

首次配置串口访问组后需重新登录。使用 `drone-nav start --dry-run` 确认当前设备参数和组件命令。

也可单次指定 `--fcu-url URL`、`--serial SERIAL` 或 `--vins-config PATH`。显式命令参数优先于环境变量，环境变量优先于配置文件。

## 定位与规划

```bash
drone-nav vins --dry-run
drone-nav start --dry-run
drone-nav vins
drone-nav start
```

两条启动链都先确认 ROS master 可用：复用已有 master，或启动一个本地 master 并等待就绪；若 `ROS_MASTER_URI` 指向远程，则等待远程 master。随后启动 MAVROS、PX4 IMU 频率请求器、RealSense 和 GPU VINS。VINS 等待 MAVROS 原始 IMU 与双目红外图像后启动，命名空间固定为 `/uav1`。`vins` 在桌面环境中增加 VINS RViz；`start` 增加 VINS→PX4 里程计桥、EGO-Planner 和 EGO RViz。

`start` 与 `quick` 等价。它们不启动 px4ctrl，不发送目标点，也不发送起飞消息。桥接组件启动前等待 IMU 和 VINS odometry，并执行重力方向检查；检查参数与参考机一致：稳定等待 8 秒、采样 2 秒、角度阈值 2°，最多重置 VINS 3 次。规划器等待 VINS odometry 和深度图。

桌面模式通过终端窗口/标签运行各组件。SSH、无桌面环境或指定 `--headless` 时在当前终端管理会话：

```bash
drone-nav start --headless
```

该模式的组件输出在 `runtime/navigation/logs/`，例如 `master.log`、`mavros.log`、`vins.log`、`bridge.log`、`planner.log`。Ctrl+C 停止本次会话；会话中组件退出也会触发其余组件清理。启动日志使用固定组件文件名，后续启动会覆盖。桌面模式由各组件终端管理，关闭相应组件终端停止该组件。

运行时 VINS 配置生成在 `runtime/navigation/vins_runtime.yaml`，输出与位姿图目录也在 `runtime/navigation/`。生成器调整运行路径，不修改 `config/vins/` 中的原始标定配置。

## 单独运行组件与手动控制

`component` 在前台运行一个组件；需要多个组件时使用多个终端。可用组件为 `master`、`mavros`、`imu-rate`、`camera`、`vins`、`bridge`、`planner`、`rviz`、`vins-rviz`、`gravity`、`controller`、`goals`。例如：

```bash
drone-nav component camera
drone-nav component planner --dry-run
drone-nav component rviz
drone-nav component bridge
```

关键组件与参考机实际 ROS 命令对应如下；统一入口负责选择依赖环境和等待输入：

| 入口 | ROS 命令 |
| --- | --- |
| `drone-nav component vins` | `rosrun vins vins_node .../runtime/navigation/vins_runtime.yaml __ns:=/uav1` |
| `drone-nav component bridge` | `roslaunch px4ctrl vins_to_px4_odom.launch` |
| `drone-nav component planner` | `roslaunch ego_planner single_run_in_exp.launch` |
| `drone-nav component rviz` | `roslaunch ego_planner rviz.launch` |
| `drone-nav controller` | `roslaunch px4ctrl run_ctrl.launch` |
| `drone-nav component goals` | `rosrun ego_planner realtime_goal.py` |

控制器和目标点分别手动启动：

```bash
drone-nav controller --dry-run
drone-nav controller
drone-nav component goals
```

`controller` 在重力检查通过后启动 px4ctrl。它使用运行工作区的 `config/ctrl_param_fpv.yaml`，与参考机相同；旧 `config/px4ctrl_bench.yaml` 不再是此入口的配置。原始配置开启 `auto_takeoff_land.enable` 和 `enable_auto_arm`，保留 RC 输入要求，起飞高度为 0.5 m。

px4ctrl 启动后等待 RC 和飞控连接。切入 AUTO_HOVER / AUTO_TAKEOFF 前需有新鲜 odometry，且不能已经收到持续的 `/position_cmd`，否则控制器拒绝切换。完成悬停与命令模式准备后再发送目标；命令控制还要求飞控已进入 OFFBOARD。

目标脚本交互输入绝对坐标 `x y z`，支持空格或逗号分隔，输入 `q` 退出。它发布到 `/move_base_simple/goal`，默认坐标系为 `world`，每个目标重复 3 次；EGO 使用该坐标值规划，不转换输入 frame，也不使用目标朝向。规划器忽略 `z < -0.1` 的目标。

起飞消息仍为独立手动命令：

```bash
drone-nav takeoff --dry-run
drone-nav takeoff
```

它发布一次 `/px4ctrl/takeoff_land`，消息为 `quadrotor_msgs/TakeoffLand`、`takeoff_land_cmd: 1`；只有运行控制器并满足其状态条件后才会进入起飞流程。

## 话题与坐标系

定位使用 `/mavros/imu/data_raw`、`/uav1/camera/infra1/image_rect_raw` 和 `/uav1/camera/infra2/image_rect_raw`。EGO 与 px4ctrl 均直接订阅 `/uav1/vins_estimator/imu_propagate`；EGO 的深度和外参分别来自 `/uav1/camera/depth/image_rect_raw` 与 `/uav1/vins_estimator/extrinsic`。

里程计桥把 VINS odometry 送至 `/mavros/odometry/out`，最多 50 Hz，仅在有新输入时发布，输入接收超时 0.15 秒后停止。桥接保留位置/姿态和输入时间戳，将 frame 标记为 `odom` / `base_link`，把世界系线速度旋转到机体系；这不是任意坐标系之间的 TF 变换。

EGO 使用 `world` 坐标系，单机 ID 为 0，规划轨迹输出 `/drone_0_planning/bspline`。轨迹服务器执行时以 100 Hz 输出 `/position_cmd`，px4ctrl 据此发布 `/mavros/setpoint_raw/attitude`，控制器自身的 IMU 使用 `/mavros/imu/data`。到达轨迹末端时轨迹服务器停止位置命令，px4ctrl 在 0.5 秒命令超时后回到 AUTO_HOVER。

当前参考参数为 EGO 最大速度 0.3 m/s、最大加速度 5.0 m/s²、最大偏航速度 30°/s；地图 30×30×3 m、分辨率 0.2 m、障碍膨胀 0.4 m。控制器质量 2.7 kg、悬停油门 0.65、姿态角上限 30°；这些参数仍需与本机机体匹配。

## 兼容入口

旧脚本保留为统一入口的兼容包装：

| 旧入口 | 当前流程 |
| --- | --- |
| `scripts/home/vi_gpu.sh`、`scripts/home/test.sh` | `drone-nav vins` |
| `scripts/home/vins_quick_start_gpu.sh` | `drone-nav start` |
| `drone-nav quick` | `drone-nav start` |
| `drone-start` 或 `drone-start --mode localization` | `drone-nav vins` |
| `drone-start --mode full` 或 `scripts/home/start_drone.sh --mode full` | `drone-nav start` |
| `drone-start --mode camera` / `--mode mavros` | 仅相机 / MAVROS 与 IMU 频率请求器 |
| `scripts/home/takeoff.sh` | `drone-nav takeoff` |
| `drone-check` | 本机 `scripts/doctor.sh` |

旧 `drone-start` 包装默认使用 `--headless`，加 `--with-rviz` 恢复桌面视图。`--mode full` 现在与参考机 `start` 一致，只包含定位、桥接和规划；控制器由 `drone-nav controller` 手动启动。

## 标定与环境隔离

`config/vins/` 中仍是上一架无人机的双目内参、相机到飞控外参及时间偏移，用于初步诊断。新相机或安装位置/方向变化后，需要重新标定并更新；控制器的质量、推力模型和增益也需按新机核验。已有报告与参数位于 `calibration/`，详细标定步骤见 [MIGRATION_ENVIRONMENT.md](MIGRATION_ENVIRONMENT.md)。

相机和控制工作区使用私有修复后的 OpenCV 3.2；GPU VINS 使用私有 CUDA OpenCV 3.3.1 和 CUDA Ceres 2.1。每个启动进程选择自己的环境，避免不同 OpenCV ABI 混入同一个 ROS 进程。手动使用时进入对应 shell：

```bash
drone-nav shell ros
drone-nav shell camera
drone-nav shell vins
drone-nav shell fast
drone-nav shell kalibr
```

`fast` 对应原有 `control` 编译环境；原有 source 方式继续可用：

```bash
source /home/z/Drone_Navigation_git/scripts/env.sh camera
source /home/z/Drone_Navigation_git/scripts/env.sh vins
source /home/z/Drone_Navigation_git/scripts/env.sh control
```

Kalibr 使用这台机器已有的 `/home/z/kalibr_workspace` 编译结果和补齐的 Python 2 科学库；选择记录在 `runtime/host.env`。可运行：

```bash
drone-nav shell kalibr
rosrun kalibr kalibr_calibrate_cameras --help
```

## 重建与重新安装

本机同类环境可运行 `bash scripts/install.sh`。该脚本下载缺失的 Ubuntu/ROS 包到用户目录，构建三个运行工作区及 GPU 依赖，安装终端入口，最后执行离线检查。系统 ROS、CUDA、RealSense SDK 2.50 和基础开发包仍使用本机已有安装。

仅更新终端入口时运行 `bash scripts/install_shell.sh`，然后打开新终端。串口权限配置为 `sudo bash scripts/setup_serial_permissions.sh`；安装脚本不会刷写 PX4 固件。本次同步验证覆盖命令展开、环境与软件检查，硬件数据、定位质量、控制和飞行效果仍需接入本机验证。
