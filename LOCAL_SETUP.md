# 本机安装与硬件接入

本机是 Jetson Xavier NX，Ubuntu 18.04.4 / L4T R32.4.3，ROS Melodic，CUDA 10.2，GPU 架构 `sm_72`。源码在独立工作副本编译；本仓库的 `workspaces/`、`dependencies/` 和旧标定作为迁移基线保留。

| 组件 | 本机运行目录 / 版本 |
| --- | --- |
| GPU VINS | `/home/amov/vinsf_gpu_ws`，GPU OpenCV 3.3.1，独立 cv_bridge |
| Ceres CUDA | `/home/amov/third_party/ceres-cuda`，2.1.0 |
| EGO / px4ctrl | `/home/amov/Fast-Drone-250-master`，系统 OpenCV 3.2 |
| RealSense ROS | `/home/amov/realsense_ws`，定制 wrapper 2.2.15，ROS SDK 2.50 |
| Kalibr | `/home/amov/kalibr_ws`，ROS1 / Python2，系统 OpenCV 3.2 |
| Kalibr Python / TeX | `/home/amov/third_party/install`，用户目录依赖 |
| OpenCV 兼容配置 | `/home/amov/drone_nav_adapt/ros_compat` |
| 日志 | `/home/amov/drone_nav_adapt/logs`、本仓库 `.runtime/logs` |

本机 `/usr/local` 另有 RealSense SDK 和 GPU OpenCV，系统 cv_bridge 的头文件导出也曾被改动。本次通过本地 CMake 配置和独立头文件目录，保证相机/规划器使用 OpenCV 3.2，GPU VINS 使用 3.3；相机不再混用 2.36 头文件和 2.50 库。不要在一个终端连续 source 多个工作区。

## 常用命令

新开终端后使用 `drone-nav`。当前终端也可直接执行：

```bash
/home/amov/.local/bin/drone-nav doctor
/home/amov/.local/bin/drone-nav start --dry-run
```

`doctor` 默认只检查软件、动态库、GPU架构和包路径；不会启动飞控、相机或控制器。更完整的 GPU 测试为：

```bash
/home/amov/Drone_Navigation_git/scripts/check_gpu.sh
```

这个测试会实际执行 GPU 特征检测、光流及 Ceres CUDA 求解。

## 接上飞控和相机后的测试

1. 飞控 USB 直连默认 `/dev/ttyACM0:57600`。设备路径不同或使用 TELEM/UART 时，修改本仓库 `config/local.env` 的 `DRONE_FCU_URL`；例如 `/dev/ttyTHS0:921600`。多个相机时设置 `DRONE_CAMERA_SERIAL`。波特率以飞控实际配置为准。
2. 首先拆桨，启动定位链：`drone-nav vins --headless`。它启动 MAVROS、200 Hz IMU 请求器、相机和 GPU VINS。观察 `.runtime/logs`；保持机体静止并按 VINS 初始化要求进行适当运动。Ctrl+C 停止。
3. 需要同时检查规划时，改用 `drone-nav start --headless`。它另外启动里程计桥和 EGO。`drone-nav doctor --hardware` 检查飞控连接、双目图像/深度/IMU/里程计的消息与时间戳。这需要另开终端。
4. 桌面模式可省略 `--headless`，启动独立终端与 RViz。`drone-nav shell kalibr` 打开标定环境；`rosrun kalibr kalibr_calibrate_imu_camera --help` 查看命令。

启动脚本不会自动运行 px4ctrl 或发布起飞/目标指令。`drone-nav controller` 是单独的人工入口，会先检查 VINS 重力方向。`drone-nav component goals` 和 `drone-nav takeoff` 也仅在明确执行时运行。

**新机标定与控制参数仍需实机确认。** 相机内外参、相机–飞控 IMU 时间偏移、深度内参、推力模型、机体方向及 PX4 参数来自旧机，软件检查通过不代表这些参数适合新机。先完成拆桨台架与重新标定，再进行飞行验证。此次不刷写飞控固件。

## 重复配置与构建

```bash
/home/amov/Drone_Navigation_git/scripts/setup_machine.sh --configure-only
/home/amov/Drone_Navigation_git/scripts/setup_machine.sh --build
```

前者配置路径、命令和终端环境；后者按 Ceres → GPU VINS → EGO/px4ctrl → 相机 → Kalibr 构建并验证，默认使用两个编译任务。本脚本复用本机已有 Jetson BSP、CUDA、ROS 和系统开发库。Kalibr 缺失软件包下载并解包到用户目录，无需更换系统驱动。

终端配置修改前备份在 `/home/amov/drone_nav_adapt/bashrc.before-drone-nav`，原启动脚本副本在同目录 `original_scripts/`。旧的自动加载多工作区行已注释，改用 `drone-nav shell PROFILE` 选择环境。MAVROS 使用本仓库 `scripts/launch/mavros_px4.launch`，不覆盖 `/opt/ros`。
