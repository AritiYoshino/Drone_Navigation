# EGO / Kalibr / VINS / PX4 环境迁移清单

> 本清单基于 `/home/amov` 本机记录。它是移植核对表，不代表目标机兼容；不包含自动安装、启动飞控或试飞步骤。只在工作副本修改源码和配置。

## 1. 先记录目标机环境

- [ ] `uname -m`：确认 ARM64/目标架构
- [ ] `lsb_release -ds`、`cat /etc/nv_tegra_release`：确认 Ubuntu、JetPack/L4T、板型
- [ ] `rosversion -d`：确认 ROS 发行版为 Melodic 或已制定兼容方案
- [ ] `gcc --version`、`cmake --version`、`nvcc --version`
- [ ] 确认 GPU 型号、CUDA compute capability、驱动与 CUDA Toolkit 匹配
- [ ] 确认 PX4 飞控板型、固件版本、airframe、MAVROS 串口/UDP 参数

## 2. 本机基线与必须核实项

| 组件 | 本机记录 | 移植检查 |
|---|---|---|
| 系统 | Ubuntu 18.04.6、ARM64、L4T R32.4.4、`t186ref` | 不要直接将 BSP/驱动复制到其他 Jetson |
| ROS / 编译 | ROS Melodic、GCC 7.5、CMake 3.10.2 | ROS1/catkin 启动链；逐项核实目标系统包 |
| CUDA / OpenCV | CUDA 10.2.89；GPU OpenCV 3.3.1 + contrib，记录为 arch 72；另有系统 OpenCV 4.1.1 | **`t186ref` 与 `sm_72` 有架构疑点**；确认目标 GPU 后重建，不混用 OpenCV/cv_bridge ABI |
| Ceres | 2.1.0，CUDA=ON | 在目标 GPU/工具链上重建并检查动态库依赖 |
| 相机 | librealsense 2.36.0；realsense-ros 2.2.15 | 核对 D435i 固件、USB3、权限、profile 与 ROS wrapper |
| MAVROS | 1.16.0 | 核实与目标 PX4 的连接、时间同步及插件配置 |

依赖版本记录见 `metadata/BUILD_PARAMETERS.json`、`metadata/DEBIAN_PACKAGE_SEEDS.json`、`metadata/DEBIAN_PACKAGE_CLOSURE.json`。包清单含仿真/可选依赖，**不要整份盲目安装**。

## 3. 分组件配置与构建

### GPU 基础、OpenCV、Ceres、相机

- [ ] 先确定目标 CUDA/GPU 架构，再构建匹配的 OpenCV + contrib、Ceres。
- [ ] 确认 VINS 使用的 OpenCV、`cv_bridge`、`image_geometry` 和链接到的 `libopencv*.so` ABI 一致；不要混合系统 OpenCV 与自编版本。
- [ ] 构建 librealsense 与 ROS wrapper；分别验证相机图像、深度和 IMU 数据。

### Kalibr

- [ ] 独立构建 `workspaces/kalibr_ws`；源码使用 ROS1/Python 2 工具链，核对 NumPy、SciPy、Matplotlib、PyYAML、igraph 等版本。
- [ ] 若要生成标定板 PDF，补装并验证原机缺少的 `python-pyx` 与 TeX。
- [ ] `cfg/april.yaml` 为 6×6、tagSize 0.088 m、tagSpacing 0.3；必须与实际打印板尺寸/间距一致。
- [ ] 备份**没有**标定输入 bag。本机文件在 `/home/amov/kalibr_ws/src/kalibr/bags/px4_d435_kalibr.bag`（约 1.24 GB）；需要复现标定时再单独归档。
- [ ] 新相机/镜头/IMU 安装后重新标定；检查重投影误差、时间偏移、IMU 单位、坐标系和温漂。

### VINS-Fusion GPU

- [ ] 独立构建 `workspaces/vinsf_gpu_ws`，包含专用 GPU 版 `cv_bridge`/`image_geometry`；避免从其他工作区混 source。
- [ ] 核对 `config/realsense_d435i/px4_d435_vins_gpu.yaml`：GPU 跟踪参数、双目分辨率/频率、PX4 IMU 输入、外参和时间偏移。
- [ ] 当前配置记录 `use_gpu=1`、`use_gpu_acc_flow=1`，CMake 有 `compute_72/sm_72`；确认目标架构后重建，并验证没有静默退回 CPU。
- [ ] 核对 Kalibr 到 VINS 的变换方向；`body_T_cam*` 不能未经坐标约定核实直接复制。

### EGO Planner / px4ctrl

- [ ] 构建 `workspaces/Fast-Drone-250-master` 中的 EGO、消息包、`px4ctrl` 与里程计桥。
- [ ] 核对 EGO 启动配置、深度相机内参、地图/速度/加速度限制、目标点；原值是特定机型实验参数。
- [ ] 核对命名空间 `/uav1`、深度 `/uav1/camera/depth/image_rect_raw`、里程计 `/uav1/vins_estimator/imu_propagate` 及所有 remap。
- [ ] 不要把 Fast-Drone 工作区的旧 VINS 二进制当成 GPU VINS 构建结果。

### PX4 / MAVROS

- [ ] 源码为 dirty `v1.12.3`；项目中随附的 `px4_fmu-v5_default.px4` 标识为 `v1.11.0`，先统一版本并确认目标板型，**不可直接刷写**。
- [ ] UAVCAN `pyuavcan` 子模块缺失，预期提交 `c58477a644d20ccf95a20c151f3a0402f271c3b8`；编译相关目标前解决，见 `metadata/SOURCE_GAPS.json`。
- [ ] 本机 PX4 build 记录是 SITL；为目标硬件单独构建匹配的 FMU 固件并核验产物、参数和校验和。
- [ ] 确认 `/mavros/state`、IMU、视觉里程计输入、EKF 融合状态与 failsafe。

## 4. 推荐验证顺序

1. 目标机系统、GPU、ROS、MAVROS 和设备权限核验。
2. OpenCV/Ceres → RealSense → Kalibr（需要时）→ GPU VINS → EGO/px4ctrl → PX4 固件。
3. 每个工作区独立构建，确认运行时库路径和 OpenCV ABI；再验证图像/深度/IMU频率、时间戳、VINS 初始化、坐标系、PX4 里程计和 EGO 避障。
4. 先做不解锁台架测试；拆桨检查控制方向、急停和 failsafe。完成安全评审前，不运行全链路起飞入口。

## 5. 当前未完成

- [ ] 目标机尚未确认/实机重编译与联调尚未完成。
- [ ] `pyuavcan` 缺口、GPU 架构疑点、PX4 源码/固件版本差异尚未解决。
- [ ] Kalibr bag 未包含在项目备份中。
- [ ] 未进行解锁或飞行验证。
