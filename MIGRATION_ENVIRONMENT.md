# 环境与后续移植说明（交给移植 AI）

> 本文基于原 Jetson 的源码、编译缓存和系统版本记录整理。目标是让后续 AI **先核验新机，再适配、配置、构建和测试**；不是可直接照抄的一键安装表，也不代表另一台无人机可直接起飞。
> 本目录是基线备份。要改源码、CMake 或 YAML，应先复制工作目录；不得在本备份内直接移植。

## 1. 任务范围及真实入口

- 算法：EGO Planner 局部规划/避障、GPU VINS-Fusion、PX4/MAVROS 外部定位、`px4ctrl` 控制，带多机轨迹通信源码。
- 当前实际全链路入口：`scripts/home/vins_quick_start_gpu.sh`；`scripts/home/vi_gpu.sh` 是较旧的 VINS/相机/RViz 启动方式。原备份中**没有**名为 `vi_gpu_quick.sh` 的源文件。
- 起飞命令原名：`scripts/home/takeoff.sh`（`Fast-Drone-250-master/shfiles/takeoff.sh` 亦有一份）。用户提到的 `take_off.sh` 并非原有脚本。
- **不要自动运行入口或起飞脚本。**全链路启动会连真实 PX4 并启动控制/规划节点；起飞脚本会发布 takeoff 指令。必须先完成人工安全检查、台架测试及新机试飞评审。

## 2. 原机观察到的环境

| 项目 | 原机记录 | 移植时处理 |
|---|---|---|
| OS/硬件 | Ubuntu 18.04.6、ARM64；L4T R32.4.4，系统板型字符串 `t186ref` | 在新机核实 Jetson 型号、JetPack/L4T、架构、内核和设备树；不要把整套系统包版本当通用版本 |
| ROS | ROS 1 Melodic、catkin；MAVROS 1.16.0 / extras / mavros_msgs | 需安装同代且与 OS 匹配的 ROS/MAVROS、消息与 launch 依赖；ROS 2 不兼容原启动链 |
| 编译器 | GCC 7.5、CMake 3.10.2；VINS C++14 | 新编译器可能需要改源码/CMake，不能保证旧代码无修改通过 |
| GPU | CUDA Toolkit 10.2.89；VINS GPU 跟踪 `use_gpu=1`、`use_gpu_acc_flow=1` | CUDA Toolkit、驱动/BSP、GPU compute capability 必须相互匹配；驱动不是普通源码依赖 |
| OpenCV | 自编 GPU OpenCV 3.3.1 + opencv_contrib，CMake 配置记录 CUDA arch 72 | GPU VINS、对应 cv_bridge、Ceres/OpenCV 链接须 ABI 一致；系统另有 OpenCV 4.1.1，ROS 自带 cv_bridge 还可能依赖 OpenCV 3.2，勿混用 |
| Ceres | Ceres 2.1.0，CUDA=ON；安装到 `dependencies/ceres-cuda` | 目标机应在正确 CUDA/GPU 架构上重建，不能复用不匹配架构的 `.so` |
| 相机 | librealsense 2.36.0；realsense-ros 2.2.15 | 相机 SDK、ROS wrapper、USB/udev 规则及 ROS/OpenCV 版本需相互匹配 |
| 标定 | Kalibr ROS 包源码；Python 2.7、NumPy/SciPy/Matplotlib/PyYAML/igraph | Kalibr 使用 ROS1/Python2 这套旧工具链；在新系统上需验证依赖可安装/可编译 |
| 图形界面 | 启动脚本使用 gnome-terminal、RViz | 原入口需要可用桌面/X11；无桌面设备须由移植 AI 另做进程编排 |

**必须处理的 GPU 疑点：**原系统标识 `t186ref`，但 OpenCV 和 VINS 编译参数是 `sm_72`（CUDA arch 72）。该信息存在硬件架构不一致的可能；本次未执行 GPU kernel 或实机飞行测试。移植 AI 应先读取新机实际 GPU/compute capability，并核验源码及运行库架构，再决定是否使用已有编译产物或重编 CUDA、OpenCV、Ceres 与 VINS。不能仅凭 `t186ref` 或 YAML 推断 GPU 支持。

## 3. 必要软件依赖清单

### 主机与 ROS

1. 适配目标 Jetson 的 Ubuntu/JetPack/L4T、NVIDIA 驱动、CUDA Toolkit，以及匹配的编译器。
2. ROS Melodic（若保留原 ROS1 代码）及 catkin、roscpp/rospy、roslaunch/rosbag/rostopic、RViz、TF/TF2、sensor/nav/geometry/std 消息、image_transport、cv_bridge、nodelet、message_generation/runtime。
3. MAVROS、mavros_extras、mavros_msgs、mavlink；按新 PX4 的连接方式设置串口或 UDP、波特率、system/component ID、时间同步及插件配置。

### 原生编译与相机/GPU

Eigen3、Boost、PCL、Ceres 2.1（启用 CUDA）、GPU OpenCV 3.3.1 与匹配 contrib、与其同版本的 VINS 工作区 cv_bridge、TBB、SuiteSparse、BLAS/LAPACK、glog/gflags、NLopt、libv4l、RealSense SDK 及 realsense-ros。另需 build-essential、CMake、pkg-config、Git、CUDA 编译工具链。准确原机包版本可参考 `metadata/DEBIAN_PACKAGE_SEEDS.json`、`DEBIAN_PACKAGE_CLOSURE.json`、`metadata/BUILD_PARAMETERS.json`；其中含整个 ROS 工作区的可选/仿真依赖，**不可把整份系统包清单不加筛选地全量安装**。

### Kalibr

Python 2 侧需要与源码兼容的 NumPy、SciPy、Matplotlib、PyYAML、python-igraph、Python/C++ catkin 包。生成 AprilTag/AprilGrid PDF 还需 PyX 与 TeX；原机**没有安装 `python-pyx`**，应在目标机显式补齐并验证 `kalibr_create_target_pdf`。原机 `metadata/PYTHON2_DEPENDENCIES.json` 是导入检查记录，不等于每个失败模块都是 Kalibr 必需项。

### 设备、界面和系统配置

- D435i USB 3、RealSense udev/权限、正确相机序列号/流配置；PX4 串口/USB 设备名和组权限。
- 当前 MAVROS 定制文件位于 `system_overrides/opt/ros/melodic/share/mavros/...`，200 Hz IMU 请求器也在该目录。它们应当**逐项比较后再应用到目标系统**，不可直接覆盖新版系统文件。
- GeographicLib 数据仅适用于当前安装/启动方式。检查 GeographicLib 的 `egm96-5` 数据集是否已安装，并使用目标 ROS 发行版提供的正确安装方法补齐；不要盲目覆盖系统数据。
- 原机 Python2 导入自编 OpenCV 时遇到过 AArch64 `libgomp` static-TLS 报错；预加载 `libgomp.so.1` 后导入成功。仅当目标机出现相同错误再评估 `LD_PRELOAD`，不要无条件全局设置。

## 4. 工作区、编译隔离与建议顺序

源码和原机 aarch64 `devel` 参考副本分别位于 `workspaces/`、`runtime_snapshot/aarch64/devel/`。参考副本含旧绝对路径/ABI，只能在**相同硬件、OS、ROS、CUDA、库版本**下做诊断参考；一般移植应从源码重建。不要复制原机 `devel` 后直接认定可运行。

建议先建工作副本并核对各包 README/CMake/package.xml，然后按设备和 ABI 分层处理：

1. 装目标 Jetson BSP/驱动、ROS1/MAVROS 与系统开发包；确认 ROS master、设备访问和 GPU toolkit。
2. 按目标 GPU 重建或匹配 GPU OpenCV + contrib；验证 Python2 `cv2` 与 C++ 共享库。
3. 构建 CUDA Ceres；核对其动态库依赖与 GPU 架构。
4. 安装/构建 librealsense，再构建相机 ROS wrapper。RealSense camera profile 与 GPU VINS profile 的 OpenCV/cv_bridge 可能不同，应分别 source、分别测试，避免错误的 `LD_LIBRARY_PATH` 混合 ABI。
5. 独立构建 `vinsf_gpu_ws`（其内含 GPU 版 cv_bridge/image_geometry）；检查生成代码确实启用 CUDA，而不是静默落入 CPU 构建。
6. 构建 `Fast-Drone-250-master`：EGO、消息、px4ctrl 和 VINS→PX4 里程计桥。不要把 Fast 工作区里旧 VINS 二进制当 GPU VINS。
7. 独立构建 `kalibr_ws`，使用目标机可用的 Python2/ROS Melodic/OpenCV；若发行版不支持该组合，先由移植 AI 给出并验证兼容方案。
8. 对全部 launch、库依赖、话题、时间戳和相机数据做**不连桨/不解锁**的测试，再台架与受控飞行验证。重新编译、所有参数配置和试飞均未在本次备份中执行。

源码的 CMake、YAML 和脚本存在 `/home/amov`、`/usr/local`、CUDA/OpenCV/Ceres 绝对路径；VINS CMake 的 GPU arch 当前硬编码 `compute_72/sm_72`。移植时只改工作副本，并将路径、架构及 CMake cache 参数一起更新。OpenCV major/minor、cv_bridge 和链接到的 `libopencv*.so` 必须一致。

## 5. 当前数据与消息链

```text
RealSense D435i 双目红外图像 ─┐
                               ├─ GPU VINS-Fusion (/uav1)
PX4/MAVROS IMU data_raw ───────┘        │
                                         ├─ /uav1/vins_estimator/imu_propagate
                                         │        ├─ px4ctrl / 外部里程计桥
                                         │        └─ EGO Planner odometry
深度图 /uav1/camera/depth/image_rect_raw ─────── EGO 地图与避障
```

现有主要配置入口：

- VINS：`workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/px4_d435_vins_gpu.yaml`；双目图像 640×480，输入 PX4 `/mavros/imu/data_raw`，`estimate_extrinsic=1`、`estimate_td=1`，旧相机 IMU 初值 `td=-0.0215941273005`。
- RealSense：`workspaces/realsense_ws/src/realsense-ros/realsense2_camera/launch/rs_camera_ego.launch`；深度/infra1/infra2 原设 640×480、15 Hz，gyro 200 Hz、accel 250 Hz，启动时请求 `unite_imu_method:=linear_interpolation`。须按实际相机固件、带宽及 VINS 帧率重新核对。
- px4ctrl 桥：`workspaces/Fast-Drone-250-master/src/realflight_modules/px4ctrl/launch/vins_to_px4_odom.launch`，旧配置从 `/uav1/vins_estimator/imu_propagate` 向 `/mavros/odometry/out` 发布 50 Hz。
- EGO 实飞启动：`.../plan_manage/launch/single_run_in_exp.launch`、`advanced_param_exp.xml`、`config/traj_server.yaml`。旧地图/速度/加速度、深度相机内参和目标点是某一机型实验值，须逐项确认。
- 控制参数：`.../px4ctrl/config/ctrl_param_fpv.yaml`；不可直接用于新机起飞。

至少核验：ROS master、`/mavros/state` 已连接、IMU/两路图像/深度消息持续且时间戳正确、VINS 初始化与重力方向、坐标系/外参和延时、视觉里程计写回 PX4、EKF 融合状态、EGO 深度投影/避障效果、控制方向与 failsafe。使用 ROS 名称空间 `/uav1`，topic remap 必须与相机和控制 launch 一致。

## 6. 标定流程和已知参数

Kalibr 工具源码在 `workspaces/kalibr_ws/src/kalibr`，原机 Kalibr 构建产物也保存在 `runtime_snapshot/aarch64/devel/kalibr_ws`。输入 bag 没有打包。AprilGrid 示例 `cfg/april.yaml` 为 6×6、tagSize 0.088 m、tagSpacing 0.3；它必须与**实际打印板尺寸/间距**一致。先采集覆盖全视场、角度、景深的高质量同步双目图像，再做双目相机标定和相机–IMU 时空标定；命令可参考 Kalibr README 及 `rosrun kalibr kalibr_calibrate_imu_camera --help`。标定后检查重投影误差、时间偏移、IMU 噪声/单位、坐标系和温漂，保存新机数据、bag、报告与硬件序列号。

当前结果在 `calibration/`，相机配置在 VINS `config/realsense_d435i/`。VINS 的 `body_T_cam*` 注释明确要求把 Kalibr 的变换取逆；不要只复制矩阵而不确认 `T_cam_imu`/`T_imu_cam` 的方向和 Kalibr/VINS 坐标约定。换相机、镜头、IMU安装方向/位置或支架后必须重标定；新机不可沿用旧 `td` 和外参作起飞参数。

## 7. PX4 版本、移植阻塞与安全边界

- PX4 源码为 dirty `v1.12.3`；随附 `px4_fmu-v5_default.px4` 内标识为 `v1.11.0`。未读取当前飞控固件、参数、board/airframe 或校验和。不要将它当作本机当前固件或直接刷到另一飞控。
- PX4 的一个原始缺失嵌套子模块为 UAVCAN `pyuavcan`，预期提交 `c58477a644d20ccf95a20c151f3a0402f271c3b8`；见 `metadata/SOURCE_GAPS.json`。PX4 编译/相关 UAVCAN 工具前需解析此缺口。
- 本机版本、dirty commit、编译缓存参数、运行时动态库记录在 `metadata/`。多数源码为带本地修改的状态，别用干净上游 clone 覆盖；先比较 diff/commit 并保护校准结果。
- 禁止仅凭“软件能启动”就解锁/起飞。先核对新机固件和 PX4 参数、RC/急停、桨叶、坐标轴/正负号、failsafe、限速限高、飞行场地和安全员；先拆桨做通信/方向检查，再按飞行安全流程由有资质人员低风险验证。
