# 本机与参考项目结构对齐

2026-10-24，以 `amov@10.53.71.32:/home/amov/Drone_Navigation_git` 为结构和启动流程参考，重构本机 `/home/z/Drone_Navigation_git`。远程参考提交为 `b32f81f8`；对比使用远程当时的实际文件内容，包含尚未提交的启动脚本修改。

## 统一后的入口

| 用途 | 当前入口 | 本机适配与兼容 |
| --- | --- | --- |
| 导航启动 | `scripts/drone_nav.sh` | `vins`、`start`、录包、回放和独立组件共用此入口 |
| 环境定义 | `scripts/env.sh` | 先 source，再执行 `drone_source_profile ros/camera/vins/fast/kalibr` |
| 私有依赖环境 | `scripts/local_env.sh` | 保留本机已验证的 OpenCV、Ceres、sysroot 和编译工作区配置 |
| MAVROS launch | `scripts/launch/mavros_px4.launch` | 使用本机原配置；旧 `launch/mavros_px4.launch` 转发并保留参数覆盖接口 |
| 软件诊断 | `scripts/doctor.py` | 从原 `doctor.sh` 提取，兼容 `--hardware`、`--require-hardware` 和 `--json` |
| 配置或构建 | `scripts/setup_machine.sh --configure-only/--build` | 分别调用本机 `install_shell.sh`、`install.sh` |
| GPU 验证 | `scripts/check_gpu.sh` | 调用本机 GPU 光流和 Ceres CUDA 数值检查 |
| 日志、运行配置和位姿图 | `.runtime/` | 原 `runtime/navigation` 整体迁入；旧路径保留符号链接 |

`scripts/nav_env.sh`、`scripts/doctor.sh` 和 `scripts/drone_nav_remote_stage.sh` 保留为兼容入口。暂存启动脚本不再包含另一套启动实现。Kalibr 也使用统一环境和 launch 路径。

各独立组件与参考机一样，在运行 ROS 命令前等待 master 就绪。`start` 保留定位、桥接、规划、重力检查后启动控制器的顺序；起飞仍使用原本机预检代码和单独命令。

`runtime/workspaces/`、`runtime/local/`、`runtime/sysroot/` 以及本机安装和构建脚本继续服务于本机环境。远程 `/home/amov` 路径、软件依赖安装布局、设备配置和实测报告没有覆盖到本机。这次对齐统一公共入口与目录约定，本机环境适配文件作为补充保留。

## 参数保留

2026-10-27 后续调整：将根目录 `config/vins/` 的 `left.yaml`、`right.yaml`、`px4_d435_vins_gpu.yaml` 完整同步到项目 `workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/` 及对应 `runtime/workspaces/` 目录。`drone-nav` 默认从项目 `workspaces/` 中读取标定配置，再生成 `.runtime/vins_runtime.yaml`；VINS 二进制与依赖继续使用运行工作区。日常配置编辑位置以项目 `workspaces/` 为准。下述比对及保留记录描述 2026-10-24 重构时的状态。

对远程 `workspaces/`、`system_overrides/`、`calibration/`、`metadata/` 的 2,764 个文件进行了内容比对，忽略 CRLF/LF 差异。本机无缺失，4 个内容差异文件均保留本机版本：

- `system_overrides/opt/ros/melodic/lib/mavros/request_px4_imu_rate.py`
- `workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/left.yaml`
- `workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/right.yaml`
- `workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/px4_d435_vins_gpu.yaml`

本机 `config/`、`calibration/`、归档与运行工作区内的配置、`runtime/host.env` 和 IMU 请求脚本共 417 个文件建立了 SHA-256 清单，重构后逐项校验，内容全部未变。保留范围包括双目内外参、时间偏移、设备默认值、EGO、px4ctrl、相机和 MAVROS 配置。

脚本中的本机频率设置、起飞预检阈值和 Kalibr 求解参数也保留；例如双目标定的 `--approx-sync 0.1` 未替换为远程的 `0.005`。运行目录调整仅改变文件路径。

## 验证

- `python3 -m unittest discover -s tests -v`：78 项通过；覆盖统一入口、master 等待、启动与清理、录包回放、起飞预检和 Kalibr。
- `bash scripts/drone_nav.sh check`：ros、camera、vins、fast、kalibr 五个环境全部通过。
- `bash scripts/drone_nav.sh doctor --json`：失败 0，提醒 0，待接硬件 0。
- MAVROS 重构前、新入口及兼容入口分别通过 `roslaunch --dump-params` 展开，输出逐字节一致。
- Shell/Python 语法检查和参数文件 SHA-256 校验通过。

未启动定位、控制或飞行流程；上述结果是软件与配置验证。

重构前文件备份、参数清单、验证日志和本次改动补丁保存在：

`/home/z/.codex/backups/drone-nav-align-20261024-194920/`
