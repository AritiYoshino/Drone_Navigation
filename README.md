# 无人机视觉定位与协同导航

本机启动命令已对齐参考无人机 `amov@10.53.71.32` 的 `Drone_Navigation_git` 脚本。统一入口为 `drone-nav`，运行时使用本机 `runtime/workspaces/` 的编译副本和私有依赖。参考机实际启动的是 `/home/amov/` 下的独立工作区，不是备份仓库 `workspaces/` 中的归档源码。

本机 `drone-nav` 默认从 `workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/` 读取 VINS 主配置和左右目内参，再生成 `.runtime/vins_runtime.yaml`。日常标定与 VINS 参数在该工作区目录编辑；根目录 `config/vins/` 保留为 2026-10-27 同步时的来源副本。

```bash
drone-nav check                 # 配置、ROS 包和动态库检查
drone-nav doctor                # 本机软件、USB 和串口诊断
drone-nav vins --dry-run        # 查看定位链启动命令
drone-nav start --dry-run       # 查看定位、桥接和 EGO 启动命令
drone-nav vins                  # MAVROS、相机、GPU VINS、VINS RViz
drone-nav start                 # MAVROS、相机、GPU VINS、里程计桥、EGO、EGO RViz
```

SSH 或无桌面运行时加 `--headless`；日志位于 `.runtime/logs/`。`start` 不启动控制器，也不发送目标或起飞消息；控制器、目标点和起飞分别由 `drone-nav controller`、`drone-nav component goals`、`drone-nav takeoff` 手动执行。控制器使用参考机原始 `px4ctrl run_ctrl.launch` 配置，其中自动起降和自动解锁功能开启。

完整命令、兼容入口、设备配置、标定和重建说明见 [LOCAL_SETUP.md](LOCAL_SETUP.md)。移植环境、GPU/ROS 依赖、话题链、标定及 PX4 说明见 [MIGRATION_ENVIRONMENT.md](MIGRATION_ENVIRONMENT.md)。本次命令同步不代表已完成硬件运行或飞行验证。

- `workspaces/`：EGO/px4ctrl、GPU VINS、RealSense、Kalibr 迁入源码及当前默认 VINS 标定配置
- `.runtime/`：运行配置、日志和位姿图目录
- `runtime/`：本机编译副本、私有依赖、录包与标定数据
- `config/`：本机设备默认值、VINS 标定同步来源副本与 RViz 配置；可用 `config/local.env` 覆盖导航入口设置
- `firmware/`：PX4 源码与现有固件文件
- `dependencies/`：Ceres、GPU OpenCV、RealSense、NLopt 源码及必要依赖
- `runtime_snapshot/aarch64/`：原机编译产物及 GPU 运行库，供匹配系统参考
- `system_overrides/`：原机 MAVROS 配置及工具副本
- `calibration/`：D435i/IMU 现有标定参数与报告
- `metadata/`：版本、编译参数、依赖及完整性信息

备份清单校验：`python3 -B verify_backup.py`。校验通过表示备份内容与清单一致；本机运行状态使用 `drone-nav check` 和 `drone-nav doctor` 检查。
