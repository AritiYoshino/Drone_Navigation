# 无人机视觉定位与协同导航备份

**当前 Jetson 的安装、启动和硬件测试说明见 [LOCAL_SETUP.md](LOCAL_SETUP.md)。** 新终端使用 `drone-nav doctor` 检查环境，`drone-nav start --dry-run` 查看启动命令。运行源码位于 `/home/amov` 下的独立工作副本；本仓库 `workspaces/` 保留迁移基线。

独立源码/配置文件夹，非压缩包。迁移基线与本机启动工具分别保留。

**移植所需环境、GPU/ROS依赖、编译隔离、话题链、标定和PX4安全事项见 [`MIGRATION_ENVIRONMENT.md`](MIGRATION_ENVIRONMENT.md)。**

GPU 全链路入口 `scripts/home/vins_quick_start_gpu.sh` 现转到 `drone-nav start`，默认启动感知、定位、里程计桥和规划；控制器需另行手动启动。`scripts/home/takeoff.sh` 仍是独立的人工起飞消息入口。硬件接入步骤见本机说明。

- `workspaces/`：EGO/px4ctrl、GPU VINS、RealSense、Kalibr 源码
- `firmware/`：PX4源码与现有固件文件
- `dependencies/`：Ceres、GPU OpenCV、RealSense、NLopt源码及必要依赖
- `runtime_snapshot/aarch64/`：原机编译产物及GPU运行库，仅供匹配系统参考
- `system_overrides/`：原机MAVROS配置副本；需先比较再应用
- `calibration/`：D435i/IMU现有标定参数与报告
- `metadata/`：版本、编译参数、依赖及完整性信息

校验文件：`python3 -B verify_backup.py`。校验通过只表示备份内容与清单一致，不表示已在目标无人机编译、连接或验证。
