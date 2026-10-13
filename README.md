# 无人机视觉定位与协同导航备份

独立源码/配置文件夹，非压缩包。原项目未修改。

**移植所需环境、GPU/ROS依赖、编译隔离、话题链、标定和PX4安全事项见 [`MIGRATION_ENVIRONMENT.md`](MIGRATION_ENVIRONMENT.md)。**

GPU全链路原入口为 `scripts/home/vins_quick_start_gpu.sh`；起飞消息脚本为 `scripts/home/takeoff.sh`。它们可能连接真实飞控；未在本次执行。移植并验证前不要启动或起飞。

- `workspaces/`：EGO/px4ctrl、GPU VINS、RealSense、Kalibr 源码
- `firmware/`：PX4源码与现有固件文件
- `dependencies/`：Ceres、GPU OpenCV、RealSense、NLopt源码及必要依赖
- `runtime_snapshot/aarch64/`：原机编译产物及GPU运行库，仅供匹配系统参考
- `system_overrides/`：原机MAVROS配置副本；需先比较再应用
- `calibration/`：D435i/IMU现有标定参数与报告
- `metadata/`：版本、编译参数、依赖及完整性信息

校验文件：`python3 -B verify_backup.py`。校验通过只表示备份内容与清单一致，不表示已在目标无人机编译、连接或验证。
