# 双目内参与相机–飞控 IMU 外参标定

本入口参考用户指定的 [CSDN 标定教程](https://blog.csdn.net/weixin_67623597/article/details/138785149) 组织采集与求解。按用户最新设置，内参、外参图像均为 30 Hz，飞控原始 IMU 为 200 Hz，双目模型为 `pinhole-radtan`。本机保留 ROS Melodic、Python 2 的现有 Kalibr 环境和私有依赖，不执行文章中的 Noetic/Python 3 安装步骤。

四个命令依次为：

```bash
drone-nav kalibr record-intrinsics
drone-nav kalibr solve-intrinsics
drone-nav kalibr record-extrinsics
drone-nav kalibr solve-extrinsics
```

所有命令支持 `--dry-run`，可先查看配置、录制话题或求解命令。采集前先停止已有导航、相机或标定会话；脚本发现冲突节点会退出。录制命令在交互终端中运行，看到开始录制提示后，在该终端按任意键结束，随后等待 rosbag 写完并保存；启动准备期间按下的键不会结束录制。

## 标定板与 IMU 参数

默认标定板文件为 `config/kalibr/april_6x6_80x80cm.yaml`，按用户确认的实物设置：

```yaml
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
codeOffset: 0
```

`tagSize` 是单个标签边长，单位为米；`tagSpacing` 是间隙与标签边长之比。因此标签间隙为 `0.088 × 0.3 = 0.0264 m`。打印后仍以实物尺寸为准；“80×80 cm”是所用板文件的名称，不能代替单标签尺寸测量。求解可用 `--target PATH` 指定其他实物对应的配置。

默认单 IMU 配置为 `config/kalibr/imu.yaml`，话题 `/mavros/imu/data_raw`，`update_rate: 200.0`。白噪声密度已根据本机静置录制的 Allan 曲线更新；偏置随机游走仍保留文章先验：

| 参数 | 当前值 | 来源 |
| --- | --- | --- |
| `accelerometer_noise_density` | `1.4132617817418410e-03` | 本机 Allan 曲线 |
| `accelerometer_random_walk` | `4.8545209750110478e-04` | 文章先验，未重新测定 |
| `gyroscope_noise_density` | `1.4683461994052150e-04` | 本机 Allan 曲线 |
| `gyroscope_random_walk` | `1.7041508754582521e-05` | 文章先验，未重新测定 |

白噪声数据来自 `runtime/calibration/imu_noise/px4_imu_static_2026-10-26-17-40-53.bag`，由已有 `imu_utils` 分析生成 Allan 数据。在 `tau=0.1..1.0 s` 区间计算 `median(sigma(tau) * sqrt(tau))`，再取三轴算术平均；陀螺仪导出值按该程序的 `57.3 * 3600` 因子从度/小时转换为 rad/s。提取明细和曲线图位于 `runtime/calibration/imu_noise/results/kalibr_conversion/`。

这版 `imu_utils` 的 `*_n` 输出包含采样率换算，`*_w` 是曲线最低点的零偏不稳定性，不能直接按同名字段复制为 Kalibr 连续时间噪声密度和偏置随机游走。本次陀螺仪各轴没有清晰一致的 `+1/2` 斜率段，因此两项偏置随机游走暂保留原先验，不将它们标记为本机实测结果。原配置备份为 `config/kalibr/imu.yaml.before-noise-calibration-20261026.bak`。

四个入口本身不自动测量或估计噪声。需要重测时，先独立静置录制约两小时原始 IMU，分析 Allan 曲线并确认参数定义，再通过 `--imu PATH` 指定噪声文件。

## 内参：固定无人机，移动标定板

```bash
drone-nav kalibr record-intrinsics --dry-run
drone-nav kalibr record-intrinsics
```

录制入口复用已有 ROS master；需要时启动本地 master，远程 master 则等待其可用。内参采集的双目红外输入为 640×480、30 fps，通过 `topic_tools relay` 转发到录制话题，保留输入帧率。标定采集关闭 RGB、深度、相机内置 IMU 和红外投射器；脚本保存投射器原值，通过动态配置关闭并验证其已关闭，避免人工红外散斑干扰标定板角点。该设置只用于标定采集，不修改普通导航相机 launch。

默认打开两个 `rqt_image_view` 窗口，分别查看左右目，并自动排列到屏幕左、右两侧；未安装该程序时使用 `image_view`。录制自动开始，终端会显示开始提示；确认两幅图都能清楚看到完整标定板后，固定无人机，手持标定板做前后、左右、上下、倾斜等多方向缓慢运动，覆盖不同距离、角度与画面位置。按文章建议采集约两分钟；运动中保持标定板在双目视野内。

录制话题为：

| 原始输入 | 转发后录入 bag 的话题 | 目标频率 |
| --- | --- | --- |
| `/uav1/camera/infra1/image_rect_raw` | `/infra_left` | 30 Hz |
| `/uav1/camera/infra2/image_rect_raw` | `/infra_right` | 30 Hz |

录制前按传感器消息时间戳测量约 3 秒的实际频率，要求左右目各至少 25.5 Hz；最多等待 20 秒让数据流稳定，缺少数据、时间戳不递增或频率持续不足时退出。30 Hz 是目标值，终端打印的实测值才是当前数据频率。

在启动终端按任意键，等待保存完成。默认文件为 `runtime/calibration/intrinsics/<时间戳>/double_cam.bag`。

随后求解：

```bash
drone-nav kalibr solve-intrinsics --dry-run
drone-nav kalibr solve-intrinsics
```

未指定 `--bag` 时，在 `runtime/calibration/intrinsics/` 中按文件修改时间选择最新的非空 `double_cam.bag`，忽略仍有 `.active` 文件的录制。实际调用对应于：

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --target /绝对路径/april_6x6_80x80cm.yaml \
  --bag /绝对路径/double_cam.bag \
  --models pinhole-radtan pinhole-radtan \
  --topics /infra_left /infra_right \
  --show-extraction --approx-sync 0.1
```

文章命令示例的 `--topic` 使用本机 Kalibr 实际参数名 `--topics`。两个相机分别使用一个 `pinhole-radtan`，近似同步容差保持文章的 `0.1` 秒。默认显示角点提取过程和报告。

Kalibr 在 bag 同目录生成 `double_cam-camchain.yaml`、`double_cam-results-cam.txt` 和 `double_cam-report-cam.pdf`；自定义 bag 文件名时使用相应文件名前缀。检查左右目角点提取和重投影误差；文章以内参重投影误差约 1 像素以内作为较好结果的参考，误差偏大时重新采集求解。下一步使用这里生成的 `*-camchain.yaml`。

## 外参：固定标定板，移动无人机

```bash
drone-nav kalibr record-extrinsics --dry-run
drone-nav kalibr record-extrinsics
```

此入口自动准备 ROS master、双目相机、MAVROS 和 PX4 IMU 频率请求器。相机仍关闭投射器、RGB、深度和相机内置 IMU，双目红外输入与内参相同，均为 640×480、30 fps，通过 `topic_tools relay` 转发并保留输入帧率。同时录制 `/mavros/imu/data_raw`，请求频率为 200 Hz；这是飞控 IMU。

| 原始输入 | 录入 bag 的话题 | 目标频率 |
| --- | --- | --- |
| `/uav1/camera/infra1/image_rect_raw` | `/infra_left` | 30 Hz |
| `/uav1/camera/infra2/image_rect_raw` | `/infra_right` | 30 Hz |
| `/mavros/imu/data_raw` | `/mavros/imu/data_raw` | 请求 200 Hz |

录制前同样使用约 3 秒的采样窗口，要求左右目各至少 25.5 Hz，IMU 至少 170 Hz；最多等待 20 秒，让 MAVROS 连接与 IMU 频率请求生效，持续不达标则退出，不开始 rosbag。30 Hz / 200 Hz 是目标值，不能仅凭启动参数认定已达到，需查看终端打印的实测结果。

将标定板固定不动，手持无人机及其固定安装的相机/飞控，缓慢做前后、左右、上下平移和不同方向转动，保持相机与飞控的安装关系不变。按文章采集约两分钟，始终让双目看到标定板。完成后在启动终端按任意键保存。

默认文件为 `runtime/calibration/extrinsics/<时间戳>/imu_cam.bag`。随后运行：

```bash
drone-nav kalibr solve-extrinsics --dry-run
drone-nav kalibr solve-extrinsics
```

未指定 `--bag` 时，在 `runtime/calibration/extrinsics/` 中按修改时间选择最新的非空 `imu_cam.bag`，忽略仍有 `.active` 文件的录制；未指定 `--cam` 时，在 `runtime/calibration/intrinsics/` 中按修改时间选择最新的 `*-camchain.yaml`。实际调用对应于文章，本机 Kalibr 的相机参数名使用 `--cams`：

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --target /绝对路径/april_6x6_80x80cm.yaml \
  --cams /绝对路径/double_cam-camchain.yaml \
  --imu /绝对路径/imu.yaml \
  --bag /绝对路径/imu_cam.bag
```

结果 `imu_cam-camchain-imucam.yaml`、`imu_cam-imu.yaml`、`imu_cam-results-imucam.txt` 和 `imu_cam-report-imucam.pdf` 保存在 bag 同目录；自定义 bag 文件名时使用相应前缀。检查两相机的重投影误差及相机–IMU 求解结果；文章以重投影误差为零点几像素作为外参结果的参考。仅生成文件不代表标定质量合格。

## 显式路径、设备和无桌面模式

录制时可指定设备和新 bag 路径，已有 `.bag` 或 `.bag.active` 文件不会被覆盖：

```bash
drone-nav kalibr record-intrinsics --serial 相机序列号 --output /数据目录/double_cam.bag
drone-nav kalibr record-extrinsics --fcu-url /dev/ttyACM0:57600 --output /数据目录/imu_cam.bag
```

指定已有数据和参数文件求解：

```bash
drone-nav kalibr solve-intrinsics --bag /数据目录/double_cam.bag --target /配置目录/april.yaml
drone-nav kalibr solve-extrinsics --bag /数据目录/imu_cam.bag \
  --cam /数据目录/double_cam-camchain.yaml --imu /配置目录/imu.yaml \
  --target /配置目录/april.yaml
```

`solve-extrinsics` 的 `--cams` 是 `--cam` 的别名，均接收双目 camchain 文件；这里只有一个飞控 IMU 配置。

自定义文件名或把输出放在默认数据目录之外时，求解请显式指定 `--bag`；自动选择只搜索上述默认目录和 bag 文件名。求解前检查 bag 的左右图像话题及消息类型，外参 bag 还需包含原始 IMU；外参输入的 camchain 必须是 640×480 双目 `pinhole-radtan`，话题对应 `/infra_left`、`/infra_right`。IMU YAML 使用顶层噪声字段、`rostopic` 和 `update_rate: 200.0`，不要把 Kalibr 生成的嵌套 `imu0` 结果文件直接当作噪声输入。

录制加 `--no-preview` 可关闭左右目预览；`--headless` 也不打开预览，但仍需保留交互终端按键结束录制。求解加 `--headless` 时不显示角点提取窗口，并传入 `--dont-show-report`；标定模型、话题、同步和输入参数保持相同，PDF 仍由 Kalibr 输出。所有命令都可追加 `--dry-run`，查看计划而不启动采集或求解。

录制正常结束或中断清理时，先完成 rosbag 写入，随后在停止相机前恢复已保存的投射器原值，再清理本次启动的进程；已有外部 ROS master 不会被停止。恢复失败会打印提示，需在下次导航前检查投射器状态。录制日志位于 bag 同目录的 `<bag文件名>-logs/`，例如 `double_cam-logs/`，包含组件日志、投射器原值 `emitter-before.json`；录制验证完成后还生成 `recording.json`。

标定命令不自动改写 `config/vins/`、飞行工作区或 PX4 参数。结果审核后再把内参、外参和时间偏移写入 `drone-nav` 默认配置目录 `workspaces/vinsf_gpu_ws/src/VINS-Fusion/config/realsense_d435i/` 中的 `left.yaml`、`right.yaml` 和 `px4_d435_vins_gpu.yaml`；根目录 `config/vins/` 保留为 2026-10-27 同步时的来源副本。先确认 Kalibr 与 VINS 的变换方向，按所用 VINS 配置的约定转换，不能直接把 `T_cam_imu` 当成 `body_T_cam`。
