# VINS IMU 200 Hz 诊断（2026-10-16）

**当前尚未修复 IMU 不足 200 Hz 的问题。** 初次排查已将频率损失定位到 PX4 内部 IMU 到 MAVLink 消息生成、发送这一段：原参数下内部 `vehicle_imu` 约为 197–201 Hz，主机实际收到约 140 Hz，VINS 预测输出随输入保持在相近频率。随后完成的台架测试否证了单独将 `IMU_GYRO_RATEMAX` 从 800 改为 1000 的修复候选，已恢复原值并再次重启核验。

## 测量条件与结果

飞控为 PX4 FMUv6C，运行 Release 1.16.2，精确固件 Git hash 为 `54f0455ffcd755534539a7cf33a09a20bf71d29d`。以下直接 USB 测量均在飞控未解锁、ROS/MAVROS/VINS 未运行时完成；源码分析对应这个固件版本。

| 检查项 | 结果 |
| --- | --- |
| 内部 IMU 源 | `vehicle_imu` 实例 0 为 197–199 Hz，实例 1 为 201 Hz；`IMU_INTEG_RATE=200` |
| 请求 200 Hz 后的 USB 接收 | HIGHRES_IMU 约 138.54 Hz；按消息采样时间戳和主机接收时间计算的频率接近 |
| 独立 8 秒接收完整性检查 | HIGHRES_IMU 142.47 Hz，ATTITUDE_QUATERNION 140.86 Hz；接收序号无缺口，相关消息 CRC 无失败 |
| 既有完整 VINS 链路 | 输入 IMU 138.54 Hz，`imu_propagate` 138.68 Hz |
| USB 发送预算 | 100000 B/s；请求 200 Hz 时 `tx rate mult=1.000`、`txerr=0` |
| 飞控负载 | CPU 约 63.9% idle；USB MAVLink 主循环平均间隔 3023.65 μs，最大 13450 μs，RMS 263.431 μs |

接收完整性检查说明：在该测量窗口内，生成并接收到的 MAVLink 包本身已经不足 200 Hz。问题可以在不运行 VINS 的条件下复现。USB CDC 连接地址中的 `:57600` 也不是此次 USB 吞吐上限。

VINS 初始化完成后，对每条输入 IMU 执行预测并发布 `imu_propagate`。配置中的 `freq: 7.5` 控制视觉优化输入帧率，不是 IMU 的频率上限。

## 提高消息请求频率的对比

以下是 HIGHRES_IMU 的实际采样时间戳频率。三路指 HIGHRES_IMU、ATTITUDE_QUATERNION、ATTITUDE。

| 临时消息请求 | 实际 HIGHRES_IMU | 说明 |
| --- | ---: | --- |
| 三路均为 200 Hz | 138.54 Hz | 复现问题 |
| 三路均为 250 Hz | 136.74 Hz | 未改善 |
| 三路均为 400 Hz | 159.17 Hz | 仍不足 200 Hz |
| 三路均为 1000 Hz | 164.64 Hz | 触发预算缩放，`rate mult=0.525` |
| 仅 HIGHRES_IMU 为 1000 Hz，其余保持原值 | 158.58 Hz | `rate mult=1.000`，仍不足 200 Hz |
| HIGHRES_IMU/QUAT 为 625 Hz，ATTITUDE 为 50 Hz | 163.13 Hz | `rate mult=0.985` |

这些结果尚未找到能得到 200 Hz 的消息请求设置。现有请求器每隔 5 秒重发同样的请求不能消除该问题；其默认最低合格阈值为 170 Hz，也不能作为已达到 200 Hz 的验收标准。命令被接受只表示请求成功，实际频率仍需测量。

## 时序线索与尚未确认的机制

当前选中的 BMI088 陀螺设置为 `IMU_GYRO_RATEMAX=800`。其 FIFO 样本周期为 500 μs，驱动把目标交付周期 1250 μs 按样本数取整到 1500 μs，因此陀螺实际交付约 666.7 Hz。现场测得 gyro 间隔 1501.9 μs、accel 间隔 1249.3 μs。

`VehicleIMU` 把目标 5 ms 积分量化为约 3 次 gyro 更新与 4 次 accel 更新。实测 `delta_angle_dt` 出现约 4506/6008 μs，`delta_velocity_dt` 约 4996 μs，说明两者没有均匀且完全对齐的 5 ms 交付时序。

该固件的 HIGHRES_IMU 由 MAVLink 主循环轮询 `vehicle_imu`，读到选中的新 IMU 数据才发送。该主题队列深度为 1，复制最新值时可能跨过中间更新；消息流另有发送间隔门控和迟到后重设相位的逻辑。因此传感器交付时序、轮询和队列覆盖是后续排查重点。

这些线索还不能证明全部差额来自同一个机制。固定均匀的 200 Hz 源加固定 3 ms 轮询，本身不足以解释全部损失。控制台 `listener` 输出受吞吐限制，其显示的 20–30 ms 间隔不能当作源的真实发布时间间隔。

## 台架结果：800 → 1000 候选失败

2026-10-16 已将 `IMU_GYRO_RATEMAX` 从 800 改为 1000，并实际重启飞控使驱动配置生效；`IMU_INTEG_RATE` 保持 200。原假设是使 gyro 的 5 次更新与 accel 的 4 次更新都接近 5 ms，从而改善交付时序。

重启后选中 gyro 的实际交付升至约 999 Hz、accel 约 800–801 Hz，确认参数已生效。但 `vehicle_imu` 实例 0 反而降至约 147–148 Hz，实例 1 约 156–157 Hz。三组各 30 秒的 USB 测量如下，主机接收频率与表中的采样时间戳频率接近。

| 消息请求设置 | 实际 HIGHRES_IMU | 接收完整性 |
| --- | ---: | --- |
| 三路均为 200 Hz | 137.1669 Hz | CRC 失败 0、序号缺口 0 |
| HIGHRES_IMU 为 200 Hz，其余恢复默认 | 137.2469 Hz | CRC 失败 0、序号缺口 0 |
| HIGHRES_IMU 为 1000 Hz，其余保持默认 | 137.6213 Hz | CRC 失败 0、序号缺口 0 |

该阶段 CPU 约 61.30–62.08% idle，传感器校验状态为 OK、无 failsafe。BMI088 的 DRDY missed、FIFO overflow、bad transfer 均为 0；两次检查中的 `vehicle_imu` accel/gyro data gap 计数均保持为 1，没有增长。这些检查没有发现持续传感器传输错误或 CPU 满载，但也不能据此排除内部调度时序问题。

因此，提高 gyro 交付率这一单参数方案未达到目标，不能作为修复部署。内部 IMU 发布率随参数下降提供了新的时序线索，其具体机制仍需验证；此前发送门控和队列覆盖各自贡献多少损失也尚未确定。本次候选没有进入完整 VINS 链路验收，不能声称正常运行链路已通过 200 Hz 验证。

## 最终状态

候选失败后，已恢复 `IMU_GYRO_RATEMAX=800` 并再次重启，确认运行中的 gyro 交付回到约 666 Hz、`vehicle_imu` 实例 0 回到约 198 Hz、实例 1 回到约 201 Hz；回滚核验通过。`IMU_INTEG_RATE` 仍为 200，USB 消息配置恢复为 HIGHRES_IMU 50 Hz、ATTITUDE_QUATERNION 50 Hz、ATTITUDE 100 Hz。未刷写固件。

后续方案仍需用内部源频率、USB 真实新样本频率与接收频率、CRC/序号和传感器健康数据共同验收，再确认完整 VINS 链路接近 200 Hz。原始诊断及台架运行记录保留在本机，本文不包含设备唯一标识。
