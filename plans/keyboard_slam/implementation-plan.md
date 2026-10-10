# 键盘控制与实时 SLAM 实现说明

更新：2026-09-23。实现入口见 [README](README.md)，现场验收见 [清单](acceptance-checklist.md)。

## 组件与隔离

`towergo_real` 包含控制状态机、SSH 键盘、SDK 输出边界、LIO 坐标处理、网页及会话工具；`towergo_livox_adapter` 提供 C++ 实时点云/IMU 转换。模拟 `command_guard`、模拟扫描处理和模拟回放入口不承担真机控制。

SDK 只由 `sdk_backend.py` 在证据验证、显式启用、控制权检查后导入并实例化。仅发送 `astribot_chassis` 的位置目标，`use_wbc=false`、`add_default_torso=false`。用独立 250 Hz 控制线程，复用 SDK 自带 ROS 线程；正常运动按实际 dt 积分，周期异常不补发积压位移。

状态为 disabled/ready/moving/fault。只接受新会话的零指令之后的新运动指令；故障锁停不自动恢复。默认后端 disabled；fake 限 Domain 125；astribot 限 Domain 25、S1 SDK 环境及完整硬件核验配置。硬件关闭 SDK 时会结束其 ROS context，重新启用需要重启进程。

## 公共接口

| 接口 | 类型和约定 |
|---|---|
| `/towergo/cmd_vel_teleop` | `TwistStamped`，frame=base_link，仅 linear.x、angular.z；20 Hz，深度 1，过期/重复/不支持轴锁停 |
| `/towergo/chassis/enable`、`disable` | `std_srvs/Trigger`，启用必须显式调用；disable 执行经验证的停止及资源释放 |
| `/towergo/control_status` | JSON String：state、reason、backend、velocity；5 Hz |
| `/towergo/lio/lidar` | Livox CustomMsg，前点云真实绝对纳秒时间转换为 timebase 与 offset_time |
| `/towergo/lio/imu` | 前 IMU，显式配置原始单位，输出 SI 加速度；不混入后 IMU |
| `/towergo/lio/odometry_raw`、`cloud_body` | FAST-LIO 原始位姿与当前帧去畸变 IMU-body 点云；原始 TF 重映射到私有话题 |
| `/odom`、`odom → base_link` | 同一位姿适配器发布，真实外参转换；不使用 SDK desired 或键盘积分充当测量 |
| `/scan` | LaserScan，scan_front 原点位于真实前雷达，轴平行底盘；未观测角度为 NaN，避免清空未知区域 |
| `/map`、`map → odom` | SLAM Toolbox 唯一发布，分辨率 0.05 m |
| `/towergo/mapping_fault` | transient-local Bool，任何输入时钟/LIO 重启/位姿跳变使会话结束；新建会话才能恢复 |
| `127.0.0.1:8765` | 只读网页，GET `/` 与 `/api/state`；不提供控制请求 |

## 硬件证据与停车约定

`hardware.unverified.yaml` 默认不能启用。必须验证 SDK 202 局部积分坐标、m/m/rad 单位、实测反馈同坐标语义、反馈时戳、控制权交接、控制周期与跟随误差、正常/异常停车和底层进程崩溃看门狗。定量阈值必须来自核验；默认空值不替换成推测值。

停车扩展文件暴露 `stop_chassis(sdk)`，由硬件配置通过绝对路径/相对路径和 SHA-256 固定。它必须按厂商核验结果实现，不能依赖当前反馈仍然新鲜，不能是无操作或未经验证的停止积分。停止失败保留故障原因，不能报告“已停车”。没有该实现则无法创建真机 SDK。

控制输入用 ROS 时间检查生成时间，用单调时钟检查 0.3 秒接收超时和控制周期。正常加速度与故障停车分开。0.2 秒按键有效期不等于实测物理制动时间。TTY 不具备 Mac 端真实按键释放/原始时间，网络延迟输入的完整识别不在本版本保证范围。

## 传感器、坐标与 LIO

FAST-LIO 固定 `2fffc570a25d0df172720bac034fbdb6a13d2162`，ikd-Tree 固定 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`。使用 lidar_type=1 的 CustomMsg 分支，避开 MID360 PointCloud2 分支按扫描角度推算时间的行为。Livox 接口 overlay 只有 wire-compatible 消息定义，不包含驱动。

C++ 转换器检查 PointCloud2 布局、XYZ/强度有限值、timestamp 类型、绝对纳秒单位、逐点顺序、四线编号及 0.2 秒最大帧跨度。float64 绝对纳秒存在约 256 ns 量化间隔，仅容许帧首小于 1 μs 的量化负误差归零，不掩盖真实时间错误。IMU 输入需要显式声明 g 或 m/s²；FAST-LIO 内部仍按静止重力模长归一化，离线检查转换前后初始化行为。

校准配置使用 `T_base_lidar`、`T_imu_lidar` 的 4×4 刚体矩阵，约定 T_A_B 把 B 系坐标变到 A 系。`T_world_base = T_world_imu × T_imu_lidar × inverse(T_base_lidar)`。首帧平地静止姿态定义 odom 原点；姿态和扫描使用相同帧末时刻。

仅处理当前去畸变点云，按实测车体轮廓/高度过滤；scan_front 保留前雷达平移原点。点云和里程计按时间戳精确配对，无配对丢弃，禁止 latest-TF 代替。前后雷达相同 raw frame 名称不代表可以融合。

位姿重置、时间回退或 LIO 发布者变化会停止整条建图 launch。公共里程计暂用保守大协方差，不宣称已标定的定位不确定性；二维 SLAM 使用位姿 TF。首版不把该里程计作为已验收的多传感器融合输入。

## 会话与产物

真实运行使用独立 overlay 和 Domain 25；离线使用专用镜像、Domain 125、无外部网络。真机 launch 不启动控制节点、模拟器、AMCL 或 Nav2 控制器。

record 分配唯一编号，保留四路原始传感器、可用底盘反馈、处理后的观测、轨迹、TF 和诊断；归档雷达配置、校准/硬件配置副本、QoS、操作者、源码哈希、依赖版本及输入审计。未指定配置时归档 unverified 模板，并如实记录。save 同时导出已收到的轨迹 JSON。录包停止前由操作者确认物理停车。session replay 仅回放四路传感器，重新生成 TF，不使用旧轨迹补齐；save 生成地图及位姿图；verify-map 将恢复实例的话题与 TF 私有化。

ROS 包、操作系统依赖版本与源码提交分别保存在 runtime_ws 的清单中。现场必须补充停车时间/距离、至少三处实测尺寸及闭环叠图。静止 bag 和假 SDK 结果不替代这些项目。

隔离镜像使用固定本地基础镜像 ID 和已核验的依赖源码缓存；运行时为上游 FAST-LIO 的硬编码 Log 目录分配临时可写空间，避免非 root 用户关闭空日志句柄时崩溃，不改算法源码。退出异常会使会话收尾失败，并保留日志。
