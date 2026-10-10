# 双雷达 FAST-LIO：实现与验证边界

当前主要输出已升级为水平切片二维占据地图，使用方法与新验证见 [SLICE_NAVIGATION.md](SLICE_NAVIGATION.md)；下文点云产物数量是此前三维版本的历史记录。三维 PCD 现在通过 `--export-pcd` 可选导出。

默认 `scripts/tg-mapping.sh` 使用双雷达融合。W/S、Q/E、空格和 X 行为保持不变。
**在线入口不设标定验收限制**，可直接启动双雷达建图。历史诊断仅作为参考：两个独立静止录包在局部重合区域的诊断配准分别产生约 1.36° 和 1.39° 旋转修正。它们不是已确认的标定误差，也不是批准采用的新外参。不得把离线软件回放成功解释为现场双雷达精度验收完成。未修改厂商配置、未自动标定、未开启硬件。

## 找到的安装定义与既有资产

以下均为 S1 的绝对路径：

- `/opt/astribot_ros/software/livox_ros_driver2/share/config/MID360_config.json`：实际启动配置。前雷达 `192.168.0.12` 外参为零，后雷达 `192.168.0.13` 到前雷达的 RPY 为 `(0.0668947487, -0.5368355616, -178.1950969876)` 度，平移 `(1,-496,84)` 毫米。
- `/opt/astribot_ros/software/livox_ros_driver2/share/livox_ros_driver2/msg_MID360_launch_rviz_msg_type.py`：加载上述配置，将两雷达映射到 front/back 话题，均标为 `livox_frame`；不能仅凭 frame_id 判断是否已变换。
- `/opt/astribot_ros/software/livox_lidar_calib/lib/livox_lidar_calib/astribot_lidar_calib.py`：将后雷达配准到前雷达，写入上述 JSON 的第二项。初值为 yaw 180°、平移 `(0,-0.488,0.08)` 米。**此脚本会修改厂商配置，本次只读，未执行。**
- `/home/astribot/TowerGO/workspace/src/astribot_navigation/scripts/build_static_map.py`：之前的双雷达静止建图资产，直接合并两路实机点云，对比原始、重复变换和逆变换，支持驱动已经应用外参。
- `/etc/config/sensor_calib.json`：前雷达到底盘另有定义；其中后雷达矩阵行列式为负，不能作为刚体旋转使用。前雷达到底盘的朝向不参与当前 IMU/LIO 地图定位。
- SDK 的 `astribot_config/robot_config/astribot_s1/model/*.urdf` 未找到独立雷达安装关节；工程仿真 scan_processor 处理的是各雷达自身坐标系，不能沿用其实机变换规则。
- `MID360_config_ori.json` 的双雷达外参均为零；`MID360_config_calibrate.json` 使用不同 IP 和安装参数，均不是当前启动配置。

## 实现

新增 `towergo_dual_livox` 包，不改原单雷达适配包。输入两路 PointCloud2 已在前雷达坐标系，因此不重复施加后雷达外参。将绝对逐点时间按共同的 100 ms 窗口分组；两路均跨过窗口终点才发布。输出按逐点时间排序，前线号 0–3、后线号 4–7，FAST-LIO 设置八线并关闭特征提取。IMU 仅使用前 IMU，g 转 m/s²，固定前雷达—IMU 外参。

每路最多一秒或 300,000 点；单消息最多 100,000 点。启动丢弃不完整开头；关闭不发布没有双路完整覆盖的尾窗。窗口跨度、输入/输出/起始丢弃/剩余缓冲点数记录到报告。短暂缺帧造成某个窗口缺一路时，丢弃该不完整窗口并记录跳过帧数和舍弃点数，下一完整双路窗口自动继续；不发布单雷达窗口。缓存按一秒及 300,000 点上限滚动丢弃过期窗口，不因暂时积压直接退出；任何一侧时间倒退、迟到或两秒无融合输出仍锁定故障。后雷达作为 READY 和运行健康的必需数据，后 IMU 仅录制。

异常通过原有健康管道撤销遥操，停止目标积分并执行目标保持和控制权释放。可能有收敛尾程；这不代替物理急停。

## 构建与离线使用

```bash
cd /home/astribot/TowerGO/workspace/src/astribot_navigation
./scripts/build_dual_livox.sh
./scripts/tg-mapping.sh --replay /home/astribot/TowerGO/workspace/data/bags/move_20260927_192650_move/bag --port 18765
```

独立安装目录为 `/home/astribot/TowerGO/workspace/dual_livox_ws`，不修改现有 runtime_ws 或厂商工作区。回放强制 localhost、Domain 125，不创建 SDK，也不控制雷达。浏览器通过 `ssh -N -L 18765:127.0.0.1:18765 S1` 访问 `http://127.0.0.1:18765`。页面显示双路新鲜度、贡献点数及标定未验收说明。

`finalize <会话目录>` 不需要新适配包，兼容旧录包。原始录包至少包含前后雷达和前 IMU；三维地图和轨迹导出格式不变。

## 标定诊断（不限制启动）

`towergo_dual_livox/config/` 保存两份静止录包诊断及 `coordinate_verification.json`；启动记录厂商配置 SHA-256；配置变化、缺少记录或尚未验收均只提示，不拒绝启动。局部 ICP 仅用于诊断，未将拟合结果用于生产点云。

验证区分两件事：

1. 坐标约定：两份录包直接叠加时后雷达点落在前雷达 10 cm 范围内的比例为约 34% 和 41%，重复变换仅约 8–9%；结合厂商标定脚本，确认两路已共用前雷达坐标系。
2. 精度验收：共同区域配准仍产生约 1.4° 修正。按保守软件验收阈值（诊断修正小于 3 cm、0.3°）未通过。阈值是本次验收标准，不是厂家精度规格；点云遮挡、采样和场景退化可能影响诊断，不能据此直接改外参。

可在实际建图时观察地图效果，再决定是否需要进一步标定；不要求先完成可靠性验收，也不需要绕过开关。保留真实的 verified 记录，不把未验收改成已通过，不运行厂商写配置标定脚本。


## 2026-10-08 交付验证结果

- 55 项 Python 测试（原遥操 25、原建图 20、新增 10）、5 项隔离 ROS 适配器测试及 9 个 C++ 分窗场景通过。覆盖大小写和控制方向回归、缺后雷达不得就绪、停流、非法线号/时间戳、锁定故障、心跳撤销、压缩网页和录包恢复。
- 完整运动录包回放约 122 秒，软件流程 complete，导出 **740,737 点、1,211 个位姿**。原始录包两雷达各 1,216 帧、两 IMU 各 24,329 条，与输入完全一致；重新逐条读取 MCAP 及轨迹时间顺序校验通过。
- 双雷达适配进程峰值 RSS 约 33 MiB、FAST-LIO 约 161 MiB，适配回调最大约 9.6 ms。网页预览最大约 89,000 点，轨迹保持在上限内；压缩后约 0.61 MB 响应经 SSH 实测约 0.98 秒。数值来自一次软件回放，不是实时性保证。
- 全长回放后，为隔离回放增加可靠输入订阅，并用约 29 秒完整录包复核：前雷达 5,819,904 点、后雷达 5,820,000 点全部进入适配器，录包消息数也完全一致；输入点数等于融合点数、初始化舍弃点数和末尾缓冲点数之和。该次导出 197,256 点、287 位姿。在线仍保持兼容厂商传感器的 best-effort 输入。
- 故障回放在第 12 秒移除后雷达，其他传感器继续，会话在进入 READY 后正确故障退出、关闭录包并标记 incomplete。之后独立 finalize 恢复 99,584 点、116 位姿，源会话仍为 incomplete，导出报告保留标定未验收信息。
- 网页已通过真实 SSH 隧道检查，双雷达点数、新鲜度和标定提示可见；没有运动按钮。
- 同一录包的单雷达基线为 395,650 点，双雷达为 740,737 点。对比图显示覆盖增多，也仍有需核查的局部边缘差异；增加点数不能证明地图精度提高。
- 未连接 SDK、未启动雷达、未发送实车动作。历史记录 `live_alignment_verified=false` 仅作为诊断信息，不影响在线启动；软件 complete 仅代表数据处理和导出完成。

S1 验证产物：

```text
/home/astribot/TowerGO/workspace/data/reports/dual_lidar_20261008_final/
  replay_validation.json      回放性能、网页和进程内存采样
  product_validation.json     MCAP 重读、点数、轨迹及输入数量比较
  single_vs_dual.png          同一路径下的单/双雷达对比
  sessions/mapping_20261008_171826_dc8e4f/
/home/astribot/TowerGO/workspace/data/reports/dual_lidar_20261008_transport/
  transport_validation.json  最终可靠传输与逐点数量守恒检查
  sessions/mapping_20261008_172440_e57de7/
/home/astribot/TowerGO/workspace/data/reports/dual_lidar_20261008_fault/
  test_result.json            后雷达停流故障联动验证
  sessions/mapping_20261008_172324_7aadd0/
```

可复现的坐标诊断工具是 `scripts/verify_dual_coordinates.py`，参数为静止 bag 路径和新报告路径；必须已加载 ROS 环境，需要录包旁的 MID360 配置与当前驱动逐字节一致，拒绝覆盖已有报告。它只读取录包，不启动 ROS 节点或硬件，不自动更新在线验证标记。

## 启动限制调整

按用户要求，取消标定状态、历史配置哈希和标定证据完整性导致的启动拒绝。缺少诊断文件也不阻断建图。保留数据格式、时间戳、传感器掉线、进程冲突、磁盘及控制心跳等运行检查。此次修改仅做无硬件测试，未启动实车。

## 启动缺帧与 Ctrl+C 修复

针对实车会话 mapping_20261008_173850_fb4dd4 的启动故障，缺少一路的短暂分窗不再直接退出，而是跳过该窗口，继续等待完整双雷达数据。计数包含 skipped_windows、front_gap_discarded、back_gap_discarded。持续停流仍由有界缓冲及两秒无输出检查处理。ROS 观察器不再安装自己的信号处理器，统一由会话管理器接管 Ctrl+C；关闭使用幂等 try_shutdown，避免重复关闭上下文。

修复验证：32 项建图 Python 测试、6 项隔离 ROS 适配器测试、11 个 C++ 场景通过。真实 FAST-LIO 回放中模拟启动连续缺失 3 帧后雷达，跳过 2 个不完整窗口后正常 READY 并导出 complete；同一录包 READY 后连续 SIGINT 两次，正常保存 incomplete 且 shutdown_errors 为空。证据：`/home/astribot/TowerGO/workspace/data/reports/dual_startup_fix_x54qn8o7/results.json`。全程没有开启雷达或连接底盘 SDK。

## 启动缓存修复

会话 mapping_20261008_174606_9c67e3 触发的是 `front: Fusion buffer exceeds one second or point limit`。旧实现将缓存保留上限当成退出阈值，在时间戳向前跳跃或两路暂时不同步时，即使只有数帧缓存也会退出。现在按完整窗口滚动裁掉过期数据，记录 front_expired_points/back_expired_points，保留内存上限并等待两路重新对齐。超过两秒未产生新融合窗口仍退出。错误报告优先使用诊断详情，诊断与关闭竞争时从本会话适配器日志补全原因。

修复验证：32 项建图 Python 测试、7 项隔离 ROS 适配器测试、13 个 C++ 场景通过。真实 FAST-LIO 回放人为丢弃双路雷达第 2–16 帧（约 1.5 秒启动数据缺口），滚动裁剪过期数据并跳过 15 个窗口后恢复，最终会话 complete，error 为空、shutdown_errors 为空。证据：`/home/astribot/TowerGO/workspace/data/reports/dual_buffer_fix_ejcqpfzp/results.json`。全程使用隔离 Domain 125，未开启雷达、未连接 SDK、未移动机器人；实车启动效果仍需重试确认。
