# S1B 真机浏览器导航交接

更新时间：2026-10-23（Asia/Shanghai）。本次为阶段性提交，**真机定位匹配及实时扫描延迟仍未解决**，不能将测试通过理解为已完成真机自主导航验收。

## 环境与用户要求

- SSH：`S1B`，用户 astribot；仓库 `/home/astribot/TowerGO/S1_locomotion`。
- 导航代码：`workspace/src/astribot_navigation`。真实 ROS Domain 25；回放/仿真 Domain 125。
- 用户要求复用键盘 SLAM、遥控、现有规划器和网页，不新增模式、验收文件、哈希或审核启动门槛。
- 页面速度默认 0.05 m/s，可设置 0–0.30；0 不提交运动任务；角速度上限 0.1 rad/s。用户已报告现场验证通过并指定最高 0.30，但未提供实测停车数据；不要编造测量结果。
- 本轮只提交与交接，不接管控制权、不重启运行服务、不发送运动。后续实现默认同样使用替身与回放验证，真实运动须由用户操作。

## 已实现与已观察到的进展

- 独立入口 `scripts/tg-navigation.sh --mode hardware`，localhost:8767，v4 地图；地图原文件不修改。
- 真实双雷达/IMU → FAST-LIO → 底盘坐标桥接 → AMCL → 原 Nav2 规划/跟随 → SDK。导航 TF 和话题使用 `/towergo/nav` 与 `tg_nav_*`。
- 复用键盘遥控的 100 Hz SDK202 局部累计坐标、filter、use_wbc=False、add_default_torso=False；不额外按 yaw 旋转、不控制上身/执行 Home。
- 复用键盘 SLAM 的 `tg_mapping.lidar_control`：没有原始输入发布者时开启厂商雷达服务（双雷达及前 IMU），已有则复用；不启动相机。回放不调用设备服务。
- 控制权使用厂商 RawRequest 交接：等待 response=success，再等待旧服务消失后以 high_control_rights=False 创建 SDK；保留本机互斥锁。已经观察到真实执行器初始化成功、rights=true。服务端可能不发布 ROS 节点名，不能凭节点列表为空判定无人占用。
- 普通零速度保持最后目标，后续可以继续。取消/断线/故障单独调用 SDK stop_robot 并锁定执行器，不自动恢复；这是整机停止，不是单独底盘刹车。此后再次导航目前需要重启会话。
- 移除了验收附件、验证标记、审核信息及外部停车模块哈希的启动门槛；max_speed 为普通配置，停车模块随代码直接导入。
- 已修复停车确认后旧超时信息未清除；无活动任务的取消不再进入停车等待；过期页面 token 返回明确刷新提示。
- 已区分过期 LIO 数据与历史连续性：过期数据允许补历史但不刷新健康；历史缺口清空插值缓存、保留原点、取消任务并要求重新初始化。真正时间倒退/位姿跳变仍报错。
- 黄色水平扫描与红色全高度障碍投影分别可开关；显示当前帧，不累计成地图。当前叠加仍依赖地图 TF，并按源时间过滤过期帧，因此现场仍经常看不到。

## 当前现场证据与未解决问题

最近检查会话：`workspace/data/reports/real_navigation/navigation_20261023_175224_75558e`。恢复时先重新读取当前状态，不假设此会话仍是最新。

- `/api/state` 曾返回 sensors_ready=true、executor.rights=true、lifecycle=true，但 initialized=false、tf=false、amcl_age≈1e20、amcl_covariance=null；没有任务，指令速度为零。
- 实时扫描源时间曾落后约 0.665 秒，而接收健康仅落后约 0.157 秒。显示阈值为 0.5 秒，扫描被隐藏。源时间与接收时间必须分开诊断，不能只改超时阈值掩盖延迟。
- nav2.log 中反复出现 AMCL initialPoseReceived，以及 requested time 比最新 odom→base TF 超前约 0.5–0.7 秒的警告；障碍层也有过期/TF 缓存时间警告。
- 页面初始化按钮直接绑定瞬时 sensors_ready，每 300 ms 启用/禁用，造成闪烁。
- 页面叠加要求先有 map TF，但用户需要用扫描辅助选择初始位置，形成交互上的循环依赖。
- 初始化请求会每 0.5 秒重发直到收到被 Web 接受的 AMCL 位姿。AMCL 回调又拒绝源时间滞后超过 0.5 秒的结果。反复重设是否妨碍收敛需要验证，不能直接当作已证实根因。
- `Geometry.deskew` 每次重建最多 256 个历史姿态；`Sensors.flush` 对 pending 点云可能重复解包/重算，且在 ROS/Web 同一处理链执行。CPU 曾出现两个 Python 进程约 108% 与 74%，尚未完成进程归属、阶段耗时与积压测量。
- 已把初始位姿消息时间改成最新可用里程计时间，但 AMCL 内部仍可能需要变换到当前时刻；**此改动并没有解决整个 TF/延迟问题**。

## 下一步：优先完成位置匹配修正和数据延迟

1. **先测量再改延迟链路。** 分别记录原始点云、LIO、底盘 odom/TF、投影 scan、AMCL 的源时间、接收时间、处理耗时、队列长度。核对时间基准与前后雷达来源，定位累计延迟发生在哪一段。
2. **消除重复工作与排队。** 检查点云重复解包、deskew 历史重建、同一线程阻塞与过深 pending 队列；优先复用键盘 SLAM 已有算法。点云处理采用可解释的有界队列/最新帧策略，不修改时间戳伪装新鲜，不降低避障覆盖来换速度；保留匹配时间的姿态插值。
3. **打通 AMCL 修正链。** 用户提供粗略位置与朝向，AMCL 根据当前雷达与静态地图修正；保证扫描时刻 TF 可用、定位节点已激活、位姿结果订阅正确。初始位置提交应成为一次明确的事务，避免反复覆盖滤波器状态；失败时给具体原因，不能永久停在“正在初始化”。
4. **改善定位前可视化。** 无 map TF 时提供机器人局部坐标实时扫描；用户选点时按所选位姿预览投影，明确标为“手动位置预览”。定位成功后切换真实 map 变换，同时区分“手动初始位置”和“定位修正位置”。预览不得被用作实际导航定位。
5. **稳定页面。** 选点按钮不随瞬时健康反复闪烁，允许稳定选点；提交时报告真实就绪原因。分别显示传感器、定位、扫描更新时间与连接状态，不能把旧数据标成实时。普通等待、错误、恢复的状态文字保持一致。
6. **确保恢复语义。** 数据中断必须取消活动任务，恢复传感器或重新定位不能自动续走。不要把停止请求等同于已静止，也不要直接放宽运动健康阈值。

验收应先完成不运动的静止定位：粗略选点后可看到 AMCL 修正，扫描与墙面叠加可评估，按钮稳定，数据延迟指标可解释；持续运行验证不会因处理积压反复掉线。之后使用录包验证暂停、延迟、缺帧与时间倒退，再由用户做实际运动验收。保留速度、停止、控制权和原仿真/遥控回归。

## 启动与验证

S1B：
```bash
/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation.sh --mode hardware
```
电脑：
```bash
ssh -N -L 8767:127.0.0.1:8767 S1B
```
浏览器 http://127.0.0.1:8767 。服务重启后必须刷新整页，旧 token 无效。修改 Python 后需要重启服务才生效，勿把磁盘修复当成当前进程已修复。

无硬件测试（导航目录）：
```bash
python3 -m pytest -q tests/test_navigation_recovery.py tests/test_real_navigation.py tests/test_navigation_chassis_interface.py tests/test_web_navigation.py tests/test_tg_teleop.py tests/test_mapping.py
bash scripts/tg-navigation.sh --mode hardware --check
```
`--check`本次提交前执行上述六组测试：112 passed, 1 skipped（27.08 秒）；暂存内容通过 git diff --cached --check。

 不创建 SDK。替身测试涵盖 SDK 初始化与控制权交接，但不是实际硬件端到端运行证据。现场确实观察到过传感器/SDK上线，尚未观察到稳定的完整 AMCL 就绪及导航到点结果。

## 提交范围

保存导航工作区的当前源码、测试及文档，包括本次会话前已存在的仿真网页/速度设置改动，作为可继续开发的检查点。不提交运行 pycache、数据目录及其链接变化、现场日志、临时 navigation.before_user_confirmation 配置备份；这些现有工作区内容原样保留。此次不 push。
