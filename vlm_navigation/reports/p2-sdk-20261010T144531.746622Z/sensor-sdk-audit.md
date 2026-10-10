# P2：机器人传感器与 SDK 接入核查

日期：2026-10-10。状态：**按用户调整后的范围完成**。

用户明确本阶段只需检查机器人有哪些传感器、哪些可利用，以及 SDK 是否能读取/启用；
不要求固定数量的图片、人工标注集、模型离线准确率或阈值设计。原先这些工作已从当前 P2 完成条件移出，未伪记为完成。

## 结论

彩色相机、深度原始数据、双雷达点云、雷达 IMU、底盘 IMU 数组及头部/躯干/底盘关节状态均已实际收到。
SDK 可以导入，相机读取方法和雷达启停方法已实测。完整 `Astribot()` 构造流程未调用，因为源码显示它会申请控制权。
本次没有底盘、头部、躯干或机械臂运动，也没有修改厂商 SDK。

推荐后续输入组合：头部/躯干彩色图用于 VLM 目标理解，双雷达及其 IMU 用于几何地图、定位与避障，
头部和躯干关节反馈用于将来计算随姿态变化的相机外参。用户随后明确导航不需要腕部相机，左右腕部已排除后续导航输入及依赖，双目作为备用。RGB-D 深度可作为补充，但需先确认单位、配准与标定。
接收数据可用不等于导航坐标、定位与碰撞约束已经验证。

## 传感器清单与实测状态

| 传感器 | 实测数据 | SDK / 接入方式 | 可利用方向与限制 |
| --- | --- | --- | --- |
| 头部 RGB-D（厂商 Bolt 配置） | RGB 1280×720，深度 640×576；约 30 Hz | `register_image_callback('head_rgbd', ...)` 已收到两路数据 | VLM 场景理解；当前能看见桌面与杯子，不再是旧记录中的朝天视角 |
| 躯干 RGB-D（Gemini 335） | RGB 1280×720，深度 640×480；约 30 Hz | `torso_rgbd` 彩色/深度均可读；SDK 解码 BGR 成功 | 首轮视觉输入候选；当前画面边缘有双手，自遮挡需考虑 |
| 左腕 RealSense D405 | RGB/深度均 640×360；约 30 Hz | `left_wrist_rgbd` 两路可读 | 近距离操作、抓取观察；按用户决定排除导航输入 |
| 右腕 RealSense D405 | RGB/深度均 640×360；约 30 Hz | `right_wrist_rgbd` 两路可读 | 按用户决定排除导航输入；本次深度空洞较明显 |
| 头部双目 | 单张 1280×480 左右拼接彩色图；约 30 Hz | `head_stereo` 彩色可读，没有对应现成深度流 | 辅助广角观察；若做立体测距还需拆分、校正与匹配 |
| 前/后 Livox MID-360 | 两路 PointCloud2，约 10 Hz；首帧各 19968 点，含无效/零点 | SDK `activate_lidar()` 启动成功，再订阅 `/livox/lidar_front`、`/livox/lidar_back` | 地图、定位、障碍几何主输入；后雷达驱动已有外参配置，后续必须核对避免重复变换 |
| 两个雷达内置 IMU | `/livox/imu_front`、`/livox/imu_back`，约 200 Hz | 标准 ROS Imu 订阅；随雷达启动 | 激光惯性里程计输入；本次加速度静止读数约 1，不能直接假设已是 m/s²；单位转换及姿态字段需按驱动核验 |
| 底盘 IMU | `/astribot_whole_body/chassis_imu` 有 9 维数组；两轮观察接收速率约 119–250 Hz | 直接 ROS Float64MultiArray；未发现专门的高层 IMU getter | 可用于姿态/运动辅助，但数组无 Header，顺序、单位、时间对齐尚未核验；另一个标准 Imu 话题没有发布者 |
| 头部/躯干/底盘关节反馈 | RobotJointState，分别 2/4/3 维 position；约 250 Hz | 已实收 ROS 数据；SDK 有 `get_current_joints_position/velocity/torque()` 等 getter，但其完整对象初始化未运行 | 相机姿态和执行反馈来源；底盘这三个数不能直接当作地图位姿或已验证里程计 |
| 麦克风阵列 | 系统枚举到 Yundea 8MICA | SDK 有音频启停方法及 ROS 录音示例；当前未见 microphone/stream 发布 | 可用于语音指令；未启用录音、未验证实际音频内容 |
| 力/力矩接口 | `/ft_data` 当前没有发布者，未收到样本 | 仅发现 ROS Wrench 接口 | 不据话题名宣称物理力传感器已可用；不是本阶段视觉导航依赖 |

## 相机载荷与标定检查

实际像素通过 `sensor_msgs/CompressedImage` 读取：

- 彩色：`/astribot_camera/<camera>/color_compress/compressed`，JPEG。
- 深度：`/astribot_camera/<camera>/depth_compress`，raw 字节；结合伴随 CameraImage 元数据，可解为 uint16 图。
- 厂商 `CameraImage.image.data` 实测为空，不能把它当普通 sensor_msgs/Image 直接取像素。
- SDK `need_decode=True` 对 JPEG 返回 **BGR**；传给采用 RGB 的模型处理器前应转换色序。
- SDK 对 depth 仍返回一维 uint8 原始字节，width/height 为 0；需根据元数据确定尺寸、字节序并解码，不自动变成米制深度。

所有保存的 JPEG 均可解码且尺寸与元数据一致；raw 深度长度与 width×height×2 一致。
首张深度非零比例：头部约 74.9%，躯干约 95.1%，左腕约 29.5%，右腕约 52.6%。
这是非零覆盖率，不是深度准确率；raw 数值未据此换算成米。

流时间戳在本次观察中均递增。接收时间减消息时间约 52–70 ms，不能据此认定曝光延迟或硬件同步精度。
相机消息 frame_id 为空；捕获期间无 TF 数据，图中 `/tf` 与 `/tf_static` 发布者均为 0，也没有 camera_info 发布流。
厂商配置有内外参，但存在具体缺口：

- 头部 RGB 与深度分别为 1280×720、640×576，不能直接共享同一像素坐标。
- 躯干标定 1280×720，深度 640×480；宽高比例不同，不能把内参统一乘 0.5 就认定配准正确。
- 腕部标定写 640×320，实际为 640×360；畸变参数均为 1，需确认其真实含义。
- 双目标定分别为 1600×1200，当前每眼 640×480；缩放关系可算，但当前是否同一裁剪/校正模式未验证。
- `filp_config_for_robot.json` 包含躯干图像翻转配置，其与内外参坐标约定的关系仍待核验。
- 部分厂商旋转矩阵有约 1e-3 量级正交误差；没有自动修正，也没有将其记为实物标定通过。
- 外参父系分别为 head/torso/left/right；运动关节到 chassis 的完整变换链未验证。

因此：彩色图可以立即作为 VLM 输入；深度到 RGB、相机到地图的米制目标投影仍需后续专项验证。

## SDK 核查发现

安装路径：`/home/astribot/astribot_sdk_aarch64`。

1. **完整 SDK 构造有控制副作用。** `AstribotInterface.__init__` 在约 102 行附近调用 `acquire_control_rights()`；获得控制权的 setter 会调用机器人 restart 和设置目标状态。`high_control_rights=False` 也不等于只读。此次仅导入类，将已安装的相机/雷达方法绑定到独立 ROS node 的轻量对象，未构造 Astribot/AstribotInterface。
2. **相机状态查询可误报。** `is_camera_activated()` 只等待 200 ms；首轮有四路返回 false，但随后相同 SDK 回调立即收到有效图像，第二轮全部 true。判断可用应以实际收到数据为准。
3. **灰度话题拼接问题。** 注册方法只给 `color_compress` 添加 `/compressed`，导致 grey 注册不到当前真实话题。SDK 测试未收到灰度数据；直接订阅 `/astribot_camera/head_rgbd/grey_compress/compressed` 则读到约 30 Hz JPEG。
4. **同一个接口对象共享回调配置。** 源码将 `user_callback_` 与 `need_decode_` 存在实例上，后一次注册会覆盖前一次。此次逐路验证；正式多相机接入宜采用独立 ROS 回调或经过修正的封装，不混用相互冲突的解码配置。
5. **雷达服务确认不等于数据就绪。** 首轮固定等待 3 秒仍未发现点云话题；第二轮等待实际发布者出现后，成功收到前后点云和 IMU。SDK 示例中的先关闭再开启流程本次未照搬。
6. **图中有话题不代表有有效数据。** 标准底盘 Imu、Wrench 没有发布者；底盘 endpoint_current_states 虽有发布者，本次未收到可解码样本，不将其作为已验证位姿输入。

本次厂商 SDK 文件保持不变，所有新脚本位于 TowerGO VLM 工作目录。

## 实测证据与结束状态

- [最终 SDK 核查记录](sdk-probe.json)：方法源码位置/哈希、相机载荷、点云字段与样本统计、IMU/关节状态、服务及话题发布者。
- [结束状态复核](post-state.json)：新 ROS 节点观察到双雷达发布者为 0、无新点云、无驱动进程；5 路彩色相机发布者仍各 1 个。
- [相机采集记录](../../assets/observations/20261010T144020.529147Z/capture.json) 与 [载荷/标定检查](../../assets/observations/20261010T144020.529147Z/inspection.json)。同目录保留原始彩色图、深度原始数据及标定文件快照。
- 首轮记录保留于 `../p2-sdk-20261010T144355.753087Z/`，明确为发现 DDS 启动等待不足的尝试，不作为雷达读数成功证据。
- 最终 SDK 记录的 `lidar_publishers_after` 仍有旧 DDS 端点缓存，因此另外使用新节点和进程清单核实恢复状态，未把缓存值当作驱动仍在运行或服务返回值当作恢复完成。

雷达已恢复检查前的关闭状态，相机保持原有开启状态。没有模型推理评测、录音、机器人运动、Git 提交或推送。

## 后续接入方式

在系统 ROS/SDK 环境单独运行观测进程，按需获取最新头部或躯干 RGB；通过图像文件或独立进程接口交给 Conda TowerGO 中的 VLM。
不将厂商 SDK 路径混入现有推理环境，也不为读相机而创建有控制副作用的完整 Astribot 对象。
下一步可做无运动的“真实相机画面 → VLM 输出”连接；几何导航前再补齐深度单位、配准、时间对齐、相机位姿和定位链路。

只读采集入口（新 SSH 终端）：

```bash
bash /home/astribot/TowerGO/vlm_navigation/scripts/capture_observations.sh
bash /home/astribot/TowerGO/vlm_navigation/scripts/probe_sensor_sdk.sh
```

第二条默认只读取当前已发布的数据；`--activate-lidar` 会临时开启当前未运行的雷达，并在结束时请求恢复关闭。
该选项只适用于已确认没有其他雷达使用者的检查场景；服务本身会启停共享驱动，不作为正式导航运行期的调用方式。
SDK 检查脚本使用轻量测试对象验证原方法，不代表完整 SDK 控制流程经过验收。
