# Qwen3.5-4B 底盘视觉语言导航

更新日期：2026-10-12\
项目：S1 / TowerGO\
分支：`feat/vlm-chassis-navigation`\
状态：P0–P2 完成；下一步 P3：底盘遥操完善与 VLM 数据采集（软件已实现，联合现场验收待完成）。

## 目标

用户通过中文表达移动目标，例如“移动到门旁边”，系统结合相机图像理解目标，利用深度/雷达、标定和机器人位姿确定可达位置，再由导航与底盘控制模块执行运动。

首选模型为 `Qwen/Qwen3.5-4B`，计划部署在 S1 的 Jetson AGX Orin 上。初版关闭思考模式，以单张图像和简短结构化输出建立可重复基线。Qwen3-VL-4B-Instruct 仅作为后续必要时的对照候选，不是当前部署任务。

## 文档入口

| 文档 | 内容 |
| --- | --- |
| [implementation-plan.md](implementation-plan.md) | 分阶段实施、依赖、交付物与完成条件 |
| [tasks.md](tasks.md) | 可勾选任务清单和当前进度 |
| [architecture.md](architecture.md) | 模块职责、数据接口、运动约束与待决事项 |
| [acceptance-checklist.md](acceptance-checklist.md) | 模型、回放、控制及现场验收 |
| [taskmemory.md](taskmemory.md) | 跨会话事实、用户决策、环境问题和下一步 |

## 当前边界

- 当前已完成模型资产、Conda `TowerGO` 环境、P1 正式基线及 P2 传感器/SDK 核查；未连接运动链路。
- 工作目录：`/home/astribot/TowerGO/vlm_navigation`，入口见 [部署说明](../../../vlm_navigation/README.md)。
- 已有双雷达 FAST-LIO、二维切片地图及底盘遥操入口；现有文档不构成真机自主导航已完成的证据。
- 相机彩色/深度、雷达、IMU 和关节读取已核查；见 [P2 报告](../../../vlm_navigation/reports/p2-sdk-20261010T144531.746622Z/sensor-sdk-audit.md)。深度单位、配准、完整时间同步及相机到底盘标定仍待验证。
- VLM 负责目标理解与选择。米制坐标、可达性、避障和连续底盘控制由几何与导航模块负责。
- 已完成 3 次新进程启动和 100 次合成图正式请求，见 [P1 基线](../../../vlm_navigation/reports/p1-20261010T134757.059795Z/baseline.md)；延迟 P50/P95 为 1.819/1.889 秒。真实导航延迟、效果和并行 SLAM 负载仍待验证。

## 下一步：P3 最小实现

复用现有键盘遥操和建图录包，先做到能驾驶、能记录、能离线导出图像给 VLM。

- 完善前后移动、转向、停止与退出，排查已有遥操异常退出；沿用现有按键和限速。
- 同一会话记录头部/躯干 RGB 与深度、双雷达/IMU、可用关节反馈和 LIO 位姿，以及遥操事件、指令和任务描述；同时归档双腕 RGB-D 和头部双目；归档范围与导航模型输入分开。
- 保存原始数据与时间索引，提供简单离线抽帧；只检查基本功能和数据可读性。

顺序：P3 遥操与采集 → P4 无运动 VLM 集成 → P5 几何导航 → P6 自主导航现场验收。
不设固定采集数量，不新增模型对比、量化、性能基准或大规模测试。细节见 [实施计划](implementation-plan.md)。

## 参考

- [Qwen3.5-4B 官方模型卡](https://huggingface.co/Qwen/Qwen3.5-4B)
- [Qwen3-VL-4B-Instruct 官方模型卡](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct)
- [现有底盘遥操说明](../../../workspace/src/astribot_navigation/docs/SSH_TELEOP.md)
- [现有切片地图说明](../../../workspace/src/astribot_navigation/docs/SLICE_NAVIGATION.md)
- [S1 控制工具说明](../../../workspace/src/astribot_navigation/docs/S1_CONTROL.md)

模型卡与软件支持会变化，部署时固定模型 revision、推理引擎版本及环境清单。

P2 范围以用户最新决定为准：只核查传感器和 SDK 接入，不设固定图片数量、标注集或模型评测阈值。

导航相机选择：头部与躯干，双目备用；按用户决定排除左右腕部相机。硬件清单保留其读取验证结果，不代表导航会使用。


## 2026-10-12 联合采集实现

用户确认底盘控制已可用，并要求双腕和双目一并归档。现已扩展 `tg-mapping`：实时建图同时保存五个相机、四路深度、可用状态及遥操事件；网页显示相机、录制状态并记录任务/事件；`tg-export.sh` 离线导出图像及时间关联。导航输入与归档范围分别定义，腕部归档不代表接入导航决策。

操作与验证见 [联合采集说明](../../../workspace/src/astribot_navigation/docs/COLLECTION.md)。实机相机/关节录制与导出已验证，现有 SLAM 隔离回放已通过；整轮实车驾驶与联合采集正常收尾仍未验收，P3 不标为整体完成。下一步：退出独立遥操后，启动该联合入口，观察网页并以 X 正常保存一轮。


2026-10-12 稳定性修复：进入人工遥操后，地图延迟、传感器缺失和采集故障只警告，不撤销键盘会话；磁盘低于 3 GiB 时只停止录制，不自动重启，保留不完整记录。启动仍需数据就绪；控制权丢失、SDK 通信失败、控制循环失效以及 SSH/会话终止仍执行停止发送和释放控制权。 软件故障注入与隔离回放通过；联合实车驾驶尚未验收。证据：[修复记录](../../../vlm_navigation/reports/stability-20261012/repair_report.md)。
