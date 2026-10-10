# Task Memory：Qwen3.5-4B 底盘导航

最后更新：2026-10-10（Asia/Shanghai）\
用途：跨会话交接。阅读本文件后再继续，区分事实、用户决定和待验证推断。

## 用户目标与已确定决策

- 用户希望在 TowerGO 中实现使用 VLM 进行底盘导航与移动。
- 已创建并切换分支：`feat/vlm-chassis-navigation`。
- 用户明确选择：计划使用 `Qwen/Qwen3.5-4B`。
- 用户明确导航不需要腕部相机：后续导航只接头部/躯干视觉，双目备用；左右腕部不作为输入或健康依赖。
- 当前请求：P2 已按用户最新要求完成传感器与 SDK 核查；不做固定数量图片、人工标注集或模型离线效果评测。
- 当前已完成模型资产、隔离环境、合成图正式离线基线及传感器/SDK 接入检查；未执行机器人运动。
- 用户选择只测新进程启动，不重启 S1、不清理系统文件缓存；冷缓存启动未测。
- Qwen3-VL-4B-Instruct 仅为讨论中的对照候选，不默认增加为第二套部署任务。

## 路径与仓库状态

- SSH 别名：`S1`；远端 hostname：`orin`；用户：`astribot`。
- 仓库：`/home/astribot/TowerGO`。
- 本专题：`/home/astribot/TowerGO/docs/plans/vlm_navigation`。
- 2026-10-10 文档创建前确认：分支 `feat/vlm-chassis-navigation`，HEAD `2615b06`，提交说明“底盘移动”。
- 仓库可能有其他会话的未提交改动；后续操作前重新检查，不重置或覆盖。
- 数据入口 `workspace/data/{bags,cache,glass,mapping,maps,reports,trajectories}` 为软链接，目标位于 `/home/astribot/Glass Semantic Knowledge/data/`，检查时均存在。
- 本机 ChatGPT 项目镜像中的 `sources/` 是只读参考，与 S1 仓库不是同一目录。

## 已实测硬件与默认环境

2026-10-10 通过 SSH 读取，数值仅代表检查时刻：

| 项目 | 结果 |
| --- | --- |
| 设备 | NVIDIA Jetson AGX Orin Developer Kit |
| 架构/CPU | aarch64，12 核 Cortex-A78AE |
| 内存 | 系统显示 61GiB，总体为 64GB 级统一内存；可用约 56GiB |
| Swap | 30GiB，检查时未使用 |
| 存储 | 根分区 938GiB，剩余约 768GiB |
| 系统 | Ubuntu 22.04.5 LTS |
| L4T | 36.4.4 |
| CUDA Toolkit | 12.6，nvcc 12.6.68 |
| TensorRT | 10.3.0 |
| 功耗模式 | MAXN（查询结果；未修改） |
| Python | 3.10.12 |
| torch 包元数据 | 2.5.0a0+872d972e41.nv24.8 |
| torchvision 包元数据 | 0.29.1 |

已确认问题：
- 默认 Python 执行 `import torch` 失败：`ImportError: libcusparseLt.so.0: cannot open shared object file`。
- `torchvision` 导入同样受该错误阻断，尚未完成其与 torch 的兼容验证。
- 默认 Python 未安装 transformers、vllm、accelerate、bitsandbytes；不代表所有虚拟环境/容器均未安装。
- nvcc 不在该 SSH shell 的 PATH；`/usr/local/cuda/bin/nvcc` 可运行。
- 普通用户执行 docker ps 被 Docker socket 权限拒绝；未据此判断 Docker 无法使用。
- nvidia-smi 能识别 Orin，但独立显存用量显示不支持；GPU 与 CPU 共享内存。
- 存在 `/dev/video0` 至 `/dev/video25` 节点；尚未验证实际 RGB/深度流，也未打开相机采集。

## 已有项目能力与边界

已读取：
- `workspace/src/astribot_navigation/docs/SSH_TELEOP.md`
- `workspace/src/astribot_navigation/docs/SLICE_NAVIGATION.md`
- `workspace/src/astribot_navigation/docs/S1_CONTROL.md`

依据现有文档：
- 有双雷达 FAST-LIO、二维切片栅格与地图导出。
- 切片地图功能不启动 Nav2 控制器；不能推断真机自主导航闭环已完成。
- 单一高度切片不足以覆盖全部矮障碍/悬空物。
- 底盘遥操使用厂商局部累计指令，不能当世界坐标或实测里程计。
- 冻结最后目标不等于立即刹停；需要核验实际尾程。
- 遥操高优先级控制权行为需要在自主导航设计中明确处理。
- 文档所述既有测试通过记录来自项目资料，本次没有重跑或独立复核。

## 模型选型依据与未证实事项

已查阅官方：
- https://huggingface.co/Qwen/Qwen3.5-4B
- https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct

Qwen3.5-4B 支持视觉输入，默认思考模式可关闭。计划初版关闭思考、单图输入、短结构化输出。
模型版本与推理支持会变化，部署时重新核对并固定 revision。

尚未证实：
- Qwen3.5-4B 在真实机器人输入上的完整决策延迟、资源占用和并行 SLAM 影响；合成图基线已完成。
- 关闭思考后的导航准确率及量化损失。
- 它是否优于 Qwen3-VL-4B-Instruct 的本任务实测效果。
- 当前已明确使用独立 TowerGO 环境与本项目模型缓存；其他环境不在本次验证范围。
不得用通用榜单、参数量、TOPS 或文本 token/s 替代导航实测。

## 设计方向

VLM 理解目标/选择候选点；几何模块计算可达位置；导航模块规划和执行；底盘适配层负责控制接口。
首选以候选 ID 连接模型与几何导航；模型不直接输出持续速度或 SDK 代码。
FP16 为计划效果基线；后续按实测决定 4-bit 量化、推理后端或模型对照。
接口细节见 [architecture.md](architecture.md)，执行阶段见 [implementation-plan.md](implementation-plan.md)。

## 当前进展与下一步

P0、P1 完成；P2 按 2026-10-10 用户调整后的“传感器盘点与 SDK 接入核查”范围完成。
原定固定数量标注集、离线模型评估及阈值不再是本次 P2 要求；未开展、未记作通过。

[P2 报告](../../../vlm_navigation/reports/p2-sdk-20261010T144531.746622Z/sensor-sdk-audit.md) 和同目录 sdk-probe.json/post-state.json 是当前接入事实来源。
- 五路彩色图、四路深度载荷实际可读；SDK 彩色解码 BGR 成功。
- 双 Livox 点云及其 IMU 可临时启用并收到数据；检查后已恢复关闭，相机保持开启。
- 底盘 IMU 数组、头/躯干/底盘关节反馈可读；没有已验证的地图位姿输入。
- 麦克风硬件枚举到，但未录音；力话题无发布者；标准底盘 Imu 无发布者。
- 相机 frame_id 为空，无 camera_info，当前 TF 无发布者；深度单位、RGB-D 配准与相机到底盘标定待核验。
- SDK 直接 Astribot() 构造会申请控制权；读传感器不应使用该初始化流程。
- SDK 状态检查有 200 ms 误报风险，灰度话题拼接有问题，多回调共用配置；优先用独立 ROS 观测进程。

下一步可将真实头部/躯干 RGB 接入 VLM，先保持无运动；几何目标投影前补齐单位、配准、时间与坐标链路。
不要在后续会话自动恢复已被用户取消的固定样本数量或阈值设计任务。

## 更新规则与日志

每次工作结束同步：
- 已完成事项和证据位置；
- 失败原因与尚未证实的假设；
- 配置、版本及设计变更；
- 最具体的下一步。
同步勾选 tasks.md，不删除仍然有效的约束；避免写入凭据。

### 2026-10-10

- 用户确定计划使用 Qwen3.5-4B。
- 检查分支/HEAD，沿用 `docs/plans/` 结构建立 `vlm_navigation/`。
- 写入 README、实施计划、架构、任务清单、验收清单及本 taskmemory。
- 本次只创建文档，未修改既有代码与机器人环境。

### 2026-10-10：环境部署（已完成当前请求）

- 工作目录 `/home/astribot/TowerGO/vlm_navigation`，Conda 环境 `/home/astribot/miniconda3/envs/TowerGO`。
- 原有 Conda 仅 base；未发现可直接复用的 Qwen 缓存。使用 conda-forge Python 3.10.22。
- NVIDIA Jetson torch 2.5.0a0 / CUDA 12.6，cuSPARSELt 0.6.2.3 仅安装到该环境。
- torchvision 0.20.1 官方源码按 SM 8.7 编译成功，与当前 NVIDIA torch 固定匹配。
- Transformers 5.3.0、accelerate 1.12.0、NumPy 1.26.4、Pillow 11.3.0。
- 禁用用户 site-packages，环境内清空 PYTHONPATH，配置动态库路径和固定包版本约束。
- S1 `/tmp` 不存在曾导致 nvcc 编译失败；TMPDIR 改为项目 `cache/build-tmp` 后构建成功。
- `reports/environment-check.json` 已验证 CUDA FP16 运算、torchvision CUDA NMS、视觉处理器输入及关闭思考模板；`pip check` 通过。
- 模型 HF revision `851bf6e806efd8d0a36b00ddf55e13ccb7b8cd0a`；ModelScope 官方同名模型两片权重的 SHA256 与其一致，使用固定文件 revision `fcb1a040bb418b0b8add6f6f6c475386abc2cb97` 加速续传。
- 模型完整下载并通过官方 LFS SHA256 校验；清理了本次续传分片，保留完整模型、源码和构建 wheel。
- SDPA 首次推理因 `enable_gqa` 不兼容失败，已在配置中固定 eager attention；未修改第三方包源码。
- 初次输出带 Markdown，严格 JSON 校验拒绝；收紧提示后两张图均返回正确裸 JSON，并正常结束。
- 最终配置：FP16、eager、关闭思考、max_new_tokens=128、max_input_tokens=4096；参考 DeltaNet 路径，尚未安装加速内核。
- 最终成功报告 `reports/smoke-inference.json`：加载约 6.69 s（不算冷启动），两次预处理到生成约 2.17/1.73 s；不算完整机器人决策延迟。
- 完整部署结果 `reports/deployment.md`，精确环境清单 `configs/pip-freeze.txt` / `configs/conda-explicit.txt`，资产清单 `configs/assets-manifest.json`。
- 激活：`source /home/astribot/TowerGO/vlm_navigation/activate.sh`；验证：进入工作目录后运行 `python scripts/check_environment.py` 和 `python scripts/smoke_inference.py`。
- 计划、任务与验收清单已同步；P1 正式基线未记为完成。没有机器人运动，没有 Git 提交或推送。

### 2026-10-10：P1 正式基线完成

- 新增共享 `scripts/inference_core.py`、私有 worker、基线命令与输出契约测试；冒烟/环境入口保留，报告默认不覆盖历史记录。
- 图像长边限制实际生效；完整请求覆盖读图至输出校验，CUDA 边界同步。
- 环境与冒烟复测通过；新进程就绪约 12.788–13.111 s，首次完整请求约 2.437–2.629 s，启动至首个有效结果约 15.418–15.707 s。
- 预热 5 次，正式 100 次全部通过；P50 1.8186 s，P95 1.8886 s，最大 2.0189 s；各输出 9 tokens。
- CUDA allocated 峰值约 9.239 GB；100 ms 采样下 hot 阶段系统可用内存最低约 37.144 GiB，swap 始终为 0；RSS 与 CUDA 不相加。
- 空/非法输出、候选/字段校验、真实截断、输入超限及独立超时终止测试通过；不代表导航服务故障验收完成。
- 详细报告：[baseline.md](../../../vlm_navigation/reports/p1-20261010T134757.059795Z/baseline.md)。脚本与样本哈希在同目录 report.json；旧脚本和哈希归档在 `vlm_navigation/reports/p1-prechange-20261010-214632/`。
- 已按用户选择调整启动验收口径并勾选 P1/A；P2–P5 未勾选。无重启、清缓存、功耗修改、相机操作、机器人运动、Git 提交或推送。

### 2026-10-10：P2 范围调整及传感器核查完成

- 用户原请求完成 P2，随后明确缩小为“检查传感器种类、可利用方向和 SDK 是否能加载”；取消本阶段固定数量样本与标注要求。
- 采集目录 `vlm_navigation/assets/observations/20261010T144020.529147Z`：真实彩色图、深度原始字节、时间戳及配置快照；inspection.json 记录载荷解析与标定疑点。
- 最终 SDK 报告 `vlm_navigation/reports/p2-sdk-20261010T144531.746622Z`，双点云约 10 Hz、雷达 IMU 约 200 Hz、关节反馈约 250 Hz；仅短时接收观测，不是长期性能保证。
- 首轮雷达检查等待发现不足，保留记录；改为等待实际发布端后再次验证成功。服务返回后 DDS 端点缓存未及时消失，新节点与进程复核 post-state.json 确认恢复关闭。
- 未修改 SDK；以导入的原方法绑定轻量只读 ROS 对象核查相机和雷达，未运行完整构造/控制权/运动流程。
- SDK 灰度读取失败已有明确话题拼接原因，直接 ROS 订阅读取成功；现存其他传感器限制写入 [报告](../../../vlm_navigation/reports/p2-sdk-20261010T144531.746622Z/sensor-sdk-audit.md)。
- 计划、tasks、验收、README 按新范围同步，几何/控制与真实任务效果未勾选。无模型评测、录音、机器人运动或 Git 提交推送。

### 2026-10-10：导航排除腕部相机

用户明确导航可以不使用腕部相机。已将左右腕部从后续导航输入和依赖中排除；硬件盘点结果保留，未关闭共享相机驱动。
