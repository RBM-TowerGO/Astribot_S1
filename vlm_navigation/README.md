# TowerGO VLM 工作目录

位置：S1 `/home/astribot/TowerGO/vlm_navigation`。关联计划：
[docs/plans/vlm_navigation](../docs/plans/vlm_navigation/README.md)。

## 环境与使用

```bash
source /home/astribot/TowerGO/vlm_navigation/activate.sh
cd /home/astribot/TowerGO/vlm_navigation
python scripts/check_environment.py
python scripts/smoke_inference.py
```

建议先 `cd /home/astribot/TowerGO/vlm_navigation`。激活后环境名为 `TowerGO`，Python 位于
`/home/astribot/miniconda3/envs/TowerGO/bin/python`。
已初始化 Conda 的终端也可直接 `conda activate TowerGO`；环境变量和动态库路径随环境激活生效，退出时恢复。
禁用用户 site-packages 并清空环境内 PYTHONPATH，避免混用系统已有的 torch/torchvision 和 tvshim。

## 目录

- `assets/models/Qwen3.5-4B/`：官方完整模型、视觉处理器、tokenizer、模板及许可证。
- `assets/downloads/`：NVIDIA PyTorch wheel、cuSPARSELt 及 torchvision 构建资产。
- `assets/samples/`：离线环境验证用合成图片；不是机器人导航数据集。
- `configs/`：模型 revision、推理参数、上游文件元数据和下载校验清单。
- `scripts/`：可恢复下载与离线推理检查入口。
- `cache/`、`logs/`：本项目缓存与部署日志，不纳入 Git。
- `reports/`：环境检查与推理结果。报告中未通过或未运行的项目不得记为完成。

大模型、下载缓存和日志由本目录 `.gitignore` 排除；不复用现有 ROS 数据目录的软链接。

## 固定部署选择

- 官方模型：`Qwen/Qwen3.5-4B`。
- revision：`851bf6e806efd8d0a36b00ddf55e13ccb7b8cd0a`。
- Conda Python 3.10、NVIDIA Jetson PyTorch 2.5.0a0 / CUDA 12.6、Transformers 5.3.0。
- cuSPARSELt 0.6.2.3 安装在 Conda 环境的 `lib/` 与 `include/`，未改动系统 CUDA。
- torchvision 0.20.1 采用官方源码，针对当前 NVIDIA torch 和 Orin SM 8.7 编译。
- 初始参数见 `configs/inference.json`：FP16、单图、关闭思考、最多 128 个输出 tokens。
- 未启用 FlashAttention / FLA / causal-conv1d；使用 Transformers 的 eager attention 与 DeltaNet 参考路径，速度待测。
  首次 SDPA 推理因 NVIDIA torch 2.5.0a0 不接受 `enable_gqa` 参数失败，现固定 eager，不修改第三方包源码。

## 下载与完整性

`python scripts/download_assets.py` 下载固定 revision 的官方资产，并验证上游提供的 LFS SHA256。
大权重也可先用 `python scripts/download_model_ranges.py` 从 Qwen 官方 ModelScope 仓库并行分片续传，再运行前一脚本生成完整清单。
ModelScope 固定文件 revision 为 `fcb1a040bb418b0b8add6f6f6c475386abc2cb97`；两片权重最终仍按固定 HF revision 的 SHA256 校验。
后一脚本会检查 HTTP Content-Range、长度及完整文件 SHA256；分片不代表完整模型。
所有资产完成后才会生成 `configs/assets-manifest.json`。

## 验证含义

`smoke_inference.py` 使用两张红蓝矩形交换左右位置的合成图、同一条指令，检查真实像素输入、
关闭思考的聊天模板、JSON 解析、候选答案和正常结束标记。测试不打开相机、不导入 ROS/机器人 SDK。
报告记录模型加载耗时、各次预处理及生成耗时、PyTorch CUDA 分配器峰值和进程 RSS。
CUDA 分配器数字不等于 Jetson 总统一内存峰值；两张图的检查不能替代导航准确率、P50/P95 或 SLAM 并行负载评估。

## 官方来源

- [Qwen 模型](https://huggingface.co/Qwen/Qwen3.5-4B)
- [Jetson PyTorch 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- [torchvision 0.20.1](https://github.com/pytorch/vision/tree/v0.20.1)

## Jetson 依赖复现

先按 `environment.yml` 创建 Conda 环境并下载资产。安装本目录保存的 NVIDIA torch wheel，
再按 `configs/requirements.lock.txt` 中的固定版本安装其余依赖（torch/torchvision 单独处理）。
cuSPARSELt 使用 `configs/cusparselt-upstream.json` 的 linux-aarch64 发行包，校验 SHA256 后将
`lib/` 和 `include/` 内容复制到 Conda 环境对应目录。激活时需包含 Conda 的 lib 路径。
解压官方 torchvision 0.20.1 源码后，执行 `bash scripts/build_torchvision.sh`。

S1 检查时 `/tmp` 不存在，因此将环境的 TMPDIR 指向本目录 `cache/build-tmp`。
部署未修改系统驱动、系统 Python 或既有 ROS 环境。CUDA、cuDNN、编译器使用现有 JetPack 依赖。

`configs/constraints.txt` 通过环境的 PIP_CONSTRAINT 生效，防止无意替换 Jetson 专用 torch。
精确包清单见 `configs/pip-freeze.txt` 与 `configs/conda-explicit.txt`；源码和构建 wheel 哈希见 `configs/torchvision-build.json`。

## 当前结果（2026-10-10）

资产下载与校验完成，环境检查及两张 FP16 合成图 JSON 推理通过。详见 [部署结果](reports/deployment.md)。
后续 P1 正式基线已完成：3 次新进程启动、5 次预热、100/100 正式请求及失败路径检查通过。见 [P1 基线](reports/p1-20261010T134757.059795Z/baseline.md)。真实观测与导航评估进入 P2。

## P1 完整基线测试

在新的 SSH 终端执行：

```bash
source /home/astribot/TowerGO/vlm_navigation/activate.sh
cd /home/astribot/TowerGO/vlm_navigation
python scripts/benchmark_inference.py
```

默认先复测依赖、输出契约、环境与两图冒烟测试，再串行启动 3 个独立推理进程；
随后另起常驻进程，预热 5 次，正式交替推理 A/B 合成图片 100 次，最后验证截断、输入超限和外部超时终止。
全程从本地模型加载，禁用联网下载，不加载 ROS、相机或底盘 SDK。

可调整参数：`--warmup 5 --requests 100 --startup-runs 3 --request-timeout 120 --startup-timeout 300`。
`--output-dir` 指定一个尚不存在的目录；默认创建带 UTC 时间戳的独立目录，不覆盖历史证据。
少于上述默认次数的试跑不能作为 P1 完成证据。环境与冒烟脚本原命令仍可使用，默认也保存到独立目录；
两者均支持 `--output` 指定尚不存在的报告文件。

报告目录包括：`report.json`（汇总、版本、源码与样本哈希）、`requests.jsonl`（每次启动/预热/正式请求）、
`*-resources.jsonl`（资源采样）、环境/冒烟报告及各阶段日志。标准输出给出实际目录。
错误、CUDA 异常或超时会保留已有记录并使程序非零退出；不会自动重试。
测试超时只终止本次创建的推理进程组，不操作现有机器人服务。

计时定义：
- 新进程启动至就绪包含 Python/依赖导入、CUDA 初始化、模型和处理器加载及进程通信。
- `load_seconds` 单独覆盖处理器和模型加载，不包括 Python 导入与首次 CUDA 初始化。
- `decision_seconds` 从读图前开始，到解码和严格输出校验后结束，CUDA 在边界同步。
  包含图片读取、缩放、模板、视觉预处理、传输和生成；不包含相机采集、模型启动和写报告。
- 控制进程往返耗时另存 `controller_roundtrip_seconds`；P50/P95 使用正式请求的完整处理耗时，采用线性插值。
- 不重启、不驱逐文件缓存；所有启动数据仅代表新进程启动，不称为冷缓存测试。

图像按长边上限等比例缩小、不主动放大；模型处理器之后仍会按 patch 网格和像素预算调整，
因此同时记录应用缩放后的尺寸、`image_grid_thw` 及视觉张量形状。
资源按名义 100 ms 间隔采样，报告实际最大间隔；采样可能漏掉瞬时峰值。
系统总内存减可用内存含后台负载，不能全部归因于模型；进程 RSS 与 CUDA 统计分别报告，不能相加。
本测试只验证合成图片上的部署稳定性，不用于宣称真实导航准确率或导航响应频率。

## P2 传感器与 SDK 核查

按用户最新要求，P2 已完成传感器种类、可利用方向与 SDK 读取/启用能力的核查；不要求固定数量图片或模型评测。
详见 [P2 核查报告](reports/p2-sdk-20261010T144531.746622Z/sensor-sdk-audit.md)。

在新的系统 ROS/SDK 终端使用（不要先激活 TowerGO Conda 环境）：

```bash
bash /home/astribot/TowerGO/vlm_navigation/scripts/capture_observations.sh
bash /home/astribot/TowerGO/vlm_navigation/scripts/probe_sensor_sdk.sh
```

前者只订阅当前相机与 TF，保存独立采集目录；后者导入已安装 SDK 的传感器方法，避开完整 Astribot 构造，默认不启停设备。
可选 `--activate-lidar` 会临时启用未运行的共享雷达驱动并在结束时请求关闭；仅在确认无人使用该驱动的检查场景使用。
传感器进程与 VLM Conda 环境分离；已有 P1 推理代码、模型和依赖保持不变。
观测图片与 raw 深度保存在 `assets/observations/`（不纳入 Git）；目录中的 capture.json 和 inspection.json 保留来源、哈希及校验结果。

用户确认的导航视觉范围：头部/躯干 RGB，双目备用；不使用左右腕部。上述采集/盘点工具覆盖全部相机是为核查硬件，不代表实际导航会使用全部通道。
