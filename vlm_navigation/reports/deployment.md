# S1 TowerGO VLM 部署结果

日期：2026-10-10；Git HEAD：`2615b06d39efe661cda86aa71288b9ec4e0d4f98`。
分支 `feat/vlm-chassis-navigation`，存在未提交改动，本次未提交或推送。

## 已完成

- 工作目录 `/home/astribot/TowerGO/vlm_navigation`，Conda 环境 `TowerGO`。
- 官方 Qwen3.5-4B 模型、tokenizer、图像/视频处理配置、模板、许可证完整下载。
- HF revision `851bf6e806efd8d0a36b00ddf55e13ccb7b8cd0a`；权重共 9,319,828,096 bytes。
- 两片权重 SHA256 均符合固定 HF revision 的 LFS 元数据，资产明细见 `../configs/assets-manifest.json`。
- Python 3.10.22、Jetson NVIDIA torch 2.5.0a0、CUDA 12.6、cuSPARSELt 0.6.2.3、
  torchvision 0.20.1（本机 SM 8.7 编译）、Transformers 5.3.0、accelerate 1.12.0。
- `pip check` 通过；CUDA FP16 运算、torchvision CUDA NMS 与视觉预处理检查通过。
- 模型离线加载与 FP16 合成图测试 2/2 通过：相同问题，红色矩形交换左右后输出分别为
  `{"candidate_id":"A"}` 和 `{"candidate_id":"B"}`；两次均正常到达 EOS，严格 JSON 校验通过。
- `enable_thinking=False` 在渲染模板时传入，并核验实际输入 token 中的关闭思考标记及最终无思考输出。

## 本次测量的范围

完整原始记录见 `smoke-inference.json`；环境证据见 `environment-check.json`。
本次模型加载耗时约 6.69 秒；此前已有加载尝试，操作系统文件缓存可能已热，不作为冷启动指标。
两次图片预处理到生成返回的耗时约 2.17 秒、1.73 秒，各输出 9 tokens；不包含摄像头采集、
提示模板构造、最终解码与 JSON 校验。不能将其等同机器人完整决策耗时。
PyTorch CUDA 分配器峰值约 9.24 GB，不是 Jetson 总统一内存峰值，不与 RSS 简单相加。
两张 512×384 合成图片仅验证环境与视觉输入，不构成真实导航准确率、P50/P95 或 SLAM 并行负载报告。
功耗查询为 MAXN；本次未修改功耗模式。

## 遇到的问题与处理

1. 默认 Python 缺失 cuSPARSELt：只在新 Conda 环境补齐 NVIDIA 发行库，并隔离用户 site-packages。
2. S1 `/tmp` 不存在导致 nvcc 构建失败：TMPDIR 指向项目缓存目录后成功构建 torchvision。
3. 当前 NVIDIA torch 的 SDPA 不支持 `enable_gqa` 参数：固定 `attn_implementation=eager`，未修改第三方包源码。
4. 首次模型能正确选目标，但附加 Markdown 代码框：收紧提示格式后通过；保留严格 JSON 校验。
   失败记录见 `smoke-inference-format-failure.json`，未把这次格式失败记为通过。

## 后续工作

真实机器人图片与重复推理性能基线、相机/深度/标定核验、严格服务接口、导航和底盘集成仍待完成。
未安装 FLA/causal-conv1d 等加速内核；当前参考实现是可运行的环境基线。
本次未打开相机、申请底盘控制权或发送运动指令。
