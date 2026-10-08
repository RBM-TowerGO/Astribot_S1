# S1 键盘遥操与实时 FAST-LIO 建图

当前版本默认双雷达共同建图，直接按下述命令启动。标定状态及历史差异仅作为提示记录，不限制建图或遥操。资产定义、独立构建及诊断依据见 [DUAL_LIDAR.md](DUAL_LIDAR.md)。

在远程电脑执行：

```bash
ssh -t -L 8765:127.0.0.1:8765 S1 \
  '/home/astribot/TowerGO/workspace/src/astribot_navigation/scripts/tg-mapping.sh'
```

在这台电脑的浏览器打开 **http://127.0.0.1:8765**。SSH 隧道把电脑上的 8765 端口转发到 S1 的只读网页服务。保持 SSH 连接；关闭浏览器不影响建图。浏览器不需要安装 ROS。

如果已经登录 S1，可在该终端运行：

```bash
cd /home/astribot/TowerGO/workspace/src/astribot_navigation
./scripts/tg-mapping.sh
```

另一个电脑终端建立隧道：`ssh -N -L 8765:127.0.0.1:8765 S1`。

## 操作流程

1. 机器人应已处于可控制状态，结束独立 tg-teleop、tg-chassis 或手柄示例。程序不会自动激活机器人或执行 Home。
2. 启动后保持实车静止。程序复用已有雷达；没有前后雷达及前 IMU 发布者时才请求开启雷达。原始录包、雷达适配与 FAST-LIO 同时启动。
3. 至少采集 10 秒静止数据，所有必需数据连续有效 2 秒后显示 READY。初始化超过 60 秒失败时不进入遥操。
4. 在 SSH 终端驾驶，在浏览器查看点云俯视图、IMU 轨迹、数据新鲜度和磁盘余量。地图支持拖动、滚轮缩放、适应地图和跟随位置。
5. 按 X 正常结束。遥操冻结目标并保持约 3 秒、交还控制权，再录 5 秒尾段，然后正常关闭 SLAM/录包并生成结果。终端会显示结果目录与 complete/incomplete 状态。

| 按键 | 行为 |
|---|---|
| W / S | 前进 / 后退，使用已确认的 SDK202_LOCAL 定义 |
| Q / E | 原地左转 / 右转 |
| A / D | 平移预留，目前停止当前积分并提示未启用 |
| 空格 | 冻结目标，继续建图 |
| + / - | 调整速度 |
| X | 正常结束会话并保存 |
| Ctrl+C | 中断并保留录包，不保证自动生成最终地图 |

默认线速度 0.05 m/s、角速度 0.10 rad/s；可用 `--linear-speed 0.03 --angular-speed 0.05` 调低。按键有效期为 0.2 秒，长按依靠系统自动重复续期。

**保持目标不是物理急停**：底盘可能继续向最后目标收敛。会话每 0.1 秒给遥操发送健康心跳；连接关闭或超过 0.5 秒未更新时，遥操停止增加目标并退出。它不能保证覆盖系统卡死、SDK 原生调用阻塞或进程强杀。实车现场应有人能操作物理急停。

## 地图与坐标定义

- SLAM 使用前后雷达 `/livox/lidar_front`、`/livox/lidar_back` 和前 IMU `/livox/imu_front`。两路雷达按真实逐点时间融合，任一路故障即退出，不降级。后 IMU 有数据时录制，不参与惯性估计。
- 使用已安装 FAST-LIO 的 MID360 参数，固定现有雷达—IMU 外参、关闭在线外参估计。运行配置完整保存在每次会话目录。
- 建图坐标来自 FAST-LIO，轨迹是 **IMU/LIO 轨迹**。不使用 SDK 的局部累计指令作为世界位姿，不依赖未经验证的底盘—雷达外参。
- 本功能输出点云及 XY 俯视投影，**不是占据栅格，不包含自由空间判定，不可直接当作已验收的导航地图**。本版没有回环优化，不保证长程漂移消除。
- 实时预览约 1 Hz，初始 0.1 m 体素，最多 100,000 点和 10,000 轨迹点，超限进一步抽稀；原始录包完整保留。最终地图从录包以 0.05 m 体素重建，不受网页抽稀影响。
- 所有 SLAM 输出及 TF 使用本次会话独立话题前缀，不覆盖厂商 TF。在线为 Domain 25，离线回放为 Domain 125 且仅 localhost。

## 保存与恢复

默认目录：

```text
/home/astribot/TowerGO/workspace/data/mapping/mapping_<时间>_<编号>/
  raw_bag/             原始 MCAP 与 metadata.yaml
  lio_bag/             配准点云、里程计 MCAP 与 metadata.yaml
  base_mid360.yaml     原始 FAST-LIO 参数
  lio_params.yaml      本次实际使用参数
  record_qos.yaml      录制 QoS
  session.json         生命周期、话题、版本、进程、错误与数据计数
  *.log                各子进程日志
  map.pcd              三维点云
  trajectory.csv       stamp_ns,x,y,z,qx,qy,qz,qw
  preview.png          点云俯视投影与轨迹
  finalize.json        导出点数、位姿数与源会话状态
```

会话编号唯一，不覆盖旧目录。最终导出使用磁盘 SQLite 体素索引和分批读取，不在内存累积全部点云。需要额外磁盘空间，启动至少 5 GiB；录制中低于 3 GiB 时中断驾驶并尽力保存。

会话失败或 SSH 断线后，先查 session.json 和进程日志。有完整 metadata 和有效 LIO 数据时，可离线恢复导出：

```bash
./scripts/tg-mapping.sh finalize /home/astribot/TowerGO/workspace/data/mapping/<会话编号>
```

此命令不加载 SDK、不启动雷达、不回放数据。已有 map.pcd / trajectory.csv / preview.png / finalize.json 时拒绝覆盖；请先将已有结果归档到别处，再明确重试。崩溃造成 MCAP 缺失或 metadata 未落盘时不会假称可恢复成功，保留原文件供单独检修。

恢复导出不会把原 incomplete 会话改写为成功；finalize.json 保留源会话状态及错误。X 正常结束时才会自动导出并将本次会话标记为 complete。生成完成后网页服务退出，浏览器可能显示连接断开，以终端及 session.json 为最终结果。

## 故障与进程归属

原始必需传感器、适配输出或 LIO 超过 2 秒未更新，适配器诊断报错，子进程退出或磁盘不足，都会撤销健康心跳。网页不是控制入口，也没有运动 HTTP API。

只向本会话创建的进程/进程组发信号，不运行 pkill，不终止其他人的录包。关闭顺序是遥操、回放器（离线时）、FAST-LIO、适配器、LIO 录包、原始录包。只有本次开启的雷达才尝试关闭；其他订阅者仍在使用时保持开启并记录原因。关闭服务超时视为结果未知，不自动反复切换。

端口被占用时启动失败，不结束原服务。可指定 `--port 8766`，并把 SSH 两端端口一起改为 8766。实车控制权、目标保持及按键行为详见 SSH_TELEOP.md。

## 无硬件验证

```bash
python3 tests/test_tg_teleop.py
python3 tests/test_mapping.py

# 在隔离 Domain 125 回放已有录包，不启动 SDK、不操作雷达服务
./scripts/tg-mapping.sh \
  --replay /home/astribot/TowerGO/workspace/data/bags/move_20260927_192650_move/bag \
  --replay-seconds 30 --port 18765
```

带 `--replay` 是离线回放，W/S 不控制机器人；实车遥操启动时不要传 `--replay`。不传 `--replay-seconds` 时回放整个录包。验证网页时另开 `ssh -N -L 18765:127.0.0.1:18765 S1`，访问电脑本地 18765 端口。

现场验收需由操作者完成：静止初始化 → 低速短按确认方向 → 缓慢绕行、网页确认地图增长 → X 退出 → 核对原始数据、点云、轨迹与控制权释放。自动化测试与离线回放不代表完成了本功能的实车联动验收。

## 2026-10-08 原单雷达版本验证记录

以下是升级前的记录，不能作为双雷达验收结论；双雷达结果见 DUAL_LIDAR.md。

- 原遥操 25 项、新增建图 20 项无硬件测试全部通过；启动器语法和 diff 格式检查通过。
- 真实 FAST-LIO 隔离回放、SSH 隧道和浏览器点云/轨迹预览通过，适应地图与跟随功能已检查。
- `mapping_20261008_164158_07598b`：约 30 秒录包自然结束后自动导出成功，状态 complete。
- `mapping_20261008_163831_10ffe7`：完整约 122 秒原始录包回放，导出 395,650 点和 1,206 个轨迹位姿。该轮发现播放器自然结束被监控竞态误判的问题，现已修复并加入回归测试；原会话保留 incomplete 状态，通过 finalize 恢复上述完整结果。
- `fault_test_v7x30ptb/mapping_20261008_164315_fc854d`：仅对测试自己启动的 FAST-LIO 注入终止信号，会话正确失败、MCAP 正常关闭；离线恢复出 42,605 点、91 个位姿，原故障状态保留。测试报告保存在该父目录的 test_result.json。
- 所有回放为 Domain 125 / localhost；没有构造真实 SDK、调用雷达启停服务或发送实车动作。在线完整联动仍需现场验收。
