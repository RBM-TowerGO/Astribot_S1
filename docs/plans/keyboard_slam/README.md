# TowerGO 键盘遥控与实时二维 SLAM

更新：2026-09-23。目标：`S1`（`astribot@10.7.154.109`）。

已新增真机软件链路：键盘节点、默认关闭的 SDK 底盘适配器、C++ Livox 输入转换器、LIO 位姿/扫描适配器、只读浏览器地图、录包/回放/地图保存与恢复检查。**软件实现不代表真机运动或移动 SLAM 已验收。** 当前缺少经过验证的底盘停车实现、控制接口证据和实测外参，默认配置会明确拒绝启用硬件及正式建图。

首版：SSH WASD，前雷达＋前 IMU 的 FAST-LIO，SLAM Toolbox 二维地图，Mac 浏览器显示。后雷达只录制；没有横移、自动导航或双雷达融合。所有底盘控制独立于录包、SLAM 和网页。

- [实现与接口](implementation-plan.md)
- [验收清单](acceptance-checklist.md)
- [原真机采集说明](../../workspace/src/astribot_navigation/docs/teleop_mapping/README.md)

## 安装与构建

在 S1 上执行；安装依赖需要管理员终端，构建使用普通用户：

```bash
cd /home/astribot/TowerGO/workspace/src/astribot_navigation
sudo bash scripts/install_real_prerequisites.sh
bash scripts/build_real.sh
source scripts/real_env.sh
```

运行 overlay 位于 `/home/astribot/TowerGO/runtime_ws`；依赖源码位于 `workspace/data/cache/real_dependencies`。FAST-LIO 固定提交 `2fffc570a25d0df172720bac034fbdb6a13d2162`，ikd-Tree 固定提交 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`。构建不修改厂商 SDK 或驱动；接口 overlay 仅重建相同的 Livox 消息定义。

## 先运行无硬件测试

先切换到本项目目录。主机单元测试会明确跳过两项原仿真 ROS 联调测试；这些测试只在隔离离线容器里运行，不能仅凭 Domain 125 判定环境可用。不要复制终端提示符或输出文本。

```bash
cd /home/astribot/TowerGO/workspace/src/astribot_navigation
bash scripts/test_unit.sh
source scripts/real_env.sh
export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 run towergo_real chassis --ros-args -p backend:=fake
```

另开两个已 source 同一环境、同样设置 Domain 125 的终端。先启动键盘并保持不按运动键，再启用假底盘：

```bash
# 终端 B，需要 ssh -t S1 分配 TTY
ros2 run towergo_real keyboard
# 终端 C
ros2 service call /towergo/chassis/enable std_srvs/srv/Trigger '{}'
```

W/S 前后、A/D 原地转向、空格停止、Q 停止退出。启用后必须先收到新零指令，再接受运动键。超过 0.3 秒失联会锁停；故障后先 disable 再 enable。假后端不能连接 Domain 25。厂商 DDS 配置在部分机器上限制发现；本地测试可 `unset FASTRTPS_DEFAULT_PROFILES_FILE`。

```bash
ros2 service call /towergo/chassis/disable std_srvs/srv/Trigger '{}'
```

SSH TTY 没有可靠的松键事件；本版本依靠按键重复和 0.2 秒有效期，初次长按可能顿挫。终端卡顿会清空输入并退出。TTY 无法提供 Mac 端按键原始时间，因此不能把本地队列校验宣称为网络中所有延迟按键的识别；SSH 重连需要重新启动和启用。

## 真机启用门槛

复制 `towergo_real/config/hardware.unverified.yaml` 与 `calibration.unverified.yaml` 到本次配置目录，按接口核验和现场测量填入数据。每份证据必须是实际文件，填写 SHA-256、核验人和日期。不能仅把 `verified` 改成 true。

```bash
ros2 run towergo_real preflight --hardware-contract /绝对路径/hardware.yaml
ros2 run towergo_real preflight --calibration /绝对路径/calibration.yaml
```

硬件还需要一份经过审查的 `stop_chassis(sdk)` 实现，覆盖反馈异常时停车。仓库**没有**提供未经核验的“保持目标”或整机 `stop_robot` 默认实现。证据/停止实现/参数缺失时，创建 SDK 之前就失败。

通过核验、厂商启动和现场停车准备后，在专用控制终端 source 厂商 SDK 的 `env.sh`，再 source `scripts/real_env.sh`，运行：

```bash
ros2 run towergo_real chassis --ros-args \
  -p backend:=astribot -p hardware_contract:=/绝对路径/hardware.yaml
```

该命令仍不自动启用。键盘先启动，独立终端调用 enable。只持有一个控制者，不自动回答 SDK 抢占提示。硬件 disable 会关闭厂商 ROS context，下一次启用需要重启控制进程。

## 录包、实时建图与浏览器

以下命令仅在相应前置验收通过后使用：

```bash
# 采集终端：确认四路数据后启动录包。物理停车后输入 stopped 才正常结束。
ros2 run towergo_real session record --data-root "$TOWERGO_WS/data" --operator 姓名 \
  --calibration /绝对路径/calibration.yaml --hardware-contract /绝对路径/hardware.yaml

# 建图终端：校准未验证时会拒绝启动；不包含底盘控制程序
ros2 launch towergo_real real_mapping.launch.py calibration:=/绝对路径/calibration.yaml
```

先静止 10 秒，独立键盘终端低速驾驶 2–3 分钟闭环，停车后再静止 10 秒。录包中断不负责停车。若录包启动了雷达且实时建图仍使用它，录包会保留驱动并在报告中标记，最后由操作者统一收尾。

Mac 建立隧道并打开 `http://127.0.0.1:8765`：

```bash
ssh -N -L 8765:127.0.0.1:8765 S1
```

网页只显示地图、轨迹、数据年龄、诊断和底盘状态，没有运动接口。失联显示过期状态，不把旧地图视为持续更新。

物理停车后保存，并验证重新加载：

```bash
ros2 run towergo_real session save --output "$TOWERGO_WS/data/maps/本次唯一编号" --physical-stop-confirmed
ros2 run towergo_real session verify-map --output "$TOWERGO_WS/data/maps/本次唯一编号"
```

保存同时生成 PGM/YAML、posegraph/data 和文件哈希报告；verify-map 用独立话题恢复地图与位姿图。保存成功不等于精度合格。

## 离线真实数据回放

在宿主机从项目源码目录执行；校准文件与它引用的证据文件应一起放在 `workspace/data` 下，使用相对证据路径：

```bash
bash scripts/real_container.sh build
bash scripts/real_container.sh run ros2 run towergo_real session replay \
  --bag /data/bags/本次编号/bag --calibration /data/config/calibration.yaml --data-root /data
```

容器固定 Domain 125、无外部网络，不挂载硬件或 SDK。仅回放四路雷达/IMU；不回放控制指令、Joy 或录包中的旧 TF。现有模拟回放入口保持不变。

## 当前交付状态

软件测试和构建的实际结果见[本次实现报告](../../workspace/data/reports/keyboard_slam_implementation/implementation-report.md)。所有现场运动、停车距离、实测外参、移动建图和地图精度项目仍以[验收清单](acceptance-checklist.md)为准；未执行不得标为通过。
