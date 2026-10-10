# 用一个脚本控制 S1

脚本：`workspace/src/astribot_navigation/scripts/s1_control.py`。

在 **S1 终端**运行，无需网页、无需安装 Codex CLI、无需手动 source SDK 环境：

```bash
cd /home/astribot/TowerGO

# 默认只读：查询驱动状态、近期关节反馈和 SDK 控制权占用。
python3 workspace/src/astribot_navigation/scripts/s1_control.py

# 一条命令：检查状态 → 必要时激活 → 检查反馈与控制权 → Home → 核对到位。
python3 workspace/src/astribot_navigation/scripts/s1_control.py home
```

`home` 会实际移动躯干、双臂和头部，并完全张开两侧夹爪。运行前确认夹爪没有夹持物、运动空间无障碍。它不向底盘发送位置目标；SDK 初始化和异常时的 `stop_robot()` 涉及整机控制权/停止，不能在其他控制程序同时操作机器人时使用。

也可从 Mac 发起，脚本和 ROS/SDK 都在 S1 执行：

```bash
ssh -t S1 'python3 /home/astribot/TowerGO/workspace/src/astribot_navigation/scripts/s1_control.py status'
ssh -t S1 'python3 /home/astribot/TowerGO/workspace/src/astribot_navigation/scripts/s1_control.py home'
```

## 操作选项

| 命令 | 行为 |
| --- | --- |
| `status`（默认） | 只查询生命周期、订阅关节反馈和发现服务；不导入 SDK、不申请控制权 |
| `configure` | 仅当控制驱动为 unconfigured 时，请求 `/supervisor/configure_system` |
| `activate` | 仅当控制驱动为 inactive 时请求激活；核实 active、硬件 active 和全部关节反馈 |
| `home` | 执行 activate 的检查流程，然后调用 SDK Home 并核对动作后的反馈 |
| `deactivate` | 仅当控制驱动为 active 时请求系统停用；不是急停，不作为 Ctrl+C 的替代 |

如果系统是 unconfigured，`home` 会明确退出并提示先运行 `configure`，不会自动配置系统。任何未知状态或状态请求超时都会阻止激活/Home。`--timeout 8` 是默认的单次服务和反馈等待时间，不是整个动作的总时限。

## 环境与接口

- 目标：S1 Orin，Ubuntu 22.04 / ARM64，ROS 2 Humble，Domain 25。
- SDK：`/home/astribot/astribot_sdk_aarch64`，核查版本 `37c83d0`。
- 脚本使用厂商 `local_setup.bash` 与现有 `/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml`，不 source 会执行 sudo/sysctl 或重写 XML 的机器人环境脚本。
- SDK 路径与依赖路径在进程内设置，不修改系统或 SDK 文件。
- 通过 `/supervisor/{configure,activate,deactivate}_system` 的 `std_srvs/srv/Trigger` 请求操作，必须检查返回的 success/message 和之后的状态。
- 通过驱动及硬件节点的 `lifecycle_msgs/srv/GetState` 读取状态。
- 订阅 `/<部位>/joint_space_states`，类型 `astribot_msgs/msg/RobotJointState`；要求头部、躯干、双臂、双夹爪和底盘都返回维度正确、有限且近期的状态，因为 SDK 初始化检查全机接口。

## 失败与退出

- 服务已经发出但响应超时，意味着结果未知，不代表操作取消。脚本不自动重试，也不继续执行 Home；重新用 `status` 查询。
- 发现已有 `/astribot/control_rights` 时退出，不强制抢占。SDK 构造期间即使出现交互询问，也会自动给空回答表示拒绝；不用 EOF 来回答，避免 SDK 的异常回退行为。
- Home 使用厂商原实现，保留其碰撞检查。未修改 SDK 内部的时长限制，不把较大的 duration 当作可靠减速方法。
- 动作完成后核对新的关节状态：关节误差阈值 0.05 rad，夹爪误差阈值 3（0–100 刻度）。这些是脚本的到位判定阈值，不是厂家精度规格。
- SDK 返回失败或到位检查失败时，若仍有控制权则尝试 `stop_robot()`，随后清理 SDK。不会自动调用系统停用服务。
- Ctrl+C 可以中断 Python 流程并在动作已开始时尝试停止；SDK 原生库调用可能延迟响应，SSH 断线或进程被强杀也不能保证执行清理。现场物理急停仍须可用。
- 状态服务异常/动作失败返回码 2，中断返回码 130；正常完成返回码 0。`status` 返回 0 仅表示状态查询完成，不表示已经激活或已经到位。

## 验证边界

交付前只运行无硬件单元测试和 S1 上的 `status`。未运行 configure、activate、home 或 deactivate；现场激活与 Home 尚未验收。创建时曾发现 ROS 服务可发现但状态响应超时，脚本会在此情形下退出，不尝试绕过。

本地测试：

```bash
python3 /Users/js/RBM/TowerGO/workspace/src/astribot_navigation/tests/test_s1_control.py
```

## 2026-09-23 状态查询超时的排查记录

只读检查确认：Orin 到 x86 控制主机网络可达，监督节点参数查询可返回，跨机的模式与诊断消息也有数据；不能把当前问题笼统归为 SSH 或 ROS 网络完全不通。

同时发现：

- `joy_node` 返回 `unconfigured`。
- Orin 没有 `/dev/input/js0`，输入设备清单未发现游戏手柄。
- 当前 joy 日志持续出现 `Couldn't open joystick /dev/input/js0. Error 2: No such file or directory`。
- `/rosout` 中监督节点持续重试配置 joy_node（采样时重试次数已超过 500）。

这是已证实的初始化故障。它是否导致所有其他节点的状态超时，仍需查看 x86 上的监督流程和驱动日志；尚不能认定唯一根因。监督配置位置由公开参数返回为 `/opt/astribot_ros/software/system_supervisor/share/system_supervisor/config/managed_nodes.yaml`，位于 x86 主机，不是 Orin 本地目录。

更新后的 `status` 会额外显示手柄设备是否存在、joy 生命周期状态、监督节点参数服务是否响应，以及最近收到的监督日志。只读查询超时不再使用“操作结果未知”的提示；只有真正发出的配置/激活/停用请求超时才使用该提示。

可先恢复手柄/接收器识别，再观察系统是否完成配置。如果要无手柄运行，应先核查厂商受支持的无手柄配置，不伪造手柄状态、不删除状态检查。此次没有更改厂商配置、重启驱动、激活或移动机器人。
