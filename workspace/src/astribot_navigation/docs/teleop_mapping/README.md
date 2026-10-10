# 手柄遥控采集与移动建图操作手册

更新：2026-09-18。机器人：`astribot@10.7.154.109`。所有命令默认在机器人 SSH 终端执行。

本文安装位置：`/home/astribot/TowerGO/workspace/src/astribot_navigation/docs/teleop_mapping/README.md`。
设计依据见同目录 `teleop-mapping-design.md`。本文编写期间没有运行遥控、启动雷达或驱动底盘；下面是供现场操作的命令。

## 0. 当前能执行到哪一步

| 操作 | 当前状态 |
|---|---|
| 识别手柄、检查 ROS 话题 | 可执行，只读 |
| 独立启动双雷达、录制点云与 IMU、关闭本次启动的驱动 | 已有接口，本文提供命令 |
| 手柄驾驶底盘 | 需要厂商已验证的遥控入口；SDK 示例存在断连检查缺口，本文不将其作为启动命令 |
| 真实移动录包 → 完整 SLAM 地图 | 尚需验证真实里程计或适配 LIO，并完成真实传感器处理；不能直接使用模拟回放入口 |
| 地图保存 | 真实建图节点产生有效 /map 后可执行，见第 8 节 |

推荐顺序：手柄与停车确认 → 位姿来源确认 → 小范围移动录包 → 离线建图 → 地图保存。首次先录包后建图，手动建图不启动 Nav2 控制器。

## 1. 登录与准备终端

从电脑登录。每个操作终端分别登录一次；不要在命令中写入密码。

```bash
ssh astribot@10.7.154.109
```

每个主机终端都执行以下环境设置。使用厂商 ROS 环境，不以 `/opt/ros/humble/setup.bash` 替代。

```bash
source /opt/astribot_ros/middle_ware/setup.bash
source /opt/astribot_ros/software/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export TOWERGO_WS=/home/astribot/TowerGO/workspace
export TOWERGO_REPO="$TOWERGO_WS/src/astribot_navigation"
cd "$TOWERGO_REPO"
```

终端分工：A 检查与雷达开关；B 录包；C 只读监测。厂商遥控使用经过验证的独立操作入口。关闭录包终端不会自动停车。

## 2. 手柄检查：此阶段不启动新的运动程序

把手柄接收器接入机器人并打开手柄，检查输入设备与节点：

```bash
cat /proc/bus/input/devices
ls -l /dev/input/by-id/ /dev/input/js* 2>/dev/null
ros2 node list
ros2 topic info /astribot_joy --verbose
ros2 topic info /joy/heartbeat --verbose
```

应看到真实手柄设备和 `/astribot_joy` 发布者。最后一次检查未发现手柄设备和该话题发布者；节点进程存在不代表手柄已连接。若 ROS 图结果可疑，等待发现完成后重试，并以实际收到消息为准。

只读查看消息，命令会等待一条消息；10 秒没有数据则超时退出：

```bash
timeout 10s ros2 topic echo /astribot_joy --once --qos-reliability best_effort
timeout 10s ros2 topic echo /joy/heartbeat --once --qos-reliability best_effort
```

当前话题有 `xbox_controller` 等订阅者。实际拨动摇杆前，现场操作员必须确认现有运动控制处于禁用状态；只读监测本身不会禁用其他控制程序。禁止往 `/astribot_joy` 发布模拟按键测试。

检查厂商映射配置：

```bash
cat /opt/astribot_ros/software/astribot_joy/share/astribot_joy/config/beitongjoy.yaml
ros2 param list /joy_node
```

记录手柄型号、前进轴、转向轴、使能按键、停止方式、连接状态检测方式。不要依据通用 Xbox 编号直接认定实体按键。

**遥控启动门槛：** 使用现场已经验证的厂商遥控流程；本次源码检查没有确认一个可推荐直接执行的安全启动命令。若现有遥控尚不可用，应先完成适配与停车测试，不继续移动采集。

SDK 的 `examples/202-chassis_joy_control_local.py`、`203-chassis_joy_control_global.py` 是接口参考：它们以 250 Hz 积分速度为位置目标；`tools/joy_tools.py` 首次收到消息后不会重新检查消息新鲜度。直接运行示例不能保证断连停车。也不要额外创建 Astribot 实例与现有遥控争用控制权。

## 3. 检查真实位姿来源

```bash
ros2 topic info /astribot_chassis/joint_space_states --verbose
ros2 topic info /astribot_chassis/endpoint_current_states --verbose
ros2 topic info /odom --verbose
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
timeout 10s ros2 topic echo /astribot_chassis/joint_space_states --once --qos-reliability best_effort
timeout 10s ros2 topic echo /odom --once --qos-reliability best_effort
timeout 10s ros2 run tf2_ros tf2_echo odom base_link
```

没有话题或没有消息时出现错误/超时是检查结果，不要用模拟器补上。需要核验：实测位置与期望位置的区别、米/弧度单位、轴方向、参考原点、时间戳、频率和重启复位行为。

后续现场短距离前进与转向试验必须由操作者用已验证的遥控进行。若反馈无法验证为里程计，选择前雷达＋前 IMU 的 LIO 路线；原始数据仍可录制用于适配，但不能承诺该录包已经满足建图条件。

## 4. 独立启动雷达

终端 A：先看雷达是否已经工作。

```bash
ros2 topic info /livox/lidar_front --verbose
ros2 topic info /livox/lidar_back --verbose
ros2 service type /astribot_lidar_control
```

服务类型应为 `std_srvs/srv/SetBool`。如果已有雷达数据，复用现有驱动，不重复启动，并记下“不是本次启动”。如果没有运行且服务可用，执行：

```bash
ros2 service call /astribot_lidar_control std_srvs/srv/SetBool '{data: true}'
```

应返回 `success: true`。再检查四路数据：

```bash
timeout 8s ros2 topic hz /livox/lidar_front
timeout 8s ros2 topic hz /livox/lidar_back
timeout 8s ros2 topic hz /livox/imu_front
timeout 8s ros2 topic hz /livox/imu_back
```

此前静止采集约为点云各 10 Hz、IMU 各 200 Hz。`timeout` 结束常返回 124，这是人为限时退出；是否正常应看期间的数据输出。频率测量受 QoS、负载影响，没有输出时检查发布端 QoS 和实际录包计数。

不要运行初始化 SDK 的雷达示例。此服务只用于雷达驱动，不需要取得底盘控制权。

## 5. 创建本次数据目录与录包配置

终端 B 在完成第 1 节环境设置后执行一次：

```bash
export TOWERGO_RUN="$TOWERGO_WS/data/bags/teleop_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TOWERGO_RUN"
printf '%s\n' "$TOWERGO_RUN"
cp /opt/astribot_ros/software/livox_ros_driver2/share/config/MID360_config.json "$TOWERGO_RUN/MID360_config.json"
cp /opt/astribot_ros/software/livox_ros_driver2/share/livox_ros_driver2/msg_MID360_launch_rviz_msg_type.py "$TOWERGO_RUN/driver_launch.py"
git -C /home/astribot/astribot_sdk_aarch64 rev-parse HEAD > "$TOWERGO_RUN/sdk_commit.txt"
ros2 topic list -t > "$TOWERGO_RUN/topics_before.txt"
df -h "$TOWERGO_WS/data"
```

记下打印的绝对路径，后续新终端需要重新设置 `TOWERGO_RUN`。此前双雷达约 46.5 秒录制 492 MB，三分钟粗估约 2 GB，建议至少预留 5 GB；实际大小可能变化。

创建传感器 QoS 配置；静态 TF 用 transient_local 接收已发布的静态关系：

```bash
cat > "$TOWERGO_RUN/record_qos.yaml" <<'YAML'
/livox/lidar_front:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/livox/lidar_back:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/livox/imu_front:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 1000
/livox/imu_back:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 1000
/astribot_chassis/joint_space_states:
  reliability: best_effort
  durability: volatile
/astribot_chassis/endpoint_current_states:
  reliability: best_effort
  durability: volatile
/astribot_joy:
  reliability: best_effort
  durability: volatile
/joy/heartbeat:
  reliability: best_effort
  durability: volatile
/odom:
  reliability: best_effort
  durability: volatile
/tf:
  reliability: best_effort
  durability: volatile
/tf_static:
  reliability: reliable
  durability: transient_local
YAML
```

## 6. 开始录制、人工驾驶与停止

终端 B：下面只录制，不发送运动指令。使用明确话题列表，不录所有话题。

```bash
ros2 bag record -s mcap \
  -o "$TOWERGO_RUN/bag" \
  --qos-profile-overrides-path "$TOWERGO_RUN/record_qos.yaml" \
  /livox/lidar_front /livox/lidar_back \
  /livox/imu_front /livox/imu_back \
  /astribot_chassis/joint_space_states \
  /astribot_chassis/endpoint_current_states \
  /astribot_joy /joy/heartbeat /odom /tf /tf_static
```

话题不存在时录包程序可能继续等待，不能把“进程在运行”当成所有数据齐全。先确认四路雷达/IMU 已订阅并有数据，然后：

1. 静止 5–10 秒，供初始状态检查。
2. 操作者使用已验证的遥控慢速前进、转向，建议初期 0.1–0.2 m/s、0.2–0.3 rad/s，禁用横移及加速模式。这些是建议值，以上录包命令不会设置底盘限速。
3. 先走 2–3 分钟的小范围闭环并回到起点，避免快速转向，保持雷达安装关系固定。
4. 通过遥控使机器人停止，确认物理静止，再保留 5–10 秒数据。
5. 在终端 B 按 Ctrl+C，等待录包进程正常退出并写出 metadata.yaml。不要用 kill -9 收尾。

遥控失效或异常移动时使用现场已确认的停车/急停方式；Ctrl+C 录包不控制底盘。切勿把录包程序的运行状态当作运动使能条件。

终端 C 可监测剩余空间及点云频率：

```bash
df -h "$TOWERGO_WS/data"
timeout 8s ros2 topic hz /livox/lidar_front
timeout 8s ros2 topic hz /livox/lidar_back
```

## 7. 检查数据与关闭雷达

终端 B（保留本次 TOWERGO_RUN 变量）：

```bash
ros2 bag info "$TOWERGO_RUN/bag" | tee "$TOWERGO_RUN/bag_info.txt"
test -s "$TOWERGO_RUN/bag/metadata.yaml"
du -sh "$TOWERGO_RUN/bag"
nano "$TOWERGO_RUN/operator_notes.md"
```

在记录中写：操作者、手柄型号和遥控入口、停车验证情况、建议限速是否实际配置、路线、是否闭环、雷达是否本次启动、异常、位姿来源是否已确认。

验收原始数据：四路传感器计数非零、持续覆盖实际驾驶时段、时间戳无明显回退；若走底盘里程计路线，实测状态及有效 TF 也必须覆盖全程。`bag info` 的计数和时长是初检，不证明每条消息的时间和坐标正确。

仅当雷达由本次启动、其他任务不再使用时，在终端 A 关闭：

```bash
ros2 service call /astribot_lidar_control std_srvs/srv/SetBool '{data: false}'
ros2 topic info /livox/lidar_front --verbose
ros2 topic info /livox/lidar_back --verbose
```

应返回 success，随后检查驱动发布是否停止。服务关闭驱动不等于已经证明雷达物理断电。

## 8. 离线建图：适配完成后才能执行

### 8.1 先明确现有入口的限制

当前 `prepare_bag` 仅支持模拟约定：必须有前点云、/odom、/tf、/tf_static，并仅保留 odom→base_link、base_link→lidar_front/back 等指定边。真实点云当前 frame_id 为 livox_frame，不能直接套用。

当前 `scan_processor` 内还固定了模拟轮廓 0.8×0.7 m 和高度筛选，未实现每点运动去畸变。`replay_mapping.launch.py` 会启动这个处理器，不能把它当成已经验收的实机回放链路。

不要手写 towergo_inputs.json 绕过检查，不要用模拟 TF 或期望位置补数据。`build_static_map.py` 和 `collect_static_lidar.py` 是静止采集/处理方案，也不是移动建图入口。

### 8.2 需要交付的适配结果

选择并验证以下一种位姿来源：实测底盘反馈转换为 odom，或前 MID360＋前 IMU 的 LIO。后者要核验每点 timestamp、加速度单位、时间同步和 LiDAR–IMU 外参；候选 FAST_LIO_ROS2 尚未在本机部署。

建议适配器输出单独的标准 bag，至少包含：

- `/scan`：真实标定与运动处理后的 LaserScan。
- `/tf`：连续的 odom→base_link，覆盖扫描时间。
- `/tf_static`：真实 base_link 到扫描坐标系关系。
- `/odom`：与上述 TF 一致的标准里程计。

标准 bag 不含 map→odom、不含控制指令或 Joy，也不含伪造的模拟真值。**转换程序当前尚未实现，所以本文没有虚构一个转换命令。只有生成并验证上述标准 bag 后，下面的 SLAM 启动与保存命令才适用。**

### 8.3 在单个隔离容器内启动 SLAM

另开主机终端，无需 source 厂商环境。检查镜像，启动专用容器；名称已占用时先检查现有容器，不覆盖别人的任务：

```bash
sudo docker image inspect towergo-offline:humble --format '{{.Id}}'
sudo docker run --rm -it --name towergo-teleop-mapping \
  --network none --ipc private --cap-drop ALL \
  --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_DOMAIN_ID=125 -e ROS_LOCALHOST_ONLY=1 \
  -v /home/astribot/TowerGO/workspace/data:/data \
  towergo-offline:humble bash
```

以下在容器内执行。标准输入约定放在 `/data/bags/teleop_normalized/bag`，这是**适配结果的约定路径，不是当前已存在的数据**：

```bash
test -s /data/bags/teleop_normalized/bag/metadata.yaml
ros2 bag info /data/bags/teleop_normalized/bag
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
  --params-file /towergo/install/astribot_nav_bringup/share/astribot_nav_bringup/config/slam.yaml
```

必须先确认 test 和 bag info 成功。此处只启动 SLAM，不启动现有模拟点云转换器。该配置订阅 /scan，使用模拟时钟，发布 map→odom。

另一个主机终端进入**同一个**容器并回放，不另起隔离容器，否则节点无法互相发现：

```bash
sudo docker exec -it towergo-teleop-mapping bash
source /opt/ros/humble/setup.bash
source /slam_overlay/install/setup.bash
source /towergo/install/setup.bash
export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
ros2 bag play /data/bags/teleop_normalized/bag --clock --rate 0.5 \
  --topics /scan /odom /tf /tf_static
```

只在这个隔离容器回放；禁止在实机 Domain 25 回放含手柄消息的原始 bag。不要启动模拟底盘、AMCL 或 Nav2 控制器。回放结束后暂时保留 SLAM 节点。

### 8.4 检查并保存地图

再开一个主机终端进入同一容器，source 与上一段相同的三个环境，然后执行：

```bash
ros2 topic info /map --verbose
ros2 service list -t
mkdir -p /data/maps/teleop_mapping
ros2 run nav2_map_server map_saver_cli -f /data/maps/teleop_mapping/map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 \
  -p free_thresh_default:=0.196 -p occupied_thresh_default:=0.65
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  '{filename: /data/maps/teleop_mapping/posegraph}'
ls -lh /data/maps/teleop_mapping
```

已有同名地图时先使用新目录，避免覆盖。确认地图保存和序列化服务成功，检查图像、YAML 和位姿图文件均非空；地图应检查墙面重影、闭环错位、未知区和实际尺寸。地图文件保存成功不等于真实精度合格。不要把仿真验收指标当作这次实机验收结果。

完成后先结束回放，再 Ctrl+C 结束 SLAM，退出各 docker exec 会话，最后退出创建容器的 bash。`--rm` 会删除该容器；挂载到 /data 的文件仍保留在主机工作区。

## 9. 常见问题

| 现象 | 处理 |
|---|---|
| 手柄节点存在但没有 Joy | 检查设备、接收器、生命周期状态和发布者；不要额外启动第二套驱动抢设备 |
| 有 Joy 但不动 | 检查厂商使能和控制权流程；不要强制获取控制权或直接运行示例试错 |
| 无 odom 或 TF | 完成真实位姿适配；禁止启动离线模拟器填补 |
| 雷达有发布者但没数据 | 检查实际消息、驱动配置、连接和 QoS；等待正常后再驾驶 |
| prepare_bag 拒绝真实 bag | 当前只支持模拟约定，是能力边界，不应伪造标记绕过 |
| 地图重影 | 检查运动去畸变、时间戳、外参、里程计，不先靠改地图参数掩盖问题 |
| Docker 权限不足 | 使用已授权的 sudo/管理员容器权限，不默认修改用户组 |
| Ctrl+C 后机器人仍动 | 录包不控制底盘；用经过现场验证的遥控停车/急停方式 |

## 10. 文档与接口依据

- 现场 SDK：`/home/astribot/astribot_sdk_aarch64/examples/202-chassis_joy_control_local.py`、`203-chassis_joy_control_global.py`、`tools/joy_tools.py`。
- [SLAM Toolbox Humble 接口与保存服务](https://github.com/SteveMacenski/slam_toolbox/tree/humble#api)。
- [teleop_twist_joy Humble](https://github.com/ros2/teleop_twist_joy/blob/humble/README.md)：Joy 到 Twist 转换不能替代 Astribot 底盘适配。
- [FAST_LIO_ROS2 的 ros2 分支](https://github.com/Ericsii/FAST_LIO_ROS2/tree/ros2)：可评估的 LIO 路线，尚未验证本机适配。

本手册给出真实可用的采集命令和满足标准输入后可用的 SLAM 命令。当前尚缺的遥控安全适配、真实里程计/LIO 和移动点云适配，必须先完成，才能声称“手柄移动建图全流程已可运行”。
