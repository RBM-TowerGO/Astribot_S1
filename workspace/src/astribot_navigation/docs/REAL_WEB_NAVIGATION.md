# S1B 真机网页导航

复用现有 v4 地图、矩形轮廓规划器、Pure Pursuit 控制器与键盘遥控的底盘目标发送协议。入口不会提交导航目标、执行 Home、激活底盘或移动上身。初始化 SDK 时会请求已有控制者交接控制权。

## 启动与操作

在 S1B 运行：

```bash
/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation.sh --mode hardware
```

电脑另开终端：

```bash
ssh -N -L 8767:127.0.0.1:8767 S1B
```

打开 http://127.0.0.1:8767 。保持机器人静止完成定位初始化，在地图上指定实际位置与朝向；定位就绪后，设置速度、选择目标位置和朝向，再点击开始。入口复用键盘 SLAM 的传感器启动流程：发现已有原始数据发布者时复用驱动；完全没有发布者时调用厂商 `/astribot_lidar_control` 开启双雷达及前 IMU。导航不需要相机，因此不启动相机。服务确认后仍等待实际数据，数据健康和定位就绪以网页状态为准。服务不可用、失败或超时会直接显示具体错误，不循环重复开启。导航退出保留共享雷达服务开启，不关闭其他应用可能使用的传感器。

默认速度 0.05 m/s，范围 0–0.30 m/s；0 不允许提交运动任务或原地转向。角速度上限 0.1 rad/s。速度参数和执行器任务限速都确认后才提交目标，任务期间速度输入锁定。其他页面只读。

## 底盘与控制权

执行器沿用键盘遥控的 `filter` 方式、`use_wbc=False`、`add_default_torso=False`，只发送底盘目标。从上一有效 SDK 目标开始，按 100 Hz 累加局部 x/y/yaw，不额外旋转平移量、不补发延迟积压位移；导航只前进和转向。

控制权交接使用厂商 `/astribot/control_rights` 的 RawRequest 接口。等待最多 3 秒取得 `success`，再等待最多 3 秒确认原服务释放，之后以 `high_control_rights=False` 创建 SDK。服务名称存在本身不再导致拒绝。交接超时或失败时不会创建 SDK，并直接显示具体原因。此流程会要求原程序让出控制权，可能中断原程序的控制。

保留既有底盘互斥锁和冲突程序检查；同一台机器上的键盘遥控应先正常退出。不会杀掉其他进程。若交接后出现新的控制者，SDK 不自动确认强抢提示。

## 暂停与停车

- 普通零速度与按键松开一致：停止累加，曾经发送过目标时继续发送自己的最后目标；后续有效速度可以继续。保持目标不等于立即刹停，机器人可能仍在追踪该目标。
- 取消、浏览器失联、定位/传感器失效或指令超时撤销任务并调用 SDK `stop_robot()`。这是整机停止接口；一旦调用，当前执行器锁定，不自动恢复，重新使用需重启会话。
- 控制权丢失后不再发送目标或停车指令。停止前尚未发送任何目标的会话不会调用整机停止。
- 页面根据有效里程计确认静止；未确认时显示正在停车，不把发出请求当作已经停住。
- 保留浏览器 0.5 秒失联、导航指令 0.3 秒超时、控制周期 0.1 秒限制及原有反馈和跟踪误差限制。

## 配置和接口

`real_navigation/navigation.yaml` 的 `max_speed: 0.3` 是导航速度上限。启动不再要求 verified、reviewed_by、reviewed_at、验收附件或模块哈希。停车调用直接使用随程序提供的 vendor_stop 模块，不再动态加载外部停车插件。仍检查数值、变换、轮廓与控制参数是否合法。

只检查配置和依赖、不创建 ROS 节点或 SDK：

```bash
./scripts/tg-navigation.sh --mode hardware --check
```

HTTP 只监听 127.0.0.1:8767；保留 token 和原有 `/api/localize`、`/api/goal`、`/api/stop`、`/api/heartbeat` 接口。真实导航使用 Domain 25，独立话题与 TF；地图源文件不修改。网页显示配置最高速度和实时健康状态，不用验收标记控制页面。

## 验证与日志

```bash
python3 -m pytest -q tests/test_real_navigation.py tests/test_navigation_chassis_interface.py tests/test_web_navigation.py tests/test_tg_teleop.py tests/test_mapping.py
```

测试覆盖交接成功、拒绝、响应超时、回复成功但未释放、完整替身初始化、无初始化运动命令、速度边界、暂停后继续、取消、失联和控制权丢失，并回归已有网页与遥控。

日志保存在 `workspace/data/reports/real_navigation/navigation_<时间>_<编号>`；子进程退出会在终端直接显示日志末尾及路径。配置检查和替身测试不表示已经完成真机启动或运动测试。此次实现不执行实际控制权交接或真机运动。
