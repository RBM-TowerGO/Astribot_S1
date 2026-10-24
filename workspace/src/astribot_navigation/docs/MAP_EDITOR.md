# 网页地图清理
启动（在本机终端运行，保持终端打开）：
```bash
ssh -L 8768:127.0.0.1:8768 S1B 'python3 /home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/real_navigation/map_editor.py --map /home/astribot/TowerGO/S1_locomotion/workspace/data/maps/web_sim_manual_20261021_v4/map.yaml'
```
打开 http://127.0.0.1:8768 。框选后点击抹除，可撤销最近 30 次擦除。
膨胀半径支持 0–2 m，橙色显示按栅格分辨率向上取整的膨胀范围，非运行中完整代价地图；实际碰撞还取决于轮廓、未知区域与预测运动。
保存生成相邻 manual_edit_* 目录，包含 map.pgm、map.yaml、navigation.yaml、来源哈希与编辑统计。
页面给出启动新地图的命令；先结束现有导航，再在 S1B 执行。原地图和当前运行导航不变。
编辑草稿在进程内，关闭服务会丢弃未保存修改。刷新页面不会丢弃草稿。
保存后的配置 inflation_radius 同时用于 global_costmap 和 local_costmap，默认仍为 0.2 m。
