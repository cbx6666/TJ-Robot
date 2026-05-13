# robot_navigation

`robot_navigation` 负责室内导航相关的 Nav2 参数、launch 入口和巡航脚本。地图仍沿用 `robot_bringup/maps/map.yaml` 和 `map.pgm`，不在导航包内复制地图资源。

## 目录结构

```text
robot_navigation/
├── config/
│   ├── nav2_params.yaml
│   ├── nav2_params_astar.yaml
│   └── nav2_params_dijkstra.yaml
├── launch/
│   ├── navigation.launch.py
│   ├── nav2_bringup.launch.py
│   └── nav2_patrol.launch.py
├── scripts/
│   └── patrol_waypoints.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 统一启动方式

首次运行或修改本包后先编译并 source 工作区。否则 `ros2 launch` 只会搜索 `/opt/ros/humble`，会出现 `Package 'robot_navigation' not found`：

```bash
cd /mnt/d/Homework/robot
source /opt/ros/humble/setup.bash
bash scripts/build.sh --symlink-install --packages-select robot_navigation
source ros_ws/install/setup.bash
```

标准入口统一使用仓库顶层脚本。默认会启动 Gazebo 底盘、静态地图 Nav2、一个 RViz，并自动运行 waypoint 巡视脚本：

```bash
bash scripts/run_nav2.sh
```

脚本内的职责划分：

- `tb3_stack.sh` 只负责 Gazebo、机器人生成和底盘话题。
- `nav2_patrol.launch.py` 负责 Nav2、RViz 和自动巡视节点。
- `nav2_bringup.launch.py` 会先启动 `map_server + AMCL`，再延迟启动 planner/controller，避免导航节点在 `map -> odom` 还没出现时卡住。
- `run_nav2.sh` 会强制关闭底盘栈里的 RViz，避免同时开两个 RViz。

常用模式：

```bash
# 默认：A* + 自动巡视 + RViz
bash scripts/run_nav2.sh

# Dijkstra + 自动巡视 + RViz
PLANNER_TYPE=dijkstra bash scripts/run_nav2.sh

# 只启动 Nav2，不自动巡视，保留 RViz 手动点目标
TB3_AUTO_PATROL=0 bash scripts/run_nav2.sh

# 不开 RViz，只在后台自动巡视
TB3_ENABLE_RVIZ=0 bash scripts/run_nav2.sh

# 使用指定地图
MAP_FILE=/path/to/map.yaml bash scripts/run_nav2.sh
```

停止：

```bash
Ctrl+C
```

如果脚本异常退出后仍有 Gazebo/RViz 残留，再执行 `bash scripts/tb3_stack.sh stop` 清理底盘栈。

## 调试入口

如果已经单独启动了底盘，也可以直接调试 Nav2 launch：

```bash
ros2 launch robot_navigation nav2_patrol.launch.py planner_type:=astar start_patrol:=true
ros2 launch robot_navigation nav2_patrol.launch.py planner_type:=dijkstra start_patrol:=true
ros2 launch robot_navigation nav2_patrol.launch.py planner_type:=astar start_patrol:=false use_rviz:=true
```

## RViz 验证

1. 确认 RViz Fixed Frame 为 `map`。
2. 仿真默认初始位姿为地图坐标 `(0, 0, 0)`；自动巡视模式会再次由 `patrol_waypoints.py` 发布该初始位姿并发送 waypoint。
3. 手动模式（`TB3_AUTO_PATROL=0`）下，可直接用 `Nav2 Goal` 或 `2D Goal Pose` 发送目标点；如果机器人位置明显不准，再用 `2D Pose Estimate` 修正 AMCL 初始位姿。
4. 切换 `PLANNER_TYPE=astar` 与 `PLANNER_TYPE=dijkstra` 后，对比同一目标点的全局路径。

## 避障验证

`local_costmap` 和 `global_costmap` 都启用了 `obstacle_layer` 并订阅 `/scan`，同时启用了 `inflation_layer`。在 Gazebo 中把动态障碍物放到机器人前方时，局部 costmap 应出现障碍代价；机器人会减速、停下、局部绕行，必要时由 BT Navigator 触发重规划或恢复行为。

关键参数位于 `config/nav2_params_astar.yaml` 和 `config/nav2_params_dijkstra.yaml`：

- `obstacle_max_range`：激光点标记障碍物的最大距离。
- `raytrace_max_range`：清除旧障碍的射线最大距离。
- `inflation_radius`：障碍物膨胀半径。
- `robot_radius`：机器人半径近似值。
- `max_vel_x`：DWB 控制器最大前进线速度。

## 当前限制

- `scripts/patrol_waypoints.py` 中的 waypoint 是基于当前地图人工预置的，换地图后需要重新标定。
- 动态避障依赖 `/scan` 能看到障碍物；玻璃、过低或过高的物体可能无法被 2D 激光稳定感知。
- 当前默认 frame 使用 TurtleBot3 常见的 `base_footprint`。如果真实底盘只发布 `base_link`，需要同步修改 AMCL、BT Navigator 和 costmap 中的 `robot_base_frame/base_frame_id`。
