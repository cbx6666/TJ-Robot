# robot_navigation

`robot_navigation` 负责室内静态地图导航、Nav2 参数、自动巡视、覆盖率监控、卡死检测和恢复动作。地图仍使用 `robot_bringup/maps/*.yaml` 与对应 `*.pgm`。

## 目录

```text
robot_navigation/
  config/
    nav2_params.yaml
    nav2_params_astar.yaml
    nav2_params_dijkstra.yaml
  launch/
    nav2_bringup.launch.py
    nav2_patrol.launch.py
    navigation.launch.py
  scripts/
    patrol_waypoints.py        # ROS executable，调用 NavigationManager
    navigation_manager.py      # 巡视状态机、路径黑名单、主动脱困
    coverage_planner.py        # 从静态地图生成覆盖 waypoint
    coverage_monitor.py        # /map + /scan 覆盖率统计
    patrol_markers.py          # RViz marker 调试
    waypoint_utils.py          # waypoint/pose 工具
```

## 启动

首次修改后先构建并 source：

```bash
cd /mnt/d/Homework/robot
source /opt/ros/humble/setup.bash
bash scripts/build.sh --symlink-install --packages-select robot_navigation robot_bringup
source ros_ws/install/setup.bash
```

默认启动 Gazebo、静态地图 Nav2、RViz、覆盖率监控和自动巡视：

```bash
bash scripts/run_nav2.sh
```

切换全局规划器：

```bash
PLANNER_TYPE=astar bash scripts/run_nav2.sh
PLANNER_TYPE=dijkstra bash scripts/run_nav2.sh
```

只启动 Nav2，手动在 RViz 点目标：

```bash
TB3_AUTO_PATROL=0 bash scripts/run_nav2.sh
```

## Nav2 参数

A* 与 Dijkstra 都使用 Nav2 官方 `nav2_navfn_planner/NavfnPlanner`：

- `config/nav2_params_astar.yaml`: `use_astar: true`
- `config/nav2_params_dijkstra.yaml`: `use_astar: false`

本项目没有手写 A* / Dijkstra 算法，算法实现由 Nav2 插件提供。

当前避障与局部规划关键参数：

- `robot_radius: 0.14`：避免窄通道被机器人半径过度占满。
- `inflation_radius: 0.38`：保留安全距离，同时避免墙边和角落全部不可通行。
- `cost_scaling_factor: 5.0`：障碍代价衰减更集中，让机器人能离障碍远一点但仍能通过窄区域。
- `obstacle_layer` 同时用于 local/global costmap，并订阅 `/scan`。
- `marking: true`、`clearing: true`：动态障碍能被标记，也能在消失后清除。
- DWB `max_vel_x: 0.22`、`BaseObstacle.scale: 0.08`、`yaw_goal_tolerance: 0.50`：降低角落速度，减少贴墙和死磕最终朝向。

## 覆盖式巡视

默认启用 `coverage_planner.py`。它会读取传入的 `map.yaml`：

1. 提取静态地图中的 free cells。
2. 按网格采样候选点。
3. 过滤 unknown、障碍、离障碍小于 `0.35m` 的点。
4. 按间距稀疏化，并用最远点采样先挑出覆盖全图的 waypoint。
5. 再用最近邻顺序串成巡视路线，减少无意义来回跑。

常用参数：

```bash
COVERAGE_SAMPLE_SPACING_M=1.10 \
COVERAGE_MAX_WAYPOINTS=40 \
WAYPOINT_MIN_OBSTACLE_DISTANCE_M=0.35 \
bash scripts/run_nav2.sh
```

如果自动生成失败，会回退到内置 fallback waypoint，fallback 中已经补了左侧区域点。

## 巡航状态机、卡死检测和恢复

当前生效逻辑集中在 `navigation_manager.py`，旧的独立卡死检测/恢复模块已经删除。
主流程是：

1. `PLANNING`：对 waypoint 生成原始点、左右/前后偏移点和 yaw 变体。
2. `ROUTE_CHECK`：每个候选点先调用 Nav2 `getPath`，生成 `path_signature`。
3. `ROUTE_REJECTED`：如果路线复用失败签名或命中 failed corridor，拒绝，不发给 Nav2。
4. `ROUTE_SELECTED`：只有通过检查的路线才会 `GOAL_SENT`。
5. `NAVIGATING`：基于 `/odom` 窗口和 `/cmd_vel` 活跃度检测是否真的在移动。
6. `STUCK_DETECTED`：记录 stuck pose 和当前 global path 周围的 failed corridor。
7. `CANCELING_GOAL`：取消 Nav2 goal，避免 Nav2 controller 抢占 `/cmd_vel`。
8. `ESCAPING`：本节点直接发布 `Twist`，执行后退、旋转、左右弧线脱困。
9. `REPLAN_AFTER_ESCAPE`：从新的 odom 位姿重新 `getPath`；仍命中 failed corridor 就拒绝。
10. `SKIP_WAYPOINT`：所有候选路线或脱困都失败时跳过当前 waypoint，避免无限规划。

关键日志：

```bash
NAV_STATE_CHANGE from=NAVIGATING to=STUCK_DETECTED reason=no_progress
FAILED_CORRIDOR_ADDED id=...
NAV_CANCEL_GOAL goal_id=... signature=...
ESCAPE_START mode=backward ...
ESCAPE_PROGRESS moved=...
ESCAPE_SUCCESS moved=...
REPLAN_AFTER_ESCAPE waypoint=...
FAILED_CORRIDOR_HIT corridor_id=...
ROUTE_REJECTED_FAILED_CORRIDOR goal_id=...
ROUTE_SELECTED goal_id=... signature=...
WAYPOINT_SKIPPED_STUCK name=...
```

常用参数：

```bash
PATROL_STUCK_WINDOW_SEC=4.0 \
PATROL_STUCK_MIN_PROGRESS_M=0.08 \
PATROL_FAILED_CORRIDOR_RADIUS_M=0.60 \
PATROL_FAILED_CORRIDOR_START_GRACE_M=0.25 \
PATROL_ESCAPE_MIN_MOVE_M=0.10 \
PATROL_ESCAPE_BACKWARD_DURATION_SEC=1.0 \
PATROL_ESCAPE_ROTATE_DURATION_SEC=0.7 \
PATROL_ESCAPE_ARC_DURATION_SEC=1.0 \
bash scripts/run_nav2.sh
```

## 覆盖率监控

`coverage_monitor.py` 订阅 `/map`、`/scan` 和 TF，把静态地图可通行栅格作为总量，激光射线经过的自由栅格标记为已扫过。

话题：

- `/navigation/coverage_status`：JSON，包含 `coverage_percent`、`covered_cells`、`total_free_cells`、`done`。
- `/navigation/coverage_done`：达到阈值后发布 `true`。
- `/navigation/coverage_grid`：RViz 中作为 OccupancyGrid 叠加查看。

调整完成阈值：

```bash
COVERAGE_REQUIRED_PERCENT=90 bash scripts/run_nav2.sh
```

## RViz 验证

Fixed Frame 使用 `map`，建议添加：

- `Map`: `/map`
- `LaserScan`: `/scan`
- `Path`: Nav2 全局路径
- `OccupancyGrid`: `/navigation/coverage_grid`
- `MarkerArray`: `/navigation/patrol_markers`
- Nav2 costmap 显示：local/global costmap

颜色含义：

- 蓝色 marker：当前目标
- 绿色 marker：已完成 waypoint
- 红色 marker：跳过 waypoint
- 橙色 marker：卡死/恢复触发位置

## 调试日志

主要日志在：

```bash
data/logs/navigation/nav2.launch.log
```

重点搜索：

```bash
grep -E "STUCK|ESCAPE|FAILED_CORRIDOR|ROUTE_|NAV_ACCEPTED|NO_MOTION|WAYPOINT_SKIPPED" data/logs/navigation/nav2.launch.log
```

如果仍然卡住，优先检查：

- `/scan` 是否有数据且 frame 正确。
- `/tf` 是否稳定存在 `map -> odom -> base_footprint`。
- `/cmd_vel` 有输出时 `/odom` 是否真的变化。
- RViz 里 local costmap 是否把前方误判成障碍。
- 当前目标是否落在 `/navigation/coverage_grid` 的未覆盖但安全区域。
- `WAYPOINT_SKIPPED_STUCK` 前后的 `FAILED_CORRIDOR_HIT` 和 `ESCAPE_*` 日志。
