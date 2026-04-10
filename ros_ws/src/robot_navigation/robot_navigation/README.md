# robot_navigation

当前包只保留两条维护中的控制方案：

- `point_to_point`
- `coverage_patrol`
  - （新增）`coverage_patrol_nav2`：使用 Nav2 的局部规划/避障执行同一套覆盖航点

## point_to_point

`point_to_point` 是一个最小闭环控制节点，用于验证底盘控制是否正常。

功能：

- 订阅 `/odom`
- 发布 `/cmd_vel`
- 先对准目标方向
- 再向目标点前进
- 可选在到点后对齐最终朝向

示例：

```bash
cd <仓库根>/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_navigation point_to_point --ros-args \
  -p goal_x:=1.0 \
  -p goal_y:=0.0
```

## coverage_patrol

`coverage_patrol` 是当前项目用于房间扫描的主方案。
它不依赖 Nav2，而是按照预定义航点覆盖房间边缘、中线和人物区域。

## coverage_patrol_nav2（推荐：更稳的避障）

`coverage_patrol_nav2` 保留“固定覆盖航点”的路线生成方式，但不再直接发 `/cmd_vel`，
而是把每个航点作为 Nav2 的 `NavigateToPose` 目标点交给局部规划器执行。

这能显著改善“小空间 + 人形障碍 + 墙”场景下的振荡与贴人问题：

- Nav2 本地代价地图用原始 `/scan` 做障碍物标记/清除
- 与主栈一致：默认 **`TB3_PERSON_SLAM_MODE=mark_then_strip`** 时 SLAM 用 **`/scan`**；若设为 **`filtered`** 则 SLAM 用 **`/scan_filtered`**

### 启动方式（与 slam_toolbox 配合）

先启动主栈（Gazebo + slam_toolbox + 可选 YOLO 人物链），确认 `/map` 正常发布；
然后启动 Nav2 + 覆盖巡航（launch 内会自动拉起 Nav2 的 planner/controller/bt/behaviors）：

```bash
ros2 launch robot_bringup nav2_coverage_patrol.launch.py
```

### 与 YOLO / 人物链的关系

推荐与主栈里的 **`human_yolo_seg`** 一起用（`TB3_ASSIST_SCAN_FILTER=1`）：

- **`tb3_stack.sh`** 通过 **`yolo_person_seg.launch.py`** 拉起 YOLO 与人物激光链（默认 **mark_then_strip**：**`person_strip_recorder` + 方位角 Marker**；**`scan_map_colored_cloud`** 默认不启以减负，需 **`TB3_ENABLE_SCAN_MAP_COLORED=1`**；**filtered** 含 **`scan_person_filter`**）。
- **默认** **`mark_then_strip`**：**SLAM 用原始 `/scan`** 建图；人物区域记在 YAML / 点云，存图后可清障或叠加可视化。
- **`TB3_PERSON_SLAM_MODE=filtered`**：**SLAM 用 `/scan_filtered`**（旧「滤人建图」）。
- **`coverage_patrol`** 避障**始终用原始 `/scan`**，避免近距离把人当「透明」。

### 设计思路

- 使用固定覆盖路线，避免 frontier 方案在当前场景中反复抖动
- 边缘和角点航点默认向房间内部收缩，减少贴墙卡死
- 前方有障碍时做轻量反应式避障
- 如果同一个目标长时间没有实质进展，会自动插入朝房间内部的 `detour` 脱困点
- 如果脱困点本身也没有进展，则跳过该恢复点继续执行后续航点

### 推荐启动流程

先启动带 YOLO 的主栈（`assist` 默认 Burger；需深度再写 `TURTLEBOT3_MODEL=waffle`）：

```bash
cd <仓库根>
TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

再启动覆盖巡航：

```bash
cd <仓库根>/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation coverage_patrol
```

或使用 launch：

```bash
ros2 launch robot_bringup coverage_patrol.launch.py
```

### 无界面运行与自动保存地图

**无图形界面（减轻负载）**：仿真仍需要 `gzserver`，只是不启动 `gzclient` 和 `RViz2`。在仓库根目录执行：

```bash
TB3_HEADLESS=1 TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

（与 `TB3_NO_GUI=1` 等价。）

**巡航走完后自动保存 SLAM 地图**：依赖 `slam_toolbox` 已发布 `/map`，并需已安装 `nav2_map_server`（`map_saver_cli`）。默认在 **`save_map_delay_sec` 秒**后再保存，给地图一点收敛时间。

在**仓库根目录**打开第二个终端（这样默认保存目录为 `./saved_maps`；否则请用环境变量或参数指定路径）：

```bash
cd /你的路径/TJ-Robot
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

ros2 run robot_navigation coverage_patrol --ros-args \
  -p use_sim_time:=true \
  -p save_map_on_complete:=true
```

或：

```bash
export TJ_ROBOT_SAVED_MAPS_DIR=/绝对路径/TJ-Robot/saved_maps
ros2 run robot_navigation coverage_patrol --ros-args \
  -p use_sim_time:=true \
  -p save_map_on_complete:=true \
  -p save_map_directory:=/绝对路径/TJ-Robot/saved_maps
```

Launch 示例：

```bash
ros2 launch robot_bringup coverage_patrol.launch.py \
  save_map_on_complete:=true \
  save_map_directory:=/绝对路径/TJ-Robot/saved_maps
```

相关参数：

- `save_map_on_complete`：是否在覆盖路径全部走完后保存地图（默认 `false`）。
- `save_map_directory`：保存目录；为空则用环境变量 `TJ_ROBOT_SAVED_MAPS_DIR`，再否则为**当前工作目录**下的 `saved_maps/`（会自动创建）。
- `save_map_basename`：文件名前缀；为空则自动生成带时间戳的名字（`coverage_map_YYYYMMDD_HHMMSS`）。
- `save_map_delay_sec`：路线完成后延迟多少秒再调用 `map_saver_cli`（默认 `2.0`）。
- `exit_after_map_save`：保存后是否退出节点（默认 `true`，便于脚本批处理）。
- `post_save_person_overlay`：存图成功后是否再生成 **彩色 PNG**（`human_yolo_seg`），把 `person_strip_recorder` 记录的 map 系人物圆叠在 SLAM 灰度图上，便于核对 YOLO+激光 与地图是否对齐（默认 `false`；需已 `colcon build human_yolo_seg`）。
- `person_regions_yaml`：人物区域文件路径（默认 `~/.ros/tj_person_strip_regions.yaml`）；与 `person_strip_recorder` 的 `output_path` 一致。
- `snapshot_overlay_from_cloud_if_no_regions`（默认 `true`）：当上述 YAML **不存在**时，是否调用 `snapshot_person_regions_from_cloud`，从 **`/human_yolo/person_laser_map_cloud`**（与 RViz 里 PointCloud2 叠在 `map` 上、**Transient Local** 保留最近一帧）生成与地图同目录的 **`*_person_regions_snapshot.yaml`** 再画叠加图。若仿真全程**从未**起过 `person_strip_recorder`（无人发布该点云），快照会得到 **空 regions**，叠加图里人数为 0——此时仍需按栈说明起 YOLO+`person_strip_recorder`。
- `person_laser_map_cloud_topic`：快照订阅的话题（默认 `/human_yolo/person_laser_map_cloud`）。

**仿真运行时能在 RViz 里看吗？**  
**`*_person_overlay.png` 不能**——它只在**存图之后**由离线命令生成，仿真过程中 RViz 里不会出现这张图。  
**默认精简显示**：RViz 开 **`/human_yolo/person_laser_map_cloud`**（**mark_then_strip** 下 `person_strip_recorder` 的 map 系人向点）+ **`/human_yolo/person_azimuth_markers`**（方位角扇形）。  
要看 **整帧激光在 map 上、人物束换色**（可设 Decay 拖尾），需 **`TB3_ENABLE_SCAN_MAP_COLORED=1`** 或 launch **`enable_scan_map_colored:=true`**，话题 **`/human_yolo/scan_map_colored_cloud`**（**PointCloud2 + RGB8**）。  
**`filtered`** 模式无 strip 累积点云；要看 `map` 上人向可开上述彩色整帧，或改用 **mark_then_strip**。

**说明**：`map_saver_cli` 输出的 **PGM 只有占用/空闲灰度**，不会带颜色；人物是否「从图里擦掉」取决于你是否再跑 `strip_saved_map_person_free`（把人物区刷成空闲）。要看「YOLO 认为人在哪」，请打开同目录下的 **`*_person_overlay.png`**（存图后），或手动：

```bash
ros2 run human_yolo_seg annotate_saved_map_person_overlay \\
  saved_maps/你的图.yaml ~/.ros/tj_person_strip_regions.yaml
```

若圆整体偏一格，可对叠加命令加 `--no-flip-y` 试与 `strip_saved_map_person_free --no-flip-y` 相同约定。  
加 **`--also-composite`** 会另存 **`*_gray_person_composite.png`**（灰度 SLAM 图转彩底 + 人物色实心小点）；`coverage_patrol` 的 `post_save_person_overlay` 已默认带上该选项。

地图文件为 `.pgm` + `.yaml`，位于上述目录；仓库 `.gitignore` 默认忽略 `saved_maps/*.pgm` 与 `*.yaml`，需要提交时请自行 `git add -f`。

### 常用参数

- `edge_margin`
  沿墙航点离墙的内缩距离。

- `corner_margin`
  角点离墙的内缩距离。

- `linear_speed_limit`, `angular_speed_limit`
  巡航速度上限。

- `obstacle_distance`
  触发避障的前向距离阈值。

- `enable_people_avoidance`
  是否订阅 `/human_yolo/person_azimuth_ranges` 并对“前方检测到人”时提前减速/停车/绕开（默认 `true`）。

- `person_front_avoid_distance`, `person_front_stop_distance`
  前方角域检测到人时的避让距离/停车距离阈值（米）。可把“人”当作更保守的动态障碍物。

- `person_virtual_range_m`
  当 YOLO 人物角域与某激光扇区重叠时，将该扇区的有效距离设为 `min(激光读数, person_virtual_range_m)`；激光漏检人时仍当作近距离障碍（默认约 `1.15` m）。**巡航避障始终用原始 `/scan`，不用 `/scan_filtered`。**

- `slowdown_distance`
  进入近距离障碍区前开始线速度缩放的距离（米）。

- `skip_people_waypoints_when_person_detected`
  当当前航点名包含 `person`/`walking actor` 且前方检测到人时，直接跳过该航点（默认 `true`），避免硬闯到人附近。

- `enable_coverage_completion`, `coverage_free_threshold`, `coverage_min_runtime_sec`
  **建图完成判定**：外圈航点完成后开始用 `/scan` 射线投射在 `/map` 网格上标记“已观测 free cell”，当 free 覆盖率 \(\ge\) `coverage_free_threshold` 且已运行至少 `coverage_min_runtime_sec` 秒时认为完成（并可触发存图退出）。

- `perimeter_waypoint_count`
  外圈航点计数阈值（默认 `7`，对应 `left edge` 到 `top center`），达到后开始上述覆盖率判定，并可生成内圈扫地机路线。

- `generate_lawnmower_after_perimeter`, `lawnmower_lane_step`
  外圈完成后自动生成“扫地机”往返扫航点；`lawnmower_lane_step` 为相邻扫线间距（米）。

- `goal_stuck_sec`
  同一目标多久没有实质进展就触发脱困。

- `detour_step`
  脱困点朝房间内部移动的距离。

- `include_people_passes`
  是否保留经过 walking actor 和 standing person 区域的航点。

### 示例

更激进一些的速度：

```bash
ros2 run robot_navigation coverage_patrol --ros-args \
  -p linear_speed_limit:=0.35 \
  -p angular_speed_limit:=1.6
```

更容易脱离边缘障碍：

```bash
ros2 run robot_navigation coverage_patrol --ros-args \
  -p goal_stuck_sec:=2.5 \
  -p detour_step:=1.8
```
