# robot_bringup

仿真 **launch**、**Gazebo 世界**、**地图与 RViz**、**slam_toolbox / Nav2 参数**、**URDF 片段与 SDF 准备脚本**。与仓库根目录 **`scripts/tb3_stack.sh`** 配合使用（栈内多数进程由脚本直接 `ros2 run`/`launch`，部分配置引用本包 `share` 路径）。

## 目录说明

| 路径 | 内容 |
|------|------|
| `launch/` | 各功能 launch：`slam_laser`、`coverage_patrol`、`nav2_coverage_patrol`、`navigation_modes`、`task_pipeline`、`rgbd_to_scan`、`bringup`、`slam_rgbd*` 等 |
| `config/` | `mapper_params_online_async_*.yaml`（slam_toolbox）、`nav2/base.yaml` + `nav2/profiles/*.yaml`、`test1.rviz`、`cartographer/` |
| `world/` | `test1.world` 等 Gazebo 世界 |
| `maps/` | 包内示例栅格地图（`test1.yaml` / `test1.pgm`） |
| `models/` | 仿真用模型（如 `moving_obstacle.sdf`） |
| `urdf/` | Burger RGB 片段、Waffle 深度 Gazebo 片段等 |
| `scripts/` | `merge_tb3_burger_rgb_urdf.py`、`prepare_burger_rgb_camera_sdf.py`、`prepare_assist_waffle_sdf.py`、`moving_obstacle_controller.py`、`tb3_teleop_keyboard_slam.py`、`depth_image_to_viz.py` 等 |

## Launch 速查

| 文件 | 典型用途 |
|------|-----------|
| `slam_laser.launch.py` | 激光 slam_toolbox（`tb3_stack` 内引用） |
| `coverage_patrol.launch.py` | 起 `coverage_patrol` 节点 |
| `nav2_coverage_patrol.launch.py` | Nav2 + `coverage_patrol_nav2` |
| `navigation_modes.launch.py` | 多导航策略入口（coverage/nav2/point_to_point/wall_follow） |
| `task_pipeline.launch.py` | 语音输入 + LLM 路由 + 任务管理 + mock 抓放 |
| `manipulation.launch.py` | mock 机械臂能力入口 |
| `rgbd_to_scan.launch.py` | 深度 → 伪激光（assist + `TB3_ASSIST_RGBD_BRIDGE=1`） |
| `bringup.launch.py` | 通用 bringup |
| `slam_rgbd.launch.py` / `slam_rgbd_cartographer.launch.py` | RGB-D / Cartographer 扩展实验 |

## Config 速查

| 文件 | 说明 |
|------|------|
| `mapper_params_online_async_full_scan.yaml` | 订阅 **`/scan`**（mark_then_strip 与对照组常用） |
| `mapper_params_online_async_scan_filtered.yaml` | 订阅 **`/scan_filtered`**（filtered 模式） |
| `mapper_params_online_async_rgbd.yaml` | RGB-D 相关 slam 参数 |
| `nav2/base.yaml` | Nav2 共享基础参数（所有场景共用） |
| `nav2/profiles/coverage_patrol.yaml` | 覆盖巡航调参覆盖层 |
| `nav2/profiles/conservative.yaml` | 保守避障调参覆盖层 |
| `test1.rviz` | 默认 RViz 显示（含人物 Marker 等 topic 预设） |

## 与 `tb3_stack.sh` 的关系

- 栈脚本通过 `ros2 pkg prefix robot_bringup` 定位 **`share/robot_bringup`**，读取 **scripts、urdf、config**；或在开发时回退到 **`ros_ws/src/robot_bringup`**。
- 修改本包后执行：`colcon build --packages-select robot_bringup && source install/setup.bash`。
- Nav2 启动默认按 **`base + profile`** 两层加载参数；可通过 `nav2_base_params`、`nav2_profile_params` 覆盖。

全仓库功能总索引：**仓库根目录 `docs/代码与功能索引.md`**。
