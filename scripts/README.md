# scripts

环境安装与仿真**主控栈**（`tb3_stack.sh`）。

全仓库「功能 → 源码路径」总索引：**[../docs/代码与功能索引.md](../docs/代码与功能索引.md)**。

## 脚本

| 文件 | 作用 |
|------|------|
| `setup_env.sh` | 安装 ROS 2 Humble、Gazebo、TurtleBot3、SLAM Toolbox 与工作区依赖（按脚本内说明）。 |
| `bootstrap.sh` | 首次环境冒烟；**源码变更后仍需自行 `colcon build`**。 |
| `tb3_stack.sh` | **主入口**：Gazebo、生成机器人与障碍、`slam_toolbox`、RViz、可选 **YOLO + 人物激光链**（strip/filter + 方位角 Marker；整帧 map 上色默认关，见 `TB3_ENABLE_SCAN_MAP_COLORED`）。子命令：`start` / `stop` / `check` / `logs` / `rviz`。 |

## 推荐启动（assist + YOLO）

`assist` 且**不写** `TURTLEBOT3_MODEL` → 默认 **Burger**。需要 Waffle 深度时再设 `TURTLEBOT3_MODEL=waffle`。

```bash
cd <仓库根目录>
bash scripts/setup_env.sh          # 按需

cd ros_ws && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && cd ..

TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

## 常用命令

```bash
bash scripts/tb3_stack.sh start
bash scripts/tb3_stack.sh check
bash scripts/tb3_stack.sh logs all
bash scripts/tb3_stack.sh stop
```

`logs` 可查 `yolo`、`yolo_map_pipeline`、`slam` 等，详见脚本内 `usage`。

## 环境变量（摘录）

- **`TB3_STACK_MODE`**：`laser` | `assist`
- **`TB3_ASSIST_SCAN_FILTER`**：`0` | `1`（`1` 且已安装包时起 YOLO 人物链）
- **`TB3_PERSON_SLAM_MODE`**：`mark_then_strip`（默认，SLAM 用 `/scan`）| `filtered`（SLAM 用 `/scan_filtered`）
- **`TB3_ENABLE_SCAN_MAP_COLORED`**：`0`（默认）| `1` 额外起 `scan_map_colored_cloud`（整帧激光投 map）
- **`TB3_ASSIST_RGBD_BRIDGE`**：`0` | `1`（assist + 深度时）
- **`TURTLEBOT3_MODEL`**：`burger` | `waffle` | `waffle_pi`
- **`YOLO_DEVICE`**：`auto` | `cpu` | `cuda:0`
- **`TB3_NO_GUI` / `TB3_HEADLESS`**：不启 gzclient / RViz2
- **`TB3_SCAN_FOV_LIMIT`**、**`TB3_SCAN_FOV_MIN_DEG`**、**`TB3_SCAN_FOV_MAX_DEG`**：filtered 模式下限制激光扇区（见脚本注释）
- **`ROBOT_START_X/Y/Z/YAW`**：生成位姿

WSL 下 `ros2` CLI 较慢；`tb3_stack.sh` 的话题检查已尽量合并为单次 `ros2 topic list`。

## 与巡航的关系

- 默认 **mark_then_strip**：建图走 **`/scan`**；**`coverage_patrol`** 仍订阅 **`/scan`** 做近距离避障。
- **`filtered`**：建图走 **`/scan_filtered`**；巡航仍建议用 **`/scan`** 避障。

存图目录、叠加图等：**`robot_navigation` 包内 README**。
