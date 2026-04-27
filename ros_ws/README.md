# ROS 2 工作区（ros_ws）

所有功能包在 **`src/`** 下；改代码后需 **`colcon build`** 再 `source install/setup.bash`。
`src` 内部分层约定见：**[`src/README.md`](src/README.md)**。

**全仓库「功能 → 路径」总索引**（含脚本与任务单）：**[../docs/代码与功能索引.md](../docs/代码与功能索引.md)**

## 包一览

| 包 | 内容 | 包内说明 |
|----|------|----------|
| **robot_bringup** | Launch、world、地图、mapper 参数、辅助脚本 | [robot_bringup/README.md](src/robot_bringup/README.md) |
| **robot_navigation** | `point_to_point`、`coverage_patrol`、Nav2 覆盖等 | [robot_navigation/README.md](src/robot_navigation/robot_navigation/README.md) |
| **human_yolo_seg** | YOLO-Seg、人物方位角、strip/filter、清障 CLI | [human_yolo_seg/README.md](src/human_yolo_seg/README.md) |
| **robot_interfaces** | 跨包消息契约（`TaskStatus`、`PersonRegion*` 等） | — |
| **robot_tasks** | 任务管理节点（当前含心跳状态发布） | — |
| **robot_interaction** | 语音输入网关与 LLM 路由接口 | [robot_interaction/README.md](src/robot_interaction/README.md) |
| **robot_manipulation** | mock 抓取/放置语义执行接口 | [robot_manipulation/README.md](src/robot_manipulation/README.md) |

## 编译

```bash
cd <仓库根>/ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

仅改部分包时：

```bash
colcon build --packages-select robot_navigation robot_bringup human_yolo_seg robot_tasks robot_interaction robot_manipulation
```

CI 会在 Linux 环境启用 `robot_interfaces` 的 rosidl 生成（`ROBOT_INTERFACES_ENABLE_ROSIDL=ON`）。

## 与 `tb3_stack.sh` 配合

先在本机起栈（仓库根目录）：

```bash
TB3_STACK_MODE=assist TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

覆盖巡航（需在 `ros_ws` 已 source）：

```bash
ros2 run robot_navigation coverage_patrol
# 或
ros2 launch robot_bringup coverage_patrol.launch.py
```

点到点调试：

```bash
ros2 run robot_navigation point_to_point --ros-args -p goal_x:=1.0 -p goal_y:=0.0
```

详细参数与存图：**[src/robot_navigation/robot_navigation/README.md](src/robot_navigation/robot_navigation/README.md)**
