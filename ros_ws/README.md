# ROS 2 工作区（ros_ws）

所有功能包在 **`src/`** 下；改代码后需 **`colcon build`** 再 `source install/setup.bash`。

## 包一览

| 包 | 内容 |
|----|------|
| **robot_bringup** | Launch、world、地图、mapper 参数、辅助脚本 |
| **robot_navigation** | `point_to_point`、`coverage_patrol`、Nav2 覆盖等 |
| **human_yolo_seg** | YOLO-Seg、人物方位角、`yolo_person_seg.launch.py`（strip/filter；可选 scan→map 彩色点云，launch 默认关） |
| **robot_interfaces** | 预留消息 |
| **robot_tasks** | 预留 |

## 编译

```bash
cd <仓库根>/ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

仅改部分包时：

```bash
colcon build --packages-select robot_navigation robot_bringup human_yolo_seg
```

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
