# ROS 2 Workspace

工作区位于 `ros_ws`，所有 ROS 2 包都在 `ros_ws/src` 下维护。

## 包说明

- `robot_bringup`
  保存 launch、地图、world、模型和 bringup 辅助脚本。

- `robot_navigation`
  保存当前维护中的运动控制节点：
  - `point_to_point`
  - `coverage_patrol`

- `human_yolo_seg`
  保存 YOLO 人体检测、方位角计算和 `/scan_filtered` 过滤链路。

- `robot_interfaces`
  预留接口包，当前只有占位消息。

- `robot_tasks`
  预留任务层，当前没有实质逻辑。

## 编译

```bash
cd /mnt/d/Homework/robot/ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只改了导航相关代码，也可以只编译相关包：

```bash
colcon build --packages-select robot_navigation robot_bringup human_yolo_seg
```

## 当前维护入口

启动带 YOLO 的主栈：

```bash
cd /mnt/d/Homework/robot
TB3_STACK_MODE=assist TURTLEBOT3_MODEL=waffle TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

启动覆盖巡航：

```bash
cd /mnt/d/Homework/robot/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation coverage_patrol
```

或使用 launch：

```bash
ros2 launch robot_bringup coverage_patrol.launch.py
```

启动点到点控制：

```bash
ros2 run robot_navigation point_to_point --ros-args -p goal_x:=1.0 -p goal_y:=0.0
```

## 说明

- 当前推荐流程必须带 YOLO 与 `/scan_filtered`
- `coverage_patrol` 默认仍然订阅 `/scan`，避免把人从近距离避障里过滤掉
- 修改 `src` 后必须重新 `colcon build`
- `install/setup.bash` 只加载已经编译好的内容，不会自动重新编译源码
