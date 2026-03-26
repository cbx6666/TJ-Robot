# robot_navigation

成员 A 当前提供两个控制入口：

- 手动控制：使用 `turtlebot3_teleop` 直接发布 `/cmd_vel`
- 点到点控制：使用 `point_to_point` 节点让机器人从起点运动到终点

## 点到点控制

这个节点不依赖 SLAM、AMCL 或避障，只做最小闭环运动控制：

- 订阅 `/odom`
- 发布 `/cmd_vel`
- 先转向目标方向
- 再向目标点前进并持续修正航向
- 可选在到点后做最终朝向对齐

示例：

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_navigation point_to_point --ros-args \
  -p goal_x:=1.0 \
  -p goal_y:=0.0
```

如果希望到达终点后再对齐朝向：

```bash
ros2 run robot_navigation point_to_point --ros-args \
  -p goal_x:=1.0 \
  -p goal_y:=0.5 \
  -p use_goal_yaw:=true \
  -p goal_yaw:=1.57
```

常用参数：

- `goal_x`, `goal_y`：终点位置，单位米，基于当前 `/odom` 坐标系
- `goal_yaw`：终点朝向，单位弧度
- `use_goal_yaw`：是否在到点后对齐朝向
- `linear_speed_limit`：最大线速度
- `angular_speed_limit`：最大角速度
- `position_tolerance`：到点判定阈值
- `heading_tolerance`：朝向判定阈值

## 使用边界

- 适合成员 A 负责的标准运动流程和基础点到点运动
- 不处理动态避障
- 不依赖 Nav2
- 不替代成员 C 后续要做的 AMCL 与统一导航整合