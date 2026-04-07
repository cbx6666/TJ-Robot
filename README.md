# Indoor Service Robot Simulation

这是一个基于 ROS 2 Humble、Gazebo 和 TurtleBot3 的室内机器人仿真项目。
当前仓库的主运行路径已经固定为：`assist + waffle + YOLO + coverage_patrol`。

## 当前主方案

- `tb3_stack.sh` 负责拉起 Gazebo、SLAM、RViz、YOLO 和激光过滤链路
- `slam_toolbox` 在启用 YOLO 时订阅 `/scan_filtered` 建图
- `coverage_patrol` 负责按固定覆盖路线扫描房间
- `coverage_patrol` 继续订阅原始 `/scan` 做近距离避障，不使用 `/scan_filtered`
- `point_to_point` 只保留为单目标点运动调试工具

## 当前功能

- TurtleBot3 室内仿真
- 激光建图 `slam_toolbox`
- 动态障碍物与人形模型
- YOLO 人体检测与 `/scan_filtered`
- 自动覆盖巡航扫描 `coverage_patrol`
- 简单点到点控制 `point_to_point`

## 仓库结构

```text
.
|- scripts/                     环境安装和一键启动脚本
|- ros_ws/                      ROS 2 工作区
|  |- src/robot_bringup         launch、地图、world、辅助脚本
|  |- src/robot_navigation      运动控制与覆盖巡航节点
|  |- src/human_yolo_seg        YOLO 人体检测和激光过滤
|  |- src/robot_interfaces      预留接口定义
|  `- src/robot_tasks           预留任务层
`- README.md
```

## 推荐启动方式

先准备环境：

```bash
cd /mnt/d/Homework/robot
bash scripts/setup_env.sh
```

编译工作区：

```bash
cd /mnt/d/Homework/robot/ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

启动带 YOLO 的主栈：

```bash
cd /mnt/d/Homework/robot
TB3_STACK_MODE=assist TURTLEBOT3_MODEL=waffle TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

新开终端启动自动覆盖巡航：

```bash
cd /mnt/d/Homework/robot/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation coverage_patrol
```

## 维护说明

- 当前主方案是 `coverage_patrol + YOLO + /scan_filtered`
- `coverage_patrol` 保留原始 `/scan` 只用于近距离避障
- `point_to_point` 仅用于手动调试运动控制
- `robot_tasks` 和 `robot_interfaces` 仍然是占位包
- 每次修改 `ros_ws/src` 下的代码后，都需要重新执行 `colcon build`

更多细节见：

- [scripts/README.md](scripts/README.md)
- [ros_ws/README.md](ros_ws/README.md)
- [ros_ws/src/robot_navigation/robot_navigation/README.md](ros_ws/src/robot_navigation/robot_navigation/README.md)
