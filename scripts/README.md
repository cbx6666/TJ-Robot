# Scripts

`scripts` 目录负责环境安装、仿真栈启动、检查和日志查看。

## 主要脚本

- `setup_env.sh`
  安装 ROS 2 Humble、Gazebo、TurtleBot3、SLAM Toolbox 和工作区依赖。

- `bootstrap.sh`
  适合首次拉起环境做基础 smoke test。
  注意：代码拉取后如果 `ros_ws/src` 有变化，仍然需要手动重新执行 `colcon build`。

- `tb3_stack.sh`
  当前项目的主控脚本，负责：
  - 启动 Gazebo
  - 生成 TurtleBot3
  - 生成动态障碍物
  - 启动 `slam_toolbox`
  - 启动 YOLO 和 `scan_person_filter`
  - 在启用 YOLO 时让 SLAM 订阅 `/scan_filtered`
  - 启动 RViz
  - 提供 `start / stop / check / logs / rviz` 子命令

## 推荐流程

```bash
cd /mnt/d/Homework/robot
bash scripts/setup_env.sh

cd ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
cd ..

TB3_STACK_MODE=assist TURTLEBOT3_MODEL=waffle TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

## 常用命令

```bash
bash scripts/tb3_stack.sh start
bash scripts/tb3_stack.sh check
bash scripts/tb3_stack.sh logs all
bash scripts/tb3_stack.sh stop
```

## 常用环境变量

- `TB3_STACK_MODE=laser|assist`
- `TB3_ASSIST_SCAN_FILTER=0|1`
- `TB3_ASSIST_RGBD_BRIDGE=0|1`
- `TURTLEBOT3_MODEL=burger|waffle`
- `YOLO_DEVICE=auto|cpu|cuda:0`
- `TB3_NO_GUI=1`
- `ROBOT_START_X / ROBOT_START_Y / ROBOT_START_Z / ROBOT_START_YAW`

## 当前建议

- 主流程固定推荐：`TB3_STACK_MODE=assist TURTLEBOT3_MODEL=waffle TB3_ASSIST_SCAN_FILTER=1`
- `coverage_patrol` 与 YOLO 一起运行时，建图走 `/scan_filtered`，巡航避障仍走 `/scan`
- `TB3_ASSIST_SCAN_FILTER=0` 只保留为调试 fallback，不再作为项目推荐启动方式
