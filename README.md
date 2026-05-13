# TJ-Robot 室内服务机器人系统

本仓库已收敛为 **ROS 2 单源结构**：运行与开发以 `ros_ws/src` 为唯一真源。

## 当前主线

- ROS 2 Humble + Gazebo + TurtleBot3 仿真。
- 激光 SLAM 地图发布（`/map`）。
- RGBD + YOLO 视觉识别（保留检测链路，移除历史角度/建图后处理链路）。
- 语音输入接口 + LLM 路由接口（`robot_interaction`）。
- mock 机械臂抓取/放置验证链路（`robot_manipulation`）。

## 仓库结构（单源）

```text
ros_ws/                ROS 2 工作空间（唯一运行源码）
  src/
    robot_bringup
    human_yolo_seg
    robot_tasks
    robot_interaction
    robot_manipulation
    robot_interfaces
scripts/               构建/运行脚本
data/                  logs/results 等运行产物
docs/                  架构与实验文档
```

## 快速运行

```bash
bash scripts/build.sh
bash scripts/run_simulation.sh         # 默认 RGBD 机器人（waffle + assist）
bash scripts/run_mapping.sh            # 仅激光 + SLAM + RViz（建图常用）
bash scripts/run_nav2.sh               # 静态地图 Nav2 + 自动巡视
bash scripts/run_full_system.sh        # 语音/LLM/任务/mock抓放全链路
```

Nav2 巡视入口已统一到 `scripts/run_nav2.sh`：脚本会启动 Gazebo 底盘、Nav2、单个 RViz，并默认运行 `robot_navigation/scripts/patrol_waypoints.py` 巡视当前地图。定位链路会先于导航链路启动，AMCL 默认使用仿真起点 `(0, 0, 0)`。手动点目标时可用：

```bash
TB3_AUTO_PATROL=0 bash scripts/run_nav2.sh
```

运行中的导航用 `Ctrl+C` 停止；若有 Gazebo/RViz 残留，再执行 `bash scripts/tb3_stack.sh stop`。

建图模式是否开启 Gazebo 界面（`gzclient`）：

```bash
bash scripts/run_mapping.sh                                # 默认只开 RViz，不开 Gazebo 界面
TB3_ENABLE_GZCLIENT=1 bash scripts/run_mapping.sh          # 同时开启 Gazebo 界面 + RViz
TB3_ENABLE_RVIZ=0 TB3_ENABLE_GZCLIENT=1 bash scripts/run_mapping.sh  # 仅 Gazebo 界面
TB3_MODEL=burger bash scripts/run_mapping.sh               # 显式切换模型（默认 waffle）
```

停止仿真：

```bash
bash scripts/tb3_stack.sh stop
```

## 约定

- 所有可运行功能只在 `ros_ws/src` 维护。
- `scripts/` 只做编排与自动化，不承载核心算法。

更多说明：
- [ros_ws/README.md](ros_ws/README.md)
- [ros_ws/src/README.md](ros_ws/src/README.md)
- [CONTRIBUTING.md](CONTRIBUTING.md)
- [docs/voice_llm_pick_framework.md](docs/voice_llm_pick_framework.md)
