# TJ-Robot 室内服务机器人系统

本仓库已收敛为 **ROS 2 单源结构**：运行与开发以 `ros_ws/src` 为唯一真源。

## 当前主线

- ROS 2 Humble + Gazebo + TurtleBot3 仿真。
- SLAM Toolbox 建图、地图保存与人区域后处理。
- YOLO 人物识别 + 激光角域融合（`human_yolo_seg`）。
- 覆盖巡航、点到点、Nav2 相关导航能力。
- 语音输入接口 + LLM 路由接口（`robot_interaction`）。
- mock 机械臂抓取/放置验证链路（`robot_manipulation`）。

## 仓库结构（单源）

```text
ros_ws/                ROS 2 工作空间（唯一运行源码）
  src/
    robot_bringup
    robot_navigation
    human_yolo_seg
    robot_tasks
    robot_interaction
    robot_manipulation
    robot_interfaces
scripts/               构建/运行/存图/清理脚本
data/                  maps/logs/results 等运行产物
docs/                  架构与实验文档
```

地图产物命名约定：

- `data/maps/baseline_raw`
- `data/maps/semantic_pre_strip`
- `data/maps/semantic_post_strip`
- `data/maps/semantic_overlays`

## 快速运行

```bash
bash scripts/build.sh
bash scripts/run_simulation.sh         # 默认 RGBD 机器人（waffle + assist）
bash scripts/run_mapping_laser.sh      # 仅激光建图
bash scripts/run_baseline_mapping.sh
bash scripts/run_semantic_mapping.sh
bash scripts/run_navigation.sh
bash scripts/run_full_system.sh        # 语音/LLM/任务/mock抓放全链路
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
