# TJ-Robot 室内服务机器人系统

这是一个基于 ROS 2 Humble、Gazebo、TurtleBot3、SLAM Toolbox、Nav2 和 YOLO 的室内服务机器人课程项目。当前结构已经收敛为紧凑的 `robot_*` 工程分层：保留现有 ROS2 工作区可运行流程，同时让建图、感知、导航、交互、任务和实验入口更容易找到。

## 当前能力

- Gazebo/TurtleBot3 仿真启动。
- SLAM Toolbox 建图和地图保存。
- baseline 普通建图流程。
- semantic mapping：YOLO/person 检测、人物区域累积、before_strip 保存、strip 后处理、after_strip 输出。
- Nav2/覆盖巡航相关入口。
- ASR、NLU、TTS、搜索任务和 task manager 的近期扩展位置。

## 收敛后的目录

```text
robot_bringup/          系统入口层：launch 与 RViz
robot_core/             任务、事件、状态、动作、topic 常量
robot_perception/       vision/speech/lidar/fusion 感知模块
robot_mapping/          slam/semantic/strip/storage 建图与地图后处理
robot_navigation/       Nav2 wrapper、目标发送、导航监控、房间导航
robot_interaction/      NLU、dialog、TTS
robot_tasks/            task manager、planner、search/mapping/navigation task
robot_experiments/      mapping/navigation/search/metrics 实验与评测
config/                 路径、topic、SLAM、Nav2、YOLO、ASR、NLU、task、experiment 配置
scripts/                一键构建、启动、保存、strip、清理脚本
data/                   maps/logs/results/datasets/recordings
docs/                   架构、运行、实验、模块设计、未来路线
ros_ws/src/             现有可编译 ROS2 包
```

主入口看 `robot_bringup/launch/` 和 `scripts/`。核心功能看 `robot_mapping/`、`robot_perception/`、`robot_navigation/`、`robot_tasks/`。

## 快速运行

```bash
bash scripts/build.sh
```

baseline 建图：

```bash
bash scripts/run_baseline_mapping.sh
bash scripts/save_map.sh raw
```

semantic mapping：

```bash
bash scripts/run_semantic_mapping.sh
bash scripts/save_map.sh before_strip
bash scripts/strip_map.sh data/maps/before_strip/map_xxx.yaml ~/.ros/tj_person_strip_regions.yaml
```

导航：

```bash
bash scripts/run_navigation.sh
```

预留入口：

```bash
bash scripts/run_voice_demo.sh
bash scripts/run_search_task.sh
bash scripts/run_full_system.sh
```

停止仿真：

```bash
bash scripts/tb3_stack.sh stop
```

## 输出目录

- `data/maps/raw`：baseline 或未处理地图。
- `data/maps/before_strip`：strip 前地图。
- `data/maps/after_strip`：strip 后地图。
- `data/maps/semantic`：语义叠加或语义地图产品。
- `data/logs`：运行日志。
- `data/results`：实验结果和指标。

`saved_maps/` 已废弃并删除，统一使用 `data/maps/*`。

## 设计约定

- `scripts/` 只做启动、构建、保存、清理，不写核心业务逻辑。
- `robot_bringup/launch/` 只做系统编排。
- 地图后处理归入 `robot_mapping/strip/`，不再单独放 `processing/`。
- 人物区域、动态对象语义信息归入 `robot_mapping/semantic/`。
- 新任务放 `robot_tasks/`，新交互放 `robot_interaction/`，新感知放 `robot_perception/`。
- 现有 ROS2 包仍在 `ros_ws/src`，保证 `colcon build`、`ros2 run`、`ros2 launch` 的兼容性。

更多说明见 [docs/architecture.md](docs/architecture.md)、[docs/runbook.md](docs/runbook.md)、[docs/experiment.md](docs/experiment.md)、[docs/module_design.md](docs/module_design.md)。
