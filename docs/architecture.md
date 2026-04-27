# 架构说明

项目采用收敛后的 ROS2 工程结构，根目录只保留少量高信号模块。

```text
robot_bringup
  -> robot_core
  -> robot_perception
  -> robot_mapping
  -> robot_navigation
  -> robot_interaction
  -> robot_tasks
  -> robot_experiments
config/scripts/data/docs
ros_ws/src
```

## robot_bringup

系统入口层。`robot_bringup/launch/` 放应用入口和组合 launch：

- `run_simulation.launch.py`
- `run_baseline_mapping.launch.py`
- `run_semantic_mapping.launch.py`
- `run_navigation.launch.py`
- `run_voice_demo.launch.py`
- `run_search_task.launch.py`
- `run_full_system.launch.py`
- `slam.launch.py`
- `nav2.launch.py`
- `perception.launch.py`
- `task_manager.launch.py`

真正可由 ROS2 安装的 launch 同步保留在 `ros_ws/src/robot_bringup/launch/`。

## robot_core

跨模块契约层，包含状态、事件、动作、任务、常量和 topic。

## robot_perception

感知层，包含：

- `vision`：YOLO 通用检测、person/object detector wrapper。
- `lidar`：LaserScan 适配与过滤。
- `speech`：ASR 和 wake word 预留。
- `fusion`：多模态融合预留。

## robot_mapping

建图与地图处理层，包含：

- `slam`：SLAM Toolbox wrapper 与 map builder。
- `semantic`：person region、semantic marker、dynamic object filter。
- `strip`：person-free map strip、map refinement。
- `storage`：map saver/loader/metadata。

`processing/` 已合并进这里。

## robot_navigation

导航层，保留 Nav2 wrapper、目标发送、导航监控和房间级导航。

## robot_interaction

人机交互层，包含 NLU、dialog、TTS。ASR 输入由 `robot_perception/speech` 产生。

## robot_tasks

任务层，负责从 intent 生成动作序列，例如：

```text
search_object(cup, living_room)
-> navigate_to_room(living_room)
-> search_object(cup)
```

## robot_experiments

实验层，按 `mapping`、`navigation`、`search`、`metrics` 组织。

## ros_ws/src

现有有效 ROS2 包继续保留：

- `robot_bringup`
- `robot_navigation`
- `human_yolo_seg`
- `robot_tasks`
- `robot_interfaces`

这些包负责真实 ROS2 构建和运行；根目录 `robot_*` 是工程化源码与系统结构入口。
