# ros_ws

`ros_ws/` 是 TJ-Robot 的 ROS 2 Humble 工作空间。所有需要通过 `ros2 run`、`ros2 launch` 或 `colcon build` 使用的源码都维护在 `ros_ws/src/`。

## 目录

```text
ros_ws/
├── src/        ROS 2 包源码，唯一真源
├── build/      colcon 中间构建产物
├── install/    colcon 安装空间
├── log/        colcon 构建日志
└── README.md
```

只有 `src/` 需要长期维护。`build/`、`install/` 和 `log/` 可以重新生成。

## ROS 包

| 包 | 职责 |
|---|---|
| `robot_bringup` | 系统级 launch、地图、Gazebo world、RViz 配置 |
| `robot_navigation` | Nav2 参数、巡视、覆盖规划和卡死恢复 |
| `human_yolo_seg` | YOLO 分割、深度定位和目标地图坐标 |
| `robot_interaction` | 文本/语音输入和 LLM 意图路由 |
| `robot_tasks` | 命令执行、取物状态机和返航 |
| `robot_manipulation` | Gazebo 真值 mock 抓取/放置 |
| `robot_interfaces` | 自定义 msg、srv 和 action |
| `robot_rviz_plugins` | RViz 模拟语音面板 |

包级说明见 [src/README.md](src/README.md)。

## 构建

推荐从仓库根目录执行：

```bash
bash scripts/build.sh
source ros_ws/install/setup.bash
```

等价的手动命令：

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

选择包构建：

```bash
bash scripts/build.sh --packages-select \
  robot_tasks robot_manipulation robot_bringup
```

修改 Python 节点、launch、配置或资源后，应重新构建对应包并重新 `source`。

## 生成物和残留文件

不要直接修改 `install/` 中的文件。运行行为和源码不一致时，依次检查：

1. 是否重新构建了对应包。
2. 当前终端是否重新执行了 `source ros_ws/install/setup.bash`。
3. 是否存在旧进程仍在运行。
4. `install/` 中是否残留源码目录已经删除的资源。

YOLO 权重尤其容易出现第 4 种情况。模型应位于：

```text
src/human_yolo_seg/models/yolo26n-seg.pt
```

或通过 `model_path` 显式指定。不能把 `install/human_yolo_seg/.../models/` 中的旧权重当成项目已包含模型。

## 系统 launch 关系

常用入口由根目录 `scripts/` 调用：

```text
scripts/run_full_system.sh
  -> Gazebo + Nav2 + YOLO
  -> robot_bringup/task_pipeline.launch.py
       -> interaction.launch.py
       -> task_manager.launch.py
       -> manipulation.launch.py
```

`task_pipeline.launch.py` 当前包含：

- 语音或 RViz 模拟文本输入
- LLM/规则意图路由
- `command_executor_node`
- `object_fetch_orchestrator_node`
- `patrol_waypoints.py`
- `mock_pick_place_node`

## 取物任务的包边界

- `human_yolo_seg`：识别物体并发布目标在 `map` 中的位置。
- `robot_tasks`：记录接令位置、搜索、接近、触发抓取、等待结果并返航。
- `robot_manipulation`：等待 2~5 秒，并根据 Gazebo 中机器人和物体真实位姿判断 mock 抓取结果。
- `robot_navigation` / Nav2：执行接近目标和返回接令位置的导航。

mock 抓取节点本身不负责返航。返航由 `object_fetch_orchestrator_node` 统一管理。

## 直接运行 launch

通常优先使用根目录脚本。需要局部调试时可以：

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

ros2 launch robot_bringup manipulation.launch.py \
  pick_delay_min_sec:=2.0 \
  pick_delay_max_sec:=5.0
```

完整 task pipeline：

```bash
ros2 launch robot_bringup task_pipeline.launch.py \
  enable_sim_speech_gui:=true
```

完整系统仍建议使用：

```bash
bash scripts/run_full_system.sh
```

## 开发约定

- 核心逻辑放在功能包内，不放入顶层脚本。
- 系统级资源和 launch 放入 `robot_bringup`。
- 导航行为放入 `robot_navigation`。
- 任务状态机放入 `robot_tasks`。
- 操作执行接口放入 `robot_manipulation`。
- 新增跨包接口时优先使用 `robot_interfaces`。
