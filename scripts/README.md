# scripts

`scripts/` 只负责环境检查、构建和进程编排。ROS 节点、状态机和算法实现都位于 `ros_ws/src/`。

所有命令建议从仓库根目录执行：

```bash
cd /mnt/d/Homework/robot
```

## 构建与环境

| 脚本 | 用途 |
|---|---|
| `setup_env.sh` | 安装或检查 ROS 2 Humble、Gazebo、TurtleBot3 等依赖 |
| `build.sh` | 使用 `colcon build --symlink-install` 构建工作空间 |
| `common.sh` | 公共路径、Nav2 lifecycle 和进程清理函数 |
| `prep_faster_whisper.py` | 准备本地 faster-whisper 模型 |
| `clean_outputs.sh` | 清理 `data/` 中的运行产物 |

完整构建：

```bash
bash scripts/build.sh
source ros_ws/install/setup.bash
```

选择包构建：

```bash
bash scripts/build.sh --packages-select \
  robot_tasks robot_manipulation robot_bringup
```

`build.sh` 会把额外参数直接传给 `colcon build`。如果根目录存在 `local_llm.env`，构建后的 `ros_ws/install/setup.bash` 会自动加载其中的变量。

## 推荐运行入口

| 脚本 | 启动内容 |
|---|---|
| `run_full_system.sh` | 仿真、静态地图 Nav2、YOLO、LLM、任务编排和 mock 抓取 |
| `run_full_system_real_mic.sh` | 全系统加真实麦克风 ASR |
| `run_voice_llm_only.sh` | 只启动 ASR/文本输入和 LLM |
| `run_simulation.sh` | Gazebo、静态地图 Nav2、RGB-D 和 YOLO |
| `run_mapping.sh` | Gazebo、SLAM Toolbox 和 RViz |
| `run_nav2.sh` | 静态地图 Nav2、覆盖规划和自动巡视 |

默认全系统：

```bash
bash scripts/run_full_system.sh
```

传递 task pipeline launch 参数：

```bash
bash scripts/run_full_system.sh \
  pick_delay_min_sec:=2.0 \
  pick_delay_max_sec:=5.0 \
  pick_max_distance_m:=1.5
```

这些参数会传给 `robot_bringup/task_pipeline.launch.py`。

## 仿真基础栈

`tb3_stack.sh` 是底层编排入口，通常由其他脚本调用：

```bash
bash scripts/tb3_stack.sh start
bash scripts/tb3_stack.sh check
bash scripts/tb3_stack.sh logs yolo
bash scripts/tb3_stack.sh stop
```

主要启动内容：

- Gazebo server，可选 `gzclient`
- TurtleBot3 Waffle
- `robot_state_publisher`
- RGB-D 相机和 `/scan` 转换
- YOLO 物体识别
- 可选 SLAM
- RViz

常用环境变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `TB3_ENABLE_RVIZ` | `1` | 是否启动 RViz |
| `TB3_ENABLE_GZCLIENT` | 依入口而定 | 是否启动 Gazebo GUI |
| `TB3_GAZEBO_HARDWARE_GL` | `0` | `0` 使用软件渲染，`1` 尝试硬件渲染 |
| `TB3_GZSERVER_WAIT_SEC` | `0` | 等待 `/spawn_entity`；`0` 表示不设上限 |
| `TB3_SPAWN_ENTITY_TIMEOUT_SEC` | `86400` | `spawn_entity.py` 超时 |
| `TB3_LOG_DIR` | 依入口而定 | 日志输出目录 |

## Nav2 与巡视

自动巡视：

```bash
bash scripts/run_nav2.sh
```

关闭自动巡视，只在 RViz 手动发送目标：

```bash
TB3_AUTO_PATROL=0 bash scripts/run_nav2.sh
```

切换全局规划配置：

```bash
PLANNER_TYPE=astar bash scripts/run_nav2.sh
PLANNER_TYPE=dijkstra bash scripts/run_nav2.sh
```

## 建图

```bash
bash scripts/run_mapping.sh
TB3_ENABLE_GZCLIENT=1 bash scripts/run_mapping.sh
TB3_ENABLE_RVIZ=0 TB3_ENABLE_GZCLIENT=1 bash scripts/run_mapping.sh
```

## 停止语义

- 在 `run_full_system.sh` 前台按 `Ctrl+C`：停止语音、LLM 和任务 pipeline，后台仿真可能仍在运行。
- `bash scripts/tb3_stack.sh stop`：停止基础仿真栈。
- `bash scripts/kill_simulation_stack.sh`：停止语音、任务、Nav2、Gazebo、RViz 和 ROS daemon，适合实验结束或清理残留进程。
- 任务运行时发送 `STOP`：取消当前取物和导航，不执行自动返航。

## 日志

全系统默认目录：

```text
data/logs/full_system/
```

单独仿真默认目录：

```text
data/logs/simulation/
```

常用命令：

```bash
tail -f data/logs/full_system/task_pipeline.launch.log
tail -f data/logs/full_system/yolo_object_seg.log
tail -f data/logs/full_system/nav2.launch.log
bash scripts/tb3_stack.sh logs all
```

## YOLO 权重提醒

源码模型默认路径为：

```text
ros_ws/src/human_yolo_seg/models/yolo26n-seg.pt
```

如果源码目录没有模型，但当前系统仍能启动，优先检查是否误用了 `ros_ws/install/` 中的旧构建残留。干净构建和新机器不会保留该文件。

`run_full_system.sh` 当前不会把命令行中的 `model_path` 转发给提前启动的 YOLO 节点。全系统使用自定义权重时，请把权重放入源码 `models/` 并重新构建；`model_path:=...` 适用于单独启动 `yolo_object_seg.launch.py`。

## lib/

`scripts/lib/` 中的文件由入口脚本 `source`，不应单独作为用户入口运行。主要模块包括：

- `full_system_env.sh`
- `tb3_sim_assist_env.sh`
- `tb3_stack_cleanup_gui.sh`
- `tb3_stack_wait.sh`
- `wsl_pulse_env.sh`
