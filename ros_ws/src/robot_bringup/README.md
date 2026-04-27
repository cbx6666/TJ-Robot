# robot_bringup

当前阶段仅保留 RGBD 主线启动编排。

## 目录说明

| 路径 | 内容 |
|------|------|
| `launch/` | 主链 launch（`perception`、`interaction`、`task_pipeline`、`full_system`、`rgbd_to_scan`） |
| `config/` | RViz 与基础配置 |
| `world/` | Gazebo 世界 |
| `models/` | 仿真模型 |
| `urdf/` | 机器人相关 URDF 片段 |
| `scripts/` | SDF/URDF 预处理与辅助脚本 |

## 主要 launch

| 文件 | 典型用途 |
|------|-----------|
| `perception.launch.py` | 启动 YOLO 识别 |
| `interaction.launch.py` | 启动语音网关 + LLM 路由 |
| `task_manager.launch.py` | 启动任务管理 |
| `manipulation.launch.py` | 启动 mock 抓放 |
| `task_pipeline.launch.py` | 启动 interaction + task + manipulation |
| `full_system.launch.py` | 一键全链路（不含旧导航/建图） |
| `rgbd_to_scan.launch.py` | 可选 depth->scan 桥接 |

## 与 `tb3_stack.sh` 关系

- `scripts/tb3_stack.sh` 负责 Gazebo、robot_state_publisher、YOLO 和可选 rgbd_to_scan 的启动。
- 修改本包后执行：`colcon build --packages-select robot_bringup && source install/setup.bash`。
