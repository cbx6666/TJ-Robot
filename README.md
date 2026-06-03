# TJ-Robot 室内服务机器人系统

本仓库已收敛为 **ROS 2 单源结构**：运行与开发以 `ros_ws/src` 为唯一真源。

## 当前主线

- ROS 2 Humble + Gazebo + TurtleBot3 仿真。
- 激光 SLAM 地图发布（`/map`）。
- RGBD + YOLO 视觉识别（保留检测链路，移除历史角度/建图后处理链路）。
- 语音输入接口 + LLM 路由接口（`robot_interaction`）。
- mock 机械臂抓取/放置验证链路（`robot_manipulation`）。

## 首次安装

从零配置 ROS 2、构建工作区、以及可选的 LLM / 本地语音依赖，见 **[docs/first-time-setup.md](docs/first-time-setup.md)**。

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
    robot_navigation
scripts/               构建与运行入口（Bash）
data/                  logs/results 等运行产物（大文件见 .gitignore）
config/                部分非 ROS 编排配置
docs/                  架构与实验文档
```

---

## 快速运行

```bash
bash scripts/build.sh
bash scripts/run_simulation.sh         # 默认一体 RGB-D（/camera/image_raw + /camera/depth/*，640×480@8Hz）
bash scripts/run_mapping.sh            # 仅激光 + SLAM + RViz（建图常用）
bash scripts/run_nav2.sh               # 静态地图 Nav2 + 自动巡视（见脚本内说明）
bash scripts/run_full_system.sh        # 【推荐】一键：仿真+Nav2+YOLO+LLM/任务（RViz 模拟语音，默认无麦）
```

**仿真一键（RViz 模拟语音）**：`run_full_system.sh` 拉起仿真、Nav2、YOLO、LLM/任务；在 **RViz → Sim Speech** 面板发送指令。首次需 `colcon build --packages-select robot_rviz_plugins robot_interaction robot_bringup`。

**真麦语音**（WSL 需 WSLg；先单独测通再开全栈）：

```bash
bash scripts/run_voice_llm_only.sh       # 仅 ASR + LLM，无仿真
bash scripts/run_full_system_real_mic.sh # 一键全栈 + 真麦
```

配置 `local_llm.env`（见 `local_llm.env.example`）。日志目录 `data/logs/full_system/`；停止实验：`bash scripts/kill_simulation_stack.sh`。环境与麦克风说明见 [docs/first-time-setup.md](docs/first-time-setup.md)。

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

---

## 约定

- 所有可运行功能只在 `ros_ws/src` 维护。
- `scripts/` 只做编排与自动化，不承载核心算法。
- `ros_ws/build`、`ros_ws/install`、`ros_ws/log` 为 colcon 生成物，勿当作源码修改；见根目录 `.gitignore`。

更多说明：

- [docs/first-time-setup.md](docs/first-time-setup.md)（首次安装与环境准备）
- [ros_ws/README.md](ros_ws/README.md)
- [ros_ws/src/README.md](ros_ws/src/README.md)
- [CONTRIBUTING.md](CONTRIBUTING.md)
- [docs/voice_llm_pick_framework.md](docs/voice_llm_pick_framework.md)
