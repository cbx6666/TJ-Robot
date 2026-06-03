# scripts

仓库级 Bash 编排：环境、构建、仿真主栈与运行入口。核心逻辑在 `ros_ws/src`。

## 环境与构建

| 脚本 | 作用 |
|------|------|
| `setup_env.sh` | 安装 ROS 2 Humble、Gazebo、TurtleBot3 等依赖 |
| `build.sh` | `colcon build` 工作区 |
| `clean_outputs.sh` | 清理 `data/` 等运行产物 |
| `common.sh` | 路径、Nav2 清理等共享函数 |
| `prep_faster_whisper.py` | 预下载/热身 faster-whisper 权重 |

## 仿真主栈

| 脚本 | 作用 |
|------|------|
| `tb3_stack.sh` | Gazebo、机器人、SLAM/YOLO 可选链路的 `start` / `stop` / `check` / `logs` |
| `kill_simulation_stack.sh` | 停语音 + 仿真 + Nav2 + daemon（实验结束用） |

## 运行入口

| 脚本 | 作用 |
|------|------|
| `run_simulation.sh` | 仿真 + 静态地图 Nav2（无 LLM/任务） |
| `run_mapping.sh` | 激光 SLAM 建图（slam_toolbox，无 Nav2） |
| `run_nav2.sh` | 静态地图 Nav2 + 自动巡视 |
| `run_full_system.sh` | **推荐** 一键：仿真 + Nav2 + YOLO + LLM/任务（RViz 模拟语音） |
| `run_full_system_real_mic.sh` | 一键全栈 + 真麦 ASR |
| `run_voice_llm_only.sh` | 仅 ASR + LLM（无仿真，联调语音用） |

## `lib/` 内部模块

由上述脚本 `source`，勿单独当入口：`full_system_env.sh`、`wsl_pulse_env.sh`、`tb3_sim_assist_env.sh`、`tb3_stack_*.sh` 等。

## 常用命令

```bash
bash scripts/build.sh
bash scripts/run_full_system.sh
bash scripts/run_voice_llm_only.sh
bash scripts/run_full_system_real_mic.sh
bash scripts/kill_simulation_stack.sh
```

Nav2 开关示例：`TB3_AUTO_PATROL=0 bash scripts/run_nav2.sh`

建图 GUI：`TB3_ENABLE_GZCLIENT=1 bash scripts/run_mapping.sh`
