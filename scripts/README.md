# scripts

脚本层仅负责构建与运行编排，不承载核心算法。

## 基础脚本

| 文件 | 作用 |
|---|---|
| `setup_env.sh` | 安装 ROS 2 Humble、Gazebo、TurtleBot3 等依赖。 |
| `build.sh` | 编译 `ros_ws`。 |
| `common.sh` | 共享环境与路径函数。 |
| `tb3_stack.sh` | 主栈：RGBD 仿真 + 激光建图(/map) + YOLO 识别 + 可选 depth->scan。 |

## 运行入口

| 文件 | 作用 |
|---|---|
| `run_simulation.sh` | 启动默认仿真（RGBD 机器人：`waffle + assist + rgbd bridge`）。 |
| `run_voice_demo.sh` | 启动语音/LLM/任务/mock抓放入口。 |
| `run_full_system.sh` | 启动完整系统（仿真 + 感知 + 语音/LLM + 任务 + mock抓放）。 |

## 常用命令

```bash
bash scripts/build.sh
bash scripts/run_simulation.sh
bash scripts/run_full_system.sh
```

停止仿真：

```bash
bash scripts/tb3_stack.sh stop
```
