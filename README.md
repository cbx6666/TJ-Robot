# TJ-Robot 室内服务机器人

TJ-Robot 是一个基于 ROS 2 Humble、Gazebo、Nav2 和 YOLO 的室内移动服务机器人项目。当前主线目标是在仿真环境中完成：

1. 接收文字或语音任务。
2. 在地图中巡视并寻找指定物体。
3. 导航到物体附近。
4. 使用 Gazebo 真值完成 mock 抓取判定。
5. 无论抓取成功还是失败，都返回接收任务时的位置。

> 当前“抓取”是任务链路验证用 mock，不包含真实机械臂轨迹规划和夹爪控制。

## 项目结构

```text
.
├── ros_ws/                  ROS 2 工作空间
│   └── src/                 所有 ROS 2 源码的唯一真源
├── scripts/                 构建、启动、停止和环境编排脚本
├── config/                  非 ROS 包级配置
├── data/                    日志、结果等运行产物
├── docs/                    安装、架构和实验文档
├── local_llm.env.example    LLM 配置示例
└── README.md
```

`ros_ws/build/`、`ros_ws/install/` 和 `ros_ws/log/` 都是生成物，不能作为源码维护。特别是模型文件可能残留在 `install/` 中，造成“源码没有模型但当前电脑仍能运行”的假象。

## 快速开始

首次安装请先阅读 [docs/first-time-setup.md](docs/first-time-setup.md)。

```bash
cd /mnt/d/Homework/robot
bash scripts/build.sh
bash scripts/run_full_system.sh
```

全系统启动后，在 RViz 的 **Sim Speech** 面板输入取物指令，例如：

```text
帮我拿杯子
```

停止全部仿真、Nav2、YOLO 和任务节点：

```bash
bash scripts/kill_simulation_stack.sh
```

## 常用运行入口

| 命令 | 用途 |
|---|---|
| `bash scripts/run_full_system.sh` | 推荐入口：仿真、Nav2、YOLO、LLM、任务和 mock 抓取 |
| `bash scripts/run_full_system_real_mic.sh` | 全系统加真实麦克风 ASR |
| `bash scripts/run_voice_llm_only.sh` | 只启动语音和 LLM，便于单独联调 |
| `bash scripts/run_simulation.sh` | 仿真、静态地图 Nav2 和视觉链路 |
| `bash scripts/run_mapping.sh` | Gazebo、SLAM Toolbox 和 RViz 建图 |
| `bash scripts/run_nav2.sh` | 静态地图 Nav2 和自动巡视 |
| `bash scripts/tb3_stack.sh stop` | 停止基础仿真栈 |

更多脚本参数见 [scripts/README.md](scripts/README.md)。

## 取物任务当前逻辑

取物状态机由 `robot_tasks/object_fetch_orchestrator_node` 负责：

```text
接收 fetch_object
  -> 记录当前 map -> base_footprint 位姿为 home_pose
  -> 覆盖式巡视并等待 YOLO 目标稳定
  -> 锁定目标坐标
  -> Nav2 导航到目标前方约 0.55 m
  -> 发布 PICK:<label>
  -> mock 等待随机 2~5 秒并使用 Gazebo 真值判定
  -> 返回 home_pose
  -> 发布 fetch_object_done
```

关键行为：

- 接令时无法取得机器人当前位姿，任务直接拒绝启动。
- mock 抓取成功后返航。
- mock 抓取失败、搜索超时、接近失败或校验超时后也会返航。
- 最终 `ok` 只有在 `pick_ok=true` 且 `return_ok=true` 时才为 `true`。
- 用户显式发送 `STOP` 属于人工中止：立即刹车并取消导航，不再自动返航。

详细接口和事件说明：

- [robot_tasks/README.md](ros_ws/src/robot_tasks/README.md)
- [robot_manipulation/README.md](ros_ws/src/robot_manipulation/README.md)
- [docs/voice_llm_pick_framework.md](docs/voice_llm_pick_framework.md)

## YOLO 模型

默认模型文件名：

```text
ros_ws/src/human_yolo_seg/models/yolo26n-seg.pt
```

模型被 `.gitignore` 排除，不会随普通 Git 提交进入仓库。首次运行前必须：

1. 把权重放入上述源码目录，然后重新构建 `human_yolo_seg`；或
2. 单独启动 YOLO launch 时通过 `model_path:=/absolute/path/yolo26n-seg.pt` 指定绝对路径。

```bash
ros2 launch human_yolo_seg yolo_object_seg.launch.py \
  model_path:=/absolute/path/yolo26n-seg.pt
```

当前 `run_full_system.sh` 的基础仿真栈使用默认模型名，因此全系统运行最稳妥的方式仍是把模型放入源码 `models/` 后重新构建。

不要依赖 `ros_ws/install/human_yolo_seg/.../models/` 中的旧文件。清理工作空间或换电脑后，这类残留文件会消失。

## 构建

完整构建：

```bash
bash scripts/build.sh
source ros_ws/install/setup.bash
```

只构建本次任务链路涉及的包：

```bash
bash scripts/build.sh --packages-select \
  robot_tasks robot_manipulation robot_bringup
source ros_ws/install/setup.bash
```

## 日志与排查

全系统日志默认位于：

```text
data/logs/full_system/
```

重点文件：

| 文件 | 内容 |
|---|---|
| `task_pipeline.launch.log` | 语音、LLM、任务编排和 mock 抓取 |
| `nav2.launch.log` | Nav2 定位、规划和控制 |
| `nav2_deferred_navigation.log` | Nav2 lifecycle 延迟启动 |
| `yolo_object_seg.log` | 模型加载、推理和目标定位 |
| `rviz2.log` | RViz 配置和显示问题 |
| `gzserver.log` | Gazebo 服务、模型和传感器 |

常用检查：

```bash
tail -f data/logs/full_system/task_pipeline.launch.log
tail -f data/logs/full_system/yolo_object_seg.log
tail -f data/logs/full_system/nav2.launch.log
```

## 文档入口

- [scripts/README.md](scripts/README.md)：脚本与启动方式
- [ros_ws/README.md](ros_ws/README.md)：工作空间和构建规则
- [ros_ws/src/README.md](ros_ws/src/README.md)：ROS 包职责
- [docs/first-time-setup.md](docs/first-time-setup.md)：首次安装
- [CONTRIBUTING.md](CONTRIBUTING.md)：开发约定
