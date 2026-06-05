# ros_ws/src

该目录是项目全部 ROS 2 包的源码根目录。

## 包职责

| 包 | 主要内容 | 不负责 |
|---|---|---|
| `robot_bringup` | 系统 launch、地图、world、RViz、仿真资源 | 导航和任务算法 |
| `robot_navigation` | Nav2、巡视、覆盖规划、卡死恢复 | Gazebo world 和任务语义 |
| `human_yolo_seg` | YOLO 推理、RGB-D 深度采样、目标地图坐标 | 任务调度和导航 |
| `robot_interaction` | 语音/文本输入、LLM 意图解析 | 直接控制底盘 |
| `robot_tasks` | 命令执行、取物状态机、返航和任务事件 | mock 抓取几何判定 |
| `robot_manipulation` | Gazebo 真值 mock 抓取和放置 | Nav2 返航 |
| `robot_interfaces` | 跨包消息、服务和动作定义 | 业务实现 |
| `robot_rviz_plugins` | RViz Sim Speech 面板 | 语音识别 |

## 取物链路

```text
robot_interaction
  -> /interaction/parsed_intent
robot_tasks/command_executor_node
  -> /task/events: fetch_object
robot_tasks/object_fetch_orchestrator_node
  -> 巡视、目标锁定、Nav2 接近
  -> /manipulation/command_text: PICK:...
robot_manipulation/mock_pick_place_node
  -> /manipulation/status_text: pick_success / pick_failed
robot_tasks/object_fetch_orchestrator_node
  -> Nav2 返回接令位置
  -> /task/events: fetch_object_done
```

正常取物失败也会返航；显式 `STOP` 会中止并跳过返航。

## 包内组织

推荐结构：

```text
<package>/
├── launch/
├── config/
├── resource/
├── <package>/nodes/
├── <package>/utils/
├── package.xml
├── setup.py 或 CMakeLists.txt
└── README.md
```

## 开发注意

- 修改源码后重新构建对应包。
- 重新构建后重新 `source ros_ws/install/setup.bash`。
- 不要在 `ros_ws/install/` 中维护代码或模型。
- 模型、大日志和实验结果应遵守 `.gitignore`，并在 README 中说明外部准备方式。
- 包之间通过 topic、service、action 或 `robot_interfaces` 通信，避免直接导入其他包的内部节点实现。
