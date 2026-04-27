# 语音取物框架规范（V0）

本文件定义“语音指令 -> 导航搜索 -> mock 抓放”的接口边界，便于后续替换真实 ASR/LLM/机械臂实现。

## 1) 运行模式

- 建图模式（仅激光）：
  - 入口：`scripts/run_mapping_laser.sh`
  - 传感：`TB3_STACK_MODE=laser`
  - 目标：构建地图并保存到 `data/maps/baseline_raw` 等目录
- 建图后任务模式（RGBD+激光）：
  - 入口：`scripts/run_full_system.sh`
  - 默认：`TURTLEBOT3_MODEL=waffle`, `TB3_STACK_MODE=assist`, `TB3_ASSIST_RGBD_BRIDGE=1`

## 2) 模块职责

- `robot_interaction/voice_gateway_node`
  - 负责语音输入接入（API 或 mock）。
  - 输出：`/interaction/voice_text`（`std_msgs/String`）。
- `robot_interaction/llm_router_node`
  - 负责把语音文本转成任务目标文本（后续可替换为真实 LLM 调用）。
  - 输出：`/task/goal_text`，`/interaction/llm_plan_text`。
- `robot_tasks/task_manager_node`
  - 负责任务分发（导航/抓取命令）。
  - 输出：`/navigation/command_text`，`/manipulation/command_text`，`/task/status(_text)`。
- `robot_navigation/navigation_command_router`
  - 导航命令入口边界，后续可绑定 Nav2 action 或多策略管理器。
- `robot_manipulation/mock_pick_place_node`
  - mock 抓放语义执行，不做精细机械臂控制。

## 3) 推荐话题/服务契约

- 话题（当前）
  - `/interaction/voice_text`
  - `/task/goal_text`
  - `/navigation/command_text`
  - `/manipulation/command_text`
  - `/task/status` 与 `/task/status_text`
- 服务（当前 mock）
  - `/manipulation/mock_pick` (`std_srvs/Trigger`)
  - `/manipulation/mock_place` (`std_srvs/Trigger`)
- 强类型接口（已预留）
  - `robot_interfaces/msg/VoiceCommand.msg`
  - `robot_interfaces/msg/TaskGoal.msg`
  - `robot_interfaces/srv/ManipulationCommand.srv`
  - `robot_interfaces/srv/NavigationCommand.srv`

## 4) 参数分层建议

- 根目录参数模板：
  - `config/interaction/voice.yaml`
  - `config/llm/orchestration.yaml`
  - `config/navigation/mapping_navigation.yaml`
  - `config/navigation/postmap_navigation.yaml`
  - `config/manipulation/mock_arm.yaml`
- Nav2 参数继续保持 `base + profiles` 两层。

## 5) 后续替换点

- 把 `voice_gateway_node` 的 mock 定时发布替换为真实语音 API 回调。
- 把 `llm_router_node` 的规则式输出替换为函数调用式 tool planner。
- 把 `navigation_command_router` 接入 Nav2 action client。
- 把 `mock_pick_place_node` 替换成真实机械臂控制适配层（保持 topic/service 契约尽量不变）。
