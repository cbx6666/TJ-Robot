# 语音取物框架规范（V0）

本文件定义“语音指令 -> 目标识别 -> mock 抓放”的接口边界，便于后续替换真实 ASR/LLM/机械臂实现。

## 1) 运行模式

- RGBD 任务模式（当前唯一主线）：
  - 入口：`scripts/run_simulation.sh` / `scripts/run_full_system.sh`
  - 默认：`TURTLEBOT3_MODEL=waffle`, `TB3_STACK_MODE=assist`, `TB3_ASSIST_RGBD_BRIDGE=1`

## 2) 模块职责

- `robot_interaction/voice_gateway_node`
  - 负责语音输入接入（API 或 mock）。
  - 输出：`/interaction/voice_text`（`std_msgs/String`）。
- `robot_interaction/llm_router_node`
  - 负责把语音文本转成任务目标文本（后续可替换为真实 LLM 调用）。
  - 输出：`/task/goal_text`，`/interaction/llm_plan_text`。
- `robot_tasks/task_manager_node`
  - 负责任务分发（当前聚焦抓取命令）。
  - 输出：`/manipulation/command_text`，`/task/status(_text)`。
- `robot_manipulation/mock_pick_place_node`
  - mock 抓放语义执行，不做精细机械臂控制。

## 3) 推荐话题/服务契约

- 话题（当前）
  - `/interaction/voice_text`
  - `/task/goal_text`
  - `/manipulation/command_text`
  - `/task/status` 与 `/task/status_text`
- 服务（当前 mock）
  - `/manipulation/mock_pick` (`std_srvs/Trigger`)
  - `/manipulation/mock_place` (`std_srvs/Trigger`)
- 强类型接口（已预留）
  - `robot_interfaces/msg/VoiceCommand.msg`
  - `robot_interfaces/msg/TaskGoal.msg`
  - `robot_interfaces/srv/ManipulationCommand.srv`

## 4) 参数分层建议

- 根目录参数模板：
  - `config/interaction/voice.yaml`
  - `config/llm/orchestration.yaml`
  - `config/manipulation/mock_arm.yaml`

## 5) 后续替换点

- 把 `voice_gateway_node` 的 mock 定时发布替换为真实语音 API 回调。
- 把 `llm_router_node` 的规则式输出替换为函数调用式 tool planner。
- 把 `mock_pick_place_node` 替换成真实机械臂控制适配层（保持 topic/service 契约尽量不变）。
