# robot_interaction

## 1. 包定位

`robot_interaction` 是交互接口包，负责语音输入与上层语言交互链路的接口封装。  
当前代码更接近可运行骨架，便于后续替换为真实 ASR、LLM 或工具调用实现。

## 2. 主要职责

- 接收或模拟语音文本输入
- 将交互输入路由到任务层
- 输出规划文本或中间交互结果

## 3. 当前节点

- `voice_gateway_node`
  - 输出：`/interaction/voice_text`
  - 用于接入或模拟语音输入
- `llm_router_node`
  - 输入：`/interaction/voice_text`
  - 输出：`/task/goal_text`、`/interaction/llm_plan_text`

## 4. 发展方向

后续可在本包内逐步接入：

- 真实 ASR 接口
- 真实 LLM 推理或工具调用能力
- 与 `robot_interfaces` 的强类型消息对接

## 5. 维护建议

- 将交互协议和接口层逻辑留在本包
- 若后续出现复杂任务决策，应通过任务层或专门 Agent 模块承接，而不是把全部逻辑堆入交互包
