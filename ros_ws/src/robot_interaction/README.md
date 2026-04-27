# robot_interaction

语音与 LLM 交互接口包（当前为可运行骨架，便于后续替换真实 API）。

## 节点

- `voice_gateway_node`
  - 输出：`/interaction/voice_text`
  - 默认支持 mock 定时输入。
- `llm_router_node`
  - 输入：`/interaction/voice_text`
  - 输出：`/task/goal_text`、`/interaction/llm_plan_text`

## 后续扩展

- 接入真实 ASR API。
- 接入真实 LLM tool-calling。
- 输出 `robot_interfaces` 强类型消息。
