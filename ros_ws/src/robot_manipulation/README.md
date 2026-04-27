# robot_manipulation

mock 机械臂语义执行包（用于验证“找对物体并执行抓放”的任务闭环）。

## 节点

- `mock_pick_place_node`
  - 话题输入：`/manipulation/command_text`
  - 状态输出：`/manipulation/status_text`
  - 服务：
    - `/manipulation/mock_pick`
    - `/manipulation/mock_place`

## 约束

- 当前不实现机械臂轨迹与夹爪细节控制。
- 仅验证 pick/place 语义流程与上层调度接口。
