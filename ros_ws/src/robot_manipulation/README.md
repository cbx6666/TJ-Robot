# robot_manipulation

## 1. 包定位

`robot_manipulation` 是操作执行接口包。  
当前阶段主要提供 mock 级 pick/place 语义执行能力，用于验证任务链路与上层接口，不代表真实机械臂控制实现。

## 2. 当前职责

- 提供 mock pick/place 服务
- 输出操作状态文本
- 为任务系统和交互系统提供可调用的操作端点

## 3. 当前节点

- `mock_pick_place_node`
  - 话题输入：`/manipulation/command_text`
  - 状态输出：`/manipulation/status_text`
  - 服务：
    - `/manipulation/mock_pick`
    - `/manipulation/mock_place`

## 4. 当前边界

本包当前不负责：

- 真实机械臂轨迹规划
- 夹爪控制细节
- 视觉伺服或抓取姿态求解

## 5. 维护建议

- 保持当前 mock 语义接口稳定，便于系统联调
- 若后续接入真实操作硬件，应在本包内逐步替换 mock 层，而不是直接改动上层调用关系
