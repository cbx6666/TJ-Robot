# ros_ws/src 结构说明

`src/` 只放 ROS 2 功能包，每个包统一按下面约定组织：

- `launch/`：启动编排入口
- `config/`：参数与 YAML 配置
- `<pkg_name>/nodes/`：可直接运行的 ROS 节点
- `<pkg_name>/tools/`：离线工具/CLI（不常驻）
- `<pkg_name>/utils/`：纯函数与公共工具（不直接起节点）
- `<pkg_name>/processing/`、`<pkg_name>/perception/`：按业务域细分模块

## 当前包分工

- `robot_bringup`：系统编排、launch、world、地图和参数
- `robot_navigation`：导航与覆盖逻辑（节点在 `robot_navigation/nodes`）
- `human_yolo_seg`：人检测、激光过滤、地图后处理（节点/工具/工具库已分层）
- `robot_tasks`：任务管理（节点在 `robot_tasks/nodes`）
- `robot_interaction`：语音输入网关与 LLM 路由接口
- `robot_manipulation`：mock 抓取/放置接口（用于端到端验证）
- `robot_interfaces`：跨包消息契约（可按开关生成 rosidl）

## 快速定位

- 想看“系统怎么启动”：先看 `robot_bringup/launch`
- 想看“某个节点逻辑”：先看 `<pkg>/<pkg>/nodes`
- 想看“离线后处理脚本”：先看 `<pkg>/<pkg>/tools`
- 想看“算法和数学工具”：先看 `<pkg>/<pkg>/utils`
