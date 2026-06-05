# 语音/文本取物任务框架

本文描述当前“用户指令 -> 视觉搜索 -> mock 抓取 -> 返回接令位置”的实际接口和状态流。

## 端到端链路

```text
RViz Sim Speech / 麦克风 ASR
  -> robot_interaction
  -> /interaction/parsed_intent
  -> robot_tasks/command_executor_node
  -> /task/events: fetch_object
  -> robot_tasks/object_fetch_orchestrator_node
       -> 记录 home_pose
       -> patrol_start
       -> YOLO 目标锁定
       -> Nav2 接近
       -> PICK 命令
  -> robot_manipulation/mock_pick_place_node
       -> 随机等待 2~5 秒
       -> Gazebo 真值判定
       -> pick_success / pick_failed
  -> object_fetch_orchestrator_node
       -> Nav2 返回 home_pose
       -> fetch_object_done
```

## 模块职责

### robot_interaction

- 接收 RViz 模拟文本或真实语音。
- 调用 LLM 或规则路由。
- 发布结构化意图到 `/interaction/parsed_intent`。

### command_executor_node

- 解析结构化命令。
- 普通导航直接调用 Nav2。
- `fetch_object` 转换为 `/task/events` 事件。

### object_fetch_orchestrator_node

- 在接令瞬间记录机器人当前位姿。
- 驱动覆盖式巡视。
- 监听并稳定化 YOLO 目标坐标。
- 计算接近点并调用 Nav2。
- 发布 mock 抓取命令。
- 收到成功或失败结果后统一返航。
- 汇总 `pick_ok`、`return_ok` 和最终 `ok`。

### mock_pick_place_node

- 不执行 Nav2。
- 不直接使用 YOLO 坐标决定成功。
- 等待 2~5 秒模拟抓取动作。
- 查询 Gazebo 机器人和物体实体位姿。
- 根据距离和朝向阈值发布结果。

## 主要话题

| 话题 | 类型 | 生产者 | 消费者 |
|---|---|---|---|
| `/interaction/parsed_intent` | `std_msgs/String` JSON | `llm_router_node` | `command_executor_node` |
| `/task/events` | `std_msgs/String` JSON | 多个任务节点 | 任务管理、巡视和取物编排 |
| `/yolo_objects/target_point_map` | `geometry_msgs/PointStamped` | YOLO | 取物编排 |
| `/yolo_objects/target_map_valid` | `std_msgs/Bool` | YOLO | 取物编排 |
| `/yolo_objects/target_label` | `std_msgs/String` | YOLO | 取物编排 |
| `/manipulation/command_text` | `std_msgs/String` | 取物编排 | mock 操作节点 |
| `/manipulation/status_text` | `std_msgs/String` | mock 操作节点 | 取物编排 |

## 任务事件

开始取物：

```json
{
  "event": "fetch_object",
  "args": {
    "object_label": "cup"
  }
}
```

记录起点并开始搜索：

```json
{
  "event": "fetch_object_started",
  "object_label": "cup",
  "phase": "search",
  "home_pose": {
    "frame_id": "map",
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  }
}
```

返航阶段：

```json
{
  "event": "fetch_object_phase",
  "phase": "returning",
  "pick_ok": true,
  "home_pose": {
    "frame_id": "map",
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  }
}
```

最终结果：

```json
{
  "event": "fetch_object_done",
  "ok": true,
  "pick_ok": true,
  "return_ok": true,
  "object_label": "cup"
}
```

## mock 抓取契约

命令：

```text
PICK:<label>;locked_target_frame=map;locked_target_xyz=x,y,z
```

结果文本至少包含下列标记之一：

```text
pick_success
pick_failed
```

编排节点只在 `PICKING` 状态处理这些结果，防止旧消息影响新任务。

默认参数：

```text
pick_delay_min_sec=2.0
pick_delay_max_sec=5.0
pick_min_distance_m=0.0
pick_max_distance_m=1.5
pick_max_yaw_error_deg=90.0
gazebo_query_timeout_sec=8.0
```

随机等待通过 ROS timer 实现，不会阻塞其他回调。Gazebo 查询超时从等待结束后开始计算。

## 返航契约

`home_pose` 是接收本次任务时的实时位姿，不是固定地图原点。

正常任务中的成功和失败都会返航：

```text
搜索失败 --------\
接近失败 ---------\
抓取失败 ----------> RETURNING -> fetch_object_done
抓取成功 ---------/
抓取结果超时 ----/
```

显式 `STOP` 是例外：

```text
STOP -> 刹车 -> 取消 Nav2 -> FAILED
     -> return_reason=return_skipped_by_stop
```

整体成功规则：

```text
ok = pick_ok && return_ok
```

## 启动与参数覆盖

```bash
bash scripts/run_full_system.sh
```

调整 mock 动作时间：

```bash
bash scripts/run_full_system.sh \
  pick_delay_min_sec:=3.0 \
  pick_delay_max_sec:=4.0
```

## 替换真实机械臂

后续接入真实机械臂时，建议保留：

- `/manipulation/command_text`
- `/manipulation/status_text`
- `pick_success` / `pick_failed` 结果语义

真实操作节点可以替换 Gazebo 查询和距离阈值，但不要把返航逻辑移入机械臂节点。返航仍应由 `object_fetch_orchestrator_node` 统一管理。
