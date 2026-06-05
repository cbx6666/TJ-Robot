# robot_tasks

`robot_tasks` 负责把解析后的用户意图转换为导航、巡视、取物和返航任务。

## 主要节点

| 节点 | 职责 |
|---|---|
| `command_executor_node` | 订阅解析后的 JSON 意图，执行导航命令或发布任务事件 |
| `object_fetch_orchestrator_node` | 完整取物状态机：记录起点、搜索、接近、抓取校验和返航 |
| `task_manager_node` | 监听任务事件并维护对外任务状态 |

主要事件总线：

```text
/task/events  std_msgs/msg/String
```

消息内容是 JSON。

## 取物状态机

```text
IDLE
  -> SEARCHING
  -> APPROACHING
  -> PICKING
  -> RETURNING
  -> DONE / FAILED
```

### 1. 接收任务和记录起点

收到：

```json
{"event":"fetch_object","args":{"object_label":"cup"}}
```

节点立即查询当前 `map -> base_footprint`，保存：

```json
{
  "frame_id": "map",
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0
}
```

该位姿是本次任务独立的 `home_pose`，不是写死的 Gazebo 原点。

如果接令时无法取得 TF，任务拒绝启动，并发布：

```json
{
  "event": "fetch_object_done",
  "ok": false,
  "pick_ok": false,
  "return_ok": false,
  "reason": "initial_pose_unavailable"
}
```

### 2. 搜索和目标锁定

节点发布 `patrol_start` 启动覆盖式巡视，并监听：

- `/yolo_objects/target_point_map`
- `/yolo_objects/target_map_valid`
- `/yolo_objects/target_label`

目标满足稳定帧数和空间跳变阈值后，停止巡视并锁定目标坐标。

### 3. 接近目标

根据机器人和目标的位置，计算距目标约 `approach_standoff_m` 的接近点，并通过 Nav2 `NavigateToPose` 导航。

接近过程中 YOLO 短暂丢失时继续使用已锁定坐标，不会立刻取消导航。

### 4. 抓取校验

导航到位后发布：

```text
PICK:cup;locked_target_frame=map;locked_target_xyz=x,y,z
```

`robot_manipulation/mock_pick_place_node` 等待随机 2~5 秒，再使用 Gazebo 实体真实位姿进行判定。

### 5. 返航

以下正常结束路径都会进入 `RETURNING`：

- mock 抓取成功
- mock 抓取失败
- 搜索超时
- 接近导航失败或超时
- 抓取校验超时
- 目标或 Gazebo 查询异常

返航前会取消旧导航目标并连续发布零速度刹车，然后向任务开始时保存的 `home_pose` 发送新的 Nav2 目标。

唯一默认不返航的情况是用户显式发送 `STOP`。这是人工紧急中止语义，会立即取消导航并发布：

```json
{
  "event": "fetch_object_done",
  "ok": false,
  "pick_ok": false,
  "return_ok": false,
  "return_reason": "return_skipped_by_stop"
}
```

## 最终结果

正常完成事件：

```json
{
  "event": "fetch_object_done",
  "ok": true,
  "pick_ok": true,
  "return_ok": true,
  "object_label": "cup"
}
```

`ok` 的计算规则：

```text
ok = pick_ok && return_ok
```

因此：

- 抓取成功但返航失败：整体失败。
- 抓取失败但返航成功：整体失败，但机器人仍回到接令位置。
- 抓取成功且返航成功：整体成功。

常见 `return_reason`：

- `initial_pose_unavailable`
- `return_nav2_unavailable`
- `return_goal_rejected`
- `return_timeout`
- `return_nav_status_<status>`
- `return_nav_send_error:<error>`
- `return_nav_result_error:<error>`

## 关键参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `approach_standoff_m` | `0.55` | 接近点与物体的距离 |
| `target_stable_samples` | `5` | 确认目标所需稳定样本 |
| `target_stable_max_jump_m` | `0.15` | 稳定目标最大空间波动 |
| `target_point_stale_sec` | `1.0` | 目标数据过期时间 |
| `search_timeout_sec` | `180.0` | 搜索超时 |
| `approach_timeout_sec` | `120.0` | 接近导航超时 |
| `pick_verify_timeout_sec` | `45.0` | 等待 mock 抓取结果超时 |
| `return_timeout_sec` | `120.0` | 返航超时 |
| `nav2_ready_timeout_sec` | `120.0` | 等待 Nav2 action 可用 |

## 调试

```bash
tail -f data/logs/full_system/task_pipeline.launch.log
```

重点搜索：

```bash
grep -E "fetch_object|home_pose|locked_target|pick_|return|返航" \
  data/logs/full_system/task_pipeline.launch.log
```
