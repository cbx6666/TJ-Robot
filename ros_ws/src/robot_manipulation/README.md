# robot_manipulation

`robot_manipulation` 提供取物任务使用的 mock 抓取/放置执行器。它用于验证任务、导航、视觉和操作接口，不代表真实机械臂控制。

## 节点

```text
mock_pick_place_node
```

接口：

| 类型 | 名称 | 说明 |
|---|---|---|
| 订阅 | `/manipulation/command_text` | 接收 `PICK:<label>` 或 `PLACE` |
| 发布 | `/manipulation/status_text` | 发布抓取/放置状态 |
| 服务 | `/manipulation/mock_pick` | 查询当前异步抓取任务状态 |
| 服务 | `/manipulation/mock_place` | 释放当前 mock 持有物 |
| 客户端 | `/get_entity_state` | 查询 Gazebo 实体真实位姿 |

## PICK 命令

任务编排节点发送的命令格式：

```text
PICK:cup;locked_target_frame=map;locked_target_xyz=1.230,2.340,0.600
```

其中 `locked_target_xyz` 用于日志和任务追踪。最终成功/失败不直接使用该 YOLO 坐标，而是查询 Gazebo 实体位姿。

## 当前判定逻辑

收到 `PICK` 后：

1. 将标签规范化，例如 `mug`、`水杯` 映射为 `cup`。
2. 检查是否已经持有其他物体。
3. 在 `pick_delay_min_sec` 到 `pick_delay_max_sec` 之间随机生成等待时间，默认 2~5 秒。
4. timer 在等待期间直接返回，不阻塞 ROS executor。
5. 等待结束后，通过 Gazebo `/get_entity_state` 查询候选物体。
6. 优先通过 TF 获取机器人 `map -> base_footprint` 位姿；不可用时查询 Gazebo 机器人实体。
7. 计算机器人与物体的平面距离和朝向误差。
8. 根据阈值发布 `pick_success` 或 `pick_failed`。

默认候选实体：

| 标签 | Gazebo 候选实体 |
|---|---|
| `cup` | `coke_can`、`cup`、`beer` |
| `bottle` | `bottle`、`beer`、`coke_can` |
| `vase` | `vase`、`Vase_01_001` |

默认判定阈值：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `pick_min_distance_m` | `0.0` | 最小平面距离 |
| `pick_max_distance_m` | `1.5` | 最大平面距离 |
| `pick_max_yaw_error_deg` | `90.0` | 机器人朝向目标的最大误差 |
| `pick_delay_min_sec` | `2.0` | mock 动作最短等待 |
| `pick_delay_max_sec` | `5.0` | mock 动作最长等待 |
| `gazebo_query_timeout_sec` | `8.0` | 等待结束后的 Gazebo 查询窗口 |
| `pick_check_period_sec` | `0.15` | 异步检查周期 |

有意设置的 2~5 秒动作等待不计入 `gazebo_query_timeout_sec`。

## 状态文本

接收命令：

```text
[manipulation] pick_command_received label=cup mock_delay_sec=3.42 ...
```

成功：

```text
[manipulation] pick_success label=cup gazebo_entity=coke_can dist=0.62 ...
```

失败：

```text
[manipulation] pick_failed reason=too_far label=cup ...
```

常见失败原因：

- `already_holding`
- `entity_not_found`
- `robot_pose_unavailable`
- `gazebo_msgs_unavailable`
- `gazebo_query_timeout`
- `too_close`
- `too_far`
- `bad_yaw`
- `invalid_object_pose`

## 启动

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash

ros2 launch robot_bringup manipulation.launch.py \
  pick_delay_min_sec:=2.0 \
  pick_delay_max_sec:=5.0 \
  pick_max_distance_m:=1.5
```

完整系统：

```bash
bash scripts/run_full_system.sh
```

## 与返航的关系

本包只发布抓取结果，不执行返航。

`robot_tasks/object_fetch_orchestrator_node` 在接收任务时记录 `home_pose`，收到 `pick_success` 或 `pick_failed` 后都调用 Nav2 返回该位置。返航行为见 [robot_tasks/README.md](../robot_tasks/README.md)。

## 边界

当前不包含：

- MoveIt 轨迹规划
- 机械臂关节控制
- 夹爪力控
- 真实抓取姿态估计
- 抓取后物体附着到机器人模型

接入真实机械臂时，应尽量保留 `/manipulation/command_text` 和 `/manipulation/status_text` 契约，让上层任务状态机无需重写。
