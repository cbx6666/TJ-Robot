# robot_navigation

当前包只保留两条维护中的控制方案：

- `point_to_point`
- `coverage_patrol`

## point_to_point

`point_to_point` 是一个最小闭环控制节点，用于验证底盘控制是否正常。

功能：

- 订阅 `/odom`
- 发布 `/cmd_vel`
- 先对准目标方向
- 再向目标点前进
- 可选在到点后对齐最终朝向

示例：

```bash
cd /mnt/d/Homework/robot/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_navigation point_to_point --ros-args \
  -p goal_x:=1.0 \
  -p goal_y:=0.0
```

## coverage_patrol

`coverage_patrol` 是当前项目用于房间扫描的主方案。
它不依赖 Nav2，而是按照预定义航点覆盖房间边缘、中线和人物区域。

### 与 YOLO 的关系

当前项目要求 `coverage_patrol` 与 YOLO 一起运行：

- `tb3_stack.sh` 启动 YOLO 与 `scan_person_filter`
- `slam_toolbox` 使用 `/scan_filtered` 建图
- `coverage_patrol` 继续使用原始 `/scan` 做近距离避障

这样做的原因是：

- 地图里要尽量过滤掉人
- 但车体近距离避障不能把人和动态障碍物一并过滤掉

### 设计思路

- 使用固定覆盖路线，避免 frontier 方案在当前场景中反复抖动
- 边缘和角点航点默认向房间内部收缩，减少贴墙卡死
- 前方有障碍时做轻量反应式避障
- 如果同一个目标长时间没有实质进展，会自动插入朝房间内部的 `detour` 脱困点
- 如果脱困点本身也没有进展，则跳过该恢复点继续执行后续航点

### 推荐启动流程

先启动带 YOLO 的主栈：

```bash
cd /mnt/d/Homework/robot
TB3_STACK_MODE=assist TURTLEBOT3_MODEL=waffle TB3_ASSIST_SCAN_FILTER=1 bash scripts/tb3_stack.sh start
```

再启动覆盖巡航：

```bash
cd /mnt/d/Homework/robot/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation coverage_patrol
```

或使用 launch：

```bash
ros2 launch robot_bringup coverage_patrol.launch.py
```

### 常用参数

- `edge_margin`
  沿墙航点离墙的内缩距离。

- `corner_margin`
  角点离墙的内缩距离。

- `linear_speed_limit`, `angular_speed_limit`
  巡航速度上限。

- `obstacle_distance`
  触发避障的前向距离阈值。

- `goal_stuck_sec`
  同一目标多久没有实质进展就触发脱困。

- `detour_step`
  脱困点朝房间内部移动的距离。

- `include_people_passes`
  是否保留经过 walking actor 和 standing person 区域的航点。

### 示例

更激进一些的速度：

```bash
ros2 run robot_navigation coverage_patrol --ros-args \
  -p linear_speed_limit:=0.35 \
  -p angular_speed_limit:=1.6
```

更容易脱离边缘障碍：

```bash
ros2 run robot_navigation coverage_patrol --ros-args \
  -p goal_stuck_sec:=2.5 \
  -p detour_step:=1.8
```
