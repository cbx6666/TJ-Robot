# 模块接口设计（单源版）

所有实现与接口以 `ros_ws/src` 下 ROS 包为准。

## `human_yolo_seg`

输入：

- `/scan`
- `/camera/image_raw`
- `/camera/camera_info`

输出：

- `/human_yolo/annotated_image`
- `/human_yolo/person_azimuth_ranges`
- `/human_yolo/person_laser_map_cloud`
- `/scan_filtered`（可选）

地图后处理链路：

```text
before_strip map + person regions
-> strip_saved_map_person_free
-> after_strip map
```

## `robot_navigation`

输入地图、TF、scan 和目标位姿，输出 `/cmd_vel` 与导航状态。
核心节点在 `ros_ws/src/robot_navigation/robot_navigation/nodes`。

## `robot_tasks`

负责任务编排和状态管理，当前主节点在 `ros_ws/src/robot_tasks/robot_tasks/nodes`。

## `robot_bringup`

负责系统级 launch 编排、参数与仿真资源，路径为 `ros_ws/src/robot_bringup`。

## `robot_interfaces`

负责消息/接口定义，供跨包契约扩展使用。
