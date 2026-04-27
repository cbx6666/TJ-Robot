# 模块接口设计

## robot_perception

输入：

- `/scan`
- `/camera/image_raw`
- `/camera/camera_info`
- future audio stream

输出：

- `/human_yolo/annotated_image`
- `/human_yolo/person_azimuth_ranges`
- `/human_yolo/person_laser_map_cloud`
- `/interaction/speech_text`

YOLO 入口：

- `robot_perception/vision/yolo_detector_node.py`
- `robot_perception/vision/person_detector_node.py`
- `robot_perception/vision/object_detector_node.py`

## robot_mapping

SLAM 输入 `/scan` 或 `/scan_filtered`，输出 `/map` 和地图文件。

地图后处理在 `robot_mapping/strip/`：

```text
before_strip map + person regions
-> strip_saved_map_person_free
-> after_strip map
```

人物区域和动态对象语义信息在 `robot_mapping/semantic/`。

## robot_navigation

输入地图、TF、scan 和 room/pose goal，输出 `/cmd_vel` 和导航状态。当前真实实现仍主要在 `ros_ws/src/robot_navigation` 包。

## robot_interaction

语音链路：

```text
ASR -> speech_text -> NLU -> RobotTask -> task manager -> TTS/log
```

示例：

```text
去客厅找杯子
```

解析为：

```text
intent = search_object
target_object = cup
target_room = living_room
required_modules = robot_navigation + robot_perception + robot_tasks
```

## robot_tasks

职责：

- 接收 NLU 或实验脚本产生的任务。
- 调用 task planner 拆解动作。
- 分发给导航、感知、建图和地图后处理。
- 记录任务状态和结果。

## robot_experiments

按实验对象组织：

- `mapping`：baseline 和 semantic mapping。
- `navigation`：导航成功率、耗时、失败原因。
- `search`：目标搜索成功率。
- `metrics`：地图质量和日志工具。
