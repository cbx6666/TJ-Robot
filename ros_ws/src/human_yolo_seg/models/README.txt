human_yolo_seg 模型目录说明
============================

1. 目录用途

本目录用于保存 `human_yolo_seg` 包运行所需的本地模型权重文件。

2. 默认文件

默认权重文件名为：

  yolo26n-seg.pt

3. 使用方式

将权重文件放入本目录后，可重新构建工作区：

  cd ros_ws
  colcon build --packages-select human_yolo_seg

构建完成后，节点默认会从以下安装路径加载权重：

  share/human_yolo_seg/models/yolo26n-seg.pt

也可以在启动时通过参数显式指定绝对路径，例如：

  ros2 launch human_yolo_seg yolo_object_seg.launch.py model_path:=/absolute/path/yolo26n-seg.pt

4. 验证建议

建议在仿真中核对（默认话题前缀 `/yolo_objects`）：

- `/yolo_objects/detection_present`
- `/yolo_objects/detection_count`
- `/yolo_objects/detection_max_conf`
- `/yolo_objects/annotated_image`

必要时可使用：

  ros2 run human_yolo_seg yolo_object_watch

对检测结果进行终端观察（兼容别名 `yolo_person_watch`）。
