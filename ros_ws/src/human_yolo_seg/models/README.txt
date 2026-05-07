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

  ros2 launch human_yolo_seg yolo_person_seg.launch.py model_path:=/absolute/path/yolo26n-seg.pt

4. 验证建议

若需要验证人物模型在仿真中的检测效果，可重点检查：

- `/human_yolo/person_present`
- `/human_yolo/person_count`
- `/human_yolo/person_max_conf`
- `/human_yolo/annotated_image`

必要时可使用：

  ros2 run human_yolo_seg yolo_person_watch

对检测结果进行终端观察。
