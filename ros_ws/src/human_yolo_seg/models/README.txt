将本地权重放在本目录，例如：
  yolo26n-seg.pt

保存后执行：cd ros_ws && colcon build --packages-select human_yolo_seg
安装后节点默认会加载 share/human_yolo_seg/models/yolo26n-seg.pt

也可用绝对路径启动：ros2 launch ... model_path:=/完整路径/yolo26n-seg.pt

----------------------------------------------------------------------
【给做 Gazebo 人物模型 / 贴图的同学：怎么验收「能被 YOLO 当成人」】

本节点用的是 COCO 的 person 类别（日常照片里那种人形），不是「任意人形网格」
都一定能过。验收看的是：机器人相机拍到的 RGB 画面里，是否像「可见的人」。

推荐流程（仿真已开、TB3_STACK_MODE=assist 或单独起了 yolo_person_seg）：

1) 终端监听（最简单）
   source /opt/ros/humble/setup.bash && source ros_ws/install/setup.bash
   ros2 topic echo /human_yolo/person_present   # true = 当前帧检出 person
   ros2 topic echo /human_yolo/person_count     # 检出框数量
   ros2 topic echo /human_yolo/person_max_conf  # 最高置信度 0~1

2) 人类可读 Watch（每秒一行中文说明）
   ros2 run human_yolo_seg yolo_person_watch

3) 图像确认（最直观）
   RViz：Add → Image → /human_yolo/annotated_image（Reliability 可设 Best effort）
   有分割/框叠在人物上即说明模型在该视角、该光照下可被当前权重识别。

若长期 person_present=false：调整人物在相机中的大小与角度、场景光照、材质颜色；
或略降低 launch 参数 conf_threshold；仍不行则需换更大分割模型（model_path）。
