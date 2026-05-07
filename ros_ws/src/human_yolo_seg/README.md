# human_yolo_seg

## 1. 包定位

`human_yolo_seg` 是视觉感知包，负责基于 YOLO 的人物/目标检测与可视化输出。  
当前主要服务于 RGBD 场景下的视觉感知验证。

## 2. 主要职责

- 启动 YOLO 检测节点
- 输出检测结果与标注图像
- 提供感知侧辅助工具

## 3. 依赖

Python 侧依赖可通过以下方式安装：

```bash
pip install -r requirements.txt
```

模型权重默认放置在 `models/` 目录，默认文件名为 `yolo26n-seg.pt`。

## 4. 目录说明

```text
human_yolo_seg/
├── launch/         感知 launch
├── models/         模型权重
├── human_yolo_seg/ Python 源码
├── requirements.txt
├── setup.py
├── package.xml
└── README.md
```

## 5. 常用入口

- `launch/yolo_person_seg.launch.py`
  - 启动 YOLO 检测链路

可执行节点包括：

- `yolo_person_seg_node`
- `person_detector_node`
- `object_detector_node`
- `yolo_person_watch`

## 6. 使用说明

通常由系统级 launch 或脚本间接调用。若需单独验证，可在工作区环境中使用：

```bash
ros2 launch human_yolo_seg yolo_person_seg.launch.py
```

## 7. 维护建议

- 模型权重与源码分离维护
- 感知算法逻辑优先保留在本包内部，不向 bringup 泄漏实现细节
