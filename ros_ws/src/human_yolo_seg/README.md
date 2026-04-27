# human_yolo_seg

当前阶段仅保留 YOLO 识别主链，用于 RGBD 路线下的视觉感知。

## 依赖

```bash
pip install -r requirements.txt
```

权重文件：放在 `models/`（默认 `yolo26n-seg.pt`），或通过参数 `model_path` 指向绝对路径。

## Launch

| 文件 | 作用 |
|------|------|
| `launch/yolo_person_seg.launch.py` | 启动 YOLO 识别节点（输出标注图像） |

## 可执行

| 命令 | 源文件 | 作用 |
|------|--------|------|
| `yolo_person_seg_node` | `human_yolo_seg/perception/vision/yolo_detector_node.py` | 主识别节点 |
| `person_detector_node` | `human_yolo_seg/perception/vision/person_detector_node.py` | 人检测适配 |
| `object_detector_node` | `human_yolo_seg/perception/vision/object_detector_node.py` | 目标检测适配 |
| `yolo_person_watch` | `human_yolo_seg/tools/yolo_person_watch.py` | 终端观察检测统计 |

## 目录结构（节选）

```text
human_yolo_seg/
  launch/yolo_person_seg.launch.py
  human_yolo_seg/
    perception/vision/
    tools/yolo_person_watch.py
  models/
  requirements.txt
  setup.py
  package.xml
```
