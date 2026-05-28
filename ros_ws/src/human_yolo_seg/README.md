# human_yolo_seg

## 1. 包定位

`human_yolo_seg` 是视觉感知包，基于 YOLO 做**多类物体**检测、3D 定位与地图可视化（默认 COCO：瓶/杯/花瓶，可配）。  
当前主要服务于 RGBD 场景下的视觉感知验证。

## 2. 主要职责

- 启动 YOLO 检测节点
- 输出检测结果与标注图像
- 基于深度图与相机内参反投影目标 3D 点（相机坐标系）
- 通过 TF2 转换并发布目标在 `map` 坐标系下的位置
- 提供同类目标的跨帧关联与追踪（默认开启）
- 提供感知侧辅助工具

## 3. 依赖

Python 侧依赖可通过以下方式安装：

```bash
pip install -r requirements.txt
```

模型权重默认放置在 `models/` 目录，默认文件名为 `yolo26n-seg.pt`。  
launch 默认 `target_class_ids_csv:=39,41,75`（瓶/杯/花瓶；仿真可乐罐多标为 bottle 或 cup）。`all` 为全 COCO（更重）。脚本 `tb3_stack.sh` 可通过 **`YOLO_TARGET_CLASS_IDS`** 覆盖。

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

- `launch/yolo_object_seg.launch.py`：**主入口**（推荐）
- `launch/yolo_person_seg.launch.py`：兼容别名，等价于上层 `IncludeLaunch`，行为与上一致。

可执行名（节选）：

- `yolo_object_seg_node`（主）
- `yolo_person_seg_node`：`yolo_object_seg_node` 的兼容别名（同一实现）
- `object_detector_node` / `yolo_detector_node`
- `person_detector_node`：历史别名，等价于上述实现
- `yolo_object_watch` / `yolo_person_watch`（后者为兼容别名）
## 6. 使用说明

通常由系统级 launch 或脚本间接调用。若需单独验证，可在工作区环境中使用：

```bash
ros2 launch human_yolo_seg yolo_object_seg.launch.py
# 推荐（仿真双相机）：注册深度到 RGB 后再做 3D
ros2 launch human_yolo_seg yolo_object_seg.launch.py enable_depth_register:=true
# 等价（旧文件名）：
ros2 launch human_yolo_seg yolo_person_seg.launch.py
```

关键输出话题（默认前缀 **`/yolo_objects`**，`stats_topic_prefix` 可调）：

- `/yolo_objects/target_objects_marker`（`visualization_msgs/MarkerArray`）：地图上**多目标**球体与文字标签
- `/yolo_objects/target_point_camera` (`geometry_msgs/PointStamped`)
- `/yolo_objects/target_point_map` (`geometry_msgs/PointStamped`，主目标单点)
- `/yolo_objects/target_map_valid` (`std_msgs/Bool`)：当前是否有有效主目标；无目标时为 `false`（避免 RViz 绿点/旧坐标残留）
- `/yolo_objects/target_label` (`std_msgs/String`)
- `/yolo_objects/desired_object_label` (`std_msgs/String`)：语音/任务语义（如 `cup`、`杯子`）。由 `command_executor` 在 `fetch_object` 时写入、`stop` 时清空；YOLO 仅在对应 COCO 类 track 中选**视野最近**目标作为绿点。
- `/yolo_objects/detection_*`：`detection_count`、`detection_max_conf`、`detection_present`
常用参数（3D 坐标转换）：

- `depth_topic`：深度图（仿真默认 `/tb3_depth_only/depth/image_raw`）
- `use_depth_sync`：是否用 `message_filters` 同步 RGB+Depth（默认 `true`，对齐 yolo_ros）
- `depth_sample_stat`：ROI 深度统计 `median`（默认）| `trimmed_mean` | `min`
- `depth_pixels_aligned`：深度已与 RGB 配准时设 `true`（如 `enable_depth_register:=true`）
- `enable_depth_register`：启动 `depth_image_proc/register` 并自动切换注册深度话题
- `target_frame_id`：目标发布坐标系（默认 `map`）
- `depth_unit_divisor`：16UC1 深度换算系数（毫米相机常用 `1000.0`）
- `max_depth_age_sec`：仅 `use_depth_sync:=false` 时生效
- `enable_target_tracking`：是否开启目标关联追踪
- `target_class_ids_csv` / `target_class_ids`：类别过滤（csv 非空时优先）
- `publish_target_markers`：是否发布地图多目标 MarkerArray
- `max_targets_3d_per_frame`：每帧参与 3D 计算的最大候选数（用于控算力）
- `min_track_hits_for_publish`：追踪命中次数达到阈值后才输出主目标与 Marker

## 7. 维护建议

- 模型权重与源码分离维护
- 感知算法逻辑优先保留在本包内部，不向 bringup 泄漏实现细节
