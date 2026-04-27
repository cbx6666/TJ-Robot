# human_yolo_seg

ROS 2 包：**RGB 上 YOLO-Seg 检人**，估计 **激光系人物方位角区间**，并与 **LaserScan** 结合实现 **实时滤扫**、**建图后 strip**、**RViz 可视化** 等。

## 依赖（pip，与 ros2 同一 Python）

```bash
pip install -r requirements.txt
```

权重：将 `.pt` 放在 `models/`（默认 `yolo26n-seg.pt`），或 `yolo_person_seg_node` 参数 `model_path` 指绝对路径。`models/*.pt` 默认 **git 忽略**。

## Launch

| 文件 | 作用 |
|------|------|
| `launch/yolo_person_seg.launch.py` | 主入口：可选 `yolo_person_seg_node`、`person_strip_recorder`、`scan_person_filter`、`scan_map_colored_cloud`、`person_azimuth_markers` 等；由 `tb3_stack.sh` 调用 |

常用参数见 launch 内 `DeclareLaunchArgument`（如 `person_slam_mode`、`enable_scan_map_colored`、`person_azimuth_mode`）。

## 可执行节点与 CLI（`setup.py` → `ros2 run`）

| 命令 | 源文件 | 作用 |
|------|--------|------|
| `yolo_person_seg_node` | `human_yolo_seg/nodes/yolo_person_seg_node.py` | 订阅 RGB/CameraInfo，发布标注图、人物角域 `JointState` |
| `scan_person_filter_node` | `human_yolo_seg/nodes/scan_person_filter_node.py` | `/scan` → `/scan_filtered`（角域内置无效） |
| `person_strip_recorder_node` | `human_yolo_seg/nodes/person_strip_recorder_node.py` | 角域内激光投 map，写 `~/.ros/tj_person_strip_regions.yaml`，可选点云 |
| `person_azimuth_markers_node` | `human_yolo_seg/nodes/person_azimuth_markers_node.py` | RViz 扇形 Marker |
| `scan_map_colored_cloud_node` | `human_yolo_seg/nodes/scan_map_colored_cloud_node.py` | 整帧 scan 投 map 上色点云 |
| `azimuth_union_node` | `human_yolo_seg/nodes/azimuth_union_node.py` | 多路角域合并为一路 |
| `yolo_person_watch` | `human_yolo_seg/tools/yolo_person_watch.py` | 订阅检测统计话题，终端简显 |
| `strip_saved_map_person_free` | `human_yolo_seg/tools/strip_saved_map.py` | **离线**：按 regions YAML 把保存地图中人区域刷空闲 |
| `annotate_saved_map_person_overlay` | `human_yolo_seg/tools/annotate_saved_map_person_overlay.py` | 生成 `*_person_overlay.png` 或 `--composite-only` 的 `*_gray_person_composite.png` |
| `snapshot_person_regions_from_cloud` | `human_yolo_seg/tools/snapshot_person_regions_from_cloud.py` | 从点云话题拉一帧生成 regions YAML |

## 核心库模块（被节点 import）

| 文件 | 作用 |
|------|------|
| `utils/person_azimuth.py` | `linear_fov` / `tf_geometry` 角域计算，`boxes_to_azimuth_data` |
| `utils/person_scan_sync_utils.py` | `norm_angle`、`angle_in_interval`、`merge_intervals`、时间戳工具 |

## 目录结构（节选）

```text
human_yolo_seg/
  launch/yolo_person_seg.launch.py
  human_yolo_seg/
    nodes/
      yolo_person_seg_node.py
      scan_person_filter_node.py
      person_strip_recorder_node.py
      person_azimuth_markers_node.py
      scan_map_colored_cloud_node.py
      azimuth_union_node.py
    tools/
      strip_saved_map.py
      annotate_saved_map_person_overlay.py
      snapshot_person_regions_from_cloud.py
      yolo_person_watch.py
    utils/
      person_azimuth.py
      person_scan_sync_utils.py
  models/          # .pt 放这里（大文件不入库）
  requirements.txt
  setup.py
  package.xml
```

全仓库功能总索引：**仓库根目录 `docs/代码与功能索引.md`**。
