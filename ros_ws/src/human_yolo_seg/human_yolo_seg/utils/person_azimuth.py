# pyright: reportMissingImports=false
"""兼容旧导入名：`person_azimuth` 已迁至 `target_azimuth`。"""
from human_yolo_seg.utils.target_azimuth import boxes_to_azimuth_data

__all__ = ["boxes_to_azimuth_data"]
