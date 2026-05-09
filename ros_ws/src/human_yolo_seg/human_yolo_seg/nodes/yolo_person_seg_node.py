# pyright: reportMissingImports=false
"""兼容旧模块名：实现已迁至 yolo_object_seg_node。"""
from human_yolo_seg.nodes.yolo_object_seg_node import YoloObjectSegNode, main as main

YoloPersonSegNode = YoloObjectSegNode

__all__ = ["YoloObjectSegNode", "YoloPersonSegNode", "main"]
