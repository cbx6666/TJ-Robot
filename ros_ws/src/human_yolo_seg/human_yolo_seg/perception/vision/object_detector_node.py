"""Generic object detector entry backed by the current YOLO implementation."""

from human_yolo_seg.nodes.yolo_person_seg_node import YoloPersonSegNode, main

__all__ = ["YoloPersonSegNode", "main"]
