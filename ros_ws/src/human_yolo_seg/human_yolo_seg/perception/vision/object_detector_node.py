"""Generic object detector entry backed by the current YOLO implementation."""

from human_yolo_seg.nodes.yolo_object_seg_node import YoloObjectSegNode, main

__all__ = ["YoloObjectSegNode", "main"]
