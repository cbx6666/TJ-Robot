"""历史可执行文件名 `person_detector_node`（仍可用）；实现与 object_detector_node / YoloObjectSegNode 一致。"""

from human_yolo_seg.nodes.yolo_object_seg_node import YoloObjectSegNode as YoloPersonSegNode, main

__all__ = ["YoloPersonSegNode", "main"]
