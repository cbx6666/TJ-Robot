"""Reserved object detector wrapper.

Future object search should configure the generic YOLO node with target class ids
and publish object-level detections for the task layer.
"""


def main() -> None:
    from .yolo_detector_node import main as yolo_main

    yolo_main()
