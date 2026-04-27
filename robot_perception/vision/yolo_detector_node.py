"""Generic YOLO detector entry.

The existing implementation is person-oriented but already exposes class ids.
This wrapper gives the project a generic vision-layer location while keeping the
current ROS package entry intact.
"""


def main() -> None:
    from human_yolo_seg.yolo_person_seg_node import main as legacy_main

    legacy_main()
