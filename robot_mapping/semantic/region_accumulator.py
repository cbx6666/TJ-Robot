"""Compatibility entry for accumulating person regions from scan+YOLO."""


def main() -> None:
    from human_yolo_seg.person_strip_recorder_node import main as legacy_main

    legacy_main()
