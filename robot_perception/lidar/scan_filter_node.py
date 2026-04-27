"""Compatibility entry for person-aware scan filtering."""


def main() -> None:
    from human_yolo_seg.scan_person_filter_node import main as legacy_main

    legacy_main()
