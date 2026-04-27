"""Compatibility entry for person azimuth markers."""


def main() -> None:
    from human_yolo_seg.person_azimuth_markers_node import main as legacy_main

    legacy_main()
