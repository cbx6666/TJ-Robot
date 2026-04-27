"""Person-region map stripping entrypoint."""


def main() -> None:
    from human_yolo_seg.strip_saved_map import main as legacy_main

    legacy_main()


if __name__ == "__main__":
    main()
