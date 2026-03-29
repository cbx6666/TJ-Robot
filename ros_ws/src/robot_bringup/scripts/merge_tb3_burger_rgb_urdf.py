#!/usr/bin/env python3
"""在 xacro 展开后的 turtlebot3_burger.urdf 末尾插入 burger_rgb_camera.urdf.fragment（RGB+光学系 TF）。"""
from __future__ import annotations

import argparse
import pathlib
import sys


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("base_urdf", type=pathlib.Path)
    p.add_argument("fragment", type=pathlib.Path)
    p.add_argument("out_urdf", type=pathlib.Path)
    args = p.parse_args()

    base = args.base_urdf.read_text(encoding="utf-8")
    frag = args.fragment.read_text(encoding="utf-8").strip()
    token = "</robot>"
    if token not in base:
        print("ERROR: base URDF 中未找到 </robot>", file=sys.stderr)
        return 1
    head, _sep, _tail = base.rpartition(token)
    merged = head.rstrip() + "\n" + frag + "\n" + token + "\n"
    args.out_urdf.parent.mkdir(parents=True, exist_ok=True)
    args.out_urdf.write_text(merged, encoding="utf-8")
    print(f"Wrote {args.out_urdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
