#!/usr/bin/env python3
"""在官方 turtlebot3_waffle.urdf 末尾、</robot> 前插入一个或多个 Gazebo 片段（XML 文本）。

部分 ROS 发行版里，xacro 展开后的 waffle URDF 已自带 diff_drive / 激光 / IMU 等 Gazebo 插件。
若再拼接 waffle_sim_plugins_gazebo_fragment.xml，会出现**双差速**等重复插件，表现为无故自转、抖动。
因此在合并前会删掉 base 中与下列插件同名的 <gazebo>...</gazebo> 整块。
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

# 与 waffle_sim_plugins_gazebo_fragment.xml 中即将注入的插件一致；只剥这些，避免误删 RGB 相机等
_DUPLICATE_PLUGINS = (
    "libgazebo_ros_diff_drive.so",
    "libgazebo_ros_joint_state_publisher.so",
    "libgazebo_ros_ray_sensor.so",
    "libgazebo_ros_imu_sensor.so",
)


def strip_duplicate_gazebo_plugin_blocks(urdf: str) -> tuple[str, int]:
    pattern = re.compile(r"<gazebo\b[^>]*>.*?</gazebo>", re.DOTALL)
    out: list[str] = []
    pos = 0
    removed = 0
    for m in pattern.finditer(urdf):
        out.append(urdf[pos : m.start()])
        block = m.group(0)
        if any(p in block for p in _DUPLICATE_PLUGINS):
            removed += 1
        else:
            out.append(block)
        pos = m.end()
    out.append(urdf[pos:])
    return "".join(out), removed


def main() -> int:
    p = argparse.ArgumentParser(
        description="合并 xacro 展开后的 URDF 与 Gazebo 片段（先仿真插件，后深度等）。",
    )
    p.add_argument("base_urdf", type=pathlib.Path, help="xacro 展开后的 turtlebot3_waffle.urdf")
    p.add_argument(
        "fragments",
        type=pathlib.Path,
        nargs="+",
        help="按顺序拼接的片段文件（如 waffle_sim_plugins + waffle_depth）",
    )
    p.add_argument("out_urdf", type=pathlib.Path, help="输出合并后的 URDF")
    args = p.parse_args()

    base = args.base_urdf.read_text(encoding="utf-8")
    base, n_stripped = strip_duplicate_gazebo_plugin_blocks(base)
    if n_stripped:
        print(
            f"NOTE: 从 base URDF 移除了 {n_stripped} 个已含 "
            f"diff_drive/激光/IMU/joint_state 的 <gazebo> 块，避免与片段重复",
            file=sys.stderr,
        )
    chunks: list[str] = []
    for frag_path in args.fragments:
        chunks.append(frag_path.read_text(encoding="utf-8").rstrip())
    frag = "\n".join(chunks)
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
