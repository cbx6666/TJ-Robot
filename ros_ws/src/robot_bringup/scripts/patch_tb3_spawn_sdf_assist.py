#!/usr/bin/env python3
"""Patch TurtleBot3 Gazebo spawn SDF for assist stack.

1) Optional: remove all <collision> under link ``camera_rgb_frame`` — official waffle
   camera box often sits in the same horizontal band as the LDS, causing a forward
   “dead sector” in /scan.
2) Optional: set hls_lfcd_lds ray range Gaussian noise stddev (default sim uses 0.01 m).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def strip_link_collisions(sdf_text: str, link_name: str) -> tuple[str, int]:
    rx = re.compile(
        rf'(<link\s+name="{re.escape(link_name)}"[^>]*>)([\s\S]*?)(</link>)',
        flags=re.MULTILINE,
    )
    m = rx.search(sdf_text)
    if not m:
        return sdf_text, 0
    inner = m.group(2)
    inner2, k = re.subn(r"<collision\b[\s\S]*?</collision>\s*", "", inner, flags=re.MULTILINE)
    if k == 0:
        return sdf_text, 0
    repl = m.group(1) + inner2 + m.group(3)
    return sdf_text[: m.start()] + repl + sdf_text[m.end() :], k


def patch_laser_ray_noise(sdf_text: str, sensor_name: str, new_stddev: str) -> tuple[str, int]:
    rx = re.compile(
        rf'(<sensor\b[^>]*name="{re.escape(sensor_name)}"[^>]*type="ray"[^>]*>)'
        rf"([\s\S]*?)"
        rf"(</sensor>)",
        flags=re.MULTILINE,
    )
    m = rx.search(sdf_text)
    if not m:
        return sdf_text, 0
    block = m.group(2)
    sub_rx = re.compile(
        r"(<noise>\s*<type>gaussian</type>\s*<mean>0\.0</mean>\s*<stddev>)([^<]+)(</stddev>\s*</noise>)",
        flags=re.MULTILINE,
    )
    block2, n = sub_rx.subn(rf"\g<1>{new_stddev}\g<3>", block, count=1)
    if n == 0:
        return sdf_text, 0
    out = sdf_text[: m.start()] + m.group(1) + block2 + m.group(3) + sdf_text[m.end() :]
    return out, n


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("sdf_file", type=Path)
    p.add_argument(
        "--laser-noise-stddev",
        default="",
        help="If set, patch first gaussian range noise (e.g. 0). Empty = skip.",
    )
    p.add_argument(
        "--strip-camera-rgb-collisions",
        action="store_true",
        help="Remove collision shapes on camera_rgb_frame (reduces LDS self-occlusion).",
    )
    args = p.parse_args()

    path = args.sdf_file
    if not path.is_file():
        print(f"ERROR: not a file: {path}", file=sys.stderr)
        return 1

    text = path.read_text(encoding="utf-8", errors="ignore")
    orig = text

    removed = 0
    if args.strip_camera_rgb_collisions:
        text, removed = strip_link_collisions(text, "camera_rgb_frame")

    laser_n = 0
    dev = args.laser_noise_stddev.strip()
    if dev:
        text, laser_n = patch_laser_ray_noise(text, "hls_lfcd_lds", dev)

    if text != orig:
        path.write_text(text, encoding="utf-8")
    print(f"STRIPPED_COLLISIONS={removed} LASER_NOISE_PATCHES={laser_n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
