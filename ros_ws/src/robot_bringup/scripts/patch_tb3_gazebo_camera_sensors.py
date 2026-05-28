#!/usr/bin/env python3
"""Patch TurtleBot3 Gazebo Classic SDF: RGB (type=camera) and depth (type=depth) sensors.

Used by scripts/tb3_stack.sh for simulation-friendly resolution and matched update_rate.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

_SENSOR_RE = re.compile(
    r"(<sensor\b[^>]*\btype=['\"](?P<type>camera|depth)['\"][^>]*>)(?P<body>.*?)(</sensor>)",
    flags=re.S,
)


def _sub1(body: str, pattern: str, repl: str, *, insert: str | None = None) -> str:
    if re.search(pattern, body, flags=re.S):
        return re.sub(pattern, repl, body, count=1, flags=re.S)
    if insert is not None:
        return insert + body
    return body


def _patch_image_size(body: str, width: int, height: int) -> str:
    if re.search(r"<image\b", body, flags=re.S):
        block = re.search(r"(<image\b[^>]*>)(.*?)(</image>)", body, flags=re.S)
        if block:
            head, inner, tail = block.groups()
            inner = _sub1(inner, r"<width>.*?</width>", f"<width>{width}</width>")
            inner = _sub1(inner, r"<height>.*?</height>", f"<height>{height}</height>")
            return body[: block.start()] + head + inner + tail + body[block.end() :]
    return _sub1(
        body,
        r"(<camera\b[^>]*>)",
        rf"\1\n          <image>\n            <width>{width}</width>\n            <height>{height}</height>\n          </image>",
        insert=None,
    )


def _patch_sensor_body(
    body: str,
    *,
    enabled: bool,
    always_on: bool,
    rate: str,
    width: int,
    height: int,
    sensor_type: str,
) -> str:
    patched = body
    ao = "true" if (enabled and always_on) else "false"
    hz = rate if enabled else "0"

    patched = _sub1(
        patched,
        r"<always_on>.*?</always_on>",
        f"<always_on>{ao}</always_on>",
        insert=f"\n        <always_on>{ao}</always_on>",
    )
    patched = _sub1(
        patched,
        r"<update_rate>.*?</update_rate>",
        f"<update_rate>{hz}</update_rate>",
        insert=f"\n        <update_rate>{hz}</update_rate>",
    )
    if enabled:
        patched = _patch_image_size(patched, width, height)
    if not enabled:
        patched = _sub1(
            patched,
            r"<visualize>.*?</visualize>",
            "<visualize>false</visualize>",
            insert="\n        <visualize>false</visualize>",
        )
    elif sensor_type == "depth":
        patched = _sub1(
            patched,
            r"<visualize>.*?</visualize>",
            "<visualize>false</visualize>",
            insert="\n        <visualize>false</visualize>",
        )
    return patched


def patch_sdf(
    text: str,
    *,
    kinds: set[str],
    enabled: bool,
    always_on: bool,
    rate: str,
    rgb_size: tuple[int, int],
    depth_size: tuple[int, int],
) -> tuple[str, int, int]:
    camera_n = 0
    depth_n = 0

    def repl(m: re.Match[str]) -> str:
        nonlocal camera_n, depth_n
        # 捕获组: 1=open, 2=type, 3=body, 4=</sensor>（勿用 group(3) 当 tail）
        head, stype, body, tail = m.group(1), m.group("type"), m.group("body"), m.group(4)
        if stype not in kinds:
            return m.group(0)
        if stype == "camera":
            camera_n += 1
            w, h = rgb_size
        else:
            depth_n += 1
            w, h = depth_size
        new_body = _patch_sensor_body(
            body,
            enabled=enabled,
            always_on=always_on,
            rate=rate,
            width=w,
            height=h,
            sensor_type=stype,
        )
        return head + new_body + tail

    return _SENSOR_RE.subn(repl, text)[0], camera_n, depth_n


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--rate", default="8")
    p.add_argument("--enable", choices=("0", "1"), default="1")
    p.add_argument("--always-on", choices=("0", "1"), default="1")
    p.add_argument("--rgb-width", type=int, default=640)
    p.add_argument("--rgb-height", type=int, default=480)
    p.add_argument("--depth-width", type=int, default=640)
    p.add_argument("--depth-height", type=int, default=480)
    p.add_argument(
        "--kinds",
        default="camera,depth",
        help="Comma-separated: camera, depth",
    )
    args = p.parse_args()

    if not args.input.is_file():
        print(f"ERROR: missing input {args.input}", file=sys.stderr)
        return 1

    kinds = {k.strip() for k in args.kinds.split(",") if k.strip()}
    if not kinds:
        print("ERROR: --kinds must include camera and/or depth", file=sys.stderr)
        return 1

    text = args.input.read_text(encoding="utf-8", errors="ignore")
    enabled = args.enable == "1"
    always_on = args.always_on == "1"
    new_text, cam_n, dep_n = patch_sdf(
        text,
        kinds=kinds,
        enabled=enabled,
        always_on=always_on,
        rate=str(args.rate).strip(),
        rgb_size=(args.rgb_width, args.rgb_height),
        depth_size=(args.depth_width, args.depth_height),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(new_text, encoding="utf-8")
    try:
        import xml.etree.ElementTree as ET

        ET.fromstring(new_text)
    except ET.ParseError as exc:
        print(f"ERROR: patched SDF is not well-formed XML: {exc}", file=sys.stderr)
        return 1
    print(f"UPDATED_CAMERA_SENSORS={cam_n}")
    print(f"UPDATED_DEPTH_SENSORS={dep_n}")
    print(f"WROTE={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
