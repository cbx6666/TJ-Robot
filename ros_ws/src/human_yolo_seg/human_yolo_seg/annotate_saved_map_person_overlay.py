# pyright: reportMissingImports=false
"""在已保存的 SLAM 地图上叠加 person_strip_recorder 记录的人物区域，输出彩色 PNG。

PGM/YAML 仍为 Nav2 用的单通道占用栅格；本工具**不修改**它们，只生成便于肉眼看对齐的
`*_person_overlay.png`（半透明洋红圆 + 中心十字），用于核对 YOLO 角域 + 激光→map 是否与 SLAM 一致。

用法示例::

  ros2 run human_yolo_seg annotate_saved_map_person_overlay \\
    saved_maps/coverage_map_20260109.yaml ~/.ros/tj_person_strip_regions.yaml
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from human_yolo_seg.strip_saved_map import _load_map_meta, _world_to_uv


def _parse_bgr(s: str) -> tuple[int, int, int]:
    parts = [int(x.strip()) for x in s.split(",")]
    if len(parts) != 3:
        raise ValueError("需为 B,G,R 三个 0–255 整数（OpenCV 顺序）")
    return tuple(max(0, min(255, p)) for p in parts)  # type: ignore[return-value]


def build_person_overlay_bgr(
    gray: np.ndarray,
    regions: list,
    res: float,
    ox: float,
    oy: float,
    flip_y: bool,
    *,
    circle_bgr: tuple[int, int, int],
    fill_alpha: float,
    draw_outline: bool,
    cross_size: int,
) -> np.ndarray:
    """gray: HxW uint8 occupancy; returns HxWx3 BGR."""
    base = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    h = gray.shape[0]
    blend_layer = np.zeros_like(base)
    pr_valid: list[tuple[int, int, int]] = []

    for r in regions:
        if not isinstance(r, dict):
            continue
        wx = float(r["x"])
        wy = float(r["y"])
        rad = float(r.get("radius", 0.08))
        u, v = _world_to_uv(wx, wy, ox, oy, res, h, flip_y)
        if not (0 <= u < base.shape[1] and 0 <= v < base.shape[0]):
            continue
        px_radius = max(1, int(round(rad / res)))
        pr_valid.append((u, v, px_radius))
        cv2.circle(blend_layer, (u, v), px_radius, circle_bgr, thickness=-1)

    alpha = max(0.0, min(1.0, float(fill_alpha)))
    out = cv2.addWeighted(base, 1.0 - alpha, blend_layer, alpha, 0.0)

    cross_bgr = (0, 255, 255)  # 黄十字，在深浅栅格上都较显眼
    for u, v, px_radius in pr_valid:
        if draw_outline:
            cv2.circle(out, (u, v), px_radius, circle_bgr, thickness=max(1, px_radius // 8))
        sz = max(3, int(cross_size))
        cv2.drawMarker(out, (u, v), cross_bgr, markerType=cv2.MARKER_CROSS, markerSize=sz, thickness=2)

    return out


def build_gray_person_composite_bgr(
    gray: np.ndarray,
    regions: list,
    res: float,
    ox: float,
    oy: float,
    flip_y: bool,
    *,
    person_bgr: tuple[int, int, int],
    dot_radius_px: int,
) -> np.ndarray:
    """灰度图转 BGR 底图，在人物区域中心画实心小圆点（导出「灰度+人点」彩图）。"""
    base = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    h, w = gray.shape[:2]
    rpx = max(1, int(dot_radius_px))
    for reg in regions:
        if not isinstance(reg, dict):
            continue
        wx = float(reg['x'])
        wy = float(reg['y'])
        u, v = _world_to_uv(wx, wy, ox, oy, res, h, flip_y)
        if 0 <= u < w and 0 <= v < h:
            cv2.circle(base, (u, v), rpx, person_bgr, thickness=-1)
    return base


def main() -> None:
    ap = argparse.ArgumentParser(
        description="在 SLAM 地图 PNG 上叠加人物区域（YOLO+激光→map 调试用，不修改 PGM）"
    )
    ap.add_argument("map_yaml", type=Path, help="map_saver 生成的 map.yaml")
    ap.add_argument(
        "regions_yaml",
        type=Path,
        help="person_strip_recorder 输出的 tj_person_strip_regions.yaml",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="输出 PNG 路径（默认与地图同目录：<stem>_person_overlay.png）",
    )
    ap.add_argument(
        "--no-flip-y",
        action="store_true",
        help="与 strip_saved_map_person_free 相同：不翻转 Y 轴时加此选项",
    )
    ap.add_argument(
        "--circle-bgr",
        type=str,
        default="255,0,255",
        help="叠加圆颜色 B,G,R（OpenCV），默认 255,0,255 洋红",
    )
    ap.add_argument(
        "--fill-alpha",
        type=float,
        default=0.42,
        help="人物圆半透明叠加强度 0–1（默认 0.42）",
    )
    ap.add_argument("--no-outline", action="store_true", help="不画圆轮廓线")
    ap.add_argument("--cross-size", type=int, default=11, help="中心十字尺寸（像素）")
    ap.add_argument(
        "--also-composite",
        action="store_true",
        help="另存 <stem>_gray_person_composite.png：灰度底图 + 人物色实心点（无半透明）",
    )
    ap.add_argument(
        "--composite-dot-radius",
        type=int,
        default=3,
        help="合成彩图上人物点半径（像素，默认 3）",
    )
    args = ap.parse_args()

    map_yaml = args.map_yaml.resolve()
    regions_path = args.regions_yaml.expanduser().resolve()

    if not regions_path.is_file():
        print(f"跳过：未找到人物区域文件 {regions_path}")
        return

    with regions_path.open(encoding="utf-8") as f:
        regions_doc = yaml.safe_load(f)
    if not isinstance(regions_doc, dict) or "regions" not in regions_doc:
        print(f"跳过：{regions_path} 格式无效（需含 regions）")
        return
    regions = regions_doc["regions"]
    if not isinstance(regions, list):
        print("跳过：regions 不是列表")
        return

    pgm_path, res, ox, oy, _w, _h = _load_map_meta(map_yaml)
    gray = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(pgm_path)

    flip_y = not bool(args.no_flip_y)
    circle_bgr = _parse_bgr(args.circle_bgr)
    bgr = build_person_overlay_bgr(
        gray,
        regions,
        res,
        ox,
        oy,
        flip_y,
        circle_bgr=circle_bgr,
        fill_alpha=args.fill_alpha,
        draw_outline=not args.no_outline,
        cross_size=args.cross_size,
    )

    legend = f"person regions: {len(regions)} (YOLO sector + laser -> map)"
    cv2.putText(
        bgr,
        legend,
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (40, 40, 255),
        2,
        cv2.LINE_AA,
    )

    if args.output is not None:
        out_path = args.output.resolve()
    else:
        out_path = map_yaml.parent / f"{map_yaml.stem}_person_overlay.png"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), bgr):
        raise OSError(f"cv2.imwrite 失败: {out_path}")
    print(f"写入 {out_path}（共 {len(regions)} 个区域；若圆与墙错位可试 --no-flip-y）")

    if args.also_composite:
        comp = map_yaml.parent / f"{map_yaml.stem}_gray_person_composite.png"
        comp_bgr = build_gray_person_composite_bgr(
            gray,
            regions,
            res,
            ox,
            oy,
            flip_y,
            person_bgr=circle_bgr,
            dot_radius_px=args.composite_dot_radius,
        )
        cv2.putText(
            comp_bgr,
            f"gray map + person dots (n={len(regions)})",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (40, 40, 255),
            2,
            cv2.LINE_AA,
        )
        if not cv2.imwrite(str(comp), comp_bgr):
            raise OSError(f"cv2.imwrite 失败: {comp}")
        print(f"写入 {comp}")


if __name__ == "__main__":
    main()
