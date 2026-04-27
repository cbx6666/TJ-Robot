# pyright: reportMissingImports=false
"""根据 person_strip_recorder 生成的 YAML，把已保存地图 PGM 中对应圆域刷成空闲像素。

坐标约定与 map_server / slam_toolbox 常见 PGM 一致：origin 为地图左下角在世界系中的位置，
图像第 0 行对应较大世界 y（opencv 行与世界 y 反向），默认 --flip-y。"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def _load_map_meta(map_yaml_path: Path) -> tuple[Path, float, float, float, int, int]:
    with map_yaml_path.open(encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    if not isinstance(meta, dict):
        raise ValueError(f"无效的 map yaml: {map_yaml_path}")
    img_name = meta.get("image")
    if not img_name:
        raise ValueError("map yaml 缺少 image 字段")
    res = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    ox, oy = float(origin[0]), float(origin[1])
    pgm_path = map_yaml_path.parent / str(img_name)
    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"无法读取 PGM: {pgm_path}")
    h, w = img.shape[:2]
    return pgm_path, res, ox, oy, w, h


def _world_to_uv(
    wx: float,
    wy: float,
    ox: float,
    oy: float,
    res: float,
    height: int,
    flip_y: bool,
) -> tuple[int, int]:
    mx = (wx - ox) / res
    my = (wy - oy) / res
    u = int(round(mx))
    if flip_y:
        v = height - 1 - int(round(my))
    else:
        v = int(round(my))
    return u, v


def _strip_regions(
    img: np.ndarray,
    regions: list,
    res: float,
    ox: float,
    oy: float,
    flip_y: bool,
    free_value: int,
) -> np.ndarray:
    out = img.copy()
    h = out.shape[0]
    for r in regions:
        if not isinstance(r, dict):
            continue
        wx = float(r["x"])
        wy = float(r["y"])
        rad = float(r.get("radius", 0.5))
        u, v = _world_to_uv(wx, wy, ox, oy, res, h, flip_y)
        px_radius = max(1, int(round(rad / res)))
        cv2.circle(out, (u, v), px_radius, int(free_value), thickness=-1)
    return out


def _write_pgm(path: Path, img: np.ndarray) -> None:
    ok = cv2.imwrite(str(path), img)
    if not ok:
        raise OSError(f"cv2.imwrite 失败: {path}")


def _write_map_yaml(src_yaml: Path, dst_yaml: Path, new_image_name: str, meta: dict) -> None:
    out = dict(meta)
    out["image"] = new_image_name
    with dst_yaml.open("w", encoding="utf-8") as f:
        f.write("# 由 strip_saved_map_person_free 生成（人物区域已刷为空闲）\n")
        yaml.safe_dump(out, f, allow_unicode=True, default_flow_style=False, sort_keys=False)


def main() -> None:
    ap = argparse.ArgumentParser(description="根据人物区域 YAML 清理已保存地图中的障碍像素")
    ap.add_argument("map_yaml", type=Path, help="slam_toolbox / map_saver 生成的 map.yaml")
    ap.add_argument(
        "regions_yaml",
        type=Path,
        help="person_strip_recorder 输出的 tj_person_strip_regions.yaml",
    )
    ap.add_argument(
        "-o",
        "--output-prefix",
        type=Path,
        default=None,
        help="输出前缀（默认与 map 同目录，文件名为原名 + _person_free）",
    )
    ap.add_argument("--free-value", type=int, default=254, help="空闲像素灰度（常见 254）")
    ap.add_argument(
        "--no-flip-y",
        action="store_true",
        help="不按 ROS 惯例翻转图像行（默认会翻转）；清障圆错位时可试此项",
    )
    args = ap.parse_args()

    map_yaml = args.map_yaml.resolve()
    regions_path = args.regions_yaml.resolve()
    with regions_path.open(encoding="utf-8") as f:
        regions_doc = yaml.safe_load(f)
    if not isinstance(regions_doc, dict) or "regions" not in regions_doc:
        raise ValueError(f"regions yaml 格式错误（需含 regions 列表）: {regions_path}")
    regions = regions_doc["regions"]
    if not isinstance(regions, list):
        raise ValueError("regions 必须是列表")

    pgm_path, res, ox, oy, w, h = _load_map_meta(map_yaml)
    with map_yaml.open(encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(pgm_path)
    flip_y = not bool(args.no_flip_y)
    cleared = _strip_regions(img, regions, res, ox, oy, flip_y, args.free_value)

    if args.output_prefix is not None:
        prefix = args.output_prefix
        out_pgm = Path(str(prefix) + ".pgm")
        out_yaml = Path(str(prefix) + ".yaml")
    else:
        stem = map_yaml.stem + "_person_free"
        out_pgm = map_yaml.parent / f"{stem}.pgm"
        out_yaml = map_yaml.parent / f"{stem}.yaml"

    _write_pgm(out_pgm, cleared)
    _write_map_yaml(map_yaml, out_yaml, out_pgm.name, meta)
    print(f"写入 {out_pgm} 与 {out_yaml}（共 {len(regions)} 个圆域）")


if __name__ == "__main__":
    main()
