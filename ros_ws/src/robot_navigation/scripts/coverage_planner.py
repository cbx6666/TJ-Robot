"""基于静态地图的覆盖式 waypoint 生成。

这里实现的是轻量工程版覆盖点生成：
1. 读取 Nav2 map.yaml 和 PGM 地图；
2. 提取自由栅格，并过滤掉离障碍物太近的点；
3. 用最远点采样选出覆盖全图的稀疏点集；
4. 用最近邻顺序串成巡航路线。

它不是完整探索算法，但能避免固定 waypoint 只覆盖地图一部分的问题。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from waypoint_utils import Waypoint


@dataclass(frozen=True)
class CoveragePlannerConfig:
    """覆盖式 waypoint 生成参数。"""

    enabled: bool = True
    sample_spacing_m: float = 1.10
    min_obstacle_distance_m: float = 0.45
    max_waypoints: int = 40
    start_x: float = 0.0
    start_y: float = 0.0


@dataclass(frozen=True)
class _MapInfo:
    """内部地图结构。

    free/blocked 使用一维数组保存，索引规则为 `row * width + col`；row 从地图底部开始。
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    free: list[bool]
    blocked: list[bool]


class CoveragePlanner:
    """从静态地图生成巡航 waypoint。"""

    def __init__(self, config: CoveragePlannerConfig) -> None:
        self._config = config

    def plan(self, map_yaml: str, fallback: list[Waypoint]) -> list[Waypoint]:
        """生成 waypoint 列表。

        地图不可读、配置关闭或没有可用点时返回 fallback，保证导航主流程不会因为覆盖规划
        失败而直接崩溃。
        """

        if not self._config.enabled or not map_yaml:
            return fallback
        try:
            info = self._load_map(map_yaml)
            points = self._sample_safe_points(info)
            selected = self._select_spread_points(
                points,
                self._config.max_waypoints,
                self._config.start_x,
                self._config.start_y,
            )
            ordered = self._order_nearest_neighbor(
                selected,
                self._config.start_x,
                self._config.start_y,
            )
        except Exception:
            return fallback

        if not ordered:
            return fallback
        return self._with_yaw(ordered)

    def _sample_safe_points(self, info: _MapInfo) -> list[Waypoint]:
        """按固定栅格间距采样安全自由点。"""

        step_cells = max(int(round(self._config.sample_spacing_m / info.resolution)), 1)
        clearance_cells = max(
            int(math.ceil(self._config.min_obstacle_distance_m / info.resolution)),
            1,
        )
        candidates: list[Waypoint] = []

        # 从地图左到右、下到上采样；最终顺序会在后面重新规划。
        for row_from_bottom in range(0, info.height, step_cells):
            for col in range(0, info.width, step_cells):
                index = row_from_bottom * info.width + col
                if not info.free[index]:
                    continue
                if not self._has_clearance(info, col, row_from_bottom, clearance_cells):
                    continue
                x = info.origin_x + (col + 0.5) * info.resolution
                y = info.origin_y + (row_from_bottom + 0.5) * info.resolution
                candidates.append(Waypoint(f"coverage_{len(candidates) + 1:02d}", x, y, 0.0))
        return self._thin_by_spacing(candidates, self._config.sample_spacing_m * 0.85)

    @staticmethod
    def _has_clearance(info: _MapInfo, col: int, row: int, radius_cells: int) -> bool:
        """检查候选栅格周围是否有足够障碍物间距。"""

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                x = col + dx
                y = row + dy
                if x < 0 or y < 0 or x >= info.width or y >= info.height:
                    return False
                index = y * info.width + x
                if info.blocked[index]:
                    return False
        return True

    @staticmethod
    def _thin_by_spacing(points: list[Waypoint], min_spacing: float) -> list[Waypoint]:
        """按最小间距稀疏化采样点，避免 waypoint 过密。"""

        kept: list[Waypoint] = []
        for point in points:
            if all(math.hypot(point.x - old.x, point.y - old.y) >= min_spacing for old in kept):
                kept.append(point)
        return kept

    @staticmethod
    def _select_spread_points(
        points: list[Waypoint],
        max_points: int,
        start_x: float,
        start_y: float,
    ) -> list[Waypoint]:
        """从候选点中挑出覆盖全图的稀疏点集。

        旧逻辑如果先最近邻排序再截断，容易只保留起点附近半张地图。这里先做最远点采样：
        每次选择“离已选集合最近距离最大”的点，保证左右、上下和边缘区域都有代表点。
        """

        if max_points <= 0 or len(points) <= max_points:
            return points[:]

        remaining = points[:]
        first_index = min(
            range(len(remaining)),
            key=lambda i: math.hypot(remaining[i].x - start_x, remaining[i].y - start_y),
        )
        selected = [remaining.pop(first_index)]

        while remaining and len(selected) < max_points:
            next_index = max(
                range(len(remaining)),
                key=lambda i: min(
                    math.hypot(remaining[i].x - old.x, remaining[i].y - old.y)
                    for old in selected
                ),
            )
            selected.append(remaining.pop(next_index))

        return selected

    @staticmethod
    def _order_nearest_neighbor(
        points: list[Waypoint], start_x: float, start_y: float
    ) -> list[Waypoint]:
        """用最近邻方式给覆盖点排序。"""

        remaining = points[:]
        ordered: list[Waypoint] = []
        current_x = start_x
        current_y = start_y
        while remaining:
            nearest_index = min(
                range(len(remaining)),
                key=lambda i: math.hypot(
                    remaining[i].x - current_x,
                    remaining[i].y - current_y,
                ),
            )
            point = remaining.pop(nearest_index)
            ordered.append(point)
            current_x = point.x
            current_y = point.y
        return ordered

    @staticmethod
    def _with_yaw(points: list[Waypoint]) -> list[Waypoint]:
        """根据下一个 waypoint 的方向给每个点补 yaw。"""

        result: list[Waypoint] = []
        for index, point in enumerate(points):
            if index + 1 < len(points):
                nxt = points[index + 1]
                yaw = math.atan2(nxt.y - point.y, nxt.x - point.x)
            elif result:
                yaw = result[-1].yaw
            else:
                yaw = 0.0
            result.append(Waypoint(point.name, point.x, point.y, yaw))
        return result

    def _load_map(self, map_yaml: str) -> _MapInfo:
        """读取 map.yaml 和对应 PGM，并转换成 _MapInfo。"""

        yaml_path = Path(map_yaml)
        values = self._read_simple_yaml(yaml_path)
        image = values.get("image", "")
        image_path = Path(image)
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        width, height, pixels = self._read_pgm(image_path)
        resolution = float(values.get("resolution", "0.05"))
        origin = self._parse_origin(values.get("origin", "[0, 0, 0]"))
        negate = int(values.get("negate", "0"))
        occupied_thresh = float(values.get("occupied_thresh", "0.65"))
        free_thresh = float(values.get("free_thresh", "0.196"))

        free = [False] * (width * height)
        blocked = [True] * (width * height)

        for image_row in range(height):
            row_from_bottom = height - 1 - image_row
            for col in range(width):
                pixel = pixels[image_row * width + col]
                occ = (pixel / 255.0) if negate else ((255 - pixel) / 255.0)
                index = row_from_bottom * width + col
                if occ <= free_thresh:
                    free[index] = True
                    blocked[index] = False
                elif occ >= occupied_thresh:
                    blocked[index] = True
                else:
                    # 灰色未知区域不作为安全 waypoint 候选。
                    blocked[index] = True

        return _MapInfo(width, height, resolution, origin[0], origin[1], free, blocked)

    @staticmethod
    def _read_simple_yaml(path: Path) -> dict[str, str]:
        """读取本项目 map.yaml 使用到的简单 key/value。"""

        values: dict[str, str] = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
        return values

    @staticmethod
    def _parse_origin(raw: str) -> tuple[float, float, float]:
        """解析 map.yaml 中的 origin: [x, y, yaw]。"""

        cleaned = raw.strip().strip("[]")
        parts = [p.strip() for p in cleaned.split(",") if p.strip()]
        while len(parts) < 3:
            parts.append("0")
        return float(parts[0]), float(parts[1]), float(parts[2])

    @staticmethod
    def _read_pgm(path: Path) -> tuple[int, int, list[int]]:
        """读取 PGM 地图，支持 P5 二进制和 P2 文本格式。"""

        data = path.read_bytes()
        cursor = 0

        def next_token() -> bytes:
            nonlocal cursor
            while cursor < len(data) and chr(data[cursor]).isspace():
                cursor += 1
            if cursor < len(data) and data[cursor] == ord("#"):
                while cursor < len(data) and data[cursor] not in (10, 13):
                    cursor += 1
                return next_token()
            start = cursor
            while cursor < len(data) and not chr(data[cursor]).isspace():
                cursor += 1
            return data[start:cursor]

        magic = next_token()
        width = int(next_token())
        height = int(next_token())
        max_value = int(next_token())
        if max_value <= 0:
            raise ValueError(f"Invalid PGM max value: {path}")

        while cursor < len(data) and chr(data[cursor]).isspace():
            cursor += 1

        if magic == b"P5":
            raw = data[cursor : cursor + width * height]
            if len(raw) != width * height:
                raise ValueError(f"Incomplete PGM data: {path}")
            return width, height, [int(v) for v in raw]
        if magic == b"P2":
            tokens = data[cursor:].split()
            values = [int(v) for v in tokens[: width * height]]
            if len(values) != width * height:
                raise ValueError(f"Incomplete PGM data: {path}")
            if max_value != 255:
                values = [int(v * 255 / max_value) for v in values]
            return width, height, values
        raise ValueError(f"Unsupported PGM format {magic!r}: {path}")
