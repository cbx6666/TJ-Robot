# pyright: reportMissingImports=false
"""与人物方位角 + LaserScan 时间同步相关的纯函数（scan_person_filter 与 person_strip_recorder 共用）。"""
from __future__ import annotations

import math
from typing import List, Tuple


def norm_angle(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def angle_in_interval(theta: float, lo: float, hi: float) -> bool:
    t = norm_angle(theta)
    lo = norm_angle(lo)
    hi = norm_angle(hi)
    if lo <= hi:
        return lo <= t <= hi
    return t >= lo or t <= hi


def merge_intervals(pairs: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if not pairs:
        return []
    flat = sorted((min(a, b), max(a, b)) for a, b in pairs)
    out: List[Tuple[float, float]] = []
    for lo, hi in flat:
        if hi - lo > math.pi * 0.95:
            continue
        if not out or lo > out[-1][1]:
            out.append((lo, hi))
        else:
            prev_lo, prev_hi = out[-1]
            out[-1] = (prev_lo, max(prev_hi, hi))
    return out


def is_zero_stamp(stamp) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0
