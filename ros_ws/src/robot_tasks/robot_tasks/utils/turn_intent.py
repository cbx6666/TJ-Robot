"""语音/LLM 原地转向：左右符号、角度解析与 args 修正（map 系：左转为正、右转为负）。"""

from __future__ import annotations

import math
import re
from typing import Any


def normalize_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_direction_sign(user_text: str) -> int | None:
    """从用户原文推断转向符号：+1 左转/逆时针，-1 右转/顺时针，None 未指明左右。"""
    t = (user_text or "").strip()
    if not t:
        return None
    low = t.lower()

    if re.search(r"向右|右转|往右|朝右", t):
        return -1
    if re.search(r"向左|左转|往左|朝左", t):
        return 1
    if "逆时针" in t:
        return 1
    if "顺时针" in t:
        return -1

    if re.search(r"\b(turn\s+)?right\b", low) or re.search(r"\bright\b", low):
        return -1
    if re.search(r"\b(turn\s+)?left\b", low) or re.search(r"\bleft\b", low):
        return 1
    if re.search(r"\bccw\b", low) or "counter-clockwise" in low or "counterclockwise" in low:
        return 1
    if re.search(r"\bcw\b", low) or "clockwise" in low:
        return -1
    return None


def parse_turn_magnitude_deg(user_text: str, default: float = 0.0) -> float:
    """从文本解析转角绝对值（度）。"""
    t = (user_text or "").strip()
    if not t:
        return abs(default)
    m = re.search(r"(-?\d+(?:\.\d+)?)\s*度", t)
    if m:
        return abs(float(m.group(1)))
    if "180" in t:
        return 180.0
    if "90" in t:
        return 90.0
    if "45" in t:
        return 45.0
    return abs(default)


def _relative_yaw_from_args(args: dict[str, Any]) -> float | None:
    for key in ("relative_yaw_deg", "yaw_delta_deg"):
        if key in args and args[key] is not None and args[key] != "":
            try:
                return float(args[key])
            except (TypeError, ValueError):
                pass
    return None


def fix_turn_args(args: dict[str, Any], user_text: str = "") -> dict[str, Any]:
    """修正 navigate_to_pose 原地转向的 relative_yaw_deg 符号（左正右负）。"""
    if not isinstance(args, dict):
        return {}
    out = dict(args)
    text = str(out.get("user_request_zh") or user_text or "").strip()
    if text and not out.get("user_request_zh"):
        out["user_request_zh"] = text

    yaw_only = bool(out.get("yaw_only") or out.get("rotate_in_place"))
    rel = _relative_yaw_from_args(out)
    if rel is None and not yaw_only:
        return out

    sign_hint = turn_direction_sign(text)
    mag = abs(rel) if rel is not None else parse_turn_magnitude_deg(text, 0.0)
    if mag <= 0.0 and text:
        mag = parse_turn_magnitude_deg(text, 0.0)
    if mag <= 0.0:
        return out

    if sign_hint is not None:
        signed = float(sign_hint) * mag
    elif rel is not None:
        signed = float(rel)
    else:
        signed = mag

    out["yaw_only"] = True
    out["relative_yaw_deg"] = signed
    out.pop("yaw_delta_deg", None)
    return out


def is_turn_nav_intent(args: dict[str, Any]) -> bool:
    if not isinstance(args, dict):
        return False
    if args.get("yaw_only") or args.get("rotate_in_place"):
        return True
    return _relative_yaw_from_args(args) is not None
