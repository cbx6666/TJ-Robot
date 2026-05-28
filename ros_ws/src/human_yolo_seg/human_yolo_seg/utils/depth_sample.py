"""RGB–Depth 投影与 ROI 稳健深度采样（对齐 yolo_ros / RealSense 对齐深度思路）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np
from sensor_msgs.msg import CameraInfo

DepthStat = Literal["median", "trimmed_mean", "min"]


@dataclass
class RgbDepthProjector:
    """RGB 检测像素 ↔ 深度图像素（视场/内参映射或已对齐同分辨率）。"""

    rgb_info: CameraInfo | None
    depth_info: CameraInfo
    rgb_w: int
    rgb_h: int
    depth_w: int
    depth_h: int
    pixels_aligned: bool = False

    @classmethod
    def from_images(
        cls,
        rgb_info: CameraInfo | None,
        depth_info: CameraInfo,
        rgb_w: int,
        rgb_h: int,
        depth_w: int,
        depth_h: int,
        pixels_aligned: bool,
    ) -> RgbDepthProjector:
        return cls(
            rgb_info=rgb_info,
            depth_info=depth_info,
            rgb_w=max(1, rgb_w),
            rgb_h=max(1, rgb_h),
            depth_w=max(1, depth_w),
            depth_h=max(1, depth_h),
            pixels_aligned=pixels_aligned,
        )

    @staticmethod
    def _scaled_k(ci: CameraInfo, img_w: int, img_h: int) -> tuple[float, float, float, float]:
        if len(ci.k) < 9:
            return (0.0, 0.0, 0.0, 0.0)
        fx, cx, fy, cy = float(ci.k[0]), float(ci.k[2]), float(ci.k[4]), float(ci.k[5])
        cw, ch = int(ci.width), int(ci.height)
        if cw <= 0 or ch <= 0 or (cw == img_w and ch == img_h):
            return fx, cx, fy, cy
        sx = float(img_w) / float(cw)
        sy = float(img_h) / float(ch)
        return fx * sx, cx * sx, fy * sy, cy * sy

    def rgb_uv_to_depth_uv(self, u_rgb: float, v_rgb: float) -> tuple[int, int]:
        if self.pixels_aligned:
            u_d = int(round(u_rgb * float(self.depth_w) / float(self.rgb_w)))
            v_d = int(round(v_rgb * float(self.depth_h) / float(self.rgb_h)))
        elif self.rgb_info is not None and len(self.rgb_info.k) >= 9:
            rfx, rcx, rfy, rcy = self._scaled_k(self.rgb_info, self.rgb_w, self.rgb_h)
            dfx, dcx, dfy, dcy = self._scaled_k(self.depth_info, self.depth_w, self.depth_h)
            if rfx > 1e-6 and rfy > 1e-6 and dfx > 1e-6 and dfy > 1e-6:
                theta_u = math.atan((float(u_rgb) - rcx) / rfx)
                theta_v = math.atan((float(v_rgb) - rcy) / rfy)
                u_d = int(round(dcx + dfx * math.tan(theta_u)))
                v_d = int(round(dcy + dfy * math.tan(theta_v)))
            else:
                u_d = int(round(u_rgb * float(self.depth_w) / float(self.rgb_w)))
                v_d = int(round(v_rgb * float(self.depth_h) / float(self.rgb_h)))
        else:
            u_d = int(round(u_rgb * float(self.depth_w) / float(self.rgb_w)))
            v_d = int(round(v_rgb * float(self.depth_h) / float(self.rgb_h)))
        return (
            max(0, min(self.depth_w - 1, u_d)),
            max(0, min(self.depth_h - 1, v_d)),
        )

    def rgb_box_depth_aabb(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> tuple[int, int, int, int]:
        """检测框四角映射到深度图上的轴对齐包围盒。"""
        corners = (
            (x1, y1),
            (x2, y1),
            (x1, y2),
            (x2, y2),
        )
        us: list[int] = []
        vs: list[int] = []
        for u, v in corners:
            ud, vd = self.rgb_uv_to_depth_uv(u, v)
            us.append(ud)
            vs.append(vd)
        return min(us), min(vs), max(us), max(vs)


def depth_value_valid(
    z: float,
    depth_min_m: float,
    depth_max_m: float,
    near_clip_reject_m: float,
) -> bool:
    if not np.isfinite(z) or z <= 0.0:
        return False
    if z < depth_min_m or z > depth_max_m:
        return False
    if z >= near_clip_reject_m:
        return False
    return True


def robust_depth_stat(
    depths: np.ndarray,
    mode: DepthStat,
    weights: np.ndarray | None = None,
) -> float | None:
    if depths.size == 0:
        return None
    d = depths.astype(np.float64)
    if weights is not None and weights.size == d.size:
        w = weights.astype(np.float64)
        w = w / max(float(w.sum()), 1e-9)
        order = np.argsort(d)
        d_sorted = d[order]
        w_sorted = w[order]
        cdf = np.cumsum(w_sorted)
        if mode == "min":
            return float(d_sorted[0])
        if mode == "median":
            idx = int(np.searchsorted(cdf, 0.5, side="left"))
            idx = min(idx, len(d_sorted) - 1)
            return float(d_sorted[idx])
        # trimmed_mean: central 60% by weight
        lo, hi = 0.2, 0.8
        mask = (cdf >= lo) & (cdf <= hi)
        if not np.any(mask):
            mask = np.ones_like(d_sorted, dtype=bool)
        return float(np.average(d_sorted[mask], weights=w_sorted[mask]))
    if mode == "min":
        return float(np.min(d))
    if mode == "median":
        return float(np.median(d))
    lo, hi = np.percentile(d, [20.0, 80.0])
    trimmed = d[(d >= lo) & (d <= hi)]
    return float(np.mean(trimmed)) if trimmed.size else float(np.mean(d))


def sample_depth_roi(
    depth_m: np.ndarray,
    projector: RgbDepthProjector,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    *,
    mask_rgb: np.ndarray | None,
    depth_min_m: float,
    depth_max_m: float,
    near_clip_reject_m: float,
    stat: DepthStat = "median",
    grid_n: int = 7,
    prefer_center: bool = True,
) -> tuple[float, float, float, float, float] | None:
    """
    在检测框对应深度 ROI 内采样，返回 (u_rgb, v_rgb, x_cam, y_cam, z_cam)。
    u_rgb/v_rgb 为框心（代表像素）；深度统计在映射后的深度网格上进行。
    """
    xa, xb = float(min(x1, x2)), float(max(x1, x2))
    ya, yb = float(min(y1, y2)), float(max(y1, y2))
    u_c = 0.5 * (xa + xb)
    v_c = 0.5 * (ya + yb)

    u0, v0, u1, v1 = projector.rgb_box_depth_aabb(xa, ya, xb, yb)
    dh, dw = depth_m.shape[:2]
    u0, v0 = max(0, u0), max(0, v0)
    u1, v1 = min(dw - 1, u1), min(dh - 1, v1)
    if u1 < u0 or v1 < v0:
        return None

    n = max(3, int(grid_n))
    us_d = np.linspace(u0, u1, num=n)
    vs_d = np.linspace(v0, v1, num=n)
    depths: list[float] = []
    weights: list[float] = []

    for vd in vs_d:
        for ud in us_d:
            ui, vi = int(round(ud)), int(round(vd))
            if ui < 0 or ui >= dw or vi < 0 or vi >= dh:
                continue
            z = float(depth_m[vi, ui])
            if not depth_value_valid(z, depth_min_m, depth_max_m, near_clip_reject_m):
                continue
            if mask_rgb is not None:
                # 反查近似 RGB 像素，仅在 mask 内保留
                if projector.pixels_aligned:
                    ur = int(round(ui * projector.rgb_w / projector.depth_w))
                    vr = int(round(vi * projector.rgb_h / projector.depth_h))
                else:
                    # 用深度像素中心角反推近似 rgb（仅用于 mask 门控）
                    dfx, dcx, dfy, dcy = projector._scaled_k(
                        projector.depth_info, projector.depth_w, projector.depth_h
                    )
                    if dfx < 1e-6 or dfy < 1e-6:
                        continue
                    theta_u = math.atan((float(ui) - dcx) / dfx)
                    theta_v = math.atan((float(vi) - dcy) / dfy)
                    if projector.rgb_info is None:
                        continue
                    rfx, rcx, rfy, rcy = projector._scaled_k(
                        projector.rgb_info, projector.rgb_w, projector.rgb_h
                    )
                    ur = int(round(rcx + rfx * math.tan(theta_u)))
                    vr = int(round(rcy + rfy * math.tan(theta_v)))
                if ur < 0 or ur >= projector.rgb_w or vr < 0 or vr >= projector.rgb_h:
                    continue
                if int(mask_rgb[vr, ur]) == 0:
                    continue
            w = 1.0
            if prefer_center:
                # 深度网格点到框心的距离权重（在深度像素空间）
                dist = math.hypot(ui - 0.5 * (u0 + u1), vi - 0.5 * (v0 + v1))
                span = math.hypot(max(1.0, u1 - u0), max(1.0, v1 - v0))
                w = max(0.05, 1.0 - dist / span)
            depths.append(z)
            weights.append(w)

    if not depths:
        return None

    z_rob = robust_depth_stat(
        np.asarray(depths, dtype=np.float64),
        stat,
        np.asarray(weights, dtype=np.float64) if prefer_center else None,
    )
    if z_rob is None:
        return None

    # 代表点：框心映射到深度后反投影
    ud, vd = projector.rgb_uv_to_depth_uv(u_c, v_c)
    dfx, dcx, dfy, dcy = projector._scaled_k(projector.depth_info, projector.depth_w, projector.depth_h)
    if dfx < 1e-6 or dfy < 1e-6:
        return None
    z_opt = float(z_rob)
    x = (float(ud) - dcx) * z_opt / dfx
    y = (float(vd) - dcy) * z_opt / dfy
    return float(u_c), float(v_c), x, y, z_opt


def sample_depth_with_optical_z(
    u_d: float,
    v_d: float,
    z_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_range_to_optical_z: bool,
) -> tuple[float, float, float]:
    if not depth_range_to_optical_z or z_m <= 0.0:
        return (
            (float(u_d) - cx) * z_m / fx,
            (float(v_d) - cy) * z_m / fy,
            float(z_m),
        )
    nx = (float(u_d) - cx) / fx
    ny = (float(v_d) - cy) / fy
    nz = 1.0
    denom = math.sqrt(nx * nx + ny * ny + nz * nz)
    if denom < 1e-6:
        z_opt = float(z_m)
    else:
        z_opt = float(z_m) * nz / denom
    x = (float(u_d) - cx) * z_opt / fx
    y = (float(v_d) - cy) * z_opt / fy
    return x, y, z_opt
