# pyright: reportMissingImports=false
"""人物方位角：tf_geometry（内参+TF+地面求交）或 linear_fov（框横坐标占图像宽度 × 水平视场，粗略、无 TF）。"""
from __future__ import annotations

import math
from typing import Any, List, Optional, Sequence, Tuple

from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.time import Time

try:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point, do_transform_vector3
except ImportError:
    from tf2_geometry_msgs import do_transform_point, do_transform_vector3  # type: ignore

from sensor_msgs.msg import CameraInfo, Image

# 底边左右像素太近时，YOLO-Seg 等会给极窄框，两角落在同一竖线上 → 方位角区间退化为 0°~0°
_MIN_BOX_BOTTOM_WIDTH_PX = 8.0


def _mask_bottom_pixels(poly: Any, ih: int) -> Optional[Tuple[float, float, float]]:
    """分割轮廓多边形 → 图像底部一带的 min/max u 与最底 v（比 axis-aligned 框更接近人体接地点宽度）。"""
    try:
        pts = list(poly)
    except TypeError:
        return None
    if len(pts) < 3:
        return None
    ys: list[float] = []
    for p in pts:
        try:
            ys.append(float(p[1]))
        except (TypeError, IndexError, ValueError):
            return None
    ymax = max(ys)
    band = max(3.0, min(14.0, float(ih) * 0.07))
    xs_bottom: list[float] = []
    for p in pts:
        try:
            if float(p[1]) >= ymax - band:
                xs_bottom.append(float(p[0]))
        except (TypeError, IndexError, ValueError):
            return None
    if len(xs_bottom) < 2:
        xs_bottom = [float(p[0]) for p in pts]
    u_left, u_right = min(xs_bottom), max(xs_bottom)
    return (u_left, u_right, ymax)


def horizontal_fov_rad_from_intrinsics(ci: CameraInfo, img_w: int) -> Optional[float]:
    """由 fx 与图像宽度估计水平视场角：2*atan(w/(2*fx))。"""
    if len(ci.k) < 9:
        return None
    fx, _cx, _fy, _cy = _scaled_intrinsics(ci, img_w, int(ci.height) if ci.height else img_w)
    if fx < 1e-6:
        return None
    return float(2.0 * math.atan(0.5 * float(img_w) / fx))


def _linear_fov_angle_rad(u: float, iw: int, cx: float, hfov_rad: float, camera_yaw_rad: float) -> float:
    """像素 u → 与 base_scan / LaserScan 一致的方位角：x 前、y 左，atan2(y,x) 左侧为正。

    图像 u 增大为「画面右侧」，对应光学系 +X；激光系左侧为 +y、为正角，故对 (u-cx) 项取负。
    """
    denom = float(max(iw - 1, 1))
    return float(-(u - cx) / denom * hfov_rad + camera_yaw_rad)


def box_linear_fov_azimuth_range(
    xyxy_row: Any,
    iw: int,
    ih: int,
    cx: float,
    hfov_rad: float,
    camera_yaw_rad: float,
    mask_xy: Any = None,
    use_mask: bool = False,
) -> Optional[Tuple[float, float]]:
    """框（或可选 mask）左右 u → 激光系近似方位角区间，无 TF、无地面求交。"""
    if iw <= 0 or ih <= 0:
        return None
    u_left: float
    u_right: float
    mp = _mask_bottom_pixels(mask_xy, ih) if use_mask and mask_xy is not None else None
    if mp is not None:
        u_left, u_right, _vb = mp
    else:
        x1, y1, x2, y2 = [float(v) for v in xyxy_row]
        u_left, u_right = min(x1, x2), max(x1, x2)
    span = u_right - u_left
    if span < _MIN_BOX_BOTTOM_WIDTH_PX:
        mid = 0.5 * (u_left + u_right)
        half = max(_MIN_BOX_BOTTOM_WIDTH_PX * 0.5, 12.0)
        u_left = mid - half
        u_right = mid + half
    u_left = max(0.0, min(float(iw - 1), u_left))
    u_right = max(0.0, min(float(iw - 1), u_right))
    if u_right <= u_left + 1e-3:
        return None
    a1 = _linear_fov_angle_rad(u_left, iw, cx, hfov_rad, camera_yaw_rad)
    a2 = _linear_fov_angle_rad(u_right, iw, cx, hfov_rad, camera_yaw_rad)
    lo, hi = (a1, a2) if a1 <= a2 else (a2, a1)
    if hi - lo > math.pi * 0.95:
        return None
    return (lo, hi)


def _scaled_intrinsics(ci: CameraInfo, img_w: int, img_h: int) -> tuple[float, float, float, float]:
    """CameraInfo 常与图像同分辨率；若不一致则按宽高比缩放 fx,fy,cx,cy（常见仿真/桥接 bug）。"""
    if len(ci.k) < 9:
        return (0.0, 0.0, 0.0, 0.0)
    fx, cx, fy, cy = float(ci.k[0]), float(ci.k[2]), float(ci.k[4]), float(ci.k[5])
    cw, ch = int(ci.width), int(ci.height)
    if cw <= 0 or ch <= 0 or (cw == img_w and ch == img_h):
        return fx, cx, fy, cy
    sx = float(img_w) / float(cw)
    sy = float(img_h) / float(ch)
    return fx * sx, cx * sx, fy * sy, cy * sy


def box_bottom_azimuth_range(
    tf_buffer: Any,
    cam_info: CameraInfo,
    image_msg: Image,
    xyxy_row: Any,
    laser_frame: str,
    ground_frame: str,
    logger,
    image_wh: Optional[Tuple[int, int]] = None,
    mask_xy: Any = None,
) -> Optional[Tuple[float, float]]:
    """单个人检测 → 激光系下方位角区间 (rad)。优先用分割 mask 底边宽度，否则用检测框底边。"""
    stamp = image_msg.header.stamp
    optical = image_msg.header.frame_id
    if not optical:
        return None

    if image_wh is not None:
        iw, ih = int(image_wh[0]), int(image_wh[1])
    else:
        iw = int(image_msg.width) if image_msg.width else int(cam_info.width)
        ih = int(image_msg.height) if image_msg.height else int(cam_info.height)
    if iw <= 0 or ih <= 0:
        return None

    u_left: float
    u_right: float
    v_bottom: float
    mp = _mask_bottom_pixels(mask_xy, ih) if mask_xy is not None else None
    if mp is not None:
        u_left, u_right, v_bottom = mp
    else:
        x1, y1, x2, y2 = [float(v) for v in xyxy_row]
        u_left, u_right = min(x1, x2), max(x1, x2)
        v_bottom = max(y1, y2)
    v_bottom = max(0.0, min(float(ih - 1), v_bottom))

    span = u_right - u_left
    if span < _MIN_BOX_BOTTOM_WIDTH_PX:
        mid = 0.5 * (u_left + u_right)
        half = max(_MIN_BOX_BOTTOM_WIDTH_PX * 0.5, 12.0)
        u_left = mid - half
        u_right = mid + half
    u_left = max(0.0, min(float(iw - 1), u_left))
    u_right = max(0.0, min(float(iw - 1), u_right))
    if u_right <= u_left + 1e-3:
        return None

    a1 = _pixel_ray_azimuth_laser(
        tf_buffer,
        cam_info,
        u_left,
        v_bottom,
        stamp,
        optical,
        laser_frame,
        ground_frame,
        logger,
        iw,
        ih,
    )
    a2 = _pixel_ray_azimuth_laser(
        tf_buffer,
        cam_info,
        u_right,
        v_bottom,
        stamp,
        optical,
        laser_frame,
        ground_frame,
        logger,
        iw,
        ih,
    )
    if a1 is None or a2 is None:
        return None
    lo, hi = (a1, a2) if a1 <= a2 else (a2, a1)
    if hi - lo > math.pi * 0.95:
        return None
    return (lo, hi)


def _pixel_ray_ground_xy(
    tf_buffer: Any,
    ci: CameraInfo,
    u: float,
    v: float,
    stamp,
    optical_frame: str,
    ground_frame: str,
    logger,
    img_w: int,
    img_h: int,
) -> Optional[Tuple[float, float]]:
    """像素射线与 ground_frame 下 z=0 平面交点 (x,y)。"""
    u = max(0.0, min(float(img_w - 1), u))
    v = max(0.0, min(float(img_h - 1), v))
    fx, cx, fy, cy = _scaled_intrinsics(ci, img_w, img_h)
    if fx < 1e-6 or fy < 1e-6:
        return None
    x_n = (u - cx) / fx
    y_n = (v - cy) / fy
    z_n = 1.0
    ln = math.sqrt(x_n * x_n + y_n * y_n + z_n * z_n)
    x_n, y_n, z_n = x_n / ln, y_n / ln, z_n / ln

    t_msg = Time.from_msg(stamp)
    try:
        t0 = Duration(seconds=0, nanoseconds=350_000_000)
        trans_g_o = tf_buffer.lookup_transform(ground_frame, optical_frame, t_msg, timeout=t0)
    except Exception as e:
        try:
            trans_g_o = tf_buffer.lookup_transform(
                ground_frame, optical_frame, Time(), timeout=t0
            )
        except Exception as e2:
            logger.debug(f"TF {ground_frame}<-{optical_frame} 失败: {e} / {e2}")
            return None

    vs = Vector3Stamped()
    vs.header.frame_id = optical_frame
    vs.header.stamp = stamp
    vs.vector.x = x_n
    vs.vector.y = y_n
    vs.vector.z = z_n
    v_g = do_transform_vector3(vs, trans_g_o)

    ps = PointStamped()
    ps.header.frame_id = optical_frame
    ps.header.stamp = stamp
    ps.point.x = 0.0
    ps.point.y = 0.0
    ps.point.z = 0.0
    o_g = do_transform_point(ps, trans_g_o)

    dz = float(v_g.vector.z)
    oz = float(o_g.point.z)
    if abs(dz) < 1e-5:
        return None
    t_hit = -oz / dz
    if t_hit < 0.0:
        return None

    px = float(o_g.point.x) + t_hit * float(v_g.vector.x)
    py = float(o_g.point.y) + t_hit * float(v_g.vector.y)
    return (px, py)


def _pixel_ray_azimuth_laser(
    tf_buffer: Any,
    ci: CameraInfo,
    u: float,
    v: float,
    stamp,
    optical_frame: str,
    laser_frame: str,
    ground_frame: str,
    logger,
    img_w: int,
    img_h: int,
) -> Optional[float]:
    gh = _pixel_ray_ground_xy(
        tf_buffer, ci, u, v, stamp, optical_frame, ground_frame, logger, img_w, img_h
    )
    if gh is None:
        return None
    px, py = gh

    t_msg = Time.from_msg(stamp)
    t0 = Duration(seconds=0, nanoseconds=350_000_000)

    pt = PointStamped()
    pt.header.frame_id = ground_frame
    pt.header.stamp = stamp
    pt.point.x = px
    pt.point.y = py
    pt.point.z = 0.0

    try:
        trans_l_g = tf_buffer.lookup_transform(laser_frame, ground_frame, t_msg, timeout=t0)
    except Exception:
        try:
            trans_l_g = tf_buffer.lookup_transform(laser_frame, ground_frame, Time(), timeout=t0)
        except Exception as e:
            logger.debug(f"TF {laser_frame}<-{ground_frame} 失败: {e}")
            return None

    p_ls = do_transform_point(pt, trans_l_g)
    return float(math.atan2(p_ls.point.y, p_ls.point.x))


def boxes_to_azimuth_data(
    tf_buffer: Any,
    cam_info: Optional[CameraInfo],
    image_msg: Image,
    boxes: Any,
    laser_frame: str,
    ground_frame: str,
    logger,
    image_wh: Optional[Tuple[int, int]] = None,
    masks_xy: Optional[Sequence[Any]] = None,
    mode: str = "linear_fov",
    linear_hfov_rad: Optional[float] = None,
    linear_camera_yaw_rad: float = 0.0,
    linear_use_mask: bool = False,
) -> List[float]:
    if boxes is None or len(boxes) == 0:
        return []
    xyxy = boxes.xyxy
    if hasattr(xyxy, "cpu"):
        xyxy = xyxy.cpu().numpy()

    low = (mode or "linear_fov").strip().lower()
    if low == "linear_fov":
        if image_wh is not None:
            iw, ih = int(image_wh[0]), int(image_wh[1])
        else:
            iw = int(image_msg.width) if image_msg.width else 0
            ih = int(image_msg.height) if image_msg.height else 0
        if iw <= 0 or ih <= 0:
            return []
        hfov = linear_hfov_rad
        if hfov is None or hfov <= 0.0:
            if cam_info is not None:
                hfov = horizontal_fov_rad_from_intrinsics(cam_info, iw)
            if hfov is None or hfov <= 0.0:
                hfov = math.radians(135.0)
        if cam_info is not None and len(cam_info.k) >= 9:
            _fx, cx, _fy, _cy = _scaled_intrinsics(cam_info, iw, ih)
            cx = float(cx)
        else:
            cx = 0.5 * float(iw)
        data: List[float] = []
        for i, row in enumerate(xyxy):
            mxy = masks_xy[i] if masks_xy is not None and i < len(masks_xy) else None
            pair = box_linear_fov_azimuth_range(
                row,
                iw,
                ih,
                cx,
                hfov,
                linear_camera_yaw_rad,
                mask_xy=mxy,
                use_mask=linear_use_mask,
            )
            if pair:
                data.extend(pair)
        return data

    if cam_info is None:
        return []
    data = []
    for i, row in enumerate(xyxy):
        mxy = masks_xy[i] if masks_xy is not None and i < len(masks_xy) else None
        pair = box_bottom_azimuth_range(
            tf_buffer,
            cam_info,
            image_msg,
            row,
            laser_frame,
            ground_frame,
            logger,
            image_wh=image_wh,
            mask_xy=mxy,
        )
        if pair:
            data.extend(pair)
    return data
