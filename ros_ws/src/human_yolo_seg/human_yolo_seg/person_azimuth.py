# pyright: reportMissingImports=false
"""RGB 检测框底边 + 相机内参 + TF：与 base_footprint 上 z=0 平面求交，再换算到激光 frame 的方位角。"""
from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple

from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.time import Time

try:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point, do_transform_vector3
except ImportError:
    from tf2_geometry_msgs import do_transform_point, do_transform_vector3  # type: ignore

from sensor_msgs.msg import CameraInfo, Image


def box_bottom_azimuth_range(
    tf_buffer: Any,
    cam_info: CameraInfo,
    image_msg: Image,
    xyxy_row: Any,
    laser_frame: str,
    ground_frame: str,
    logger,
) -> Optional[Tuple[float, float]]:
    """单个人框 (x1,y1,x2,y2) 像素 → 激光系下方位角区间 (rad)。"""
    x1, y1, x2, y2 = [float(v) for v in xyxy_row]
    u_left, u_right = x1, x2
    v_bottom = max(y1, y2)
    stamp = image_msg.header.stamp
    optical = image_msg.header.frame_id
    if not optical:
        return None

    a1 = _pixel_ray_azimuth_laser(
        tf_buffer, cam_info, u_left, v_bottom, stamp, optical, laser_frame, ground_frame, logger
    )
    a2 = _pixel_ray_azimuth_laser(
        tf_buffer, cam_info, u_right, v_bottom, stamp, optical, laser_frame, ground_frame, logger
    )
    if a1 is None or a2 is None:
        return None
    lo, hi = (a1, a2) if a1 <= a2 else (a2, a1)
    if hi - lo > math.pi * 0.95:
        return None
    return (lo, hi)


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
) -> Optional[float]:
    if len(ci.k) < 9:
        return None
    fx, cx, fy, cy = float(ci.k[0]), float(ci.k[2]), float(ci.k[4]), float(ci.k[5])
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
) -> List[float]:
    if cam_info is None or boxes is None or len(boxes) == 0:
        return []
    xyxy = boxes.xyxy
    if hasattr(xyxy, "cpu"):
        xyxy = xyxy.cpu().numpy()
    data: List[float] = []
    for row in xyxy:
        pair = box_bottom_azimuth_range(
            tf_buffer, cam_info, image_msg, row, laser_frame, ground_frame, logger
        )
        if pair:
            data.extend(pair)
    return data
