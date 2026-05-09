# pyright: reportMissingImports=false
"""YOLO 物体节点共用的推理前检查与 2D 叠图（从 yolo_object_seg_node 拆出以控制单文件体量）。"""

from __future__ import annotations

import os
import sys
from typing import Any

import numpy as np

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def check_numpy_compatible_cv_bridge() -> None:
    """Humble 的 cv_bridge 针对 NumPy 1.x 编译，NumPy 2.x 会在 imgmsg_to_cv2 等处崩溃。"""
    major = int(np.__version__.split(".", 1)[0])
    if major >= 2:
        print(
            f"ERROR: 当前 NumPy {np.__version__} 与 ROS 2 Humble 的 cv_bridge 不兼容（常见报错: _ARRAY_API）。\n"
            "  请使用与 ros2 相同的 python3 执行: pip install 'numpy>=1.23,<2'\n"
            "  然后重启 yolo 节点。requirements.txt 已约束 numpy<2。",
            file=sys.stderr,
        )
        raise RuntimeError("NumPy>=2 is incompatible with cv_bridge on ROS 2 Humble")


def try_import_ultralytics():
    try:
        from ultralytics import YOLO  # type: ignore

        return YOLO
    except ImportError as e:
        print(
            "ERROR: 未安装 ultralytics/torch。请使用与 ros2 相同的 python3：\n"
            "  pip install ultralytics torch torchvision opencv-python-headless\n"
            "或: pip install -r <工作空间>/src/human_yolo_seg/requirements.txt",
            file=sys.stderr,
        )
        raise e


def resolve_model_path(logger, model_path: str) -> str:
    """优先：存在的绝对/相对路径；否则 share/human_yolo_seg/models/<文件名>；否则原样交给 Ultralytics。"""
    expanded = os.path.expanduser(model_path.strip())
    if os.path.isfile(expanded):
        resolved = os.path.abspath(expanded)
        logger.info(f"模型路径（用户指定文件）: {resolved}")
        return resolved
    basename = os.path.basename(expanded)
    try:
        share = get_package_share_directory("human_yolo_seg")
        cand = os.path.join(share, "models", basename)
        if os.path.isfile(cand):
            logger.info(f"模型路径（包内 models/）: {cand}")
            return cand
    except PackageNotFoundError:
        logger.warning("未找到已安装的 human_yolo_seg share（请先 colcon build）")
    logger.info(f"按 Ultralytics 解析模型: {expanded}")
    return expanded


def detection_box_count_and_max_conf(results: list[Any]) -> tuple[int, float]:
    if not results:
        return 0, 0.0
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return 0, 0.0
    confs = boxes.conf
    if hasattr(confs, "detach"):
        arr = confs.detach().cpu().float().numpy()
        return int(len(boxes)), float(arr.max()) if arr.size else 0.0
    return int(len(boxes)), float(np.max(confs)) if confs is not None else 0.0


def annotate_boxes_bgr(img: np.ndarray, results: list[Any]) -> np.ndarray:
    """仅画框+标签（避免 Results.plot() 在分割模型上绘制整幅 mask，CPU 开销极大）。"""
    import cv2

    out = np.ascontiguousarray(img)
    if not results:
        return out
    r0 = results[0]
    boxes = getattr(r0, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return out
    xyxy = boxes.xyxy
    if hasattr(xyxy, "detach"):
        xyxy = xyxy.detach().cpu().numpy()
    confs = boxes.conf
    if hasattr(confs, "detach"):
        confs = confs.detach().cpu().numpy()
    classes = boxes.cls
    if hasattr(classes, "detach"):
        classes = classes.detach().cpu().numpy()
    names = getattr(r0, "names", {}) if hasattr(r0, "names") else {}
    color = (0, 220, 0)
    white = (255, 255, 255)
    for i in range(len(xyxy)):
        x1, y1, x2, y2 = (int(xyxy[i][j]) for j in range(4))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2, lineType=cv2.LINE_AA)
        cls_id = int(classes[i]) if len(classes) > i else -1
        lab = str(names.get(cls_id, cls_id)) if isinstance(names, dict) else str(cls_id)
        c = float(confs[i]) if len(confs) > i else 0.0
        caption = f"{lab} {c:.2f}"
        (tw, th), bl = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1 - 4, th + 6)
        cv2.rectangle(out, (x1, ty - th - 6), (x1 + tw + 4, ty + bl - 2), color, -1)
        cv2.putText(out, caption, (x1 + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, white, 1, cv2.LINE_AA)
    return out
