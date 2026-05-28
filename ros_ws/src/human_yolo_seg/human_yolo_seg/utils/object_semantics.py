"""语音/任务语义标签 → COCO 检测类别（与 YOLO 推理 class id 一致）。"""

from __future__ import annotations

# Ultralytics COCO 80 类（与 yolo_object_seg 默认过滤 39,41,75 对齐）
COCO_NAME_TO_ID: dict[str, int] = {
    "bottle": 39,
    "cup": 41,
    "vase": 75,
    "chair": 56,
    "book": 73,
    "cell phone": 67,
    "cell_phone": 67,
    "phone": 67,
    "remote": 65,
}

# 中文/英文别名 → COCO 名（可多对一；可乐罐仿真常标为 bottle 或 cup）
_ALIASES: list[tuple[str, tuple[str, ...]]] = [
    ("杯子", ("cup",)),
    ("杯", ("cup",)),
    ("水杯", ("cup",)),
    ("茶杯", ("cup",)),
    ("瓶子", ("bottle",)),
    ("瓶", ("bottle",)),
    ("可乐", ("bottle", "cup")),
    ("可乐罐", ("bottle", "cup")),
    ("花瓶", ("vase",)),
    ("椅子", ("chair",)),
    ("书", ("book",)),
    ("书本", ("book",)),
    ("手机", ("cell phone",)),
    ("遥控器", ("remote",)),
    ("纸巾", ()),  # 无稳定 COCO 类，不映射
]


def resolve_desired_coco_class_ids(label: str) -> set[int] | None:
    """解析任务要求的物体类别。

    Returns:
        None: 空/通用 object → 不按语义过滤（沿用全局 target_class_ids 内全部候选）。
        set(): 无法映射到已知 COCO 类（调用方应停止发布错误绿点）。
        非空 set: 仅在这些 class id 的 track 中选目标点。
    """
    raw = (label or "").strip()
    if not raw or raw.lower() in ("object", "any", "none", "*"):
        return None

    low = raw.lower().replace("_", " ")
    if low in COCO_NAME_TO_ID:
        return {int(COCO_NAME_TO_ID[low])}

    for name, cid in COCO_NAME_TO_ID.items():
        if name.replace(" ", "") == low.replace(" ", ""):
            return {int(cid)}

    ids: set[int] = set()
    for zh, coco_names in _ALIASES:
        if zh in raw:
            for cn in coco_names:
                if cn in COCO_NAME_TO_ID:
                    ids.add(int(COCO_NAME_TO_ID[cn]))
    if ids:
        return ids

    # 英文短语：bottle / cup 等已处理；剩余尝试子串
    for name, cid in COCO_NAME_TO_ID.items():
        if name in low:
            ids.add(int(cid))
    return ids if ids else set()
