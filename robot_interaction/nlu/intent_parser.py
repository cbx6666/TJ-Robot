from __future__ import annotations

from robot_core.task import RobotTask, TaskIntent


ROOM_ALIASES = {
    "客厅": "living_room",
    "厨房": "kitchen",
    "卧室": "bedroom",
    "书房": "study",
    "门口": "entrance",
}

OBJECT_ALIASES = {
    "杯子": "cup",
    "水杯": "cup",
    "椅子": "chair",
    "包": "bag",
    "门": "door",
    "人": "person",
}


def parse_intent(text: str, task_id: str = "voice_command") -> RobotTask:
    compact = text.strip().replace(" ", "")
    target_room = next((v for k, v in ROOM_ALIASES.items() if k in compact), None)
    target_object = next((v for k, v in OBJECT_ALIASES.items() if k in compact), None)

    if any(word in compact for word in ("找", "寻找", "搜索")) and target_object:
        return RobotTask(
            task_id=task_id,
            intent=TaskIntent.SEARCH_OBJECT,
            target_room=target_room,
            target_object=target_object,
            required_modules=["robot_navigation", "robot_perception", "robot_tasks"],
            parameters={"raw_text": text},
        )
    if any(word in compact for word in ("去", "到", "导航")) and target_room:
        return RobotTask(
            task_id=task_id,
            intent=TaskIntent.NAVIGATE_TO_ROOM,
            target_room=target_room,
            required_modules=["robot_navigation"],
            parameters={"raw_text": text},
        )
    if any(word in compact for word in ("建图", "地图")):
        return RobotTask(
            task_id=task_id,
            intent=TaskIntent.BUILD_MAP,
            required_modules=["robot_mapping"],
            parameters={"raw_text": text},
        )
    if any(word in compact for word in ("停止", "停下", "取消")):
        return RobotTask(
            task_id=task_id,
            intent=TaskIntent.STOP,
            required_modules=["robot_tasks"],
            parameters={"raw_text": text},
        )
    return RobotTask(
        task_id=task_id,
        intent=TaskIntent.UNKNOWN,
        parameters={"raw_text": text},
    )
