from __future__ import annotations

from robot_core.action import ActionType, RobotAction
from robot_core.task import RobotTask, TaskIntent


def plan_task(task: RobotTask) -> list[RobotAction]:
    if task.intent == TaskIntent.SEARCH_OBJECT:
        actions: list[RobotAction] = []
        if task.target_room:
            actions.append(
                RobotAction(ActionType.NAVIGATE_TO_ROOM, {"room": task.target_room})
            )
        actions.append(
            RobotAction(
                ActionType.SEARCH_OBJECT,
                {"object": task.target_object, "room": task.target_room},
            )
        )
        return actions
    if task.intent == TaskIntent.NAVIGATE_TO_ROOM:
        return [RobotAction(ActionType.NAVIGATE_TO_ROOM, {"room": task.target_room})]
    if task.intent == TaskIntent.BUILD_MAP:
        return [RobotAction(ActionType.BUILD_MAP)]
    if task.intent == TaskIntent.STOP:
        return [RobotAction(ActionType.STOP)]
    return []
