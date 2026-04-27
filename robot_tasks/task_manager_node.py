"""Task manager placeholder.

This module currently provides pure planning helpers. A ROS node can later
subscribe to `/interaction/parsed_intent` and dispatch actions from here.
"""

from robot_interaction.nlu.intent_parser import parse_intent
from robot_tasks.task_planner import plan_task


def plan_from_text(text: str):
    return plan_task(parse_intent(text))


def main() -> None:
    for action in plan_from_text("去客厅找杯子"):
        print(action)
