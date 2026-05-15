# pyright: reportMissingImports=false
"""Task manager bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    parsed_intent_topic = LaunchConfiguration("parsed_intent_topic")
    task_events_topic = LaunchConfiguration("task_events_topic")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "parsed_intent_topic",
                default_value="/interaction/parsed_intent",
            ),
            DeclareLaunchArgument("task_events_topic", default_value="/task/events"),
            DeclareLaunchArgument(
                "navigate_to_pose_action",
                default_value="navigate_to_pose",
                description="Nav2 bt_navigator 的 NavigateToPose 动作名",
            ),
            Node(
                package="robot_tasks",
                executable="task_manager_node",
                name="task_manager",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_tasks",
                executable="command_executor_node",
                name="command_executor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "parsed_intent_topic": parsed_intent_topic,
                        "task_events_topic": task_events_topic,
                        "navigate_to_pose_action": navigate_to_pose_action,
                    }
                ],
            ),
            Node(
                package="robot_tasks",
                executable="patrol_smallhouse",
                name="patrol_smallhouse",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "task_events_topic": task_events_topic,
                    }
                ],
            ),
        ]
    )
