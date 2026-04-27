# pyright: reportMissingImports=false
"""Interaction interface bringup: voice input gateway + LLM router."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_mock_voice = LaunchConfiguration("enable_mock_voice")
    voice_api_url = LaunchConfiguration("voice_api_url")
    llm_api_url = LaunchConfiguration("llm_api_url")
    llm_model = LaunchConfiguration("llm_model")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_mock_voice", default_value="true"),
        DeclareLaunchArgument("voice_api_url", default_value=""),
        DeclareLaunchArgument("llm_api_url", default_value=""),
        DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
        Node(
            package="robot_interaction",
            executable="voice_gateway_node",
            name="voice_gateway",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "enable_mock_input": enable_mock_voice,
                    "voice_api_url": voice_api_url,
                }
            ],
        ),
        Node(
            package="robot_interaction",
            executable="llm_router_node",
            name="llm_router",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "llm_api_url": llm_api_url,
                    "llm_model": llm_model,
                }
            ],
        ),
    ])
