# pyright: reportMissingImports=false
"""Task pipeline composition: interaction + task manager + mock manipulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _inc(share: str, launch_file: str, args: dict[str, LaunchConfiguration]):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", launch_file)),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_mock_voice = LaunchConfiguration("enable_mock_voice")
    voice_api_url = LaunchConfiguration("voice_api_url")
    llm_api_url = LaunchConfiguration("llm_api_url")
    llm_model = LaunchConfiguration("llm_model")

    share = get_package_share_directory("robot_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_mock_voice", default_value="true"),
        DeclareLaunchArgument("voice_api_url", default_value=""),
        DeclareLaunchArgument("llm_api_url", default_value=""),
        DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
        _inc(
            share,
            "interaction.launch.py",
            {
                "use_sim_time": use_sim_time,
                "enable_mock_voice": enable_mock_voice,
                "voice_api_url": voice_api_url,
                "llm_api_url": llm_api_url,
                "llm_model": llm_model,
            },
        ),
        _inc(share, "task_manager.launch.py", {"use_sim_time": use_sim_time}),
        _inc(share, "manipulation.launch.py", {"use_sim_time": use_sim_time}),
    ])
