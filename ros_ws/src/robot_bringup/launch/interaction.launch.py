# pyright: reportMissingImports=false
"""Interaction interface bringup: voice input gateway + LLM router."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_bringup")
    default_prompt = os.path.join(share, "config", "voice_llm_system_prompt.txt")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_mock_voice = LaunchConfiguration("enable_mock_voice")
    speech_text_log_path = LaunchConfiguration("speech_text_log_path")
    voice_api_url = LaunchConfiguration("voice_api_url")
    asr_backend = LaunchConfiguration("asr_backend")
    llm_api_url = LaunchConfiguration("llm_api_url")
    llm_model = LaunchConfiguration("llm_model")
    llm_api_key_env = LaunchConfiguration("llm_api_key_env")
    system_prompt_file = LaunchConfiguration("system_prompt_file")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_mock_voice", default_value="false"),
        DeclareLaunchArgument(
            "speech_text_log_path",
            default_value="",
            description="非空则 voice_gateway 将每次识别文本追加写入该 UTF-8 文件",
        ),
        DeclareLaunchArgument("voice_api_url", default_value=""),
        DeclareLaunchArgument(
            "asr_backend",
            default_value="mock",
            description="mock | whisper_file | whisper_mic | none（whisper_mic=本机麦克风实时转写）",
        ),
        DeclareLaunchArgument(
            "llm_api_url",
            default_value="",
            description="OpenAI 兼容 Chat API 根，如 https://api.deepseek.com/v1；空则用环境变量 TJ_LLM_API_URL",
        ),
        DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
        DeclareLaunchArgument(
            "llm_api_key_env",
            default_value="TJ_LLM_API_KEY",
            description="从此环境变量读取 API Key，避免写进 launch",
        ),
        DeclareLaunchArgument(
            "system_prompt_file",
            default_value=default_prompt,
            description="LLM 系统提示词文件（UTF-8）",
        ),
        Node(
            package="robot_interaction",
            executable="voice_gateway_node",
            name="voice_gateway",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "enable_mock_input": enable_mock_voice,
                    "speech_text_log_path": speech_text_log_path,
                    "voice_api_url": voice_api_url,
                    "asr_backend": asr_backend,
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
                    "llm_api_key_env": llm_api_key_env,
                    "system_prompt_file": system_prompt_file,
                }
            ],
        ),
    ])
