# pyright: reportMissingImports=false
"""Interaction interface bringup: voice input gateway + LLM router."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    mic_device_index = LaunchConfiguration("mic_device_index")
    mic_speech_rms_threshold = LaunchConfiguration("mic_speech_rms_threshold")
    mic_min_speech_sec = LaunchConfiguration("mic_min_speech_sec")
    mic_health_log_path = LaunchConfiguration("mic_health_log_path")
    mic_health_interval_sec = LaunchConfiguration("mic_health_interval_sec")
    mic_stall_alert_sec = LaunchConfiguration("mic_stall_alert_sec")
    enable_sim_speech_gui = LaunchConfiguration("enable_sim_speech_gui")
    speech_text_topic = LaunchConfiguration("speech_text_topic")

    _asr_is_none = PythonExpression(["'", asr_backend, "' == 'none'"])
    _gui_on = PythonExpression(["'", enable_sim_speech_gui, "' == 'true'"])

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
        DeclareLaunchArgument(
            "mic_device_index",
            default_value="-1",
            description="whisper_mic 输入设备 index；-1 自动（WSL 优先 pulse/0）",
        ),
        DeclareLaunchArgument(
            "mic_speech_rms_threshold",
            default_value="0.02",
            description="whisper_mic 有声判定 RMS 门限（默认 0.02，原 0.01 易被环境噪触发）",
        ),
        DeclareLaunchArgument(
            "mic_min_speech_sec",
            default_value="0.38",
            description="whisper_mic 切段最短有效语音秒数",
        ),
        DeclareLaunchArgument(
            "mic_health_log_path",
            default_value="",
            description="非空则周期性写入麦克风健康 JSONL（推荐 data/logs/full_system/voice_gateway_health.jsonl）",
        ),
        DeclareLaunchArgument("mic_health_interval_sec", default_value="5.0"),
        DeclareLaunchArgument("mic_stall_alert_sec", default_value="4.0"),
        DeclareLaunchArgument("publish_task_goal_from_fetch", default_value="false"),
        DeclareLaunchArgument(
            "enable_sim_speech_gui",
            default_value="false",
            description="true 时启动 tkinter 模拟语音窗（无 RViz 面板时用）",
        ),
        DeclareLaunchArgument(
            "speech_text_topic",
            default_value="/interaction/speech_text",
        ),
        Node(
            package="robot_interaction",
            executable="voice_gateway_node",
            name="voice_gateway",
            output="screen",
            condition=UnlessCondition(_asr_is_none),
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "enable_mock_input": enable_mock_voice,
                    "speech_text_log_path": speech_text_log_path,
                    "voice_api_url": voice_api_url,
                    "asr_backend": asr_backend,
                    "mic_device_index": mic_device_index,
                    "mic_speech_rms_threshold": mic_speech_rms_threshold,
                    "mic_min_speech_sec": mic_min_speech_sec,
                    "mic_health_log_path": mic_health_log_path,
                    "mic_health_interval_sec": mic_health_interval_sec,
                    "mic_stall_alert_sec": mic_stall_alert_sec,
                }
            ],
        ),
        Node(
            package="robot_interaction",
            executable="sim_speech_gui_node",
            name="sim_speech_gui",
            output="screen",
            condition=IfCondition(_gui_on),
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "speech_text_topic": speech_text_topic,
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
                    "publish_task_goal_from_fetch": LaunchConfiguration(
                        "publish_task_goal_from_fetch"
                    ),
                }
            ],
        ),
    ])
