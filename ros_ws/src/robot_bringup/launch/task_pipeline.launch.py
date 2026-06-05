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
    speech_text_log_path = LaunchConfiguration("speech_text_log_path")
    voice_api_url = LaunchConfiguration("voice_api_url")
    asr_backend = LaunchConfiguration("asr_backend")
    llm_api_url = LaunchConfiguration("llm_api_url")
    llm_model = LaunchConfiguration("llm_model")
    llm_api_key_env = LaunchConfiguration("llm_api_key_env")
    system_prompt_file = LaunchConfiguration("system_prompt_file")
    parsed_intent_topic = LaunchConfiguration("parsed_intent_topic")
    task_events_topic = LaunchConfiguration("task_events_topic")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")
    map_yaml = LaunchConfiguration("map_yaml")
    mic_device_index = LaunchConfiguration("mic_device_index")
    mic_speech_rms_threshold = LaunchConfiguration("mic_speech_rms_threshold")
    mic_min_speech_sec = LaunchConfiguration("mic_min_speech_sec")
    mic_health_log_path = LaunchConfiguration("mic_health_log_path")
    mic_health_interval_sec = LaunchConfiguration("mic_health_interval_sec")
    mic_stall_alert_sec = LaunchConfiguration("mic_stall_alert_sec")
    enable_sim_speech_gui = LaunchConfiguration("enable_sim_speech_gui")
    pick_min_distance_m = LaunchConfiguration("pick_min_distance_m")
    pick_max_distance_m = LaunchConfiguration("pick_max_distance_m")
    pick_max_yaw_error_deg = LaunchConfiguration("pick_max_yaw_error_deg")
    pick_delay_min_sec = LaunchConfiguration("pick_delay_min_sec")
    pick_delay_max_sec = LaunchConfiguration("pick_delay_max_sec")

    share = get_package_share_directory("robot_bringup")
    default_map = os.path.join(share, "maps", "map.yaml")
    default_prompt = os.path.join(share, "config", "voice_llm_system_prompt.txt")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_mock_voice", default_value="false"),
        DeclareLaunchArgument("speech_text_log_path", default_value=""),
        DeclareLaunchArgument("voice_api_url", default_value=""),
        DeclareLaunchArgument("asr_backend", default_value="mock"),
        DeclareLaunchArgument("llm_api_url", default_value=""),
        DeclareLaunchArgument("llm_model", default_value="gpt-4o-mini"),
        DeclareLaunchArgument("llm_api_key_env", default_value="TJ_LLM_API_KEY"),
        DeclareLaunchArgument("system_prompt_file", default_value=default_prompt),
        DeclareLaunchArgument(
            "parsed_intent_topic",
            default_value="/interaction/parsed_intent",
        ),
        DeclareLaunchArgument("task_events_topic", default_value="/task/events"),
        DeclareLaunchArgument("navigate_to_pose_action", default_value="navigate_to_pose"),
        DeclareLaunchArgument("map_yaml", default_value=default_map),
        DeclareLaunchArgument("mic_device_index", default_value="-1"),
        DeclareLaunchArgument(
            "mic_speech_rms_threshold",
            default_value="0.02",
            description="whisper_mic RMS 门限，环境噪大可用 0.025~0.04",
        ),
        DeclareLaunchArgument("mic_min_speech_sec", default_value="0.38"),
        DeclareLaunchArgument("mic_health_log_path", default_value=""),
        DeclareLaunchArgument("mic_health_interval_sec", default_value="5.0"),
        DeclareLaunchArgument("mic_stall_alert_sec", default_value="4.0"),
        DeclareLaunchArgument("enable_sim_speech_gui", default_value="false"),
        DeclareLaunchArgument("pick_min_distance_m", default_value="0.0"),
        DeclareLaunchArgument("pick_max_distance_m", default_value="1.5"),
        DeclareLaunchArgument("pick_max_yaw_error_deg", default_value="90.0"),
        DeclareLaunchArgument("pick_delay_min_sec", default_value="2.0"),
        DeclareLaunchArgument("pick_delay_max_sec", default_value="5.0"),
        _inc(
            share,
            "interaction.launch.py",
            {
                "use_sim_time": use_sim_time,
                "enable_mock_voice": enable_mock_voice,
                "speech_text_log_path": speech_text_log_path,
                "voice_api_url": voice_api_url,
                "asr_backend": asr_backend,
                "llm_api_url": llm_api_url,
                "llm_model": llm_model,
                "llm_api_key_env": llm_api_key_env,
                "system_prompt_file": system_prompt_file,
                "mic_device_index": mic_device_index,
                "mic_speech_rms_threshold": mic_speech_rms_threshold,
                "mic_min_speech_sec": mic_min_speech_sec,
                "mic_health_log_path": mic_health_log_path,
                "mic_health_interval_sec": mic_health_interval_sec,
                "mic_stall_alert_sec": mic_stall_alert_sec,
                "enable_sim_speech_gui": enable_sim_speech_gui,
            },
        ),
        _inc(
            share,
            "task_manager.launch.py",
            {
                "use_sim_time": use_sim_time,
                "parsed_intent_topic": parsed_intent_topic,
                "task_events_topic": task_events_topic,
                "navigate_to_pose_action": navigate_to_pose_action,
                "map_yaml": map_yaml,
            },
        ),
        _inc(
            share,
            "manipulation.launch.py",
            {
                "use_sim_time": use_sim_time,
                "pick_min_distance_m": pick_min_distance_m,
                "pick_max_distance_m": pick_max_distance_m,
                "pick_max_yaw_error_deg": pick_max_yaw_error_deg,
                "pick_delay_min_sec": pick_delay_min_sec,
                "pick_delay_max_sec": pick_delay_max_sec,
            },
        ),
    ])
