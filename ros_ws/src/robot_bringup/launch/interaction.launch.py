# pyright: reportMissingImports=false
"""Interaction bringup placeholder for ASR, NLU, dialog, and TTS."""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [LogInfo(msg="Interaction layer reserved: ASR -> NLU -> task manager -> TTS.")]
    )
