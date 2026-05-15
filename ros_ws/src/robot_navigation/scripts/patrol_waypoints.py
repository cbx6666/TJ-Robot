#!/usr/bin/env python3
"""Patrol executable.

核心实现拆分在 navigation_manager.py 及其配套模块中；此文件只保留 ROS
入口，方便 ros2 launch 执行。
"""

from __future__ import annotations

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

from navigation_manager import NavigationManager


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()
    manager = NavigationManager(navigator)
    try:
        manager.run()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
