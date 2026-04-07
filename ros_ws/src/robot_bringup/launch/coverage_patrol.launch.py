# pyright: reportMissingImports=false
"""Launch coverage_patrol after tb3_stack.sh has started the sim stack.

When YOLO is enabled, slam_toolbox should consume /scan_filtered for mapping,
while coverage_patrol keeps raw /scan for short-range obstacle avoidance.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('include_people_passes', default_value='true'),
        DeclareLaunchArgument('edge_margin', default_value='1.35'),
        DeclareLaunchArgument('linear_speed_limit', default_value='0.30'),
        DeclareLaunchArgument('obstacle_distance', default_value='0.50'),
        DeclareLaunchArgument('goal_stuck_sec', default_value='4.0'),
        Node(
            package='robot_navigation',
            executable='coverage_patrol',
            name='coverage_patrol',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'include_people_passes': LaunchConfiguration('include_people_passes'),
                'edge_margin': LaunchConfiguration('edge_margin'),
                'linear_speed_limit': LaunchConfiguration('linear_speed_limit'),
                'obstacle_distance': LaunchConfiguration('obstacle_distance'),
                'goal_stuck_sec': LaunchConfiguration('goal_stuck_sec'),
            }],
        ),
    ])
