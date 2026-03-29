# pyright: reportMissingImports=false
"""RGB-D 建图（无 YOLO）：深度 -> 点云 -> /scan_rgbd -> slam_toolbox。

要求：TurtleBot3 **Waffle** 或 **Waffle Pi**（带深度相机仿真）。
激光建图请用 slam_laser.launch.py + Burger 或 Waffle 上的 /scan。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_camera_info_topic = LaunchConfiguration('depth_camera_info_topic')

    default_rgbd_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'mapper_params_online_async_rgbd.yaml',
    )

    rgbd_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'rgbd_to_scan.launch.py',
            ]),
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('depth_image_topic', depth_image_topic),
            ('depth_camera_info_topic', depth_camera_info_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock',
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/tb3_depth_only/depth/image_raw',
            description='深度图话题（与 waffle_depth_gazebo_fragment 中 camera_name 一致）',
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/tb3_depth_only/depth/camera_info',
            description='深度 CameraInfo',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_rgbd_params,
            description='slam_toolbox 参数（须含 scan_topic: /scan_rgbd）',
        ),
        rgbd_to_scan,
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
