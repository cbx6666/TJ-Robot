# pyright: reportMissingImports=false
"""RGB-D 建图：深度 -> /scan_rgbd -> Cartographer 2D（质量优先可选后端）。

与 slam_rgbd.launch.py（slam_toolbox）共用同一条 depth -> 虚拟激光链；仅 SLAM 后端不同。
依赖系统包：sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

若启动报错找不到 occupancy_grid_node，请执行：
  ros2 pkg executables cartographer_ros
并对照 Humble 发行版可执行文件名修改本 launch 中 occupancy 节点。
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
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_camera_info_topic = LaunchConfiguration('depth_camera_info_topic')

    bringup_share = get_package_share_directory('robot_bringup')
    cart_cfg_dir = os.path.join(bringup_share, 'config', 'cartographer')

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
            ('scan_topic', '/scan_rgbd'),
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cart_cfg_dir,
            '-configuration_basename', 'turtlebot3_rgbd_2d.lua',
        ],
        remappings=[
            ('scan', '/scan_rgbd'),
        ],
    )

    # Humble Debian 中可执行名多为 occupancy_grid_node；若不符请按 `ros2 pkg executables` 修改
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '0.5',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/tb3_depth_only/depth/image_raw',
            description='深度图（与 URDF 深度插件 camera_name 一致）',
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/tb3_depth_only/depth/camera_info',
            description='深度 CameraInfo',
        ),
        rgbd_to_scan,
        cartographer_node,
        occupancy_grid_node,
    ])
