# pyright: reportMissingImports=false
"""深度图 -> PointCloud2 -> LaserScan(/scan_rgbd)，供 slam_toolbox 使用。

默认话题对齐本仓库合并的 Gazebo 插件（camera_name=tb3_depth_only，挂在 camera_depth_frame）：
  /tb3_depth_only/depth/image_raw + /tb3_depth_only/depth/camera_info
若实际与模型不一致，请用 launch 参数覆盖（可先 ros2 topic list 确认）。

另启动 depth_image_to_viz：把 32FC1/16UC1 转为 mono8，便于 RViz Image 显示。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('robot_bringup')
    viz_py = os.path.join(share, 'scripts', 'depth_image_to_viz.py')
    depth_in = LaunchConfiguration('depth_image_topic').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time')
    depth_image = LaunchConfiguration('depth_image_topic')
    depth_info = LaunchConfiguration('depth_camera_info_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    target_frame = LaunchConfiguration('target_frame')

    return [
        ExecuteProcess(
            cmd=[
                'python3',
                viz_py,
                '--ros-args',
                '-p',
                'use_sim_time:=true',
                '-p',
                f'in_topic:={depth_in}',
            ],
            output='screen',
        ),
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_to_cloud',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('image_rect', depth_image),
                ('camera_info', depth_info),
                ('points', cloud_topic),
            ],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='rgbd_to_laserscan',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {
                    'target_frame': target_frame,
                    'transform_tolerance': 0.05,
                    'min_height': 0.15,
                    'max_height': 0.65,
                    'angle_min': -1.0,
                    'angle_max': 1.0,
                    'angle_increment': 0.00872,
                    'scan_time': 0.1,
                    'range_min': 0.35,
                    # 与 Gazebo 深度相机 far clip / TurtleBot3 激光量程上限（约 3.5 m）一致
                    'range_max': 3.5,
                    'use_inf': True,
                    'concurrency_level': 1,
                },
            ],
            remappings=[
                ('cloud_in', cloud_topic),
                ('scan', scan_topic),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='与 Gazebo 仿真时钟一致',
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/tb3_depth_only/depth/image_raw',
            description='仿真深度图（与 URDF 片段中 camera_name 一致）',
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/tb3_depth_only/depth/camera_info',
            description='与深度图同分辨率的 CameraInfo',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/rgbd/depth_points',
            description='中间点云话题',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_rgbd',
            description='输出虚拟激光，供 slam_toolbox 订阅',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_footprint',
            description='pointcloud_to_laserscan 投影坐标系',
        ),
        OpaqueFunction(function=launch_setup),
    ])
