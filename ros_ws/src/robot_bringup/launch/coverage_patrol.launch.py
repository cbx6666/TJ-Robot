# pyright: reportMissingImports=false
"""Launch coverage_patrol after tb3_stack.sh has started the sim stack.

slam_toolbox may use /scan_filtered for mapping; coverage_patrol uses raw /scan.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('route_margin', default_value='0.7'),
        DeclareLaunchArgument('linear_speed_limit', default_value='0.60'),
        DeclareLaunchArgument('obstacle_distance', default_value='0.50'),
        DeclareLaunchArgument('goal_stuck_sec', default_value='4.0'),
        DeclareLaunchArgument('route_goal_stuck_sec', default_value='7.5'),
        DeclareLaunchArgument('slowdown_distance', default_value='2.20'),
        DeclareLaunchArgument('wall_follow_side', default_value='right'),
        DeclareLaunchArgument('perimeter_min_traveled_m', default_value='22.0'),
        DeclareLaunchArgument('perimeter_lap_radius_m', default_value='1.05'),
        DeclareLaunchArgument('perimeter_lap_min_separation_m', default_value='2.8'),
        DeclareLaunchArgument('perimeter_force_finish_traveled_m', default_value='0.0'),
        DeclareLaunchArgument('p1_corner_arc_chord_m', default_value='0.30'),
        DeclareLaunchArgument('p1_corner_turn_threshold_deg', default_value='70.0'),
        DeclareLaunchArgument('perimeter_pure_turn_drive', default_value='true'),
        DeclareLaunchArgument('perimeter_align_heading_deg', default_value='6.0'),
        DeclareLaunchArgument('laser_range_fallback_m', default_value='8.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('save_map_on_complete', default_value='false'),
        DeclareLaunchArgument('save_map_directory', default_value=''),
        DeclareLaunchArgument('save_map_basename', default_value=''),
        DeclareLaunchArgument('save_map_delay_sec', default_value='2.0'),
        DeclareLaunchArgument('exit_after_map_save', default_value='true'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('post_save_person_overlay', default_value='false'),
        DeclareLaunchArgument(
            'person_regions_yaml',
            default_value='',
            description='人物区域 YAML；空则 ~/.ros/tj_person_strip_regions.yaml',
        ),
        DeclareLaunchArgument('snapshot_overlay_from_cloud_if_no_regions', default_value='true'),
        DeclareLaunchArgument(
            'person_laser_map_cloud_topic',
            default_value='/human_yolo/person_laser_map_cloud',
        ),
        Node(
            package='robot_navigation',
            executable='coverage_patrol',
            name='coverage_patrol',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'route_margin': ParameterValue(LaunchConfiguration('route_margin'), value_type=float),
                'linear_speed_limit': LaunchConfiguration('linear_speed_limit'),
                'obstacle_distance': LaunchConfiguration('obstacle_distance'),
                'goal_stuck_sec': LaunchConfiguration('goal_stuck_sec'),
                'route_goal_stuck_sec': ParameterValue(
                    LaunchConfiguration('route_goal_stuck_sec'), value_type=float
                ),
                'slowdown_distance': ParameterValue(
                    LaunchConfiguration('slowdown_distance'), value_type=float
                ),
                'wall_follow_side': LaunchConfiguration('wall_follow_side'),
                'perimeter_min_traveled_m': ParameterValue(
                    LaunchConfiguration('perimeter_min_traveled_m'), value_type=float
                ),
                'perimeter_lap_radius_m': ParameterValue(
                    LaunchConfiguration('perimeter_lap_radius_m'), value_type=float
                ),
                'perimeter_lap_min_separation_m': ParameterValue(
                    LaunchConfiguration('perimeter_lap_min_separation_m'), value_type=float
                ),
                'perimeter_force_finish_traveled_m': ParameterValue(
                    LaunchConfiguration('perimeter_force_finish_traveled_m'), value_type=float
                ),
                'p1_corner_arc_chord_m': ParameterValue(
                    LaunchConfiguration('p1_corner_arc_chord_m'), value_type=float
                ),
                'p1_corner_turn_threshold_deg': ParameterValue(
                    LaunchConfiguration('p1_corner_turn_threshold_deg'), value_type=float
                ),
                'perimeter_pure_turn_drive': LaunchConfiguration('perimeter_pure_turn_drive'),
                'perimeter_align_heading_deg': ParameterValue(
                    LaunchConfiguration('perimeter_align_heading_deg'), value_type=float
                ),
                'laser_range_fallback_m': ParameterValue(
                    LaunchConfiguration('laser_range_fallback_m'), value_type=float
                ),
                # 节点侧 declare 为字符串；须 value_type=str，否则 YAML 会变成 bool 与 STRING 声明冲突
                'save_map_on_complete': ParameterValue(
                    LaunchConfiguration('save_map_on_complete'), value_type=str
                ),
                'save_map_directory': LaunchConfiguration('save_map_directory'),
                'save_map_basename': LaunchConfiguration('save_map_basename'),
                'save_map_delay_sec': ParameterValue(
                    LaunchConfiguration('save_map_delay_sec'), value_type=float
                ),
                'exit_after_map_save': ParameterValue(
                    LaunchConfiguration('exit_after_map_save'), value_type=str
                ),
                'map_topic': LaunchConfiguration('map_topic'),
                'post_save_person_overlay': ParameterValue(
                    LaunchConfiguration('post_save_person_overlay'), value_type=str
                ),
                'person_regions_yaml': LaunchConfiguration('person_regions_yaml'),
                'snapshot_overlay_from_cloud_if_no_regions': ParameterValue(
                    LaunchConfiguration('snapshot_overlay_from_cloud_if_no_regions'), value_type=str
                ),
                'person_laser_map_cloud_topic': LaunchConfiguration('person_laser_map_cloud_topic'),
            }],
        ),
    ])
