# pyright: reportMissingImports=false
"""启动 YOLO-Seg 人体验证节点，并可一键附带「人物激光链」（默认精简）：

- mark_then_strip（默认）：YOLO + person_strip_recorder（map 上人点 + YAML）+ 方位角 Marker；**不**起整帧彩色 scan 点云（省一倍 /scan 处理）
- filtered：scan_person_filter；可选 enable_scan_map_colored:=true 再启整帧 map 上色（无 strip 时便于看人向激光）

单相机（仿真）：
  ros2 launch human_yolo_seg yolo_person_seg.launch.py use_sim_time:=true

仅人物链、不起 YOLO（双相机第三路）：
  ros2 launch human_yolo_seg yolo_person_seg.launch.py use_sim_time:=true enable_yolo:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(s: str) -> bool:
    return str(s).strip().lower() in ('true', '1', 'yes')


def _launch_setup(context, *_args, **_kwargs):
    use_sim = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    enable_yolo = _as_bool(LaunchConfiguration('enable_yolo').perform(context))
    enable_pipeline = _as_bool(LaunchConfiguration('enable_person_map_pipeline').perform(context))
    enable_markers = _as_bool(LaunchConfiguration('enable_person_azimuth_markers').perform(context))
    mode = LaunchConfiguration('person_slam_mode').perform(context).strip().lower()
    conf = float(LaunchConfiguration('conf_threshold').perform(context))
    imgsz = int(LaunchConfiguration('imgsz').perform(context))
    image_topic = LaunchConfiguration('image_topic').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    device = LaunchConfiguration('device').perform(context)
    pub_stats = _as_bool(LaunchConfiguration('publish_detection_stats').perform(context))
    stats_prefix = LaunchConfiguration('stats_topic_prefix').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    pub_az = _as_bool(LaunchConfiguration('publish_person_azimuths').perform(context))
    person_azimuth_topic = LaunchConfiguration('person_azimuth_topic').perform(context)
    laser_frame_id = LaunchConfiguration('laser_frame_id').perform(context)
    ground_frame_id = LaunchConfiguration('ground_frame_id').perform(context)
    person_azimuth_mode = LaunchConfiguration('person_azimuth_mode').perform(context).strip().lower()
    linear_hfov = float(LaunchConfiguration('person_azimuth_linear_hfov_deg').perform(context))
    linear_yaw = float(LaunchConfiguration('person_azimuth_linear_camera_yaw_deg').perform(context))
    linear_use_mask = _as_bool(LaunchConfiguration('person_azimuth_linear_use_mask').perform(context))
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    limit_fov = _as_bool(LaunchConfiguration('limit_scan_to_fov').perform(context))
    fov_min = float(LaunchConfiguration('fov_min_deg').perform(context))
    fov_max = float(LaunchConfiguration('fov_max_deg').perform(context))
    scan_filtered_out = LaunchConfiguration('scan_filtered_topic').perform(context)
    enable_colored = _as_bool(LaunchConfiguration('enable_scan_map_colored').perform(context))

    actions = []

    if enable_yolo:
        actions.append(
            Node(
                package='human_yolo_seg',
                executable='yolo_person_seg_node',
                name='yolo_person_seg',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim,
                        'image_topic': image_topic,
                        'output_topic': output_topic,
                        'model_path': model_path,
                        'conf_threshold': conf,
                        'imgsz': imgsz,
                        'device': device,
                        'publish_detection_stats': pub_stats,
                        'stats_topic_prefix': stats_prefix,
                        'camera_info_topic': camera_info_topic,
                        'publish_person_azimuths': pub_az,
                        'person_azimuth_topic': person_azimuth_topic,
                        'laser_frame_id': laser_frame_id,
                        'ground_frame_id': ground_frame_id,
                        'person_azimuth_mode': person_azimuth_mode,
                        'person_azimuth_linear_hfov_deg': linear_hfov,
                        'person_azimuth_linear_camera_yaw_deg': linear_yaw,
                        'person_azimuth_linear_use_mask': linear_use_mask,
                    },
                ],
            ),
        )

    if enable_pipeline:
        if enable_colored:
            actions.append(
                Node(
                    package='human_yolo_seg',
                    executable='scan_map_colored_cloud_node',
                    name='scan_map_colored_cloud',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': use_sim,
                            'scan_topic': scan_topic,
                            'azimuth_topic': person_azimuth_topic,
                        },
                    ],
                ),
            )
        if mode == 'filtered':
            actions.append(
                Node(
                    package='human_yolo_seg',
                    executable='scan_person_filter_node',
                    name='scan_person_filter',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': use_sim,
                            'scan_in': scan_topic,
                            'scan_out': scan_filtered_out,
                            'azimuth_topic': person_azimuth_topic,
                            'limit_scan_to_fov': limit_fov,
                            'fov_min_deg': fov_min,
                            'fov_max_deg': fov_max,
                        },
                    ],
                ),
            )
        else:
            actions.append(
                Node(
                    package='human_yolo_seg',
                    executable='person_strip_recorder_node',
                    name='person_strip_recorder',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': use_sim,
                            'scan_topic': scan_topic,
                            'azimuth_topic': person_azimuth_topic,
                        },
                    ],
                ),
            )

    # 仅随人物激光链启一次 markers（单路 YOLO 时与节点同启即可）
    if enable_markers and enable_pipeline:
        actions.append(
            Node(
                package='human_yolo_seg',
                executable='person_azimuth_markers_node',
                name='person_azimuth_markers',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim,
                        'azimuth_topic': person_azimuth_topic,
                        'laser_frame_id': laser_frame_id,
                    },
                ],
            ),
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gazebo / 仿真时钟时设为 true',
        ),
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='true',
            description='是否启动 yolo_person_seg_node（双相机时前两路可 false+仅 pipeline 第三路）',
        ),
        DeclareLaunchArgument(
            'enable_person_map_pipeline',
            default_value='true',
            description='是否启动人物激光链：mark_then_strip=strip+方位角 Marker；filtered=scan_filter',
        ),
        DeclareLaunchArgument(
            'enable_scan_map_colored',
            default_value='false',
            description='是否额外启动 scan_map_colored_cloud（整帧 /scan 投 map 上色；默认关以省算力；filtered 且需 map 上人向可看可开 true）',
        ),
        DeclareLaunchArgument(
            'enable_person_azimuth_markers',
            default_value='true',
            description='是否在 RViz 用 MarkerArray 显示人物方位角弧段（话题 /human_yolo/person_azimuth_markers）',
        ),
        DeclareLaunchArgument(
            'person_slam_mode',
            default_value='mark_then_strip',
            description='mark_then_strip | filtered（与 TB3_PERSON_SLAM_MODE 一致）',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan 输入（person_strip / 可选 colored / filter）',
        ),
        DeclareLaunchArgument(
            'scan_filtered_topic',
            default_value='/scan_filtered',
            description='filtered 模式下 scan_person_filter 输出话题',
        ),
        DeclareLaunchArgument(
            'limit_scan_to_fov',
            default_value='false',
            description='filtered 模式下是否按相机 FOV 截断 scan（与 TB3_SCAN_FOV_LIMIT 对应）',
        ),
        DeclareLaunchArgument('fov_min_deg', default_value='-70.0'),
        DeclareLaunchArgument('fov_max_deg', default_value='70.0'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='RGB 输入（TurtleBot3 Waffle 仿真常见为此话题）',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/human_yolo/annotated_image',
            description='叠加分割结果，供 RViz Image 显示',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolo26n-seg.pt',
            description='权重文件名（在 share/.../models/）或绝对路径',
        ),
        DeclareLaunchArgument('conf_threshold', default_value='0.25'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument(
            'device',
            default_value='auto',
            description='auto=有 CUDA 则用 cuda:0 否则 cpu；也可 cuda:0 / cpu / 0',
        ),
        DeclareLaunchArgument(
            'publish_detection_stats',
            default_value='true',
            description='是否发布 /human_yolo/person_count 等统计话题（建模验收；不需要可 false）',
        ),
        DeclareLaunchArgument(
            'stats_topic_prefix',
            default_value='/human_yolo',
            description='统计话题前缀：{prefix}/person_count、person_max_conf、person_present',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
            description='与 RGB 对齐的 CameraInfo（用于人物方位角 / scan 掩膜）',
        ),
        DeclareLaunchArgument(
            'publish_person_azimuths',
            default_value='true',
            description='是否发布 person_azimuth（sensor_msgs/JointState，header=RGB 时间戳，供 scan 同步）',
        ),
        DeclareLaunchArgument(
            'person_azimuth_topic',
            default_value='/human_yolo/person_azimuth_ranges',
            description='方位角区间话题 [lo0,hi0,...]（弧度，激光系）',
        ),
        DeclareLaunchArgument(
            'laser_frame_id',
            default_value='base_scan',
            description='与 LaserScan.header.frame_id 一致的 TF frame（算 atan2）',
        ),
        DeclareLaunchArgument(
            'ground_frame_id',
            default_value='base_footprint',
            description='地面近似平面所在 frame（z=0 求交）',
        ),
        DeclareLaunchArgument(
            'person_azimuth_mode',
            default_value='linear_fov',
            description='linear_fov=框横坐标×水平视场（默认粗略、无 TF）；tf_geometry=内参+TF+地面',
        ),
        DeclareLaunchArgument(
            'person_azimuth_linear_hfov_deg',
            default_value='0.0',
            description='linear_fov 时水平视场角（度）；0=由 CameraInfo 的 fx 推算，若无 CameraInfo 则用 135°',
        ),
        DeclareLaunchArgument(
            'person_azimuth_linear_camera_yaw_deg',
            default_value='0.0',
            description='linear_fov 时相机光轴相对激光 forward 的 yaw（度），前视与 base_scan 一致时常为 0',
        ),
        DeclareLaunchArgument(
            'person_azimuth_linear_use_mask',
            default_value='false',
            description='linear_fov 时是否用分割 mask 底边代替框的横坐标',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
