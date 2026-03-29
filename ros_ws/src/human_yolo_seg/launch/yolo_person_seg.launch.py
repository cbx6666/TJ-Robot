# pyright: reportMissingImports=false
"""启动 YOLO-Seg 人体验证节点（不依赖人物模型已就绪；有图像即可测）。

示例（仿真 + use_sim_time）：
  ros2 launch human_yolo_seg yolo_person_seg.launch.py use_sim_time:=true

默认使用包内 models/yolo26n-seg.pt（需先放入并 colcon build）；或：
  ros2 launch human_yolo_seg yolo_person_seg.launch.py model_path:=/绝对路径/xxx.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    conf = float(LaunchConfiguration('conf_threshold').perform(context))
    imgsz = int(LaunchConfiguration('imgsz').perform(context))
    image_topic = LaunchConfiguration('image_topic').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    device = LaunchConfiguration('device').perform(context)
    pub_stats = LaunchConfiguration('publish_detection_stats').perform(context).lower() in (
        'true',
        '1',
        'yes',
    )
    stats_prefix = LaunchConfiguration('stats_topic_prefix').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    pub_az = LaunchConfiguration('publish_person_azimuths').perform(context).lower() in (
        'true',
        '1',
        'yes',
    )
    person_azimuth_topic = LaunchConfiguration('person_azimuth_topic').perform(context)
    laser_frame_id = LaunchConfiguration('laser_frame_id').perform(context)
    ground_frame_id = LaunchConfiguration('ground_frame_id').perform(context)

    return [
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
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gazebo / 仿真时钟时设为 true',
        ),
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
            description='是否发布 /human_yolo/person_count 等统计话题（建模验收）',
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
        OpaqueFunction(function=_launch_setup),
    ])
