from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use sim time'
        ),
        DeclareLaunchArgument(
            name='namespace',
            default_value='/husky',
            description='Namespace to run stella slam nodes in'
        ),
        DeclareLaunchArgument(
            name='config',
            default_value='/WorkingData/Stella/config.yaml',
            description='Namespace to run stella slam nodes in'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config')
    orb_vocab_file = PathJoinSubstitution(['/WorkingData/Stella/', 'orb_vocab.fbow'])
    
    stella_slam_ros_pkg_share = FindPackageShare('stella_vslam_ros')
    
    localization_param_file=PathJoinSubstitution([stella_slam_ros_pkg_share, 'config', 'localization.yaml'])
    
    slam_group = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='stella_vslam_ros',
            executable='run_slam',
            name="stella_slam",
            output='screen',
            arguments=[
                ['-c', config_file],
                ['-v', orb_vocab_file]
            ],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'odom2d': True},
                {'camera_frame': 'camnav_link'}
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('camera/color/image_raw', 'sensors/camnav/color/image_raw'),
                ('camera/depth/image_raw', 'sensors/camnav/depth/image_rect_raw'),
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='stella_ekf_node',
            output='screen',
            parameters=[
                ParameterFile(param_file=localization_param_file, allow_substs=True),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[    
              ('odometry/filtered', 'sensors/odom/filtered'),
              ('/diagnostics', 'sensors/diagnostics'),
              ('/tf', 'tf'),
              ('/tf_static', 'tf_static'),
            ]
        )
    ])

    nodes = [
        slam_group,
    ]

    return LaunchDescription(declared_arguments + nodes)
