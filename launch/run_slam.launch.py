from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition

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
        DeclareLaunchArgument(
            name='enable_visual_odom',
            default_value='false',
            choices=['false', 'true'],
            description='Publishes odom TF based on camera position'
        ),
        DeclareLaunchArgument(
            name='publish_tf',
            default_value='true',
            choices=['false', 'true'],
            description='Publishes map->odom TF based on camera position'
        ),
        DeclareLaunchArgument(
            name='odom2d',
            default_value='true',
            choices=['false', 'true'],
            description='Published map->odom TF is on XY plane'
        ),
        DeclareLaunchArgument(
            name='use_exact_time',
            default_value='false',
            choices=['false', 'true'],
            description=''
        ),
        DeclareLaunchArgument(
            name='vocab_file',
            default_value='/Models/orb_vocab.fbow',
            description='Features extraction vocabulary file',
        ),
        DeclareLaunchArgument(
            name='params_file',
            default_value=PathJoinSubstitution([FindPackageShare('stella_vslam_ros'), 'config', 'stella-vslam.yaml']),
            description=''
        ),
    ]
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config')
    publish_tf = LaunchConfiguration("publish_tf")
    odom2d = LaunchConfiguration('odom2d')
    use_exact_time = LaunchConfiguration('use_exact_time')
    
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
                ['-v', LaunchConfiguration('vocab_file')]
            ],
            parameters=[
                ParameterFile(param_file=LaunchConfiguration('params_file'), allow_substs=True),
                {'use_sim_time': use_sim_time}, 
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('camera/image_raw', 'camnav/color/image_raw'),
                ('camera/color/image_raw', 'camnav/color/image_raw'),
                ('camera/depth/image_raw', 'camnav/aligned_depth_to_color/image_raw'),
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
            ],
            condition=IfCondition(LaunchConfiguration('enable_visual_odom'))
        )
    ])

    nodes = [
        slam_group,
    ]

    return LaunchDescription(declared_arguments + nodes)
