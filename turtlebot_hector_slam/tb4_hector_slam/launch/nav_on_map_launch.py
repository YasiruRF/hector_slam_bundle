import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Phase 2: autonomous navigation on a map saved from a Hector run.

    Hector must NOT be running: here map_server publishes /map and AMCL
    publishes map -> odom. Running both would give two /map and two
    map -> odom sources fighting each other.
    """
    pkg_dir = get_package_share_directory('tb4_hector_slam')
    nav2_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml = LaunchConfiguration('map')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'autostart': 'true',
        }.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'nav2_params_file': nav2_params_file,
        }.items())

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'hector_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Full path to the saved map .yaml (from map_saver_cli)'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2.yaml'),
            description='Full path to the Nav2 parameters file (with amcl/map_server)'),
        # localization_launch only uses `namespace` to rewrite the params; the
        # nodes must be pushed into it explicitly (navigation_launch does its own).
        GroupAction([
            PushRosNamespace(namespace),
            localization,
        ]),
        nav,
        rviz,
    ])
