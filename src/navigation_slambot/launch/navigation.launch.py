import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('navigation_slambot'),'config')
    map_file = os.path.join(config_dir,'new_map.yaml')
    param_file = os.path.join(config_dir,'nav2_params.yaml')
    rviz_config_dir = os.path.join(config_dir,'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('new_robot_description'),'/launch','/gazebo.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': param_file}.items(),

        ),
    Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('navigation_slambot'), 'config', 'new_map.yaml')}]
    ),
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_dir],
        output='screen'

        ),
    ])