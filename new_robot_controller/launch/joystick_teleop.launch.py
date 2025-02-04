from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_teleop = os.path.join(get_package_share_directory('new_robot_controller'),'config','joy_teleop.yaml')
    joy_params = os.path.join(get_package_share_directory('new_robot_controller'),'config','joy_config.yaml')
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': True}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_teleop, {'use_sim_time': True}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )


    return LaunchDescription([
        joy_node,
        teleop_node,
        # twist_stamper       
    ])