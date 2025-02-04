from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    
    # wheel_radius_arg = DeclareLaunchArgument(
    #     "wheel_radius",
    #     default_value="0.1",
    # )
    # wheel_separation_arg = DeclareLaunchArgument(
    #     "wheel_separation",
    #     default_value="0.4",
    # )
    
    # wheel_radius = LaunchConfiguration("wheel_radius")
    # wheel_separation = LaunchConfiguration("wheel_separation")
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["new_robot_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            # use_simple_controller_arg,
            # use_python_arg,
            # wheel_radius_arg,
            # wheel_separation_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            # simple_controller,
        ]
    )