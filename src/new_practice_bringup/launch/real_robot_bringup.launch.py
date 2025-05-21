import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("new_robot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("new_robot_controller"),
            "launch",
            "simple_controller.launch.py"
        )
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("new_robot_controller"),
        "config",
        "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/new_robot_controller/cmd_vel_unstamped')]
    )
    
    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("new_robot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    # )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        twist_mux,
    ])