from launch.actions import IncludeLaunchDescription, TimerAction
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Include the Gazebo launch file
    gazebo_sim = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("new_robot_description"),
                     "launch",
                     "gazebo.launch.py")
    )

    # Add a simple delay before launching the joystick and controller
    delayed_launch = TimerAction(
        period=6.0,  # Adjust the delay as needed (5 seconds here)
        actions=[
            IncludeLaunchDescription(
                os.path.join(get_package_share_directory("new_robot_controller"),
                             "launch",
                             "joystick_teleop.launch.py")
            ),
            IncludeLaunchDescription(
                os.path.join(get_package_share_directory("new_robot_controller"),
                             "launch",
                             "simple_controller.launch.py")
            )
        ]
    )

    return LaunchDescription([gazebo_sim, delayed_launch])

