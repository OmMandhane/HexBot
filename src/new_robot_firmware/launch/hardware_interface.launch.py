import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    # Robot description from xacro
    robot_description = Command([
        "xacro ", os.path.join(
            get_package_share_directory("new_robot_description"),
            "urdf",
            "my_robot.urdf.xacro"
        ), 
        " is_sim:=False"
    ])

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Twist mux configuration
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

    # Controller Manager Node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("new_robot_controller"),
                "config",
                "new_robot_controllers.yaml",
            ),
        ],
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    # Diff Drive Controller Spawner
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["new_robot_controller"]
    )

    # Delay controller manager to allow robot description to load
    delayed_controller_manager = TimerAction(
        period=3.0, actions=[controller_manager]
    )

    # Spawn joint_state_broadcaster after controller_manager starts
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Spawn wheel controller after joint_state_broadcaster starts
    delayed_wheel_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[wheel_controller_spawner],
        )
    )
    lidar = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("hls_lfcd_lds_driver"),
            "launch",
            "hlds_laser.launch.py"
        ),
    )
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("new_slam"),
            "launch",
            "slam.launch.py"
        ),
    )
    # Launch description
    return LaunchDescription([
        robot_state_publisher_node,
        twist_mux,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_wheel_controller_spawner,
        lidar,
        slam,  
    ])