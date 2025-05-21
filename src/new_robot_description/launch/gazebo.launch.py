import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paths to packages and files
    new_robot_description = get_package_share_directory("new_robot_description")
    new_robot_description_prefix = get_package_prefix("new_robot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Declare robot model argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(new_robot_description, "urdf", "my_robot.urdf.xacro"),
        description="Absolute path to robot URDF file",
    )

    # Path to world file
    world_path = os.path.join(new_robot_description, "worlds", "warehouse.world")

    # Set GAZEBO_MODEL_PATH environment variable
    model_path = os.path.join(new_robot_description, "models")
    model_path += pathsep + os.path.join(new_robot_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Robot description
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    # Nodes to spawn controllers
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

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},
        ],
        output="screen",
    )

    # Gazebo parameters file
    gazebo_params_file = os.path.join(
        get_package_share_directory("new_robot_description"), "config", "gazebo_params.yaml"
    )

    # Start Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )

    # Start Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "my_robot",
            "-topic",
            "robot_description",
        ],
        output="screen",

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

    # Launch description
    return LaunchDescription(
        [
            env_var,
            model_arg,
            start_gazebo_server,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            start_gazebo_client,
            robot_state_publisher_node,
            spawn_robot,
            twist_mux
        ]
    )
