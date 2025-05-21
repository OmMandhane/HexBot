from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import UnlessCondition, IfCondition

# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    vfh_algo = Node(
        package="vfh",
        executable="vfh_3",
    )   

    vfh_control = Node(
        package="vfh",
        executable="controller",
    )     

    return LaunchDescription(
        [
            vfh_algo,
            vfh_control,
        ]
    )

