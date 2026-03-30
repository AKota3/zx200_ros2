import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    zx200_navigation_dir=get_package_share_directory("zx200_navigation")

    ekf_localization_launch_file_path=os.path.join(zx200_navigation_dir,"launch","ekf_localization.launch.py")
    zx200_navigation_launch_file_path=os.path.join(zx200_navigation_dir,"launch","zx200_navigation.launch.py")

    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zx200_navigation_launch_file_path),
        ),
#         Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='world_to_map',
#         arguments=['--x','21395.178',
#         '--y','14034.450',
#         '--z','28.552',
#         '--roll','0',
#  '      --pitch','0',
#         '--yaw','0',
#         '--frame-id', 'world',
#         '--child-frame-id', 'map']),
        Node(
            package='zx200_navigation',
            executable='world_coodinate_converter',
        )



    ])