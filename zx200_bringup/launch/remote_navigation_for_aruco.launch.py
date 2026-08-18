import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    zx200_navigation_dir=get_package_share_directory("zx200_navigation")

    ekf_localization_launch_file_path=os.path.join(
        zx200_navigation_dir,"launch","ekf_localization_for_aruco.launch.py"
    )
    zx200_navigation_launch_file_path=os.path.join(
        zx200_navigation_dir,"launch","zx200_navigation.launch.py"
    )
    zx200_unity_dir = get_package_share_directory("zx200_unity")
    rviz_file = os.path.join(
        zx200_unity_dir, "rviz2", "zx200_standby.rviz"
    )
    zx200_description_path = get_package_share_path('zx200_description')
    default_model_path = zx200_description_path / 'urdf/zx200.xacro'
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='zx200')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    straight_only_arg = DeclareLaunchArgument('straight_only', default_value='false')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),description='Absolute path to robot urdf file')



    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    straight_only = LaunchConfiguration('straight_only')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,
        straight_only_arg,
        model_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
            launch_arguments={
                'robot_name': robot_name,
                'use_namespace': 'true',
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zx200_navigation_launch_file_path),
            launch_arguments={
                'robot_name': robot_name,
                'use_namespace': 'true',
                'use_sim_time': use_sim_time,
                'straight_only': straight_only,
            }.items(),
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
            parameters=[{
                'offset_x': 0.0,
                'offset_y': 0.0,
                'offset_z': 0.0,
            }],
        ),

        GroupAction([

            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}]
            ),

            Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["--display-config", rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        ])



    ])