import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='zx200')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    zx200_navigation_dir=get_package_share_directory("zx200_navigation")
    zx200_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(zx200_navigation_dir, 'config', 'zx200_ekf.yaml'))

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            Node(
                package='zx200_navigation',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[
                    {'odom_topic': 'odom_pose', 
                     'odom_frame': '/odom',
                     'base_link_frame': '/base_link',
                     'use_sim_time': use_sim_time,
                    },
                ]
            ),
            Node(
                package='zx200_navigation',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "map",
                                'odom_child_frame': "base_link",
                                'poseStamped_topic_name': "global_pose",
                                'odom_topic_name': "global_pose_odom",
                                'use_sim_time': use_sim_time}]
            ),     
            # Node(
            #     package = 'zx200_navigation',
            #     executable = 'message_converter_odom',
            #     name = "message_converter_odom",
            #     output = "screen",
            #     parameters=[{'input_topic': "odom",
            #                 'output_topic': 'fixed_odom',
            #                 'use_sim_time': use_sim_time}],
            # ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output='screen',
                remappings=[('odometry/filterd','odometry/global_output')],
                parameters=[zx200_ekf_yaml_file,
                            {
                                'odom0' : 'odom_pose',
                                'odom1' : 'global_pose_odom_map',

                            }]
            ),
            Node(
                package='zx200_navigation',
                executable='connection_ditector',
                name='safety_node',
                output='screen',
                parameters=[{
                                'input_pose_topic' : 'global_pose',
                                'input_velosity_topic' : 'nav2_cmd_vel',
                                'output_velosity_topic' : 'cmd_vel_nav',
                                'pose_msg_type' : "pose_stamped",
                            }]
            )
        ])
    ])