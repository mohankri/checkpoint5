import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'attach_shelf'


def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument(
        'obstacle', default_value='0.2',
        description='Obstacle distance [m]')
    degrees_arg = DeclareLaunchArgument(
        'degrees', default_value='-90',
        description='Rotation after stop [deg]')
    rviz_config_file_name_arg = DeclareLaunchArgument(
        'rviz_config_file_name', default_value='checkpoint5.rviz',
        description='RViz config file name')
    final_approach_arg = DeclareLaunchArgument(
        'final_approach', default_value='false',
        description='Perform final approach to the shelf')

    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    rviz_config_file_name = LaunchConfiguration('rviz_config_file_name')
    final_approach = LaunchConfiguration('final_approach')


    pre_approach_v2 = Node(
        package=PACKAGE_NAME,
        executable='pre_approach_v2',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'obstacle': obstacle,
            'degrees': degrees,
            'final_approach': final_approach,
        }],
    )
    
    # deferred join: resolved at LAUNCH time, when the arg has a value
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(PACKAGE_NAME), 'rviz', rviz_config_file_name])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        rviz_config_file_name_arg,
        final_approach_arg,
        LogInfo(msg=['obstacle: ', obstacle]),
        LogInfo(msg=['degrees: ', degrees]),
        LogInfo(msg=['rviz config: ', rviz_config_file_name]),
        LogInfo(msg=['final_approach: ', final_approach]),
        pre_approach_v2,
        rviz_node,
    ])