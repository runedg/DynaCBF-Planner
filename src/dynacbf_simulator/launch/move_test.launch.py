import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('dynacbf_simulator')
    config_file_path = os.path.join(share_dir, 'config/config.yaml')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=config_file_path,
        description='Path to the ROS 2 parameters file to use.'
    )

    move_test_node = Node(
        package='dynacbf_simulator',
        executable='move_test',
        name='move_test',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        params_declare,
        move_test_node
    ])