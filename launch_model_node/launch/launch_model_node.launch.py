import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('launch_model_node'), 'config', 'launch_model_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='launch_model.py export-yaml が出力した YAML'),
        DeclareLaunchArgument('input_topic', default_value='/test/belt_launcher/target_distance'),
        DeclareLaunchArgument('output_topic', default_value='/test/belt_launcher/auto_target_velocity'),
        Node(
            package='launch_model_node',
            executable='launch_model_node',
            name='launch_model_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'input_topic': LaunchConfiguration('input_topic'),
                 'output_topic': LaunchConfiguration('output_topic')},
            ],
        ),
    ])
