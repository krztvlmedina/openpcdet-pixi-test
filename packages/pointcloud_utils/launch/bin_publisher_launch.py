#!/usr/bin/env python3
"""
Launch file for bin_publisher_node

Usage:
    ros2 launch pointcloud_utils bin_publisher_launch.py bin_directory:=/path/to/bin/files
    ros2 launch pointcloud_utils bin_publisher_launch.py bin_directory:=/path/to/bin/files publish_rate:=5.0
    ros2 launch pointcloud_utils bin_publisher_launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('pointcloud_utils')
    default_config_file = os.path.join(pkg_dir, 'config', 'bin_publisher.yaml')

    # Declare launch arguments
    bin_directory_arg = DeclareLaunchArgument(
        'bin_directory',
        default_value='',
        description='Directory containing .bin files to publish'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publish rate in Hz'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop through files continuously'
    )

    use_file_timestamps_arg = DeclareLaunchArgument(
        'use_file_timestamps',
        default_value='false',
        description='Use file modification timestamps instead of current time'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='velodyne',
        description='Frame ID for published point clouds'
    )

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='velodyne_points',
        description='Topic name for publishing'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Verbose logging'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to config file (overrides other parameters if provided)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    def launch_setup(context, *args, **kwargs):
        # Get launch configurations
        config_file = LaunchConfiguration('config_file').perform(context)
        bin_directory = LaunchConfiguration('bin_directory').perform(context)

        # Determine if we should use config file or parameters
        use_config_file = os.path.exists(config_file) and bin_directory == ''

        if use_config_file:
            # Use config file
            node = Node(
                package='pointcloud_utils',
                executable='bin_publisher_node',
                name='bin_publisher_node',
                output='screen',
                parameters=[config_file],
                arguments=['--ros-args', '--log-level',
                          LaunchConfiguration('log_level')]
            )
        else:
            # Use command-line parameters
            node = Node(
                package='pointcloud_utils',
                executable='bin_publisher_node',
                name='bin_publisher_node',
                output='screen',
                parameters=[{
                    'bin_directory': LaunchConfiguration('bin_directory'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    'loop': LaunchConfiguration('loop'),
                    'use_file_timestamps': LaunchConfiguration('use_file_timestamps'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'topic': LaunchConfiguration('topic'),
                    'verbose': LaunchConfiguration('verbose'),
                }],
                arguments=['--ros-args', '--log-level',
                          LaunchConfiguration('log_level')]
            )

        return [node]

    return LaunchDescription([
        bin_directory_arg,
        publish_rate_arg,
        loop_arg,
        use_file_timestamps_arg,
        frame_id_arg,
        topic_arg,
        verbose_arg,
        config_file_arg,
        log_level_arg,
        OpaqueFunction(function=launch_setup)
    ])
