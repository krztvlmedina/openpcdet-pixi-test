#!/usr/bin/env python3
"""
Launch file for the interpolation side of performance evaluation.
Runs in the velodyne_ros container.

Launches:
  - bin_publisher_node (publishes .bin files as PointCloud2)
  - pointcloud_interpolation_node (with perf tracking enabled)
  - Optional: foxglove_bridge (if headless=false)

Usage:
    ros2 launch dynamic_lidar_interpolation perf_eval_interpolation_launch.py \
        config_file:=/path/to/interpolation_config.yaml \
        bin_directory:=/path/to/bin/files \
        config_name:=optimized \
        publish_rate:=10.0

    # With Foxglove visualization:
    ros2 launch dynamic_lidar_interpolation perf_eval_interpolation_launch.py \
        config_file:=/path/to/config.yaml \
        bin_directory:=/path/to/bin/files \
        headless:=false
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def resolve_config_path(context, *args, **kwargs):
    """Resolve the interpolation config file path."""
    config_file_param = context.launch_configurations.get('config_file', '')

    default_config = os.path.join(
        get_package_share_directory('dynamic_lidar_interpolation'),
        'config',
        'interpolation_config.yaml'
    )

    if not config_file_param:
        context.launch_configurations['resolved_config_file'] = default_config
        print(f"Using default config: {default_config}")
        return []

    if not os.path.isabs(config_file_param):
        resolved_path = os.path.abspath(os.path.join(os.getcwd(), config_file_param))
    else:
        resolved_path = config_file_param

    if os.path.isfile(resolved_path):
        context.launch_configurations['resolved_config_file'] = resolved_path
        print(f"Using config file: {resolved_path}")
    else:
        context.launch_configurations['resolved_config_file'] = default_config
        print(f"Config file not found: {resolved_path}")
        print(f"Falling back to default config: {default_config}")

    return []


def cleanup_foxglove_port(context, *args, **kwargs):
    """Clean up port 8765 if used by a previous foxglove_bridge."""
    try:
        result = subprocess.run(
            ['lsof', '-t', '-i:8765'],
            stdout=subprocess.PIPE, text=True, check=False
        )
        pid = result.stdout.strip()
        if pid:
            subprocess.run(['kill', pid], check=False)
            print(f"Terminated process using port 8765 (PID: {pid}).")
    except Exception as e:
        print(f"Error during cleanup: {e}")


def launch_setup(context, *args, **kwargs):
    """Dynamically set up launch nodes based on configuration."""
    config = context.launch_configurations['resolved_config_file']
    headless = context.launch_configurations.get('headless', 'true').lower() == 'true'
    config_name = context.launch_configurations.get('config_name', 'default')
    bin_directory = context.launch_configurations.get('bin_directory', '')
    publish_rate = context.launch_configurations.get('publish_rate', '10.0')
    log_level = context.launch_configurations.get('log_level', 'info')

    nodes = []

    # bin_publisher_node
    if bin_directory:
        bin_publisher = Node(
            package='pointcloud_utils',
            executable='bin_publisher_node',
            name='bin_publisher_node',
            output='screen',
            parameters=[{
                'bin_directory': bin_directory,
                'publish_rate': float(publish_rate),
                'loop': False,
                'use_file_timestamps': False,
                'frame_id': 'velodyne',
                'topic': 'velodyne_points',
                'verbose': True,
            }],
            arguments=['--ros-args', '--log-level', log_level]
        )
        nodes.append(bin_publisher)

    # interpolation_node with perf tracking parameters overlaid
    interpolation_node = Node(
        package='dynamic_lidar_interpolation',
        executable='pointcloud_interpolation_node',
        name='pointcloud_interpolation_node',
        output='screen',
        parameters=[
            config,
            {
                'performance.enable_perf_tracking': True,
                'performance.config_name': config_name,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    nodes.append(interpolation_node)

    # Optional foxglove_bridge
    if not headless:
        try:
            foxglove_launch_file = os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
            foxglove_bridge = IncludeLaunchDescription(
                AnyLaunchDescriptionSource(foxglove_launch_file)
            )
            nodes.append(foxglove_bridge)
        except Exception:
            print("foxglove_bridge package not found, skipping visualization.")

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='',
            description='Path to interpolation config YAML'),
        DeclareLaunchArgument('config_name', default_value='default',
            description='Config name label for perf tracking'),
        DeclareLaunchArgument('bin_directory', default_value='',
            description='Directory containing .bin files'),
        DeclareLaunchArgument('publish_rate', default_value='10.0',
            description='Publish rate in Hz'),
        DeclareLaunchArgument('headless', default_value='true',
            description='Run without visualization (true/false)'),
        DeclareLaunchArgument('log_level', default_value='info',
            description='ROS 2 logging level'),

        OpaqueFunction(function=resolve_config_path),
        LogInfo(msg="Launching interpolation pipeline with performance tracking"),
        OpaqueFunction(function=launch_setup),
    ])
