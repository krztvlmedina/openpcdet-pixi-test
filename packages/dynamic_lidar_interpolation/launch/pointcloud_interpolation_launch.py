import os
import subprocess
from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def resolve_config_path(context, *args, **kwargs):
    """Resolve the config file path relative to execution directory, fallback to default if not found."""
    config_file_param = context.launch_configurations.get('config_file', '')

    # Get default config path
    default_config = os.path.join(
        get_package_share_directory('dynamic_lidar_interpolation'),
        'config',
        'interpolation_config.yaml'
    )

    # If config_file_param is empty or same as default, use default
    if not config_file_param or config_file_param == default_config:
        context.launch_configurations['resolved_config_file'] = default_config
        print(f"Using default config: {default_config}")
        return []

    # Resolve relative path from current working directory
    if not os.path.isabs(config_file_param):
        resolved_path = os.path.abspath(os.path.join(os.getcwd(), config_file_param))
    else:
        resolved_path = config_file_param

    # Check if the resolved path exists
    if os.path.isfile(resolved_path):
        context.launch_configurations['resolved_config_file'] = resolved_path
        print(f"Using config file: {resolved_path}")
    else:
        context.launch_configurations['resolved_config_file'] = default_config
        print(f"Config file not found: {resolved_path}")
        print(f"Falling back to default config: {default_config}")

    return []

def cleanup_existing_process(context, *args, **kwargs):
    # Check if port 8765 is in use and terminate the process
    try:
        result = subprocess.run(
            ['lsof', '-t', '-i:8765'],
            stdout=subprocess.PIPE,
            text=True,
            check=False
        )
        pid = result.stdout.strip()
        if pid:
            subprocess.run(['kill', pid], check=False)
            print(f"Terminated process using port 8765 (PID: {pid}).")
        else:
            print("No process was using port 8765.")
    except Exception as e:
        print(f"Error during cleanup: {e}")

def generate_launch_description():
    # Declare a launch argument for the log level
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Set the ROS 2 logging level (e.g., debug, info, warn, error, fatal)'
    )

    # Declare a launch argument for the config file path
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to the interpolation configuration YAML file (relative to execution directory, or absolute)'
    )

    # Use the user-provided log level or the default value
    log_level = LaunchConfiguration('log_level')

    # Use the resolved config file path
    config = LaunchConfiguration('resolved_config_file')

    foxglove_launch_file = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_file)
    )

    pointcloud_interpolation_node = Node(
        package='dynamic_lidar_interpolation',
        executable='pointcloud_interpolation_node',
        name='pointcloud_interpolation_node',
        output='screen',
        parameters=[config],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        log_level_arg,
        config_file_arg,
        OpaqueFunction(function=resolve_config_path),
        OpaqueFunction(function=cleanup_existing_process),
        LogInfo(msg="Launching foxglove_bridge on ws://localhost:8765"),
        foxglove_bridge_launch,
        pointcloud_interpolation_node,
    ])
