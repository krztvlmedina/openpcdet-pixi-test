import os
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def resolve_config_path(context, *args, **kwargs):
    """Resolve the config file path relative to execution directory, fallback to default if not found."""
    config_file_param = context.launch_configurations.get('config_file', '')

    # Get default optimized config path
    default_config = os.path.join(
        get_package_share_directory('dynamic_lidar_interpolation'),
        'config',
        'interpolation_config_optimized.yaml'
    )

    # If config_file_param is empty, use optimized default
    if not config_file_param:
        context.launch_configurations['resolved_config_file'] = default_config
        print(f"Using optimized config: {default_config}")
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
        print(f"Falling back to optimized config: {default_config}")

    return []

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

    # OPTIMIZED CONFIGURATION:
    # 1. Uses optimized executable with hardcoded depth=1 (QoS overrides don't work reliably)
    # 2. Frame dropping built-in to prevent stale message processing
    # 3. Performance statistics logging every 100 frames
    pointcloud_interpolation_node = Node(
        package='dynamic_lidar_interpolation',
        executable='pointcloud_interpolation_node_optimized',  # OPTIMIZED: Use the optimized executable
        name='pointcloud_interpolation_node',
        output='screen',
        parameters=[config],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        log_level_arg,
        config_file_arg,
        OpaqueFunction(function=resolve_config_path),
        LogInfo(msg="Launching OPTIMIZED interpolation node with minimal latency configuration"),
        pointcloud_interpolation_node,
    ])
