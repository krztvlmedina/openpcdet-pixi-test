#!/usr/bin/env python3
"""
Standalone script to interpolate point clouds from .bin files.

This script reads a .bin file, converts it to a ROS2 PointCloud2 message,
applies interpolation using the dynamic_lidar_interpolation package,
and saves the result back to a .bin file.

@author Abdalrahman M. Amer
@linkedin https://www.linkedin.com/in/abdalrahman-m-amer
@github https://github.com/geekgineer

@license AGPL-3.0
"""

import argparse
import sys
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

try:
    import bin_file_utils
except ImportError:
    # Try to import from the same directory
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bin_file_utils


class BinFileInterpolator(Node):
    """
    ROS2 Node for interpolating point clouds from .bin files.
    """

    def __init__(self, input_file: str, output_file: str, config: dict):
        super().__init__('bin_file_interpolator')

        self.input_file = input_file
        self.output_file = output_file
        self.config = config
        self.interpolated_cloud = None
        self.received_result = False

        # Declare parameters from config
        self._declare_parameters_from_config(config)

        # Create publisher for original point cloud
        self.original_cloud_pub = self.create_publisher(
            PointCloud2,
            '/velodyne_points',
            10
        )

        # Create subscriber for interpolated point cloud
        self.interpolated_cloud_sub = self.create_subscription(
            PointCloud2,
            config.get('topics', {}).get('interpolated_point_cloud_topic', '/interpolated_point_cloud'),
            self.interpolated_callback,
            10
        )

        self.get_logger().info(f"Initialized BinFileInterpolator")
        self.get_logger().info(f"Input: {input_file}")
        self.get_logger().info(f"Output: {output_file}")

    def _declare_parameters_from_config(self, config: dict):
        """Declare ROS2 parameters from configuration dictionary."""
        # LiDAR parameters
        lidar_config = config.get('lidar', {})
        self.declare_parameter('lidar.max_range', lidar_config.get('max_range', float('inf')))
        self.declare_parameter('lidar.min_range', lidar_config.get('min_range', 0.0))

        # Interpolation parameters
        interp_config = config.get('interpolation', {})
        self.declare_parameter('interpolation.method', interp_config.get('method', 'linear'))
        self.declare_parameter('interpolation.scale_factor_x', interp_config.get('scale_factor_x', 1.0))
        self.declare_parameter('interpolation.scale_factor_y', interp_config.get('scale_factor_y', 2.0))
        self.declare_parameter('interpolation.interpolation_max_var', interp_config.get('interpolation_max_var', 50.0))
        self.declare_parameter('interpolation.apply_variance_filter', interp_config.get('apply_variance_filter', False))
        self.declare_parameter('interpolation.rotation_angle_x', interp_config.get('rotation_angle_x', 0.0))
        self.declare_parameter('interpolation.extrapolation_value', interp_config.get('extrapolation_value', 'NaN'))
        self.declare_parameter('interpolation.sensor_translation', interp_config.get('sensor_translation', [0.0, 0.0, 0.0]))

        # Range image parameters
        range_config = config.get('range_image', {})
        self.declare_parameter('range_image.angular_resolution_x', range_config.get('angular_resolution_x', 0.25))
        self.declare_parameter('range_image.angular_resolution_y', range_config.get('angular_resolution_y', 2.05))
        self.declare_parameter('range_image.max_angle_width', range_config.get('max_angle_width', 360.0))
        self.declare_parameter('range_image.max_angle_height', range_config.get('max_angle_height', 180.0))
        self.declare_parameter('range_image.min_ang_fov', range_config.get('min_ang_fov', 0.0))
        self.declare_parameter('range_image.max_ang_fov', range_config.get('max_ang_fov', 360.0))

    def interpolated_callback(self, msg: PointCloud2):
        """Callback for receiving interpolated point cloud."""
        self.get_logger().info(f"Received interpolated cloud with {msg.width * msg.height} points")
        self.interpolated_cloud = msg
        self.received_result = True

    def xyz_array_to_pointcloud2(self, points: np.ndarray, frame_id: str = "velodyne") -> PointCloud2:
        """
        Convert numpy array of XYZ points to PointCloud2 message.

        Args:
            points: Nx3 numpy array
            frame_id: Frame ID for the point cloud

        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = points.shape[0]

        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True

        # Pack data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))

        msg.data = b''.join(buffer)

        return msg

    def pointcloud2_to_xyz_array(self, msg: PointCloud2) -> np.ndarray:
        """
        Convert PointCloud2 message to numpy array.

        Args:
            msg: PointCloud2 message

        Returns:
            Nx3 numpy array with XYZ coordinates
        """
        # Parse point cloud data
        points = []
        for i in range(0, len(msg.data), msg.point_step):
            x, y, z = struct.unpack_from('fff', msg.data, i)
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])

        return np.array(points, dtype=np.float32)

    def process(self):
        """Main processing function."""
        try:
            # Read input .bin file
            self.get_logger().info(f"Reading {self.input_file}...")
            xyz_points = bin_file_utils.bin_to_xyz_array(self.input_file)
            self.get_logger().info(f"Loaded {xyz_points.shape[0]} points")

            # Convert to PointCloud2
            cloud_msg = self.xyz_array_to_pointcloud2(xyz_points)

            # Publish the original cloud
            self.get_logger().info("Publishing original point cloud for interpolation...")
            self.original_cloud_pub.publish(cloud_msg)

            # Wait for interpolated result
            self.get_logger().info("Waiting for interpolated result...")
            rate = self.create_rate(10)  # 10 Hz
            timeout = 10.0  # seconds
            elapsed = 0.0

            while not self.received_result and elapsed < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed += 0.1

            if not self.received_result:
                self.get_logger().error("Timeout waiting for interpolated point cloud!")
                return False

            # Convert interpolated cloud back to numpy array
            self.get_logger().info("Converting interpolated cloud to .bin format...")
            interpolated_xyz = self.pointcloud2_to_xyz_array(self.interpolated_cloud)

            # Write output .bin file
            self.get_logger().info(f"Writing {interpolated_xyz.shape[0]} points to {self.output_file}...")
            bin_file_utils.xyz_array_to_bin(interpolated_xyz, self.output_file)

            self.get_logger().info("Interpolation complete!")
            self.get_logger().info(f"Original points: {xyz_points.shape[0]}")
            self.get_logger().info(f"Interpolated points: {interpolated_xyz.shape[0]}")

            return True

        except Exception as e:
            self.get_logger().error(f"Error during processing: {e}")
            import traceback
            traceback.print_exc()
            return False


def load_config(config_file: str = None) -> dict:
    """
    Load configuration from YAML file or return defaults.

    Args:
        config_file: Path to YAML config file (optional)

    Returns:
        Configuration dictionary
    """
    default_config = {
        'lidar': {
            'max_range': float('inf'),
            'min_range': 0.0,
        },
        'interpolation': {
            'method': 'linear',
            'scale_factor_x': 1.0,
            'scale_factor_y': 2.0,
            'interpolation_max_var': 50.0,
            'apply_variance_filter': False,
            'rotation_angle_x': 0.0,
            'extrapolation_value': 'NaN',
            'sensor_translation': [0.0, 0.0, 0.0],
        },
        'range_image': {
            'angular_resolution_x': 0.25,
            'angular_resolution_y': 2.05,
            'max_angle_width': 360.0,
            'max_angle_height': 180.0,
            'min_ang_fov': 0.0,
            'max_ang_fov': 360.0,
        },
        'topics': {
            'lidar_topic': 'velodyne_points',
            'interpolated_point_cloud_topic': 'interpolated_point_cloud',
        }
    }

    if config_file is None:
        return default_config

    try:
        import yaml
        with open(config_file, 'r') as f:
            user_config = yaml.safe_load(f)

        # Merge user config with defaults
        def deep_update(base, update):
            for key, value in update.items():
                if isinstance(value, dict) and key in base:
                    deep_update(base[key], value)
                else:
                    base[key] = value

        deep_update(default_config, user_config)
        return default_config
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
        print("Using default configuration.")
        return default_config


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Interpolate point clouds from .bin files using dynamic_lidar_interpolation'
    )
    parser.add_argument('input_file', type=str, help='Input .bin file path')
    parser.add_argument('output_file', type=str, help='Output .bin file path')
    parser.add_argument('--config', type=str, default=None, help='Configuration YAML file (optional)')
    parser.add_argument('--method', type=str, default='linear',
                        choices=['linear', 'nearest', 'bilateral', 'edgeAware', 'spline'],
                        help='Interpolation method')
    parser.add_argument('--scale-x', type=float, default=1.0, help='Scale factor in X direction')
    parser.add_argument('--scale-y', type=float, default=2.0, help='Scale factor in Y direction')

    parsed_args = parser.parse_args()

    # Validate input file
    if not Path(parsed_args.input_file).exists():
        print(f"Error: Input file not found: {parsed_args.input_file}")
        return 1

    # Load configuration
    config = load_config(parsed_args.config)

    # Override with command-line arguments
    config['interpolation']['method'] = parsed_args.method
    config['interpolation']['scale_factor_x'] = parsed_args.scale_x
    config['interpolation']['scale_factor_y'] = parsed_args.scale_y

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        # Create and run the interpolator node
        node = BinFileInterpolator(
            parsed_args.input_file,
            parsed_args.output_file,
            config
        )

        # Process the file
        success = node.process()

        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

        return 0 if success else 1

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
