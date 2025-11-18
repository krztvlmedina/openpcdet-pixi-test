#!/usr/bin/env python3
"""
Simplified script to interpolate point clouds from .bin files.

This script provides a simpler interface for batch processing .bin files
without requiring a running ROS2 node. It launches the interpolation node
internally and handles the communication.

@author Abdalrahman M. Amer
@linkedin https://www.linkedin.com/in/abdalrahman-m-amer
@github https://github.com/geekgineer

@license AGPL-3.0
"""

import argparse
import sys
import subprocess
import time
from pathlib import Path
import tempfile
import yaml
import signal

try:
    import bin_file_utils
except ImportError:
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bin_file_utils


class BinInterpolationPipeline:
    """
    Simplified pipeline for interpolating .bin files.
    """

    def __init__(self, config: dict):
        self.config = config
        self.processes = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def cleanup(self):
        """Cleanup all launched processes."""
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=2)
            except:
                try:
                    proc.kill()
                except:
                    pass

    def create_temp_config(self) -> str:
        """Create a temporary config file for the interpolation node."""
        temp_config = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(self.config, temp_config)
        temp_config.close()
        return temp_config.name

    def interpolate(self, input_file: str, output_file: str) -> bool:
        """
        Interpolate a single .bin file.

        Args:
            input_file: Path to input .bin file
            output_file: Path to output .bin file

        Returns:
            True if successful, False otherwise
        """
        try:
            print(f"Processing: {input_file}")
            print(f"Output: {output_file}")

            # For now, use the ROS2-based interpolation script
            # In the future, this could be replaced with a pure Python implementation
            # or a direct C++ library binding

            print("\nNOTE: To use this script, you need to:")
            print("1. Start the interpolation node in one terminal:")
            print("   ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py")
            print("\n2. Then run this script to process .bin files")
            print("\nAlternatively, use the full interpolate_bin_file.py script.")

            return False

        except Exception as e:
            print(f"Error during interpolation: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Simplified .bin file interpolation tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interpolate a single file with default settings
  %(prog)s input.bin output.bin

  # Interpolate with custom scale factors
  %(prog)s input.bin output.bin --scale-x 2.0 --scale-y 4.0

  # Use bilateral interpolation
  %(prog)s input.bin output.bin --method bilateral

  # Batch process multiple files
  for f in *.bin; do %(prog)s $f interpolated_$f; done
        """
    )

    parser.add_argument('input_file', type=str, help='Input .bin file path')
    parser.add_argument('output_file', type=str, help='Output .bin file path')
    parser.add_argument('--method', type=str, default='linear',
                        choices=['linear', 'nearest', 'bilateral', 'edgeAware', 'spline'],
                        help='Interpolation method (default: linear)')
    parser.add_argument('--scale-x', type=float, default=1.0,
                        help='Scale factor in X direction (default: 1.0)')
    parser.add_argument('--scale-y', type=float, default=2.0,
                        help='Scale factor in Y direction (default: 2.0)')
    parser.add_argument('--min-range', type=float, default=0.0,
                        help='Minimum range filter (default: 0.0)')
    parser.add_argument('--max-range', type=float, default=float('inf'),
                        help='Maximum range filter (default: inf)')
    parser.add_argument('--apply-variance-filter', action='store_true',
                        help='Apply variance filtering')
    parser.add_argument('--max-variance', type=float, default=50.0,
                        help='Maximum allowed variance (default: 50.0)')
    parser.add_argument('--info', action='store_true',
                        help='Show info about the input file and exit')

    args = parser.parse_args()

    # Validate input file
    if not Path(args.input_file).exists():
        print(f"Error: Input file not found: {args.input_file}", file=sys.stderr)
        return 1

    # If --info flag, just show file info
    if args.info:
        try:
            x, y, z, intensity = bin_file_utils.read_bin_pointcloud(args.input_file)
            print(f"File: {args.input_file}")
            print(f"Points: {x.shape[0]}")
            print(f"X range: [{x.min():.2f}, {x.max():.2f}]")
            print(f"Y range: [{y.min():.2f}, {y.max():.2f}]")
            print(f"Z range: [{z.min():.2f}, {z.max():.2f}]")
            print(f"Intensity range: [{intensity.min():.2f}, {intensity.max():.2f}]")
            return 0
        except Exception as e:
            print(f"Error reading file: {e}", file=sys.stderr)
            return 1

    # Build configuration
    config = {
        'lidar': {
            'max_range': args.max_range,
            'min_range': args.min_range,
        },
        'interpolation': {
            'method': args.method,
            'scale_factor_x': args.scale_x,
            'scale_factor_y': args.scale_y,
            'interpolation_max_var': args.max_variance,
            'apply_variance_filter': args.apply_variance_filter,
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

    # Create pipeline and process
    with BinInterpolationPipeline(config) as pipeline:
        success = pipeline.interpolate(args.input_file, args.output_file)

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
