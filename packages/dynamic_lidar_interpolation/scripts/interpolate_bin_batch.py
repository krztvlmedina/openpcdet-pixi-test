#!/usr/bin/env python3
"""
Batch script to interpolate point clouds from .bin files in a directory.

This script processes all .bin files in a source directory and outputs
interpolated results to a target directory using the same configuration
as interpolate_bin_simple.

@license AGPL-3.0
"""

import argparse
import sys
from pathlib import Path
import yaml
import numpy as np

try:
    import bin_file_utils
except ImportError:
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bin_file_utils

try:
    import interpolation_core
except ImportError:
    print("WARNING: interpolation_core module not found.", file=sys.stderr)
    print("Please build the C++ Python bindings first:", file=sys.stderr)
    print("  1. Install pybind11: pip install pybind11", file=sys.stderr)
    print("  2. Build the package: colcon build --packages-select dynamic_lidar_interpolation", file=sys.stderr)
    print("  3. Source the workspace: source install/setup.bash", file=sys.stderr)
    interpolation_core = None


class BinInterpolationPipeline:
    """
    Simplified pipeline for interpolating .bin files using C++ bindings.
    """

    def __init__(self, config: dict):
        self.config = config

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def interpolate(self, input_file: str, output_file: str) -> bool:
        """
        Interpolate a single .bin file.

        Args:
            input_file: Path to input .bin file
            output_file: Path to output .bin file

        Returns:
            True if successful, False otherwise
        """
        if interpolation_core is None:
            print("ERROR: interpolation_core module not available.", file=sys.stderr)
            print("Please build the Python bindings first (see warning above).", file=sys.stderr)
            return False

        try:
            print(f"Processing: {input_file}")
            print(f"Output: {output_file}")

            # Read input .bin file
            x, y, z, intensity = bin_file_utils.read_bin_pointcloud(input_file)
            num_input_points = len(x)
            print(f"  Input points: {num_input_points}")

            # Prepare point cloud as numpy array (N, 3)
            points = np.column_stack([x, y, z]).astype(np.float32)

            # Extract parameters from config
            lidar_config = self.config.get('lidar', {})
            range_config = self.config.get('range_image', {})
            interp_config = self.config.get('interpolation', {})

            # Handle extrapolation value
            extrap_value = interp_config.get('extrapolation_value', float('nan'))
            if isinstance(extrap_value, str) and extrap_value.lower() == 'nan':
                extrap_value = float('nan')

            # Perform interpolation
            print(f"  Method: {interp_config.get('method', 'linear')}")
            interpolated_points = interpolation_core.interpolate(
                points,
                angular_res_x=range_config.get('angular_resolution_x', 0.25),
                angular_res_y=range_config.get('angular_resolution_y', 2.05),
                max_angle_width=range_config.get('max_angle_width', 360.0),
                max_angle_height=range_config.get('max_angle_height', 180.0),
                interpolation_method=interp_config.get('method', 'linear'),
                scale_factor_x=interp_config.get('scale_factor_x', 1.0),
                scale_factor_y=interp_config.get('scale_factor_y', 2.0),
                min_range=lidar_config.get('min_range', 0.0),
                max_range=lidar_config.get('max_range', float('inf')),
                apply_variance_filter=interp_config.get('apply_variance_filter', False),
                max_allowed_variance=interp_config.get('interpolation_max_var', 50.0),
                sensor_translation=interp_config.get('sensor_translation', [0.0, 0.0, 0.0]),
                rotation_angle_x=interp_config.get('rotation_angle_x', 0.0),
                extrapolation_value=extrap_value,
                min_ang_fov=range_config.get('min_angle_fov', 0.0),
                max_ang_fov=range_config.get('max_angle_fov', 360.0),
            )

            num_output_points = len(interpolated_points)
            print(f"  Output points: {num_output_points} (+{num_output_points - num_input_points})")

            # Extract coordinates
            x_out = interpolated_points[:, 0]
            y_out = interpolated_points[:, 1]
            z_out = interpolated_points[:, 2]

            # Create intensity array (zeros for new points)
            intensity_out = np.zeros(num_output_points, dtype=np.float32)

            # Write output .bin file
            bin_file_utils.write_bin_pointcloud(output_file, x_out, y_out, z_out, intensity_out)
            print(f"  Saved to: {output_file}\n")

            return True

        except Exception as e:
            print(f"Error during interpolation: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return False


def load_config_from_file(config_path: str) -> dict:
    """Load configuration from YAML file."""
    config_file = Path(config_path)
    if not config_file.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Extract parameters from ROS2-style config structure if present
    if 'pointcloud_interpolation_node' in config:
        if 'ros__parameters' in config['pointcloud_interpolation_node']:
            config = config['pointcloud_interpolation_node']['ros__parameters']

    return config


def process_directory(source_dir: Path, target_dir: Path, config: dict) -> int:
    """
    Process all .bin files in source directory.

    Args:
        source_dir: Source directory containing .bin files
        target_dir: Target directory for interpolated files
        config: Configuration dictionary

    Returns:
        0 if successful, 1 otherwise
    """
    # Find all .bin files in source directory
    bin_files = sorted(source_dir.glob('*.bin'))

    if not bin_files:
        print(f"No .bin files found in {source_dir}", file=sys.stderr)
        return 1

    print(f"Found {len(bin_files)} .bin file(s) in {source_dir}")
    print(f"Target directory: {target_dir}\n")

    # Create target directory if it doesn't exist
    if not target_dir.exists():
        print(f"Creating target directory: {target_dir}")
        target_dir.mkdir(parents=True, exist_ok=True)
    else:
        # Check if target directory has any files
        existing_files = list(target_dir.iterdir())
        if existing_files:
            print(f"ERROR: Target directory already contains files:", file=sys.stderr)
            print(f"  {target_dir}", file=sys.stderr)
            print(f"  Found {len(existing_files)} file(s)", file=sys.stderr)
            print(f"  Please use an empty directory or remove existing files first.", file=sys.stderr)
            return 1

    print()

    # Process each file
    success_count = 0
    failed_count = 0

    with BinInterpolationPipeline(config) as pipeline:
        for i, input_file in enumerate(bin_files, 1):
            print(f"[{i}/{len(bin_files)}] Processing: {input_file.name}")

            output_file = target_dir / input_file.name

            if pipeline.interpolate(str(input_file), str(output_file)):
                success_count += 1
            else:
                failed_count += 1
                print(f"  FAILED: {input_file.name}\n", file=sys.stderr)

    # Print summary
    print("=" * 60)
    print(f"Batch processing complete!")
    print(f"  Successful: {success_count}/{len(bin_files)}")
    if failed_count > 0:
        print(f"  Failed: {failed_count}/{len(bin_files)}")
    print("=" * 60)

    return 0 if failed_count == 0 else 1


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Batch .bin file interpolation tool for directories',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process all .bin files in a directory
  %(prog)s /path/to/source /path/to/target --config config/interpolation_config.yaml

  # Process with relative paths
  %(prog)s ./input_data ./output_data --config config.yaml
        """
    )

    parser.add_argument('source_dir', type=str, help='Source directory containing .bin files')
    parser.add_argument('target_dir', type=str, help='Target directory for interpolated files (must be empty or non-existent)')
    parser.add_argument('--config', type=str, required=True,
                        help='Path to YAML configuration file')

    args = parser.parse_args()

    # Validate source directory
    source_dir = Path(args.source_dir)
    if not source_dir.exists():
        print(f"Error: Source directory not found: {source_dir}", file=sys.stderr)
        return 1

    if not source_dir.is_dir():
        print(f"Error: Source path is not a directory: {source_dir}", file=sys.stderr)
        return 1

    # Prepare target directory path
    target_dir = Path(args.target_dir)

    # Load configuration from file
    try:
        config = load_config_from_file(args.config)
    except Exception as e:
        print(f"Error loading configuration: {e}", file=sys.stderr)
        return 1

    # Process directory
    return process_directory(source_dir, target_dir, config)


if __name__ == '__main__':
    sys.exit(main())
