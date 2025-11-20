#!/usr/bin/env python3
"""
Standalone script to interpolate point clouds from .bin files using a configuration file.

This script loads all interpolation parameters from a YAML configuration file
and uses the C++ interpolation algorithms via Python bindings (no ROS2 required).

Usage:
    python interpolate_with_config.py input.bin output.bin --config config.yaml
    python interpolate_with_config.py input.bin output.bin --config /path/to/config.yaml

@author Dynamic LiDAR Interpolation Contributors
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
    print("ERROR: interpolation_core module not found.", file=sys.stderr)
    print("Please build the C++ Python bindings first:", file=sys.stderr)
    print("  1. Install pybind11: pip install pybind11", file=sys.stderr)
    print("  2. Build the package: colcon build --packages-select dynamic_lidar_interpolation", file=sys.stderr)
    print("  3. Source the workspace: source install/setup.bash", file=sys.stderr)
    sys.exit(1)


def load_config(config_path: str) -> dict:
    """
    Load interpolation configuration from a YAML file.

    Args:
        config_path: Path to the YAML configuration file

    Returns:
        Dictionary containing configuration parameters

    Raises:
        FileNotFoundError: If config file doesn't exist
        yaml.YAMLError: If config file is invalid YAML
    """
    config_file = Path(config_path)

    if not config_file.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_file, 'r') as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise yaml.YAMLError(f"Error parsing YAML configuration: {e}")

    # Extract parameters from ROS2-style config structure if present
    if 'pointcloud_interpolation_node' in config:
        if 'ros__parameters' in config['pointcloud_interpolation_node']:
            config = config['pointcloud_interpolation_node']['ros__parameters']

    return config


def get_default_config() -> dict:
    """
    Get default configuration parameters.

    Returns:
        Dictionary with default configuration values
    """
    return {
        'lidar': {
            'max_range': float('inf'),
            'min_range': 0.0,
        },
        'range_image': {
            'angular_resolution_x': 0.25,
            'angular_resolution_y': 2.05,
            'max_angle_width': 360.0,
            'max_angle_height': 180.0,
            'min_angle_fov': 0.0,
            'max_angle_fov': 360.0,
        },
        'interpolation': {
            'method': 'linear',
            'scale_factor_x': 1.0,
            'scale_factor_y': 2.0,
            'interpolation_max_var': 50.0,
            'apply_variance_filter': False,
            'rotation_angle_x': 0.0,
            'extrapolation_value': float('nan'),
            'sensor_translation': [0.0, 0.0, 0.0],
        },
    }


def merge_configs(default: dict, user: dict) -> dict:
    """
    Deep merge user configuration with defaults.

    Args:
        default: Default configuration dictionary
        user: User configuration dictionary

    Returns:
        Merged configuration dictionary
    """
    result = default.copy()

    for key, value in user.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_configs(result[key], value)
        else:
            result[key] = value

    return result


def interpolate_bin_file(
    input_file: str,
    output_file: str,
    config: dict,
    verbose: bool = False
) -> bool:
    """
    Interpolate a .bin point cloud file using the provided configuration.

    Args:
        input_file: Path to input .bin file
        output_file: Path to output .bin file
        config: Configuration dictionary
        verbose: Print detailed progress information

    Returns:
        True if successful, False otherwise
    """
    try:
        if verbose:
            print(f"Reading input file: {input_file}")

        # Read input .bin file
        x, y, z, intensity = bin_file_utils.read_bin_pointcloud(input_file)
        num_input_points = len(x)

        if verbose:
            print(f"  Input points: {num_input_points}")
            print(f"  X range: [{x.min():.2f}, {x.max():.2f}] m")
            print(f"  Y range: [{y.min():.2f}, {y.max():.2f}] m")
            print(f"  Z range: [{z.min():.2f}, {z.max():.2f}] m")

        # Prepare point cloud as numpy array (N, 3)
        points = np.column_stack([x, y, z]).astype(np.float32)

        # Extract parameters from config
        lidar_config = config.get('lidar', {})
        range_config = config.get('range_image', {})
        interp_config = config.get('interpolation', {})

        # Handle extrapolation value (could be 'NaN' string or float)
        extrap_value = interp_config.get('extrapolation_value', float('nan'))
        if isinstance(extrap_value, str) and extrap_value.lower() == 'nan':
            extrap_value = float('nan')

        if verbose:
            print(f"\nInterpolation settings:")
            print(f"  Method: {interp_config.get('method', 'linear')}")
            print(f"  Scale factors: X={interp_config.get('scale_factor_x', 1.0)}, "
                  f"Y={interp_config.get('scale_factor_y', 2.0)}")
            print(f"  Angular resolution: X={range_config.get('angular_resolution_x', 0.25)}°, "
                  f"Y={range_config.get('angular_resolution_y', 2.05)}°")
            print(f"  Variance filter: {interp_config.get('apply_variance_filter', False)}")

        # Perform interpolation
        if verbose:
            print("\nPerforming interpolation...")

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

        if verbose:
            print(f"  Output points: {num_output_points}")
            print(f"  Points added: {num_output_points - num_input_points}")

        # Extract coordinates
        x_out = interpolated_points[:, 0]
        y_out = interpolated_points[:, 1]
        z_out = interpolated_points[:, 2]

        # Create intensity array (zeros for new points)
        # TODO: Could implement intensity interpolation here
        intensity_out = np.zeros(num_output_points, dtype=np.float32)

        if verbose:
            print(f"\nWriting output file: {output_file}")

        # Write output .bin file
        bin_file_utils.write_bin_pointcloud(output_file, x_out, y_out, z_out, intensity_out)

        if verbose:
            print("Success!")

        return True

    except Exception as e:
        print(f"Error during interpolation: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Interpolate .bin point cloud files using a configuration file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default config file location
  %(prog)s input.bin output.bin --config config/interpolation_config.yaml

  # Use custom config file
  %(prog)s input.bin output.bin --config /path/to/my_config.yaml

  # Show detailed progress
  %(prog)s input.bin output.bin --config config.yaml --verbose

  # Show info about input file
  %(prog)s input.bin output.bin --info

Configuration File Format:
  The YAML configuration file should contain the following sections:
    - lidar: Range filtering parameters (min_range, max_range)
    - range_image: Angular resolution and FOV settings
    - interpolation: Method, scale factors, variance filtering

  See config/interpolation_config.yaml for a complete example.
        """
    )

    parser.add_argument('input_file', type=str, help='Input .bin file path')
    parser.add_argument('output_file', type=str, help='Output .bin file path')
    parser.add_argument('--config', type=str, required=True,
                        help='Path to YAML configuration file')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Print detailed progress information')
    parser.add_argument('--info', action='store_true',
                        help='Show info about the input file and exit')

    args = parser.parse_args()

    # Validate input file
    if not Path(args.input_file).exists():
        print(f"Error: Input file not found: {args.input_file}", file=sys.stderr)
        return 1

    # Show info and exit if requested
    if args.info:
        try:
            x, y, z, intensity = bin_file_utils.read_bin_pointcloud(args.input_file)
            print(f"File: {args.input_file}")
            print(f"Points: {x.shape[0]}")
            print(f"X range: [{x.min():.2f}, {x.max():.2f}] m")
            print(f"Y range: [{y.min():.2f}, {y.max():.2f}] m")
            print(f"Z range: [{z.min():.2f}, {z.max():.2f}] m")
            if intensity is not None:
                print(f"Intensity range: [{intensity.min():.2f}, {intensity.max():.2f}]")
            return 0
        except Exception as e:
            print(f"Error reading file: {e}", file=sys.stderr)
            return 1

    # Load configuration
    try:
        if args.verbose:
            print(f"Loading configuration from: {args.config}")

        user_config = load_config(args.config)
        default_config = get_default_config()
        config = merge_configs(default_config, user_config)

    except Exception as e:
        print(f"Error loading configuration: {e}", file=sys.stderr)
        return 1

    # Perform interpolation
    success = interpolate_bin_file(
        args.input_file,
        args.output_file,
        config,
        verbose=args.verbose
    )

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
