# .bin File Interpolation Scripts

This directory contains Python scripts for interpolating point clouds stored in `.bin` files using the `dynamic_lidar_interpolation` package.

## Overview

The `.bin` file format is commonly used in autonomous driving datasets (e.g., KITTI, nuScenes) where point clouds are stored as binary files with interleaved `[x, y, z, intensity]` values.

## Files

### `bin_file_utils.py`
Utility functions for reading and writing `.bin` files.

**Usage:**
```python
from bin_file_utils import read_bin_pointcloud, write_bin_pointcloud

# Read a .bin file
x, y, z, intensity = read_bin_pointcloud('input.bin')

# Write a .bin file
write_bin_pointcloud('output.bin', x, y, z, intensity)
```

**Command-line usage:**
```bash
# Display statistics about a .bin file
python bin_file_utils.py input.bin
```

### `interpolate_bin_file.py`
Main script for interpolating `.bin` files using the ROS2 interpolation node.

**Prerequisites:**
- ROS2 Humble (or compatible version)
- `dynamic_lidar_interpolation` package built and sourced
- Running ROS2 core and interpolation node

**Usage:**
```bash
# Basic usage with default settings
python interpolate_bin_file.py input.bin output.bin

# With custom interpolation method
python interpolate_bin_file.py input.bin output.bin --method bilateral

# With custom scale factors
python interpolate_bin_file.py input.bin output.bin --scale-x 2.0 --scale-y 4.0

# With custom configuration file
python interpolate_bin_file.py input.bin output.bin --config custom_config.yaml
```

**Complete workflow:**
```bash
# Terminal 1: Start the interpolation node
ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py

# Terminal 2: Process your .bin file
python interpolate_bin_file.py input.bin output_interpolated.bin --method linear --scale-y 2.0
```

### `interpolate_bin_simple.py`
Simplified interface for batch processing (work in progress).

**Usage:**
```bash
# Show file information
python interpolate_bin_simple.py input.bin output.bin --info

# Interpolate with custom settings
python interpolate_bin_simple.py input.bin output.bin --method spline --scale-y 3.0
```

## .bin File Format

The `.bin` format used is compatible with KITTI and similar datasets:

```
File structure: [x1, y1, z1, i1, x2, y2, z2, i2, ..., xN, yN, zN, iN]
Data type: float32
Total size: N_points × 4 × 4 bytes
```

Where:
- `x, y, z`: 3D coordinates in meters
- `i`: Intensity/reflectance value

## Interpolation Methods

Available interpolation methods:
- **linear** (bilinear): Fast, good quality (recommended)
- **nearest**: Fastest, preserves original points
- **bilateral**: Preserves edges, reduces noise
- **edgeAware**: Maintains sharp boundaries
- **spline**: Smoothest results, slowest

## Configuration

You can customize interpolation behavior using a YAML config file:

```yaml
lidar:
  max_range: 100.0  # Maximum range in meters
  min_range: 0.5    # Minimum range in meters

interpolation:
  method: "linear"
  scale_factor_x: 1.0  # Horizontal angular resolution multiplier
  scale_factor_y: 2.0  # Vertical angular resolution multiplier
  apply_variance_filter: false
  interpolation_max_var: 50.0

range_image:
  angular_resolution_x: 0.25  # degrees
  angular_resolution_y: 2.05  # degrees
  max_angle_width: 360.0
  max_angle_height: 180.0
```

## Examples

### Example 1: Basic Interpolation
```bash
# Interpolate with 2x vertical resolution
python interpolate_bin_file.py velodyne_frame_001.bin velodyne_dense_001.bin --scale-y 2.0
```

### Example 2: Batch Processing
```bash
#!/bin/bash
# Process all .bin files in a directory

for file in velodyne_points/*.bin; do
    filename=$(basename "$file" .bin)
    echo "Processing $filename..."
    python interpolate_bin_file.py \
        "$file" \
        "interpolated/${filename}_dense.bin" \
        --method bilateral \
        --scale-y 2.0
done
```

### Example 3: Using with Python
```python
import bin_file_utils
import numpy as np

# Read original point cloud
x, y, z, intensity = bin_file_utils.read_bin_pointcloud('input.bin')

print(f"Original points: {len(x)}")

# After running interpolation (using the ROS2 node)
# Read the interpolated result
x_i, y_i, z_i, intensity_i = bin_file_utils.read_bin_pointcloud('output_interpolated.bin')

print(f"Interpolated points: {len(x_i)}")
print(f"Density increase: {len(x_i) / len(x):.2f}x")
```

## Integration with OpenPCDet

These scripts are designed to work with OpenPCDet and similar 3D detection frameworks:

```python
# Example: Preprocess KITTI dataset with interpolation
import os
from pathlib import Path

velodyne_dir = Path('data/kitti/training/velodyne')
output_dir = Path('data/kitti/training/velodyne_interpolated')
output_dir.mkdir(exist_ok=True)

for bin_file in velodyne_dir.glob('*.bin'):
    output_file = output_dir / bin_file.name
    # Run interpolation (assumes ROS2 node is running)
    os.system(f"python interpolate_bin_file.py {bin_file} {output_file} --scale-y 2.0")
```

## Troubleshooting

### "ModuleNotFoundError: No module named 'rclpy'"
Make sure you have sourced your ROS2 workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/your_ros2_ws/install/setup.bash
```

### "Timeout waiting for interpolated point cloud"
Ensure the interpolation node is running:
```bash
ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py
```

### "Invalid .bin file format"
Verify your .bin file has the correct format (multiple of 4 float32 values):
```bash
python bin_file_utils.py your_file.bin
```

## Performance Tips

1. **For batch processing**: Keep the interpolation node running and process files sequentially
2. **Scale factors**: Higher values create denser clouds but increase processing time
3. **Method selection**: Use 'linear' for best speed/quality tradeoff
4. **Variance filtering**: Only enable if you need noise reduction (adds overhead)

## License

AGPL-3.0 - See main package LICENSE file for details.
