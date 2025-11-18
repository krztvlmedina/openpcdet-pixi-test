#!/usr/bin/env python3
"""
Utility functions for reading and writing point clouds in .bin format.

This module provides functions to convert between .bin files (commonly used in
autonomous driving datasets) and PCL point clouds for use with the
dynamic_lidar_interpolation package.

@author Abdalrahman M. Amer
@linkedin https://www.linkedin.com/in/abdalrahman-m-amer
@github https://github.com/geekgineer

@license AGPL-3.0
"""

import numpy as np
from typing import Tuple, Optional


def read_bin_pointcloud(bin_file_path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Read a point cloud from a .bin file.

    The .bin file is expected to contain point cloud data in the format:
    [x1, y1, z1, intensity1, x2, y2, z2, intensity2, ...]

    Args:
        bin_file_path: Path to the .bin file

    Returns:
        Tuple of (x, y, z, intensity) arrays

    Raises:
        FileNotFoundError: If the file does not exist
        ValueError: If the file format is invalid
    """
    # Load the binary file
    arr = np.fromfile(bin_file_path, dtype=np.float32)

    # Validate that we have the correct number of elements (multiple of 4)
    if arr.shape[0] % 4 != 0:
        raise ValueError(
            f"Invalid .bin file format. Expected multiple of 4 values, got {arr.shape[0]}"
        )

    # Reshape and extract x, y, z, intensity
    num_points = arr.shape[0] // 4
    x = arr[0::4]
    y = arr[1::4]
    z = arr[2::4]
    intensity = arr[3::4]

    return x, y, z, intensity


def write_bin_pointcloud(
    output_path: str,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    intensity: Optional[np.ndarray] = None
) -> None:
    """
    Write a point cloud to a .bin file.

    The output format is: [x1, y1, z1, intensity1, x2, y2, z2, intensity2, ...]

    Args:
        output_path: Path where the .bin file will be saved
        x: X coordinates array
        y: Y coordinates array
        z: Z coordinates array
        intensity: Intensity values array (optional, defaults to zeros)

    Raises:
        ValueError: If the arrays have different lengths
    """
    # Validate input arrays
    if not (x.shape[0] == y.shape[0] == z.shape[0]):
        raise ValueError(
            f"Arrays must have the same length. Got x={x.shape[0]}, y={y.shape[0]}, z={z.shape[0]}"
        )

    # Use zero intensity if not provided
    if intensity is None:
        intensity = np.zeros_like(x)
    elif intensity.shape[0] != x.shape[0]:
        raise ValueError(
            f"Intensity array must have the same length as x, y, z. Got {intensity.shape[0]}"
        )

    # Create interleaved array
    num_points = x.shape[0]
    arr = np.zeros(num_points * 4, dtype=np.float32)
    arr[0::4] = x
    arr[1::4] = y
    arr[2::4] = z
    arr[3::4] = intensity

    # Write to file
    arr.tofile(output_path)


def bin_to_xyz_array(bin_file_path: str) -> np.ndarray:
    """
    Read a .bin file and return XYZ points as Nx3 array.

    Args:
        bin_file_path: Path to the .bin file

    Returns:
        Nx3 numpy array with [x, y, z] coordinates
    """
    x, y, z, _ = read_bin_pointcloud(bin_file_path)
    return np.stack([x, y, z], axis=1)


def xyz_array_to_bin(xyz_points: np.ndarray, output_path: str, intensity: Optional[np.ndarray] = None) -> None:
    """
    Write XYZ points to a .bin file.

    Args:
        xyz_points: Nx3 numpy array with [x, y, z] coordinates
        output_path: Path where the .bin file will be saved
        intensity: Optional intensity values (Nx1 array)
    """
    if xyz_points.shape[1] != 3:
        raise ValueError(f"Expected Nx3 array, got shape {xyz_points.shape}")

    x = xyz_points[:, 0]
    y = xyz_points[:, 1]
    z = xyz_points[:, 2]

    write_bin_pointcloud(output_path, x, y, z, intensity)


if __name__ == "__main__":
    # Example usage
    import sys

    if len(sys.argv) < 2:
        print("Usage: python bin_file_utils.py <input.bin>")
        print("This will read the .bin file and print statistics.")
        sys.exit(1)

    input_file = sys.argv[1]

    try:
        x, y, z, intensity = read_bin_pointcloud(input_file)
        print(f"Successfully read {input_file}")
        print(f"Number of points: {x.shape[0]}")
        print(f"X range: [{x.min():.2f}, {x.max():.2f}]")
        print(f"Y range: [{y.min():.2f}, {y.max():.2f}]")
        print(f"Z range: [{z.min():.2f}, {z.max():.2f}]")
        print(f"Intensity range: [{intensity.min():.2f}, {intensity.max():.2f}]")
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)
