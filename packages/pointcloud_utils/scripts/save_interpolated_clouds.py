#!/usr/bin/env python3
"""
save_interpolated_clouds.py — ROS2 subscriber that saves interpolated point
clouds as KITTI-format .bin files.

Subscribes to a PointCloud2 topic and saves each message as a .bin file in
the output directory, named after the corresponding input .bin stem (by
sorted order, since bin_publisher_node plays files in alphabetical order with
loop=false).

Used by run_full_pipeline.sh to generate static interpolated datasets from the
ROS2 interpolation pipeline without any real-time constraints.

Usage:
  python3 save_interpolated_clouds.py \
      --input-dir  /data/reduced-kitti/velodyne \
      --output-dir /data/interpolated-kitti/nearest_extreme/velodyne \
      --topic      interpolated_point_cloud \
      [--timeout   120]
"""

import argparse
import sys
import os
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudSaverNode(Node):

    def __init__(self, args):
        super().__init__('cloud_saver')

        input_dir  = Path(args.input_dir)
        self._out  = Path(args.output_dir)
        self._out.mkdir(parents=True, exist_ok=True)

        self._stems = sorted(p.stem for p in input_dir.glob('*.bin'))
        if not self._stems:
            self.get_logger().error(f'No .bin files found in {input_dir}')
            raise SystemExit(1)

        self._idx     = 0
        self._total   = len(self._stems)
        self._timeout = args.timeout

        self.get_logger().info(
            f'Expecting {self._total} frames from {args.topic} → {self._out}')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PointCloud2, args.topic, self._cb, qos)

        # Watchdog: shut down if no message arrives within timeout seconds
        self._watchdog = self.create_timer(self._timeout, self._on_timeout)
        self._last_received = self.get_clock().now()

    def _cb(self, msg: PointCloud2):
        if self._idx >= self._total:
            return

        # Reset watchdog
        self._watchdog.cancel()
        self._watchdog = self.create_timer(self._timeout, self._on_timeout)

        stem     = self._stems[self._idx]
        out_path = self._out / f'{stem}.bin'

        try:
            pts = np.array(
                list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                              skip_nans=True)),
                dtype=np.float32,
            )
            if pts.ndim == 1:
                pts = pts.reshape(-1, 1)
            # Ensure 4-column layout
            if pts.shape[1] < 4:
                pad = np.zeros((pts.shape[0], 4 - pts.shape[1]), dtype=np.float32)
                pts = np.hstack([pts, pad])
            pts[:, :4].tofile(str(out_path))
            self.get_logger().info(
                f'[{self._idx + 1}/{self._total}] {stem}: {len(pts)} pts → {out_path.name}')
        except Exception as exc:
            self.get_logger().warn(f'[{self._idx + 1}/{self._total}] {stem}: {exc}')
            # Write empty file so downstream scripts don't miss the frame
            out_path.write_bytes(b'')

        self._idx += 1
        if self._idx >= self._total:
            self.get_logger().info('All frames saved. Shutting down.')
            rclpy.shutdown()

    def _on_timeout(self):
        self.get_logger().error(
            f'No message received for {self._timeout}s '
            f'(saved {self._idx}/{self._total}). Shutting down.')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Save interpolated ROS2 PointCloud2 messages as KITTI .bin files')
    parser.add_argument('--input-dir',  required=True, metavar='DIR',
                        help='Source .bin directory (for stem ordering)')
    parser.add_argument('--output-dir', required=True, metavar='DIR',
                        help='Destination directory for saved .bin files')
    parser.add_argument('--topic', default='interpolated_point_cloud',
                        help='PointCloud2 topic to subscribe to')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Seconds to wait for next message before aborting (default: 30)')

    args = parser.parse_args()

    rclpy.init()
    try:
        node = CloudSaverNode(args)
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
