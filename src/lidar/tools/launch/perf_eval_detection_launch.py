#!/usr/bin/env python3
"""
Launch file for the detection side of performance evaluation.
Runs in the openpcdet-prebuilt container.

Launches:
  - pcdet_node (ros2_node.py with perf tracking enabled)
  - perf_collector_node (collects and correlates metrics)

Usage:
    ros2 launch <package> perf_eval_detection_launch.py \
        cfg_file:=src/lidar/tools/cfgs/kitti_models/interpolated/parta2_anchor.yaml \
        ckpt:=data/pretrained-models/PartA2_7940.pth \
        model_name:=parta2_anchor \
        config_name:=optimized \
        output_dir:=/app/data/perf_results \
        warmup_frames:=10 \
        max_frames:=500

Note: Since this is a Python-only launch (no ament package), run nodes directly:
    # Terminal 1: Detection node
    python3 src/lidar/tools/ros2_node.py \
        --cfg_file <cfg> --ckpt <ckpt> \
        --pointcloud_topic interpolated_point_cloud \
        --enable_perf_tracking --model_name parta2_anchor

    # Terminal 2: Collector node
    python3 src/lidar/tools/perf_collector_node.py --ros-args \
        -p output_dir:=/app/data/perf_results \
        -p config_name:=optimized \
        -p model_name:=parta2_anchor \
        -p warmup_frames:=10 \
        -p max_frames:=500
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """Set up detection and collector nodes."""
    cfg_file = context.launch_configurations.get('cfg_file', '')
    ckpt = context.launch_configurations.get('ckpt', '')
    model_name = context.launch_configurations.get('model_name', 'unknown')
    config_name = context.launch_configurations.get('config_name', 'default')
    output_dir = context.launch_configurations.get('output_dir', '/tmp/perf_results')
    warmup_frames = context.launch_configurations.get('warmup_frames', '10')
    max_frames = context.launch_configurations.get('max_frames', '500')
    pointcloud_topic = context.launch_configurations.get('pointcloud_topic', 'interpolated_point_cloud')

    nodes = []

    # Detection node (pcdet_node)
    pcdet_node = ExecuteProcess(
        cmd=[
            sys.executable, 'src/lidar/tools/ros2_node.py',
            '--cfg_file', cfg_file,
            '--ckpt', ckpt,
            '--pointcloud_topic', pointcloud_topic,
            '--enable_perf_tracking',
            '--model_name', model_name,
        ],
        output='screen',
    )
    nodes.append(pcdet_node)

    # Performance collector node
    collector_node = ExecuteProcess(
        cmd=[
            sys.executable, 'src/lidar/tools/perf_collector_node.py',
            '--ros-args',
            '-p', f'output_dir:={output_dir}',
            '-p', f'config_name:={config_name}',
            '-p', f'model_name:={model_name}',
            '-p', f'warmup_frames:={warmup_frames}',
            '-p', f'max_frames:={max_frames}',
        ],
        output='screen',
    )
    nodes.append(collector_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cfg_file', default_value='',
            description='Path to PCDet model config YAML'),
        DeclareLaunchArgument('ckpt', default_value='',
            description='Path to pretrained model checkpoint'),
        DeclareLaunchArgument('model_name', default_value='unknown',
            description='Model name for perf tracking'),
        DeclareLaunchArgument('config_name', default_value='default',
            description='Interpolation config name for perf tracking'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/perf_results',
            description='Output directory for perf results'),
        DeclareLaunchArgument('warmup_frames', default_value='10',
            description='Number of warmup frames to skip'),
        DeclareLaunchArgument('max_frames', default_value='500',
            description='Maximum frames to record (0=unlimited)'),
        DeclareLaunchArgument('pointcloud_topic', default_value='interpolated_point_cloud',
            description='Input point cloud topic for detection'),

        LogInfo(msg="Launching detection pipeline with performance tracking"),
        OpaqueFunction(function=launch_setup),
    ])
