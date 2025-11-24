#!/usr/bin/env python3
import argparse
from pathlib import Path

import copy
import struct
import math
import numpy as np
import torch
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header

try:
    import open3d
    import open3d_custom_vis_utils as V
    OPEN3D_FLAG = True
except:
    OPEN3D_FLAG = False

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils


class Ros2Dataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None):
        """
        Dataset for processing single point cloud frames
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.current_points = None
        self.current_frame_id = 0

    def __len__(self):
        return 1

    def __getitem__(self, index):
        if self.current_points is None:
            raise ValueError("No point cloud data available")

        input_dict = {
            'points': self.current_points,
            'frame_id': self.current_frame_id,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict

    def set_points(self, points, frame_id):
        """Set the current point cloud data"""
        self.current_points = points
        self.current_frame_id = frame_id


class PCDetNode(Node):
    def __init__(self, args, cfg):
        super().__init__('pcdet_node')
        
        self.logger_pcdet = common_utils.create_logger()
        self.logger_pcdet.info('-----------------ROS2 PCDet Node-------------------------')
        
        # Initialize dataset
        self.demo_dataset = Ros2Dataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            root_path=Path('.'),
            logger=self.logger_pcdet
        )
        self.processing_frame = False
        
        self.get_logger().info(f"Dataset initialized with classes: {cfg.CLASS_NAMES}")
        self.get_logger().info("Building model...")

        # Initialize model
        self.model = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(cfg.CLASS_NAMES),
            dataset=self.demo_dataset
        )
        self.get_logger().info("Model built successfully.")
        self.get_logger().info(f"Loading model from checkpoint {args.ckpt}...")

        self.model.load_params_from_file(filename=args.ckpt, logger=self.logger_pcdet, to_cpu=True)
        self.model.cuda()
        self.model.eval()
        
        self.class_names = cfg.CLASS_NAMES
        self.frame_count = 0
        
        self.get_logger().info("Model loaded.")
        self.get_logger().info(f"Subscribing to point cloud topic: {args.pointcloud_topic}")
        # ROS2 subscribers and publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            args.pointcloud_topic,
            self.pointcloud_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=2)
        )

        # Publisher for corrected point cloud
        self.corrected_pc_pub = self.create_publisher(
            PointCloud2,
            'corrected_pointcloud',
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'detected_objects',
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=15)
        )
        
        # Open3D visualization (optional)
        self.use_visualization = args.visualize
        if self.use_visualization and OPEN3D_FLAG:
            self.vis = open3d.visualization.Visualizer()
            self.vis.create_window()
            self.pcdFrame = V.PCDFrame()
            self.vis.get_render_option().point_size = 1.0
            self.vis.get_render_option().background_color = np.zeros(3)
            
            if args.draw_origin:
                axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
                self.vis.add_geometry(axis_pcd)
        
        self.get_logger().info('PCDet node initialized and ready')

    def pointcloud_callback(self, cloud_msg):
        """Callback function for point cloud messages"""
        self.get_logger().info(f'Processing frame {self.frame_count} received in timestamp {time.time():.3f}')
        start_time = time.time()
        if self.processing_frame:
            self.get_logger().warn('Still processing previous frame, skipping this one')
            self.frame_count += 1
            self.get_logger().info(f'Frame {self.frame_count} processed in {time.time() - start_time:.2f} seconds')
            return
        
        self.processing_frame = True
        current_frame = self.frame_count
        # data_len = len(cloud_msg.data)
        # expected_len = cloud_msg.row_step * cloud_msg.height

        # self.get_logger().info(
        #     f"PointCloud2 dims: width={cloud_msg.width}, height={cloud_msg.height}, "
        #     f"point_step={cloud_msg.point_step}, row_step={cloud_msg.row_step}, "
        #     f"data_len={data_len}, expected_len={expected_len}"
        # Convert ROS2 PointCloud2 to numpy array

        stepTime = time.time()
        points = self.pointcloud2_to_array(cloud_msg)

        self.get_logger().info(f'PCL conversion processed in {time.time() - stepTime:.2f} seconds')
        
        if points is None or len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return
        
        # Publish corrected point cloud
        stepTime = time.time()
        self.publish_corrected_pointcloud(points, cloud_msg.header)
        self.get_logger().info(f'Corrected PCL published in {time.time() - stepTime:.2f} seconds')

        # Set points in dataset
        self.demo_dataset.set_points(points, self.frame_count)
        
        # Run inference
        stepTime = time.time()
        try:
            with torch.no_grad():
                data_dict = self.demo_dataset[0]
                data_dict = self.demo_dataset.collate_batch([data_dict])
                load_data_to_gpu(data_dict)
                pred_dicts, _ = self.model.forward(data_dict)
                
                # Extract predictions
                pred_boxes = pred_dicts[0]['pred_boxes'].cpu().numpy()
                pred_scores = pred_dicts[0]['pred_scores'].cpu().numpy()
                pred_labels = pred_dicts[0]['pred_labels'].cpu().numpy()
                
                self.get_logger().info(f'Detected {len(pred_boxes)} objects')
                self.get_logger().info(f'Current frame: {cloud_msg.header.frame_id}, Timestamp: {cloud_msg.header.stamp.sec}.{cloud_msg.header.stamp.nanosec}')
                # Publish markers
                self.publish_markers(pred_boxes, pred_scores, pred_labels, cloud_msg.header)
                
                # Update visualization if enabled
                if self.use_visualization and OPEN3D_FLAG:
                    V.update_scene(
                        points=data_dict['points'][:, 1:],
                        pcdFrame=self.pcdFrame,
                        vis=self.vis,
                        ref_boxes=pred_dicts[0]['pred_boxes'],
                        ref_scores=pred_dicts[0]['pred_scores'],
                        ref_labels=pred_dicts[0]['pred_labels']
                    )
                
        except Exception as e:
            self.get_logger().error(f'Error during inference: {str(e)}')
            self.processing_frame = False

        self.get_logger().info(f'Inference ran in {time.time() - stepTime:.2f} seconds')
        current_frame += 1
        self.get_logger().info(f'Frame {self.frame_count} processed in {time.time() - start_time:.2f} seconds')
        self.frame_count += 1
        self.processing_frame = False

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array (defensive against truncated data)."""

        # Basic diagnostics
        data_len = len(cloud_msg.data) if cloud_msg.data is not None else 0
        expected_len = int(cloud_msg.row_step) * int(cloud_msg.height)
        point_step = int(cloud_msg.point_step) if cloud_msg.point_step is not None else 0

        # If something is wrong with message metadata vs payload, try a best-effort trim
        if data_len != expected_len and point_step > 0 and data_len > 0:
            # self.get_logger().warn(
            #     f"PointCloud2 data length mismatch: data_len={data_len} != row_step*height={expected_len}. "
            #     "Attempting best-effort parsing by trimming the buffer to full points."
            # )
            n_full_points = data_len // point_step
            trimmed_bytes = cloud_msg.data[: n_full_points * point_step]

            # Make a shallow copy of the message and adjust payload + layout so read_points won't blow up
            try:
                trimmed_msg = copy.copy(cloud_msg)
                trimmed_msg.data = trimmed_bytes
                trimmed_msg.width = n_full_points
                trimmed_msg.height = 1
                trimmed_msg.row_step = n_full_points * point_step
                cloud_for_read = trimmed_msg
            except Exception:
                # If copy fails for any reason, fall back to original (we'll manual-parse below)
                cloud_for_read = cloud_msg
        else:
            cloud_for_read = cloud_msg

        # Try the normal, convenient parser first (preserves all field names)
        points_list = []
        try:
            for p in point_cloud2.read_points(cloud_for_read, skip_nans=True):
                # keep the same assumptions: x,y,z and optionally intensity
                if len(p) >= 4:
                    points_list.append([p[0], p[1], p[2], p[3]])
                elif len(p) >= 3:
                    points_list.append([p[0], p[1], p[2], 0.0])
        except struct.error as e:
            # read_points tried to unpack past buffer or hit other struct errors. Do manual conservative parse.
            self.get_logger().warn(f"pc2.read_points failed with struct.error: {e}. Falling back to manual parse.")
            # Build a map of field name -> (offset, datatype)
            field_map = {}
            for f in cloud_msg.fields:
                # sensor_msgs.msg.PointField datatype for FLOAT32 is 7
                # we'll only support FLOAT32 for x,y,z,intensity here (most common)
                field_map[f.name] = (int(f.offset), int(getattr(f, 'datatype', -1)))

            # Determine offsets for x,y,z,intensity if present
            wanted = []
            for name in ("x", "y", "z", "intensity"):
                if name in field_map and field_map[name][1] == 7:  # 7 == FLOAT32
                    wanted.append((name, field_map[name][0]))
                else:
                    # Not present or not float32 -> use None to fill 0.0 for that field
                    wanted.append((name, None))

            # Endianness
            endian = '<' if not cloud_msg.is_bigendian else '>'
            float_fmt = endian + 'f'
            if point_step == 0:
                self.get_logger().error("point_step == 0, cannot parse points.")
                return None

            total_points = (len(cloud_msg.data) // point_step)
            for i in range(total_points):
                base = i * point_step
                try:
                    x = y = z = None
                    intensity = 0.0
                    # x
                    if wanted[0][1] is not None:
                        x = struct.unpack_from(float_fmt, cloud_msg.data, base + wanted[0][1])[0]
                    # y
                    if wanted[1][1] is not None:
                        y = struct.unpack_from(float_fmt, cloud_msg.data, base + wanted[1][1])[0]
                    # z
                    if wanted[2][1] is not None:
                        z = struct.unpack_from(float_fmt, cloud_msg.data, base + wanted[2][1])[0]
                    # intensity
                    if wanted[3][1] is not None:
                        intensity = struct.unpack_from(float_fmt, cloud_msg.data, base + wanted[3][1])[0]

                    # If x/y/z are missing (None) skip this point because we can't interpret it
                    if x is None or y is None or z is None:
                        continue

                    # skip NaN/Inf like skip_nans=True
                    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                        continue

                    points_list.append([x, y, z, float(intensity)])
                except struct.error:
                    # If a struct error occurs for this point, skip it (incomplete tail)
                    continue

        # If still empty, return None (same behavior as your original)
        if len(points_list) == 0:
            return None

        points = np.array(points_list, dtype=np.float32)
        return points

    def publish_corrected_pointcloud(self, points, header):
        """Publish corrected point cloud as PointCloud2 message"""
        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create new header with same frame_id and timestamp
        corrected_header = Header()
        corrected_header.stamp = header.stamp
        corrected_header.frame_id = header.frame_id
        
        # Convert numpy array to PointCloud2
        corrected_msg = point_cloud2.create_cloud(corrected_header, fields, points)
        
        # Publish
        self.corrected_pc_pub.publish(corrected_msg)
        self.get_logger().info(f'Published corrected point cloud with {len(points)} points')

    def publish_markers(self, boxes, scores, labels, header):
        """Publish detected bounding boxes as ROS2 markers"""
        marker_array = MarkerArray()
        
        # Delete old markers
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Create markers for each detected object
        for i, (box, score, label) in enumerate(zip(boxes, scores, labels)):
            # Box format: [x, y, z, dx, dy, dz, heading]
            marker = Marker()
            marker.header = header
            marker.ns = "detections"
            marker.id = i + 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # Position
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            
            # Orientation (from heading)
            heading = float(box[6])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = np.sin(heading / 2)
            marker.pose.orientation.w = np.cos(heading / 2)
            
            # Scale
            marker.scale.x = float(box[3])  # length
            marker.scale.y = float(box[4])  # width
            marker.scale.z = float(box[5])  # height
            
            # Color based on class
            marker.color = self.get_color_for_class(int(label))
            # self.get_logger().info(f'Color for label {label}: {marker.color}')
            marker.color.a = min(float(score), 1.0)
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2 seconds
            
            marker_array.markers.append(marker)
            
            # Add text label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "labels"
            text_marker.id = i + 1 + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(box[0])
            text_marker.pose.position.y = float(box[1])
            text_marker.pose.position.z = float(box[2]) + float(box[5]) / 2 + 0.5
            
            class_name = self.class_names[int(label) - 1]
            text_marker.text = f"{class_name}: {score:.2f}"
            text_marker.scale.z = 0.5
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 200000000
            
            marker_array.markers.append(text_marker)
        
        self.get_logger().info(f'Publishing {len(marker_array.markers)-1} markers')
        # self.get_logger().info(f'Markers: {marker_array.markers}')
        self.marker_pub.publish(marker_array)


    def get_color_for_class(self, label_idx):
        """Return color based on class label"""
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5),  # Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5),  # Green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),  # Blue
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5),  # Yellow
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5),  # Magenta
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5),  # Cyan
        ]
        return colors[(label_idx - 1) % len(colors)]

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.use_visualization and OPEN3D_FLAG:
            self.vis.destroy_window()
        super().destroy_node()


def parse_config():
    parser = argparse.ArgumentParser(description='ROS2 PCDet Node')
    parser.add_argument('--cfg_file', type=str, default='cfgs/kitti_models/second.yaml',
                        help='specify the config for demo')
    parser.add_argument('--ckpt', type=str, required=True,
                        help='specify the pretrained model')
    parser.add_argument('--pointcloud_topic', type=str, default='/points',
                        help='ROS2 topic for point cloud messages')
    parser.add_argument('--visualize', action='store_true',
                        help='enable Open3D visualization')
    parser.add_argument('--draw_origin', action='store_true',
                        help='draw origin in visualization')

    args = parser.parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)

    return args, cfg


def main(args=None):
    # Parse arguments
    parsed_args, parsed_cfg = parse_config()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node
    node = PCDetNode(parsed_args, parsed_cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()