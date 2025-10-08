import numpy as np
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def pointcloud2_to_array(cloud_msg):
    """
    Convert a sensor_msgs/PointCloud2 message to a NumPy array with x, y, z, and intensity.
    
    Args:
        cloud_msg: sensor_msgs/PointCloud2 message
        
    Returns:
        numpy array of shape (N, 4) where columns are [x, y, z, intensity]
    """
    # Get the point cloud as a structured numpy array
    pc_array = rnp.point_cloud2.pointcloud2_to_array(cloud_msg)
    # intensity_array = pc2.read_points_numpy(cloud_msg, field_names=("intensity"))
    
    # Convert structured array to regular array
    points = np.zeros((pc_array.shape[0], 4), dtype=np.float32)

    points[:, 0] = pc_array['x']
    points[:, 1] = pc_array['y']
    points[:, 2] = pc_array['z']
    points[:, 3] = np.linalg.norm(pc_array['intensity'])
    
    return points


def process_rosbag(bag_path, topic_name, output_dir='./data/npy_data'):
    """
    Extract all PointCloud2 messages from a rosbag and save as numpy arrays.
    
    Args:
        bag_path: Path to the rosbag file/directory
        topic_name: Name of the PointCloud2 topic (e.g., '/velodyne_points')
        output_dir: Directory to save the numpy arrays
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    print("Processing rosbag...")
    with Reader(bag_path) as reader:
        # Get connections for the specific topic
        connections = [c for c in reader.connections if c.topic == topic_name]
        
        if not connections:
            print(f"Topic '{topic_name}' not found in rosbag!")
            print("Available topics:")
            for c in reader.connections:
                print(f"  - {c.topic} ({c.msgtype})")
            return
        
        print(f"Processing topic: {topic_name}")
        print(f"Message type: {connections[0].msgtype}")
        
        # Iterate through messages
        count = 0
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # Deserialize the message
            msg = deserialize_cdr(rawdata, connection.msgtype)
            
            # Convert to numpy array
            points = pointcloud2_to_array(msg)
            
            # Save with timestamp in filename
            filename = f"pointcloud_{timestamp}.npy"
            filepath = os.path.join(output_dir, filename)
            np.save(filepath, points)
            
            count += 1
            if count % 10 == 0:
                print(f"Processed {count} point clouds...")
        
        print(f"Done! Saved {count} point clouds to {output_dir}")


def process_single_message(bag_path, topic_name, message_index=0):
    """
    Extract a single PointCloud2 message from a rosbag.
    
    Args:
        bag_path: Path to the rosbag file/directory
        topic_name: Name of the PointCloud2 topic
        message_index: Which message to extract (0 = first message)
        
    Returns:
        numpy array of shape (N, 4) where columns are [x, y, z, intensity]
    """
    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        
        if not connections:
            raise ValueError(f"Topic '{topic_name}' not found in rosbag!")
        
        for idx, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=connections)):
            if idx == message_index:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                return pointcloud2_to_array(msg)
        
        raise IndexError(f"Message index {message_index} not found (only {idx+1} messages available)")


def combine_all_pointclouds(bag_path, topic_name):
    """
    Combine all PointCloud2 messages from a rosbag into a single numpy array.
    Useful for creating a map or for batch processing.
    
    Args:
        bag_path: Path to the rosbag file/directory
        topic_name: Name of the PointCloud2 topic
        
    Returns:
        numpy array of shape (N, 4) where columns are [x, y, z, intensity]
    """
    all_points = []
    
    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        
        if not connections:
            raise ValueError(f"Topic '{topic_name}' not found in rosbag!")
        
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            points = pointcloud2_to_array(msg)
            all_points.append(points)
        
        # Concatenate all point clouds
        combined = np.vstack(all_points)
        print(f"Combined {len(all_points)} point clouds into array of shape {combined.shape}")
        
        return combined


# Example usage
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Extract PointCloud2 messages from ROS bags to NumPy arrays',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process all clouds and save separately
  python script.py --bag /path/to/bag --topic /velodyne_points --mode all --output-dir ./clouds
  
  # Get a single cloud
  python script.py --bag /path/to/bag --topic /velodyne_points --mode single --npy-path single.npy --index 0
  
  # Combine all clouds into one file
  python script.py --bag /path/to/bag --topic /velodyne_points --mode combined --npy-path combined.npy
        """
    )
    
    parser.add_argument('--bag', '--bag-path', dest='bag_path', required=True,
                        help='Path to the ROS bag file or directory')
    parser.add_argument('--topic', '--topic-name', dest='topic_name', required=True,
                        help='Name of the PointCloud2 topic (e.g., /velodyne_points)')
    parser.add_argument('--mode', choices=['all', 'single', 'combined'], required=True,
                        help='Processing mode: "all" (save each cloud separately), '
                             '"single" (extract one cloud), "combined" (merge all clouds)')
    parser.add_argument('--output-dir', default='./output_clouds',
                        help='Output directory for "all" mode (default: ./output_clouds)')
    parser.add_argument('--npy-path', default='pointcloud.npy',
                        help='Output .npy file path for "single" or "combined" mode (default: pointcloud.npy)')
    parser.add_argument('--index', type=int, default=0,
                        help='Message index for "single" mode (default: 0)')
    
    args = parser.parse_args()
    
    # Execute based on mode
    if args.mode == 'all':
        process_rosbag(
            bag_path=args.bag_path,
            topic_name=args.topic_name,
            output_dir=args.output_dir
        )
        
    elif args.mode == 'single':
        points = process_single_message(
            bag_path=args.bag_path,
            topic_name=args.topic_name,
            message_index=args.index
        )
        print(f"Point cloud shape: {points.shape}")
        np.save(args.npy_path, points)
        print(f"Saved to {args.npy_path}")
        
    elif args.mode == 'combined':
        all_points = combine_all_pointclouds(
            bag_path=args.bag_path,
            topic_name=args.topic_name
        )

        np.save(args.npy_path, all_points)
        print(f"Saved combined cloud to {args.npy_path}")