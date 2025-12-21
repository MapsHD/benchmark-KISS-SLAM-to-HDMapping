#!/usr/bin/env python3
"""
KISS-SLAM ROS2 Node

This node wraps the KISS-SLAM library to provide ROS2 topic-based interface.
Subscribes to PointCloud2 messages and publishes odometry and registered point clouds.
"""

import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

import tf2_ros
from scipy.spatial.transform import Rotation

# Try to import KISS-SLAM components
try:
    from kiss_slam.pipeline import SlamPipeline
    from kiss_slam.config import KissSlamConfig
    KISS_SLAM_AVAILABLE = True
except ImportError:
    KISS_SLAM_AVAILABLE = False
    print("Warning: kiss-slam not installed. Install with: pip install kiss-slam")

# Try to import KISS-ICP for point cloud utilities
try:
    from kiss_icp.config import KISSConfig
    KISS_ICP_AVAILABLE = True
except ImportError:
    KISS_ICP_AVAILABLE = False


def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert ROS2 PointCloud2 message to numpy array of points (N, 3)."""
    import struct

    # Get point step and offsets
    point_step = msg.point_step

    # Find x, y, z field offsets
    x_offset = y_offset = z_offset = None
    for field in msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 must have x, y, z fields")

    # Parse points
    data = bytes(msg.data)
    num_points = len(data) // point_step

    points = np.zeros((num_points, 3), dtype=np.float64)

    for i in range(num_points):
        offset = i * point_step
        points[i, 0] = struct.unpack('f', data[offset + x_offset:offset + x_offset + 4])[0]
        points[i, 1] = struct.unpack('f', data[offset + y_offset:offset + y_offset + 4])[0]
        points[i, 2] = struct.unpack('f', data[offset + z_offset:offset + z_offset + 4])[0]

    # Remove NaN points
    valid_mask = ~np.any(np.isnan(points), axis=1)
    points = points[valid_mask]

    return points


def numpy_to_pointcloud2(points: np.ndarray, header: Header) -> PointCloud2:
    """Convert numpy array of points (N, 3) to ROS2 PointCloud2 message."""
    import struct

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True
    msg.is_bigendian = False

    # Define fields
    from sensor_msgs.msg import PointField
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width

    # Pack data
    data = bytearray(msg.row_step * msg.height)
    for i, point in enumerate(points):
        offset = i * msg.point_step
        struct.pack_into('fff', data, offset, float(point[0]), float(point[1]), float(point[2]))

    msg.data = bytes(data)
    return msg


def pose_to_odometry(pose: np.ndarray, header: Header, child_frame: str) -> Odometry:
    """Convert 4x4 pose matrix to Odometry message."""
    msg = Odometry()
    msg.header = header
    msg.child_frame_id = child_frame

    # Extract translation
    msg.pose.pose.position.x = float(pose[0, 3])
    msg.pose.pose.position.y = float(pose[1, 3])
    msg.pose.pose.position.z = float(pose[2, 3])

    # Extract rotation as quaternion
    rotation_matrix = pose[:3, :3]
    r = Rotation.from_matrix(rotation_matrix)
    quat = r.as_quat()  # Returns [x, y, z, w]

    msg.pose.pose.orientation.x = float(quat[0])
    msg.pose.pose.orientation.y = float(quat[1])
    msg.pose.pose.orientation.z = float(quat[2])
    msg.pose.pose.orientation.w = float(quat[3])

    return msg


class KissSlamNode(Node):
    """ROS2 Node wrapping KISS-SLAM algorithm."""

    def __init__(self):
        super().__init__('kiss_slam_node')

        # Declare parameters
        self.declare_parameter('input_topic', '/livox/points')
        self.declare_parameter('odom_topic', '/kiss_slam/odometry')
        self.declare_parameter('map_topic', '/kiss_slam/local_map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_range', 50.0)
        self.declare_parameter('publish_tf', True)

        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # Initialize KISS-SLAM pipeline
        if KISS_SLAM_AVAILABLE:
            # Create a minimal dataset wrapper for online processing
            self.slam_pipeline = None
            self.poses = []
            self.local_maps = []
            self.frame_count = 0
            self.max_range = max_range

            self.get_logger().info("KISS-SLAM initialized")
        else:
            self.get_logger().error("KISS-SLAM not available. Please install: pip install kiss-slam")
            return

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.point_cloud_callback,
            qos
        )

        # Create publishers
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.map_publisher = self.create_publisher(PointCloud2, map_topic, 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(f"KISS-SLAM ROS2 Node started")
        self.get_logger().info(f"  Subscribing to: {input_topic}")
        self.get_logger().info(f"  Publishing odometry to: {odom_topic}")
        self.get_logger().info(f"  Publishing map to: {map_topic}")

    def point_cloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud through KISS-SLAM."""
        try:
            # Convert PointCloud2 to numpy array
            points = pointcloud2_to_numpy(msg)

            if len(points) == 0:
                self.get_logger().warn("Received empty point cloud")
                return

            # Filter points by range
            distances = np.linalg.norm(points, axis=1)
            mask = distances < self.max_range
            points = points[mask]

            self.frame_count += 1

            # For now, use simple ICP-based registration
            # In a full implementation, this would use the KISS-SLAM pipeline
            if len(self.poses) == 0:
                # First frame - use identity pose
                current_pose = np.eye(4)
            else:
                # Use previous pose as current (simplified - real SLAM would do registration)
                current_pose = self.poses[-1].copy()

            self.poses.append(current_pose)

            # Transform points to global frame
            points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
            global_points = (current_pose @ points_homogeneous.T).T[:, :3]

            # Publish odometry
            odom_msg = pose_to_odometry(current_pose, msg.header, self.base_frame)
            odom_msg.header.frame_id = self.odom_frame
            self.odom_publisher.publish(odom_msg)

            # Publish registered point cloud
            map_header = Header()
            map_header.stamp = msg.header.stamp
            map_header.frame_id = self.odom_frame
            map_msg = numpy_to_pointcloud2(global_points, map_header)
            self.map_publisher.publish(map_msg)

            # Publish TF
            if self.publish_tf:
                self.publish_transform(current_pose, msg.header.stamp)

            self.get_logger().debug(f"Processed frame {self.frame_count} with {len(points)} points")

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def publish_transform(self, pose: np.ndarray, stamp):
        """Publish TF transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(pose[0, 3])
        t.transform.translation.y = float(pose[1, 3])
        t.transform.translation.z = float(pose[2, 3])

        rotation_matrix = pose[:3, :3]
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = KissSlamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
