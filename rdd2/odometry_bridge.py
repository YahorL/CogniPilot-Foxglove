#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import message_filters
from typing import Optional, Tuple

class OdometryDifferencePublisher(Node):
    """ROS2 node that computes and publishes differences between two odometry topics.
    
    This node subscribes to two odometry topics, computes position and orientation
    differences, converts quaternions to Euler angles, and publishes the results
    to separate topics for visualization in Foxglove.
    """

    def __init__(self) -> None:
        """Initialize the odometry difference publisher node."""
        super().__init__('odometry_difference_publisher')
        
        # Declare parameters
        self.declare_parameter('odom_true', '/odom')
        self.declare_parameter('odom_est', '/cerebri/out/odometry')
        self.declare_parameter('sync_tolerance', 0.1)  # seconds
        
        # Get parameters
        self.odom_true = self.get_parameter('odom_true').get_parameter_value().string_value
        self.odom_est = self.get_parameter('odom_est').get_parameter_value().string_value
        self.sync_tolerance = self.get_parameter('sync_tolerance').get_parameter_value().double_value
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        
        # Create synchronized subscribers
        self.odom1_sub = message_filters.Subscriber(
            self, Odometry, self.odom_true, qos_profile=qos_profile
        )
        self.odom2_sub = message_filters.Subscriber(
            self, Odometry, self.odom_est, qos_profile=qos_profile
        )
        
        # Synchronize messages
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.odom1_sub, self.odom2_sub], 
            queue_size=10, 
            slop=self.sync_tolerance
        )
        self.time_sync.registerCallback(self.synchronized_callback)
        
        # Publishers for differences
        self.pos_diff_pub = self.create_publisher(
            Vector3Stamped, '/foxglove/pos_diff', 10
        )
        self.ori_diff_pub = self.create_publisher(
            Vector3Stamped, '/foxglove/att_diff', 10
        )
        
        # Publishers for Euler angles
        self.euler1_pub = self.create_publisher(
            Vector3Stamped, '/foxglove/att_true', 10
        )
        self.euler2_pub = self.create_publisher(
            Vector3Stamped, '/foxglove/att_est', 10
        )
        
        self.get_logger().info(
            f'Odometry difference publisher started. '
                f'Subscribing to {self.odom_true} and {self.odom_est}'
        )
    
    def quaternion_to_euler_zyx(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Convert quaternion to Euler angles (yaw-pitch-roll, ZYX order) in degrees.
        
        Args:
            x: Quaternion x component
            y: Quaternion y component
            z: Quaternion z component
            w: Quaternion w component
            
        Returns:
            Euler angles as [yaw, pitch, roll] in degrees
        """
        rotation = R.from_quat([x, y, z, w])
        euler_rad = rotation.as_euler('zyx', degrees=False)
        return np.rad2deg(euler_rad)
    
    def quaternion_difference(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Compute relative rotation from q2 to q1.
        
        Args:
            q1: Target quaternion [x, y, z, w] format
            q2: Reference quaternion [x, y, z, w] format
        
        Returns:
            Relative rotation quaternion q_diff = q1 * q2^(-1)
        """
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        r_diff = r1 * r2.inv()
        return r_diff.as_quat()
    
    def wrap_to_pi_degrees(self, angle_deg: float) -> float:
        """Wrap angle to [-180, 180] range in degrees."""
        return ((angle_deg + 180.0) % 360.0) - 180.0
    
    def synchronized_callback(self, odom1_msg: Odometry, odom2_msg: Odometry) -> None:
        """Process synchronized odometry messages and publish differences.
        
        Args:
            odom1_msg: Odometry message from first topic (typically ground truth)
            odom2_msg: Odometry message from second topic (typically estimated)
        """
        try:
            # Use the timestamp from the first message as reference
            timestamp = odom1_msg.header.stamp
            frame_id = odom1_msg.header.frame_id
            
            # Extract positions
            pos1 = np.array([
                odom1_msg.pose.pose.position.x,
                odom1_msg.pose.pose.position.y,
                odom1_msg.pose.pose.position.z
            ])
            
            pos2 = np.array([
                odom2_msg.pose.pose.position.x,
                odom2_msg.pose.pose.position.y,
                odom2_msg.pose.pose.position.z
            ])
            
            # Extract quaternions
            quat1 = np.array([
                odom1_msg.pose.pose.orientation.x,
                odom1_msg.pose.pose.orientation.y,
                odom1_msg.pose.pose.orientation.z,
                odom1_msg.pose.pose.orientation.w
            ])
            
            quat2 = np.array([
                odom2_msg.pose.pose.orientation.x,
                odom2_msg.pose.pose.orientation.y,
                odom2_msg.pose.pose.orientation.z,
                odom2_msg.pose.pose.orientation.w
            ])
            
            # Compute position difference (topic1 - topic2)
            pos_diff = pos1 - pos2
            
            # Compute orientation difference using quaternions
            quat_diff = self.quaternion_difference(quat1, quat2)
            euler_diff = self.quaternion_to_euler_zyx(quat_diff[0], quat_diff[1], quat_diff[2], quat_diff[3])
            
            # Wrap angles to [-180, 180] degrees
            euler_diff = np.array([self.wrap_to_pi_degrees(angle) for angle in euler_diff])
            
            # Convert quaternions to Euler angles
            euler1 = self.quaternion_to_euler_zyx(quat1[0], quat1[1], quat1[2], quat1[3])
            euler2 = self.quaternion_to_euler_zyx(quat2[0], quat2[1], quat2[2], quat2[3])
            
            # Create and publish position difference
            pos_diff_msg = Vector3Stamped()
            pos_diff_msg.header = Header(stamp=timestamp, frame_id=frame_id)
            pos_diff_msg.vector.x = float(pos_diff[0])
            pos_diff_msg.vector.y = float(pos_diff[1])
            pos_diff_msg.vector.z = float(pos_diff[2])
            self.pos_diff_pub.publish(pos_diff_msg)
            
            # Create and publish orientation difference
            ori_diff_msg = Vector3Stamped()
            ori_diff_msg.header = Header(stamp=timestamp, frame_id=frame_id)
            ori_diff_msg.vector.x = float(euler_diff[0])  # yaw difference
            ori_diff_msg.vector.y = float(euler_diff[1])  # pitch difference
            ori_diff_msg.vector.z = float(euler_diff[2])  # roll difference
            self.ori_diff_pub.publish(ori_diff_msg)
            
            # Create and publish Euler angles for topic1
            euler1_msg = Vector3Stamped()
            euler1_msg.header = Header(stamp=timestamp, frame_id=frame_id)
            euler1_msg.vector.x = float(euler1[0])  # yaw
            euler1_msg.vector.y = float(euler1[1])  # pitch
            euler1_msg.vector.z = float(euler1[2])  # roll
            self.euler1_pub.publish(euler1_msg)
            
            # Create and publish Euler angles for topic2
            euler2_msg = Vector3Stamped()
            euler2_msg.header = Header(stamp=timestamp, frame_id=frame_id)
            euler2_msg.vector.x = float(euler2[0])  # yaw
            euler2_msg.vector.y = float(euler2[1])  # pitch
            euler2_msg.vector.z = float(euler2[2])  # roll
            self.euler2_pub.publish(euler2_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing odometry messages: {str(e)}')

def main(args: Optional[list] = None) -> None:
    """Main function to run the odometry difference publisher node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    
    try:
        node = OdometryDifferencePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 