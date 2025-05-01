#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from transforms3d.euler import quat2euler, euler2quat
import time

class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )
        
        # Create publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/drone/visualization_marker_array',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/drone/pose_with_covariance',
            10
        )
        
        # Initialize marker array
        self.markers = MarkerArray()
        self.setup_markers()
        
        # Add debugging timer
        self.create_timer(1.0, self.debug_timer_callback)
        
        self.get_logger().info('Pose visualizer node initialized')

    def debug_timer_callback(self):
        """Periodic debug info"""
        self.get_logger().info('Pose visualizer is running...')

    def setup_markers(self):
        """Setup visualization markers for the drone."""
        # Body frame axes
        self.markers.markers = []
        
        # X axis (red)
        x_marker = self.create_arrow_marker(
            [1, 0, 0, 1],  # red
            0
        )
        self.markers.markers.append(x_marker)
        
        # Y axis (green)
        y_marker = self.create_arrow_marker(
            [0, 1, 0, 1],  # green
            1
        )
        self.markers.markers.append(y_marker)
        
        # Z axis (blue)
        z_marker = self.create_arrow_marker(
            [0, 0, 1, 1],  # blue
            2
        )
        self.markers.markers.append(z_marker)

    def create_arrow_marker(self, color, id):
        """Create an arrow marker for visualization."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.5  # shaft diameter
        marker.scale.y = 0.1  # head diameter
        marker.scale.z = 0.1  # head length
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        return marker

    def update_markers(self, position, orientation):
        """Update marker poses based on drone position and orientation."""
        # Get rotation matrix from quaternion
        roll, pitch, yaw = quat2euler(orientation)
        
        # Update each axis marker
        for i, marker in enumerate(self.markers.markers):
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            
            # Set orientation based on axis
            if i == 0:  # X axis
                q = euler2quat(roll, pitch, yaw)
            elif i == 1:  # Y axis
                q = euler2quat(roll, pitch + np.pi/2, yaw)
            else:  # Z axis
                q = euler2quat(roll + np.pi/2, pitch, yaw)
                
            marker.pose.orientation.w = q[0]
            marker.pose.orientation.x = q[1]
            marker.pose.orientation.y = q[2]
            marker.pose.orientation.z = q[3]

    def create_pose_with_covariance(self, position, orientation):
        """Create PoseWithCovarianceStamped message."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set position
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        
        # Set orientation
        msg.pose.pose.orientation.w = orientation[0]
        msg.pose.pose.orientation.x = orientation[1]
        msg.pose.pose.orientation.y = orientation[2]
        msg.pose.pose.orientation.z = orientation[3]
        
        # Set covariance (example values - adjust based on your needs)
        # 6x6 matrix [x, y, z, roll, pitch, yaw]
        covariance = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        msg.pose.covariance = covariance.flatten().tolist()
        
        return msg

    def odom_callback(self, msg):
        """Handle incoming odometry messages."""
        try:
            # Print some debug information about the message
            self.get_logger().debug(f"Received odometry message: position type: {type(msg.position)}")
            
            # Create a new position array with explicit conversion to float
            position = []
            for i in range(3):
                try:
                    if i < len(msg.position):
                        position.append(float(msg.position[i]))
                    else:
                        position.append(0.0)  # Default if index doesn't exist
                except (IndexError, TypeError, ValueError) as e:
                    self.get_logger().error(f"Error converting position[{i}]: {e}")
                    position.append(0.0)
            
            # Create a new orientation array with explicit conversion to float
            orientation = []
            for i in range(4):
                try:
                    if i < len(msg.q):
                        orientation.append(float(msg.q[i]))
                    else:
                        orientation.append(0.0 if i > 0 else 1.0)  # Default quaternion [1,0,0,0]
                except (IndexError, TypeError, ValueError) as e:
                    self.get_logger().error(f"Error converting q[{i}]: {e}")
                    orientation.append(0.0 if i > 0 else 1.0)
            
            # Update and publish markers with the safe values
            self.update_markers(position, orientation)
            self.marker_pub.publish(self.markers)
            
            # Create and publish pose with covariance
            pose_msg = self.create_pose_with_covariance(position, orientation)
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {str(e)}')

def main():
    rclpy.init()
    
    pose_visualizer = PoseVisualizer()
    
    try:
        rclpy.spin(pose_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        pose_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()