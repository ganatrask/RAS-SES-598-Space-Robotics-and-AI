#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from visualization_msgs.msg import Marker, MarkerArray

class DebugCylinderDetector(Node):
    """
    Simplified debugging version of the cylinder detector node
    that just logs information about topics and messages.
    """
    def __init__(self):
        super().__init__('debug_cylinder_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.depth_image_subscriber = self.create_subscription(
            Image,
            '/drone/front_depth',  # Original topic
            self.depth_image_callback,
            10)
            
        # Create a timer to check available topics
        self.create_timer(5.0, self.check_topics)
        
        # Publishers
        self.cylinder_center_publisher = self.create_publisher(
            Point,
            '/geometry/cylinder_center',
            10)
        
        self.cylinder_info_publisher = self.create_publisher(
            Float32MultiArray,
            '/geometry/cylinder_info',
            10)
        
        self.get_logger().info('Debug Cylinder Detector initialized')
        self.get_logger().info('Subscribed to depth image topic: /depth_camera')
        
    def depth_image_callback(self, msg):
        """Log information about received depth image."""
        try:
            # Log that we've received an image
            self.get_logger().info(f'Received depth image with encoding: {msg.encoding}, size: {msg.width}x{msg.height}')
            
            # Try to convert it
            try:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                self.get_logger().info(f'Successfully converted depth image to numpy array with shape {depth_image.shape}')
                
                # Create a dummy cylinder detection
                self.publish_dummy_cylinder(depth_image)
                
            except Exception as e:
                self.get_logger().error(f'Error converting depth image: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def publish_dummy_cylinder(self, depth_image):
        """Publish a dummy cylinder detection for testing."""
        try:
            # Get the center of the image
            height, width = depth_image.shape[:2]
            center_y, center_x = height // 2, width // 2
            
            # Create a dummy center point
            center_msg = Point()
            center_msg.x = float(center_x)
            center_msg.y = float(center_y)
            center_msg.z = 5.0  # dummy depth
            self.cylinder_center_publisher.publish(center_msg)
            
            # Create dummy cylinder info
            info_msg = Float32MultiArray()
            info_msg.data = [2.0, 10.0, 90.0, 0.9]  # width, height, angle, confidence
            self.cylinder_info_publisher.publish(info_msg)
            
            self.get_logger().info('Published dummy cylinder detection')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dummy cylinder: {str(e)}')
    
    def check_topics(self):
        """Check for available topics."""
        import subprocess
        try:
            # Run 'ros2 topic list' command
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                   capture_output=True, text=True, check=True)
            
            # Log the available topics
            topics = result.stdout.strip().split('\n')
            self.get_logger().info('Available topics:')
            for topic in topics:
                self.get_logger().info(f'  - {topic}')
                
            # Check for depth camera topics specifically
            depth_topics = [t for t in topics if 'depth' in t.lower()]
            if depth_topics:
                self.get_logger().info('Depth camera topics found:')
                for topic in depth_topics:
                    self.get_logger().info(f'  - {topic}')
            else:
                self.get_logger().warn('No depth camera topics found!')
                
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error running ros2 topic list: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error checking topics: {str(e)}')


def main():
    rclpy.init()
    detector = DebugCylinderDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()