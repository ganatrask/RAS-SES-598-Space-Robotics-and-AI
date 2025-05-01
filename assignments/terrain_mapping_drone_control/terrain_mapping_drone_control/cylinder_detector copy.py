#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from visualization_msgs.msg import Marker, MarkerArray

class CylinderDetector(Node):
    """
    ROS2 node for detecting and measuring cylindrical rock formations from point cloud data.
    """
    def __init__(self):
        super().__init__('cylinder_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False
        
        # Initialize data structures for cylinder tracking
        self.cylinders = []  # List to store detected cylinders
        
        # Parameters
        self.declare_parameter('min_points', 100)
        self.declare_parameter('max_radius', 10.0)
        self.declare_parameter('min_radius', 1.0)
        self.declare_parameter('ransac_threshold', 0.1)
        self.declare_parameter('ransac_iterations', 1000)
        self.declare_parameter('detection_threshold', 0.7)
        
        self.min_points = self.get_parameter('min_points').value
        self.max_radius = self.get_parameter('max_radius').value
        self.min_radius = self.get_parameter('min_radius').value
        self.ransac_threshold = self.get_parameter('ransac_threshold').value
        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        
        # Subscribers
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/drone/front_depth/points',
            self.pointcloud_callback,
            10)
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/drone/front_depth/camera_info',
            self.camera_info_callback,
            10)
            
        self.depth_image_subscriber = self.create_subscription(
            Image,
            '/drone/front_depth',
            self.depth_image_callback,
            10)
        
        # Publishers
        self.cylinder_center_publisher = self.create_publisher(
            Point,
            '/geometry/cylinder_center',
            10)
        
        self.cylinder_info_publisher = self.create_publisher(
            Float32MultiArray,
            '/geometry/cylinder_info',
            10)
            
        self.cylinder_pose_publisher = self.create_publisher(
            PoseStamped,
            '/geometry/cylinder_pose',
            10)
            
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization/cylinders',
            10)
            
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/cylinder_detector/debug_image',
            10)
        
        self.get_logger().info('Cylinder detector node initialized')
    
    def camera_info_callback(self, msg):
        """Process incoming camera calibration data."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera calibration received')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
    
    def depth_image_callback(self, msg):
        """Process depth image to detect cylinders."""
        try:
            # Convert ROS Image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Create a copy for visualization and processing
            depth_display = depth_image.copy()
            
            # Replace NaN values with 0 for visualization
            depth_display[np.isnan(depth_display)] = 0
            
            # Set depth range for visualization (0-10 meters)
            depth_min = 0.0
            depth_max = 10.0
            
            # Clip and normalize the depth values for visualization
            depth_normalized = np.clip(depth_display, depth_min, depth_max)
            depth_normalized = ((depth_normalized - depth_min) * 255 / (depth_max - depth_min))
            depth_normalized = depth_normalized.astype(np.uint8)
            
            # Apply histogram equalization to enhance contrast
            depth_normalized = cv2.equalizeHist(depth_normalized)
            
            # Create debug image (BGR format for visualization)
            debug_image = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
            
            # Edge detection using Canny
            edges = cv2.Canny(blurred, 50, 150)
            
            # Find contours for potential cylinder boundaries
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw all contours in green (light)
            cv2.drawContours(debug_image, contours, -1, (0,100,0), 1)
            
            # Process contours to find cylinder-like shapes
            best_cylinder = None
            best_confidence = 0.0
            
            for contour in contours:
                # Calculate contour properties
                area = cv2.contourArea(contour)
                
                # Filter small contours
                if area < 1000:  # Minimum area threshold
                    continue
                
                # Fit an ellipse to the contour if possible
                if len(contour) >= 5:
                    ellipse = cv2.fitEllipse(contour)
                    (center_x, center_y), (width, height), angle = ellipse
                    
                    # Calculate aspect ratio and confidence score
                    aspect_ratio = min(width, height) / max(width, height)
                    angle_confidence = 1.0 - abs(angle - 90) / 90
                    size_confidence = min(width, height) / 100  # Normalize by expected size
                    confidence = aspect_ratio * angle_confidence * size_confidence
                    
                    # Filter for cylinder-like shapes (nearly vertical ellipses)
                    if aspect_ratio > 0.3 and abs(angle - 90) < 30:
                        # Update best cylinder if confidence is higher
                        if confidence > best_confidence:
                            best_cylinder = (ellipse, confidence)
                            best_confidence = confidence
                        
                        # Draw the ellipse in blue
                        cv2.ellipse(debug_image, ellipse, (255,0,0), 2)
                        
                        # Get depth at center of ellipse
                        center_x, center_y = int(center_x), int(center_y)
                        if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                            center_depth = depth_image[center_y, center_x]
                            if not np.isnan(center_depth):  # Only show valid depth values
                                # Draw crosshair at center
                                cv2.line(debug_image, (center_x-15, center_y), (center_x+15, center_y), (255,255,0), 2)
                                cv2.line(debug_image, (center_x, center_y-15), (center_x, center_y+15), (255,255,0), 2)
                                
                                # Add text for measurements
                                cv2.putText(debug_image, 
                                          f'D:{center_depth:.2f}m AR:{aspect_ratio:.2f} C:{confidence:.2f}', 
                                          (center_x-50, center_y-20), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 
                                          0.5, 
                                          (255,255,0), 
                                          1)
            
            # Publish best cylinder info if found
            if best_cylinder is not None:
                ellipse, confidence = best_cylinder
                (center_x, center_y), (width, height), angle = ellipse
                center_x, center_y = int(center_x), int(center_y)
                
                if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                    center_depth = depth_image[center_y, center_x]
                    if not np.isnan(center_depth):
                        # Publish cylinder center
                        center_msg = Point()
                        center_msg.x = float(center_x)
                        center_msg.y = float(center_y)
                        center_msg.z = float(center_depth)
                        self.cylinder_center_publisher.publish(center_msg)
                        
                        # Estimate real-world dimensions
                        # This is a simplification - actual implementation would use proper camera calibration
                        # to convert from pixels to real-world units based on depth
                        real_width = width * center_depth / self.camera_matrix[0, 0] if self.camera_matrix is not None else width * 0.001 * center_depth
                        real_height = height * center_depth / self.camera_matrix[1, 1] if self.camera_matrix is not None else height * 0.001 * center_depth
                        
                        # Publish cylinder info (width, height, angle, confidence)
                        info_msg = Float32MultiArray()
                        info_msg.data = [float(real_width), float(real_height), float(angle), float(confidence)]
                        self.cylinder_info_publisher.publish(info_msg)
                        
                        self.get_logger().info(
                            f"Detected cylinder: center=({center_x}, {center_y}, {center_depth:.2f}m), "
                            f"dimensions={real_width:.2f}x{real_height:.2f}m, confidence={confidence:.2f}"
                        )
            
            # Draw center crosshair
            height, width = depth_image.shape[:2]
            center_y, center_x = height // 2, width // 2
            cv2.line(debug_image, (center_x-20, center_y), (center_x+20, center_y), (0,255,255), 2)
            cv2.line(debug_image, (center_x, center_y-20), (center_x, center_y+20), (0,255,255), 2)
            
            # Add center depth text
            center_depth = depth_image[center_y, center_x]
            if not np.isnan(center_depth):
                cv2.putText(debug_image, 
                          f'Center: {center_depth:.2f}m', 
                          (10, height-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          1.0, 
                          (0,255,255), 
                          2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_publisher.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')