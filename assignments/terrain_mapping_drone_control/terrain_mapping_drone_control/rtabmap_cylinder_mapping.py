#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
import subprocess
import os
import sys
import time
from datetime import datetime

class RTABMapCylinderReconstruction(Node):
    """
    Node to manage RTAB-Map 3D reconstruction of the cylinders.
    This node:
    1. Synchronizes camera and odometry data
    2. Provides a service to export the 3D mesh when the mission is complete
    3. Manages the RTAB-Map database and visualization
    """
    def __init__(self):
        super().__init__('rtabmap_cylinder_reconstruction')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('database_path', 'cylinder_reconstruction.db')
        self.declare_parameter('mesh_export_path', 'cylinder_mesh.ply')
        self.declare_parameter('mapping_enabled', True)
        
        self.database_path = self.get_parameter('database_path').value
        self.mesh_export_path = self.get_parameter('mesh_export_path').value
        self.mapping_enabled = self.get_parameter('mapping_enabled').value
        
        # Create database directory if it doesn't exist
        db_dir = os.path.dirname(self.database_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir)
        
        # State variables
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.last_odom_pose