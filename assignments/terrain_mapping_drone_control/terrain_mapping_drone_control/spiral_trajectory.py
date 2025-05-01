#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class SpiralTrajectory(Node):
    """
    A ROS 2 node for executing a spiral search trajectory with PX4.
    The drone starts from a specified height and performs a spiral pattern
    to search for cylinders.
    """
    def __init__(self):
        super().__init__('spiral_trajectory')

        # Declare parameters
        self.declare_parameter('initial_height', 12.0)      # meters
        self.declare_parameter('spiral_diameter', 20.0)     # meters
        self.declare_parameter('descent_rate', 0.0)         # m/s - set to 0 to maintain height
        self.declare_parameter('spiral_period', 15.0)       # seconds per revolution
        self.declare_parameter('min_height', 5.0)           # minimum height
        self.declare_parameter('height_threshold', 0.3)     # meters
        
        # Load parameters
        self.INITIAL_HEIGHT = self.get_parameter('initial_height').value
        self.SPIRAL_DIAMETER = self.get_parameter('spiral_diameter').value
        self.DESCENT_RATE = self.get_parameter('descent_rate').value
        self.SPIRAL_PERIOD = self.get_parameter('spiral_period').value
        self.MIN_HEIGHT = self.get_parameter('min_height').value
        self.HEIGHT_REACHED_THRESHOLD = self.get_parameter('height_threshold').value
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
            
        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
            
        # Subscribe to cylinder detection
        self.cylinder_center_subscriber = self.create_subscription(
            Point, '/geometry/cylinder_center',
            self.cylinder_center_callback, 10)
        self.cylinder_info_subscriber = self.create_subscription(
            Float32MultiArray, '/geometry/cylinder_info',
            self.cylinder_info_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = None
        self.vehicle_status = None
        self.start_time = time.time()
        self.arm_time = None
        self.cylinders_detected = []
        self.last_spiral_expansion = 0.0
        
        # State machine
        self.state = "INIT"  # States: INIT, TAKEOFF, SEARCH, APPROACH, LAND
        
        # Create a timer to publish control commands
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        # Log parameters
        self.get_logger().info(f"Starting spiral trajectory with initial height: {self.INITIAL_HEIGHT}m")
        self.get_logger().info(f"Spiral diameter: {self.SPIRAL_DIAMETER}m, period: {self.SPIRAL_PERIOD}s")

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry data."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle status data."""
        self.vehicle_status = msg
    
    def cylinder_center_callback(self, msg):
        """Store cylinder center from detector."""
        # Store position for processing
        self.get_logger().debug(f"Cylinder detected at pixel: ({msg.x:.1f}, {msg.y:.1f}) depth {msg.z:.2f}m")
    
    def cylinder_info_callback(self, msg):
        """Process cylinder dimensions from detector."""
        if len(msg.data) >= 4:
            # Extract data: [width, height, angle, confidence]
            width = msg.data[0]
            height = msg.data[1]
            confidence = msg.data[3]
            
            if confidence > 0.7:  # Only consider high confidence detections
                self.get_logger().info(f"Detected cylinder: width={width:.2f}m, height={height:.2f}m, confidence={confidence:.2f}")
                
                # Get current position
                if self.vehicle_odometry:
                    pos = self.current_position()
                    self.cylinders_detected.append({
                        'width': width,
                        'height': height,
                        'confidence': confidence,
                        'drone_pos': pos
                    })
                    self.get_logger().info(f"Recorded cylinder at drone position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.arm_time = time.time()
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def current_position(self):
        """Get current position in NED frame."""
        try:
            if self.vehicle_odometry:
                return [
                    float(self.vehicle_odometry.position[0]),
                    float(self.vehicle_odometry.position[1]),
                    float(self.vehicle_odometry.position[2])
                ]
            return [0.0, 0.0, 0.0]
        except (IndexError, AttributeError, TypeError):
            return [0.0, 0.0, 0.0]

    def current_height(self):
        """Get current height (negative of NED z-coordinate)."""
        try:
            if self.vehicle_odometry:
                return -float(self.vehicle_odometry.position[2])
            return 0.0
        except (IndexError, AttributeError, TypeError):
            return 0.0

    def is_at_target_height(self):
        """Check if the drone has reached the target height."""
        return abs(self.current_height() - self.INITIAL_HEIGHT) < self.HEIGHT_REACHED_THRESHOLD

    def calculate_spiral_position(self, time_elapsed):
        """Calculate position along spiral trajectory."""
        # Calculate angular position
        angle = (2 * math.pi * time_elapsed) / self.SPIRAL_PERIOD
        
        # Gradually increase radius up to spiral diameter
        max_radius = self.SPIRAL_DIAMETER / 2.0
        # Allow up to 20 seconds to reach full radius
        radius = min(max_radius, max_radius * (time_elapsed / 20.0))
        
        # Calculate position
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Height calculation - maintain height unless descent requested
        if self.DESCENT_RATE > 0:
            # Descend gradually if descent rate is set
            current_z = -(self.INITIAL_HEIGHT - self.DESCENT_RATE * time_elapsed)
            # Stop descent at minimum height
            z = max(current_z, -self.MIN_HEIGHT)
        else:
            # Maintain initial height
            z = -self.INITIAL_HEIGHT
        
        # Calculate yaw to always point towards the center
        yaw = angle + math.pi
        
        return x, y, z, yaw
    
    def check_progress(self):
        """Check if takeoff is progressing."""
        if self.arm_time and self.state == "TAKEOFF":
            elapsed = time.time() - self.arm_time
            if elapsed > 5.0 and self.current_height() < 1.0:
                self.get_logger().warning("Takeoff not progressing, retrying arm and offboard commands")
                self.arm()
                self.engage_offboard_mode()

    def control_loop(self):
        """Timer callback for control loop."""
        # Check if odometry data is available
        if not self.vehicle_odometry and self.offboard_setpoint_counter > 20:
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().warning("Waiting for vehicle odometry data...")
            self.offboard_setpoint_counter += 1
            return
            
        # Publish offboard control mode
        self.publish_offboard_control_mode()
        
        # Initialize after 10 cycles
        if self.offboard_setpoint_counter == 10:
            # Send initial setpoint
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)
            
            # Start the takeoff sequence
            self.engage_offboard_mode()
            self.arm()
            self.state = "TAKEOFF"
            self.start_time = time.time()
            self.arm_time = time.time()
            self.get_logger().info("Vehicle armed and offboard mode enabled")
            
        # Execute state machine
        if self.state == "INIT":
            # Waiting for initialization
            pass
            
        elif self.state == "TAKEOFF":
            # Take off to target height
            self.publish_trajectory_setpoint(
                x=0.0, 
                y=0.0,
                z=-self.INITIAL_HEIGHT,
                yaw=0.0
            )
            
            # Log height every second (10 cycles at 10Hz)
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Taking off... Current height: {self.current_height():.2f}m")
                
                # Periodically check progress and retry if stuck
                self.check_progress()
            
            # Transition to SEARCH once we reach target height
            if self.is_at_target_height():
                self.get_logger().info(f"Reached target height of {self.INITIAL_HEIGHT}m, starting search pattern")
                self.state = "SEARCH"
                self.start_time = time.time()
                
        elif self.state == "SEARCH":
            # Execute spiral search pattern
            time_elapsed = time.time() - self.start_time
            x, y, z, yaw = self.calculate_spiral_position(time_elapsed)
            
            # Send position command
            self.publish_trajectory_setpoint(x=x, y=y, z=z, yaw=yaw)
            
            # Log position every 2 seconds
            if self.offboard_setpoint_counter % 20 == 0:
                self.get_logger().info(
                    f"Searching... Position: ({x:.2f}, {y:.2f}, {-z:.2f}m), "
                    f"Search time: {time_elapsed:.2f}s, "
                    f"Cylinders found: {len(self.cylinders_detected)}"
                )
            
            # Check for cylinders found - if we've found enough, start approach
            if len(self.cylinders_detected) >= 2:
                self.get_logger().info(f"Found {len(self.cylinders_detected)} cylinders, preparing to approach tallest")
                
                # Find tallest cylinder
                tallest = None
                max_height = 0.0
                
                for cyl in self.cylinders_detected:
                    if cyl['height'] > max_height:
                        max_height = cyl['height']
                        tallest = cyl
                
                if tallest:
                    self.get_logger().info(f"Tallest cylinder has height {tallest['height']:.2f}m")
                    self.target_cylinder = tallest
                    self.state = "APPROACH"
                    self.start_time = time.time()
                    
            # Check for time limit - after 3 minutes, land even if no cylinders found
            if time_elapsed > 180.0:
                self.get_logger().warning("Search time limit reached, landing")
                self.state = "LAND"
            
        elif self.state == "APPROACH":
            # In a full implementation, this would approach the cylinder
            # For now, let's just land
            self.get_logger().info("Approach state not implemented, landing at current position")
            self.state = "LAND"
            
        elif self.state == "LAND":
            # Land the drone
            pos = self.current_position()
            
            self.publish_trajectory_setpoint(
                x=pos[0],  # Land at current horizontal position
                y=pos[1],
                z=0.0,  # Land at ground level
                yaw=0.0
            )
            
            # Log height every second
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Landing... Current height: {self.current_height():.2f}m")
                
            # Disarm once landed
            if self.current_height() < 0.2:
                self.disarm()
                self.get_logger().info("Mission complete, vehicle disarmed")

        # Increment counter
        self.offboard_setpoint_counter += 1


def main():
    rclpy.init()
    node = SpiralTrajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()