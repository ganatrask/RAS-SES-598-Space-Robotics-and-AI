#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleControlMode, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math
import time

class CylinderLandingController(Node):
    """
    ROS2 node that manages a mission to:
    1. Take off to search height
    2. Perform a spiral search pattern to find cylinders
    3. Identify the tallest cylinder
    4. Plan a path to land on top of it
    """
    def __init__(self):
        super().__init__('cylinder_landing_controller')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Mission parameters
        self.declare_parameter('takeoff_height', 12.0)       # meters
        self.declare_parameter('search_height', 12.0)        # meters
        self.declare_parameter('search_speed', 2.0)          # m/s
        self.declare_parameter('position_threshold', 0.5)    # meters
        self.declare_parameter('heading_threshold', 0.1)     # radians
        self.declare_parameter('search_radius', 20.0)        # meters
        self.declare_parameter('min_confidence', 0.7)        # minimum confidence for cylinder detection
        
        # Get parameters
        self.TAKEOFF_HEIGHT = self.get_parameter('takeoff_height').value
        self.SEARCH_HEIGHT = self.get_parameter('search_height').value
        self.SEARCH_SPEED = self.get_parameter('search_speed').value
        self.POSITION_THRESHOLD = self.get_parameter('position_threshold').value
        self.HEADING_THRESHOLD = self.get_parameter('heading_threshold').value
        self.SEARCH_RADIUS = self.get_parameter('search_radius').value
        self.MIN_CONFIDENCE = self.get_parameter('min_confidence').value
        
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
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
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.cylinder_center = None
        self.cylinder_info = None
        self.start_time = time.time()
        
        # Detected cylinders list [x, y, z, height, radius, confidence]
        self.detected_cylinders = []
        
        # State machine
        self.states = {
            'INIT': 0,
            'TAKEOFF': 1,
            'SEARCH': 2,
            'APPROACH': 3,
            'DESCEND': 4,
            'LAND': 5
        }
        self.current_state = self.states['INIT']
        
        # Create timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        self.get_logger().info('Cylinder Landing Controller initialized')
        self.get_logger().info(f'Takeoff height: {self.TAKEOFF_HEIGHT}m')
        self.get_logger().info(f'Search height: {self.SEARCH_HEIGHT}m')

    def vehicle_odometry_callback(self, msg):
        """Store vehicle position from odometry."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Store vehicle status."""
        self.vehicle_status = msg

    def cylinder_center_callback(self, msg):
        """Store cylinder center position."""
        self.cylinder_center = msg

    def cylinder_info_callback(self, msg):
        """Store cylinder information."""
        self.cylinder_info = msg.data  # [width, height, angle, confidence]
        
        # Log cylinder detection
        if self.cylinder_info and self.cylinder_center and self.cylinder_info[3] > self.MIN_CONFIDENCE:
            self.get_logger().info(
                f'Detected cylinder: width={self.cylinder_info[0]:.2f}m, '
                f'height={self.cylinder_info[1]:.2f}m, '
                f'confidence={self.cylinder_info[3]:.2f}'
            )
            
            # Store detected cylinder if confidence is high enough
            if self.cylinder_info[3] > self.MIN_CONFIDENCE:
                cylinder_data = [
                    self.current_position()[0],  # x
                    self.current_position()[1],  # y
                    self.current_position()[2],  # z
                    self.cylinder_info[1],       # height
                    self.cylinder_info[0] / 2,   # radius (half of width)
                    self.cylinder_info[3]        # confidence
                ]
                
                # Check if this cylinder was already detected (within 2m)
                is_new = True
                for cyl in self.detected_cylinders:
                    dist = math.sqrt((cyl[0] - cylinder_data[0])**2 + 
                                     (cyl[1] - cylinder_data[1])**2)
                    if dist < 2.0:
                        is_new = False
                        # Update with better confidence
                        if cylinder_data[5] > cyl[5]:
                            self.get_logger().info('Updating existing cylinder data')
                            cyl = cylinder_data
                        break
                
                if is_new:
                    self.get_logger().info('Adding new cylinder to detected list')
                    self.detected_cylinders.append(cylinder_data)
                    self.get_logger().info(f'Total cylinders detected: {len(self.detected_cylinders)}')

    def arm(self):
        """Send arm command to vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send disarm command to vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
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
            return [
                self.vehicle_odometry.position[0],
                self.vehicle_odometry.position[1],
                self.vehicle_odometry.position[2]
            ]
        except (IndexError, AttributeError):
            return [0.0, 0.0, 0.0]

    def current_height(self):
        """Get current height (negative of NED z-coordinate)."""
        try:
            return -self.vehicle_odometry.position[2]
        except (IndexError, AttributeError):
            return 0.0

    def is_at_height(self, target_height):
        """Check if drone has reached target height within threshold."""
        current_height = self.current_height()
        return abs(current_height - target_height) < self.POSITION_THRESHOLD

    def calculate_spiral_position(self, time_elapsed):
        """Calculate position along spiral trajectory."""
        # Calculate angular position
        spiral_period = 10.0  # seconds per revolution
        angular_speed = 2.0 * math.pi / spiral_period
        angle = angular_speed * time_elapsed
        
        # Spiral outward with time
        radius = min(self.SEARCH_RADIUS, 0.5 * time_elapsed)
        
        # Calculate position
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = -self.SEARCH_HEIGHT  # Negative for NED frame
        
        # Calculate yaw to always point towards center
        yaw = angle + math.pi
        
        return x, y, z, yaw

    def find_tallest_cylinder(self):
        """Find the tallest cylinder from detected cylinders."""
        if not self.detected_cylinders:
            return None
        
        # Sort by height
        sorted_cylinders = sorted(self.detected_cylinders, 
                                 key=lambda cyl: cyl[3], 
                                 reverse=True)
        return sorted_cylinders[0]

    def control_loop(self):
        """Timer callback for control loop."""
        # Initialize offboard control after 10 cycles
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.start_time = time.time()
            self.current_state = self.states['TAKEOFF']
            self.get_logger().info("Vehicle armed and offboard mode enabled")

        # Publish offboard control mode
        self.publish_offboard_control_mode()

        # Execute state machine
        if self.current_state == self.states['INIT']:
            # Wait for initialization
            pass
            
        elif self.current_state == self.states['TAKEOFF']:
            # Take off to search height
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.TAKEOFF_HEIGHT,  # Negative because of NED frame
                yaw=0.0
            )
            
            # Log current height every 10 cycles (approximately 1 second)
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Taking off... Current height: {self.current_height():.2f}m")
            
            # Transition to SEARCH state once target height is reached
            if self.is_at_height(self.TAKEOFF_HEIGHT):
                self.current_state = self.states['SEARCH']
                self.start_time = time.time()
                self.get_logger().info(f"Reached takeoff height of {self.TAKEOFF_HEIGHT}m, starting search pattern")
                
        elif self.current_state == self.states['SEARCH']:
            # Execute spiral search pattern
            time_elapsed = time.time() - self.start_time
            x, y, z, yaw = self.calculate_spiral_position(time_elapsed)
            
            self.publish_trajectory_setpoint(x=x, y=y, z=z, yaw=yaw)
            
            # Log position every 10 cycles
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(
                    f"Searching... Position: ({x:.2f}, {y:.2f}, {-z:.2f}m), "
                    f"Search time: {time_elapsed:.2f}s"
                )
            
            # If we've found cylinders and searched for at least 30 seconds, proceed
            if len(self.detected_cylinders) >= 2 and time_elapsed > 30.0:
                tallest_cylinder = self.find_tallest_cylinder()
                
                if tallest_cylinder:
                    self.get_logger().info(
                        f"Found tallest cylinder at ({tallest_cylinder[0]:.2f}, "
                        f"{tallest_cylinder[1]:.2f}) with height {tallest_cylinder[3]:.2f}m"
                    )
                    self.current_state = self.states['APPROACH']
                    self.start_time = time.time()
            
            # Safety timeout - if search takes too long (e.g., 120 seconds), land
            if time_elapsed > 120.0:
                self.get_logger().warning("Search timeout reached, proceeding with available data")
                tallest_cylinder = self.find_tallest_cylinder()
                
                if tallest_cylinder:
                    self.get_logger().info(
                        f"Best cylinder found at ({tallest_cylinder[0]:.2f}, "
                        f"{tallest_cylinder[1]:.2f}) with height {tallest_cylinder[3]:.2f}m"
                    )
                    self.current_state = self.states['APPROACH']
                    self.start_time = time.time()
                else:
                    self.get_logger().warning("No cylinders found, returning to start position")
                    self.current_state = self.states['LAND']
                
        elif self.current_state == self.states['APPROACH']:
            # Approach tallest cylinder
            tallest_cylinder = self.find_tallest_cylinder()
            
            if tallest_cylinder:
                # Move to position above cylinder
                x = tallest_cylinder[0]
                y = tallest_cylinder[1]
                z = -self.SEARCH_HEIGHT  # Stay at search height during approach
                
                self.publish_trajectory_setpoint(x=x, y=y, z=z, yaw=0.0)
                
                # Log position every 10 cycles
                if self.offboard_setpoint_counter % 10 == 0:
                    current_pos = self.current_position()
                    distance = math.sqrt(
                        (current_pos[0] - x)**2 + 
                        (current_pos[1] - y)**2
                    )
                    self.get_logger().info(
                        f"Approaching cylinder... Distance: {distance:.2f}m, "
                        f"Height: {self.current_height():.2f}m"
                    )
                
                # Check if we've reached the cylinder
                current_pos = self.current_position()
                distance = math.sqrt(
                    (current_pos[0] - x)**2 + 
                    (current_pos[1] - y)**2
                )
                
                if distance < self.POSITION_THRESHOLD:
                    self.get_logger().info("Reached cylinder position, starting descent")
                    self.current_state = self.states['DESCEND']
                    self.start_time = time.time()
            else:
                # No cylinder found, return to start
                self.get_logger().warning("Lost cylinder data, returning to start")
                self.current_state = self.states['LAND']
                
        elif self.current_state == self.states['DESCEND']:
            # Descend to land on top of the cylinder
            tallest_cylinder = self.find_tallest_cylinder()
            
            if tallest_cylinder:
                # Get cylinder position and height
                x = tallest_cylinder[0]
                y = tallest_cylinder[1]
                cylinder_height = tallest_cylinder[3]
                
                # Calculate target height (add small margin)
                target_height = cylinder_height + 0.2  # 20cm above cylinder
                z = -target_height  # Negative for NED frame
                
                self.publish_trajectory_setpoint(x=x, y=y, z=z, yaw=0.0)
                
                # Log position every 10 cycles
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().info(
                        f"Descending to cylinder... Current height: {self.current_height():.2f}m, "
                        f"Target height: {target_height:.2f}m"
                    )
                
                # Check if we've reached the cylinder top
                if abs(self.current_height() - target_height) < self.POSITION_THRESHOLD:
                    self.get_logger().info("Reached cylinder top, landing")
                    self.current_state = self.states['LAND']
                    self.start_time = time.time()
            else:
                # No cylinder found, return to start
                self.get_logger().warning("Lost cylinder data, returning to start")
                self.current_state = self.states['LAND']
                
        elif self.current_state == self.states['LAND']:
            # In this simplified version, we'll just land at current position
            current_pos = self.current_position()
            
            self.publish_trajectory_setpoint(
                x=current_pos[0],
                y=current_pos[1],
                z=0.0,  # Land at ground level or cylinder top
                yaw=0.0
            )
            
            # Log position every 10 cycles
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(
                    f"Landing... Current height: {self.current_height():.2f}m"
                )
            
            # If very close to the ground, disarm
            if self.current_height() < 0.2:
                self.disarm()
                self.get_logger().info("Mission complete, vehicle disarmed")

        # Increment counter
        self.offboard_setpoint_counter += 1


def main():
    rclpy.init()
    controller = CylinderLandingController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
