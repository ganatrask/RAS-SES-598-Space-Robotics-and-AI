#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time

class SimpleOffboardController(Node):
    def __init__(self):
        super().__init__('simple_offboard_controller')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for controlling the drone
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
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = None
        self.vehicle_status = None
        self.arm_time = None
        
        # Define target altitude
        self.target_altitude = 5.0  # meters
        
        # Create main control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        # Create a timer for arming retry
        self.arm_retry_timer = self.create_timer(5.0, self.check_progress)  # 5 seconds timeout
        
        self.get_logger().info('Simple Offboard Controller initialized')
        self.get_logger().info(f'Target altitude: {self.target_altitude} meters')
    
    def vehicle_odometry_callback(self, msg):
        """Store the vehicle odometry data."""
        self.vehicle_odometry = msg
        # Print altitude occasionally
        if self.offboard_setpoint_counter % 10 == 0 and hasattr(self, 'vehicle_odometry') and self.vehicle_odometry is not None:
            try:
                altitude = -float(msg.position[2])  # NED frame, so -z is altitude
                self.get_logger().info(f'Current altitude: {altitude:.2f} meters')
            except Exception as e:
                self.get_logger().error(f'Error getting altitude: {str(e)}')
    
    def vehicle_status_callback(self, msg):
        """Store the vehicle status data."""
        self.vehicle_status = msg
    
    def arm(self):
        """Send arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.arm_time = time.time()
        self.get_logger().info('Arm command sent')
    
    def disarm(self):
        """Send disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
    
    def engage_offboard_mode(self):
        """Send command to switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')
    
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
    
    def current_altitude(self):
        """Get the current altitude of the drone."""
        if self.vehicle_odometry is not None:
            try:
                # NED frame, so altitude is -z
                return -float(self.vehicle_odometry.position[2])
            except (IndexError, TypeError, AttributeError) as e:
                self.get_logger().error(f'Error calculating altitude: {str(e)}')
                return 0.0
        return 0.0
    
    def check_progress(self):
        """Check if the drone is making progress in taking off."""
        if self.arm_time is not None:
            elapsed = time.time() - self.arm_time
            altitude = self.current_altitude()
            
            if elapsed > 5.0 and altitude < 0.5:  # If 5 seconds passed and altitude is still low
                self.get_logger().warning('Takeoff not progressing, retrying arm and offboard commands')
                self.arm()
                self.engage_offboard_mode()
                
                # Ensure we're commanding the drone to takeoff
                self.publish_trajectory_setpoint(0.0, 0.0, -self.target_altitude, 0.0)
    
    def control_loop(self):
        """Main control loop executed at 10Hz."""
        # Publish the offboard control mode
        self.publish_offboard_control_mode()
        
        # Counter for proper initialization sequence
        if self.offboard_setpoint_counter == 10:
            # Send offboard and arm commands
            self.engage_offboard_mode()
            self.arm()
            
            self.get_logger().info('Vehicle armed and offboard mode enabled')
        
        # Command the drone to fly to the target altitude
        # Negative Z in the NED frame is up
        self.publish_trajectory_setpoint(
            x=0.0,
            y=0.0,
            z=-self.target_altitude,
            yaw=0.0
        )
        
        if self.offboard_setpoint_counter % 50 == 0:  # Log every 5 seconds
            # Check progress
            altitude = self.current_altitude()
            
            # Send a debug log showing our commands
            self.get_logger().info(f'Commanding takeoff to altitude: {self.target_altitude}m, Current altitude: {altitude:.2f}m')
            
            # If we're close to the target altitude, consider the takeoff complete
            if abs(altitude - self.target_altitude) < 0.5:
                self.get_logger().info(f'Reached target altitude of {self.target_altitude}m')
        
        # Increment the counter
        self.offboard_setpoint_counter += 1


def main():
    rclpy.init()
    controller = SimpleOffboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()