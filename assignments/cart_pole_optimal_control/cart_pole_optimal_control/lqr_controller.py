#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg

import csv
import time
import os
from datetime import datetime
import atexit

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
        
        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
        
        # LQR cost matrices
        self.Q = np.diag([1.0, 1.0, 10.0, 10.0])  # State cost
        self.R = np.array([[0.1]])  # Control cost
        
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        
        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force', 
            10
        )
        
        # Verify publisher created successfully
        if self.cart_cmd_pub:
            self.get_logger().info('Force command publisher created successfully')
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to earthquake force topic if available
        self.earthquake_force = 0.0
        self.earthquake_sub = self.create_subscription(
            Float64,
            '/cart_pole/earthquake_force',
            self.earthquake_force_callback,
            10
        )
        
        # Setup data logging
        self.setup_logging()
        
        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)
        
        # Register shutdown callback
        atexit.register(self.on_shutdown)
        
        self.get_logger().info('Cart-Pole LQR Controller initialized')
    
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
    
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            # Get indices for cart and pole joints
            cart_idx = msg.name.index('cart_to_base')  # Cart position/velocity
            pole_idx = msg.name.index('pole_joint')    # Pole angle/velocity
            
            # State vector: [x, ẋ, θ, θ̇]
            self.x = np.array([
                [msg.position[cart_idx]],     # Cart position (x)
                [msg.velocity[cart_idx]],     # Cart velocity (ẋ)
                [msg.position[pole_idx]],     # Pole angle (θ)
                [msg.velocity[pole_idx]]      # Pole angular velocity (θ̇)
            ])
            
            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={msg.position[cart_idx]:.3f}, cart_vel={msg.velocity[cart_idx]:.3f}, pole_angle={msg.position[pole_idx]:.3f}, pole_vel={msg.velocity[pole_idx]:.3f}')
                self.state_initialized = True
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}, msg={msg.name}')
    
    def earthquake_force_callback(self, msg):
        """Callback to capture the earthquake force."""
        self.earthquake_force = msg.data
    
    def setup_logging(self):
        """Set up data logging and performance metrics tracking."""
        # Create logs directory if it doesn't exist
        log_dir = os.path.join(os.path.expanduser('~'), 'ros', 'drone_ws', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        # Create log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f'lqr_data_{timestamp}.csv')
        
        # Initialize CSV file with headers
        self.log_file = open(log_file, 'w')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['time', 'cart_position', 'cart_velocity', 
                                 'pole_angle', 'pole_angle_deg', 'pole_velocity', 
                                 'control_force', 'earthquake_force'])
        
        # Initialize tracking variables
        self.start_time = time.time()
        self.max_cart_pos = 0.0
        self.max_pole_angle = 0.0
        self.sum_control_force = 0.0
        self.count = 0
        self.last_print_time = 0
        
        self.get_logger().info(f"Logging data to {log_file}")
        return log_file

    def control_loop(self):
        """Compute and apply LQR control."""
        try:
            if not self.state_initialized:
                self.get_logger().warn('State not initialized yet')
                return
            
            # Get current time
            current_time = time.time() - self.start_time

            # Compute control input u = -Kx
            u = -self.K @ self.x
            force = float(u[0])
            
            # Log data to CSV
            self.csv_writer.writerow([
                current_time,                  # Time
                float(self.x[0]),              # Cart position
                float(self.x[1]),              # Cart velocity
                float(self.x[2]),              # Pole angle (radians)
                np.degrees(float(self.x[2])),  # Pole angle (degrees)
                float(self.x[3]),              # Pole angular velocity
                force,                         # Control force applied
                self.earthquake_force          # Earthquake force
            ])
            
            # Update metrics tracking
            self.max_cart_pos = max(self.max_cart_pos, abs(float(self.x[0])))
            self.max_pole_angle = max(self.max_pole_angle, abs(float(self.x[2])))
            self.sum_control_force += abs(force)
            self.count += 1
            
            # Log control input periodically
            if abs(force - self.last_control) > 0.1 or self.control_count % 100 == 0:
                self.get_logger().info(f'State: {self.x.T}, Control force: {force:.3f}N')
            
            # Every 5 seconds, print current metrics
            if int(current_time) % 5 == 0 and int(current_time) != int(self.last_print_time):
                self.get_logger().info(f"Time: {current_time:.2f}s, Max Cart Disp: {self.max_cart_pos:.3f}m, " +
                                     f"Max Pole Angle: {np.degrees(self.max_pole_angle):.2f}°, " +
                                     f"Avg Control: {self.sum_control_force/max(1, self.count):.3f}N")
                self.last_print_time = current_time
            
            # Publish control command
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
            
            self.last_control = force
            self.control_count += 1
            
            # Check for failure conditions (cart out of bounds or pole angle too large)
            if abs(float(self.x[0])) > 2.5:  # Cart position limit
                self.get_logger().warning(f"FAILURE: Cart position {float(self.x[0]):.3f} exceeds limit of ±2.5m")
                self.on_shutdown()  # Record metrics on failure
                
            if abs(np.degrees(float(self.x[2]))) > 45.0:  # Pole angle limit (45 degrees)
                self.get_logger().warning(f"FAILURE: Pole angle {np.degrees(float(self.x[2])):.3f}° exceeds limit of ±45°")
                self.on_shutdown()  # Record metrics on failure
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
    
    def on_shutdown(self):
        """Calculate and print final metrics when shutting down."""
        try:
            # Calculate final metrics
            avg_control_effort = self.sum_control_force / max(1, self.count)
            duration = time.time() - self.start_time
            
            # Calculate stability score (using the formula from student example)
            stability_score = max(0, 10 - (self.max_cart_pos * 2) - 
                                 (np.degrees(self.max_pole_angle)/5) - 
                                 (avg_control_effort/20))
            
            # Print final metrics
            self.get_logger().info("\n=== FINAL PERFORMANCE METRICS ===")
            self.get_logger().info(f"Duration of stable operation: {duration:.2f} s")
            self.get_logger().info(f"Maximum cart displacement: {self.max_cart_pos:.3f} m")
            self.get_logger().info(f"Maximum pendulum angle deviation: {np.degrees(self.max_pole_angle):.3f}°")
            self.get_logger().info(f"Average control effort: {avg_control_effort:.3f} N")
            self.get_logger().info(f"Stability score: {stability_score:.2f}/10")
            
            # Close log file
            if hasattr(self, 'log_file') and self.log_file:
                self.log_file.close()
                self.get_logger().info(f"Data saved to {self.log_file.name}")
                
        except Exception as e:
            self.get_logger().error(f"Error in shutdown: {e}")

def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.on_shutdown()  # Ensure metrics are recorded on Ctrl+C
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
