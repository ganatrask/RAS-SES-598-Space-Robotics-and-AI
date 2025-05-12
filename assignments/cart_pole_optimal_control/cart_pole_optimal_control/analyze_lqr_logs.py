#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import argparse
from datetime import datetime

def analyze_lqr_data(csv_file):
    """Analyze LQR controller performance from CSV log file."""
    print(f"Analyzing LQR data from: {csv_file}")
    
    # Read the CSV file
    try:
        data = pd.read_csv(csv_file)
        print(f"Successfully loaded data with {len(data)} rows")
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return
    
    # Check for required columns
    required_cols = ['time', 'cart_position', 'pole_angle_deg', 'control_force']
    missing_cols = [col for col in required_cols if col not in data.columns]
    if missing_cols:
        print(f"Error: Missing required columns: {missing_cols}")
        print(f"Available columns: {data.columns.tolist()}")
        return
    
    # Calculate performance metrics
    max_cart_pos = data['cart_position'].abs().max()
    max_pole_angle = data['pole_angle_deg'].abs().max()
    avg_control_effort = data['control_force'].abs().mean()
    duration = data['time'].max()
    
    # Calculate stability score
    stability_score = max(0, 10 - (max_cart_pos * 2) - (max_pole_angle/5) - (avg_control_effort/20))
    
    # Print performance metrics
    print("\n=== PERFORMANCE METRICS ===")
    print(f"Duration of stable operation: {duration:.2f} s")
    print(f"Maximum cart displacement: {max_cart_pos:.3f} m")
    print(f"Maximum pendulum angle deviation: {max_pole_angle:.3f}°")
    print(f"Average control effort: {avg_control_effort:.3f} N")
    print(f"Stability score: {stability_score:.2f}/10")
    
    # Create output directory for plots
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"logs/lqr_analysis_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)
    
    plt.figure(figsize=(12, 10))
    
    # Plot 1: Cart Position
    plt.subplot(3, 1, 1)
    plt.plot(data['time'], data['cart_position'])
    plt.axhline(y=2.5, color='r', linestyle='--', label='Upper Limit')
    plt.axhline(y=-2.5, color='r', linestyle='--', label='Lower Limit')
    plt.title('Cart Position Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.grid(True)
    plt.legend()
    
    # Plot 2: Pole Angle
    plt.subplot(3, 1, 2)
    plt.plot(data['time'], data['pole_angle_deg'])
    plt.title('Pole Angle Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.grid(True)
    
    # Plot 3: Control Force
    plt.subplot(3, 1, 3)
    plt.plot(data['time'], data['control_force'])
    plt.title('Control Force Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/baseline_performance.png")
    
    # additional plots for earthquake disturbance if available
    if 'earthquake_force' in data.columns:
        plt.figure(figsize=(12, 6))
        plt.plot(data['time'], data['earthquake_force'])
        plt.title('Earthquake Disturbance Force Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.grid(True)
        plt.savefig(f"{output_dir}/earthquake_force.png")
    
    # phase plots
    plt.figure(figsize=(12, 10))
    
    # Phase plot 1: Cart Position vs. Velocity
    plt.subplot(2, 1, 1)
    plt.scatter(data['cart_position'], data['cart_velocity'], c=data['time'], cmap='viridis', s=2)
    plt.colorbar(label='Time (s)')
    plt.title('Cart Phase Space (Position vs. Velocity)')
    plt.xlabel('Position (m)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    
    # Phase plot 2: Pole Angle vs. Angular Velocity
    plt.subplot(2, 1, 2)
    plt.scatter(data['pole_angle_deg'], data['pole_velocity'], c=data['time'], cmap='viridis', s=2)
    plt.colorbar(label='Time (s)')
    plt.title('Pole Phase Space (Angle vs. Angular Velocity)')
    plt.xlabel('Angle (degrees)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/phase_plots.png")
    
    # text file save
    with open(f"{output_dir}/performance_summary.txt", 'w') as f:
        f.write("=== PERFORMANCE METRICS ===\n")
        f.write(f"LQR Controller Parameters: Q = diag([1.0, 1.0, 10.0, 10.0]), R = [0.1]\n")
        f.write(f"Duration of stable operation: {duration:.2f} s\n")
        f.write(f"Maximum cart displacement: {max_cart_pos:.3f} m\n")
        f.write(f"Maximum pendulum angle deviation: {max_pole_angle:.3f}°\n")
        f.write(f"Average control effort: {avg_control_effort:.3f} N\n")
        f.write(f"Stability score: {stability_score:.2f}/10\n")
    
    print(f"\nAnalysis complete. Results saved to {output_dir}/ directory")
    return output_dir, data

def find_latest_log():
    """Find the most recent LQR log file."""
    log_pattern = os.path.expanduser("~/ros/drone_ws/logs/lqr_data_*.csv")
    log_files = glob.glob(log_pattern)
    
    if not log_files:
        print(f"No log files found matching pattern: {log_pattern}")
        return None
    
    # Sort by modification time, newest first
    latest_log = max(log_files, key=os.path.getmtime)
    return latest_log

def main():
    parser = argparse.ArgumentParser(description='Analyze LQR controller logs')
    parser.add_argument('--file', type=str, help='CSV log file to analyze')
    args = parser.parse_args()
    
    # Use provided file or find the latest log
    log_file = args.file if args.file else find_latest_log()
    
    if log_file:
        analyze_lqr_data(log_file)
    else:
        print("No log file specified or found. Please provide a log file path.")

if __name__ == "__main__":
    main()
