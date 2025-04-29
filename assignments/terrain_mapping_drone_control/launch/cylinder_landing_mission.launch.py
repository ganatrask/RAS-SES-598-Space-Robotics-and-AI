#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch description for the full cylinder landing mission.
    This includes:
    - The cylinder landing simulator environment
    - Cylinder detector node
    - Cylinder landing controller node
    - RViz for visualization
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Include the base cylinder landing simulation launch file
    cylinder_landing_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'cylinder_landing.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'px4_autopilot_path': px4_autopilot_path
        }.items()
    )
    
    # Cylinder detector node
    cylinder_detector_node = Node(
        package='terrain_mapping_drone_control',
        executable='cylinder_detector.py',
        name='cylinder_detector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_points': 100,
            'max_radius': 10.0,
            'min_radius': 1.0,
            'ransac_threshold': 0.1,
            'ransac_iterations': 1000,
            'detection_threshold': 0.7
        }],
        output='screen'
    )
    
    # Cylinder landing controller node
    cylinder_landing_controller_node = Node(
        package='terrain_mapping_drone_control',
        executable='cylinder_landing_controller.py',
        name='cylinder_landing_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'takeoff_height': 15.0,       # meters
            'search_height': 15.0,        # meters
            'search_speed': 2.0,          # m/s
            'position_threshold': 0.5,    # meters
            'heading_threshold': 0.1,     # radians
            'search_radius': 20.0,        # meters
            'min_confidence': 0.7         # minimum confidence for cylinder detection
        }],
        output='screen'
    )
    
    # Pose visualizer for the drone
    pose_visualizer_node = Node(
        package='terrain_mapping_drone_control',
        executable='pose_visualizer.py',
        name='pose_visualizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Define all launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'config', 'drone_viz.rviz'),
            description='Path to RViz config file')
    ]
    
    # Return the LaunchDescription object
    return LaunchDescription(
        launch_args + [
            cylinder_landing_sim,
            TimerAction(
                period=5.0,  # Wait for simulation to start
                actions=[cylinder_detector_node]
            ),
            TimerAction(
                period=6.0,  # Starts after detector
                actions=[pose_visualizer_node]
            ),
            TimerAction(
                period=7.0,  # Starts after pose visualizer
                actions=[cylinder_landing_controller_node]
            ),
            TimerAction(
                period=8.0,  # Start RViz last
                actions=[rviz_node]
            )
        ]
    )