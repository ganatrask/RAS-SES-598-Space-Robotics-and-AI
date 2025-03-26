from setuptools import setup

package_name = 'terrain_mapping_drone_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
            'launch/cylinder_landing.launch.py',
            'launch/rtabmap.launch.py',
            'launch/terrain_mapping.launch.py',
        ]),

        ('share/' + package_name + '/models', []),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shyam Ganatra',
    maintainer_email='ganatras@asu.edu',
    description='A ROS2 package for controlling PX4 drone in spiral trajectory',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiral_trajectory = terrain_mapping_drone_control.spiral_trajectory:main',
            'pose_visualizer = terrain_mapping_drone_control.pose_visualizer:main',
            'aruco_tracker = terrain_mapping_drone_control.aruco_tracker:main',
            'geometry_tracker = terrain_mapping_drone_control.geometry_tracker:main',
            'feature_tracker = terrain_mapping_drone_control.feature_tracker:main',
            'cylinder_landing_node = terrain_mapping_drone_control.cylinder_landing_node:main',
            'cylinder_detector_node = terrain_mapping_drone_control.cylinder_detector_node:main',
        ],
    },
)
