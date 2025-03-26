cmake_minimum_required(VERSION 3.8)
project(terrain_mapping_drone_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
