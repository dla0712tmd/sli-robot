cmake_minimum_required(VERSION 3.5)
project(dynamixel_mpu9250)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python executables
ament_python_install_package(${PROJECT_NAME})

# Install other files
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  
)

ament_export_dependencies(rclpy dynamixel_sdk sensor_msgs)
ament_package()
