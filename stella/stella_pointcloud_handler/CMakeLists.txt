################################################################################
# Set minimum required version of cmake, project name
################################################################################
cmake_minimum_required(VERSION 3.8)
project(stella_pointcloud_handler)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_ros REQUIRED)

################################################################################
# Build
################################################################################
add_executable(pointcloud_to_laserscan_tf_node src/pointcloud_to_laserscan_tf_node.cpp)
add_executable(pointcloud_passthrough_filter src/pointcloud_passthrough_filter.cpp)

ament_target_dependencies(pointcloud_to_laserscan_tf_node rclcpp sensor_msgs tf2 tf2_ros pcl_conversions pcl_ros)
ament_target_dependencies(pointcloud_passthrough_filter rclcpp sensor_msgs pcl_conversions pcl_ros)

################################################################################
# Install
################################################################################
install(TARGETS
  pointcloud_to_laserscan_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_passthrough_filter
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
