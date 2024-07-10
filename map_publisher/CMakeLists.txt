cmake_minimum_required(VERSION 3.8)
project(map_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Add include directories
include_directories(include)

# Create an executable
add_executable(map_publisher_node src/map_publisher_node.cpp)

# Link dependencies to the executable
ament_target_dependencies(map_publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  my_robot_interfaces
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the executable
install(
  TARGETS map_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Create an Ament package
ament_package()
