cmake_minimum_required(VERSION 3.8)
project(elevator_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Add the plugin library
add_library(elevator_control_plugin SHARED src/elevator_control_plugin.cpp)

ament_target_dependencies(elevator_control_plugin
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)

install(TARGETS elevator_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()