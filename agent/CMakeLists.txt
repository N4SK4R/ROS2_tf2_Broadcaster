cmake_minimum_required(VERSION 3.8)
project(agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(service_test src/test.cpp)
ament_target_dependencies(service_test rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros my_robot_interfaces)

install(
  TARGETS service_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
