cmake_minimum_required(VERSION 3.8)
project(PathPlanners)

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

include_directories(include)

add_executable(planner_test src/test_planner.cpp)
ament_target_dependencies(planner_test rclcpp std_msgs geometry_msgs visualization_msgs my_robot_interfaces)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS planner_test
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
