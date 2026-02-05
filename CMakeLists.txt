cmake_minimum_required(VERSION 3.10)
project(mpc_cartesian_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mpc_cartesian_planner_lib
  src/cartesian_path_planner.cpp
  src/trajectory_publisher.cpp
  src/trajectory_monitor.cpp
)
ament_target_dependencies(mpc_cartesian_planner_lib
  rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  ocs2_mobile_manipulator ocs2_pinocchio_interface
  pinocchio Eigen3
)

add_executable(trajectory_tt_publisher_node src/trajectory_tt_publisher_node.cpp)
target_link_libraries(trajectory_tt_publisher_node mpc_cartesian_planner_lib)
ament_target_dependencies(trajectory_tt_publisher_node
  rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  ocs2_mobile_manipulator ocs2_pinocchio_interface
  pinocchio Eigen3
)

add_executable(trajectory_progress_monitor_node src/trajectory_progress_monitor_node.cpp)
target_link_libraries(trajectory_progress_monitor_node mpc_cartesian_planner_lib)
ament_target_dependencies(trajectory_progress_monitor_node
  rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  ocs2_mobile_manipulator ocs2_pinocchio_interface
  pinocchio Eigen3
)

install(TARGETS 
  mpc_cartesian_planner_lib 
  trajectory_tt_publisher_node 
  trajectory_progress_monitor_node
  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
