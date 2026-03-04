cmake_minimum_required(VERSION 3.10)
project(mpc_cartesian_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development Development.Module REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(mobile_manipulator_mpc REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Humble's rosidl_generator_py still reaches the legacy FindPythonInterp path
# through python_cmake_module. Export the resolved Python3 interpreter so
# PYTHON_EXECUTABLE is defined during rosidl interface generation.
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH "Python interpreter for rosidl generation" FORCE)

# Some Humble/aarch64 environments expose Python3_SOABI with a leading dot
# (e.g. ".cpython-310-aarch64-linux-gnu"). rosidl_generator_py then appends
# its own dot and produces broken module names like "..cpython-...so".
if(DEFINED Python3_SOABI)
  string(REGEX REPLACE "^\\.+"
    ""
    Python3_SOABI
    "${Python3_SOABI}")
  set(PYTHON_SOABI "${Python3_SOABI}" CACHE STRING "Python SOABI for rosidl generation" FORCE)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteScrewMove.action"
  "action/ExecuteLinearMove.action"
  "action/ExecuteTargetPose.action"
  "action/ExecuteFigureEight.action"
  "action/ExecuteCombinedScrewMove.action"
  "action/ExecuteCombinedLinearMove.action"
  "action/ExecuteCombinedTargetPose.action"
  DEPENDENCIES action_msgs
)

add_library(mpc_cartesian_planner_lib
  src/cartesian_trajectory_planner.cpp
  src/trajectory_publisher.cpp
  src/trajectory_monitor.cpp
)
ament_target_dependencies(mpc_cartesian_planner_lib
  rclcpp rclcpp_action std_msgs std_srvs controller_manager_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  mobile_manipulator_mpc ocs2_pinocchio_interface
  pinocchio Eigen3
)

add_executable(trajectory_tt_publisher_node src/trajectory_tt_publisher_node.cpp)
target_link_libraries(trajectory_tt_publisher_node mpc_cartesian_planner_lib)
ament_target_dependencies(trajectory_tt_publisher_node
  rclcpp rclcpp_action std_msgs std_srvs controller_manager_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  mobile_manipulator_mpc ocs2_pinocchio_interface
  pinocchio Eigen3
)

add_executable(trajectory_progress_monitor_node src/trajectory_progress_monitor_node.cpp)
target_link_libraries(trajectory_progress_monitor_node mpc_cartesian_planner_lib)
ament_target_dependencies(trajectory_progress_monitor_node
  rclcpp rclcpp_action std_msgs std_srvs controller_manager_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  mobile_manipulator_mpc ocs2_pinocchio_interface
  pinocchio Eigen3
)

add_executable(trajectory_tt_action_server_node
  src/trajectory_tt_action_server.cpp
  src/trajectory_tt_action_server_support.cpp
  src/trajectory_tt_action_server_node.cpp
)
target_link_libraries(trajectory_tt_action_server_node mpc_cartesian_planner_lib)
ament_target_dependencies(trajectory_tt_action_server_node
  rclcpp rclcpp_action std_msgs std_srvs controller_manager_msgs geometry_msgs visualization_msgs tf2_ros
  ocs2_core ocs2_mpc ocs2_msgs ocs2_ros_interfaces
  mobile_manipulator_mpc ocs2_pinocchio_interface
  pinocchio Eigen3
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(trajectory_tt_action_server_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(trajectory_tt_action_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(TARGETS 
  mpc_cartesian_planner_lib 
  trajectory_tt_publisher_node 
  trajectory_progress_monitor_node
  trajectory_tt_action_server_node
  
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
