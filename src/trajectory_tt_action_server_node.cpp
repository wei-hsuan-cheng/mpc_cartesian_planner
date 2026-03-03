#include "rclcpp/rclcpp.hpp"

#include "mpc_cartesian_planner/trajectory_tt_action_server.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = mpc_cartesian_planner::createTrajectoryTTActionServerNode();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
