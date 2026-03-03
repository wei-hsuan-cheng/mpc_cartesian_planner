#pragma once

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace mpc_cartesian_planner {

std::shared_ptr<rclcpp::Node> createTrajectoryTTActionServerNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

}  // namespace mpc_cartesian_planner
