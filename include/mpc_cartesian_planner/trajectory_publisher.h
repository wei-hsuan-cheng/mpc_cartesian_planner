#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"

namespace mpc_cartesian_planner {

class TrajectoryPublisher {
 public:
  TrajectoryPublisher(rclcpp::Node& node, const std::string& robotName, bool latch = false);

  // Publish EE target as [px py pz qx qy qz qw]
  void publishEeTarget(const PlannedCartesianTrajectory& traj, const ocs2::SystemObservation& obs);

  const std::string& topic() const { return topic_; }

 private:
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr pub_;
  std::string topic_;
};

}  // namespace mpc_cartesian_planner
