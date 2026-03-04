#pragma once

#include <cstddef>
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
  // Publish base target as [x y yaw] or [x y z yaw pitch roll] depending on base_state_dim.
  void publishBaseTarget(const PlannedBaseTrajectory& traj, std::size_t base_state_dim, std::size_t base_input_dim);
  // Publish arm joint target as q_arm.
  void publishJointTarget(const PlannedJointTrajectory& traj);

  // Backward-compatible alias for legacy EE-only call sites.
  const std::string& topic() const { return ee_topic_; }
  const std::string& eeTopic() const { return ee_topic_; }
  const std::string& baseTopic() const { return base_topic_; }
  const std::string& jointTopic() const { return joint_topic_; }

 private:
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr ee_pub_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr base_pub_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr joint_pub_;
  std::string ee_topic_;
  std::string base_topic_;
  std::string joint_topic_;
};

}  // namespace mpc_cartesian_planner
