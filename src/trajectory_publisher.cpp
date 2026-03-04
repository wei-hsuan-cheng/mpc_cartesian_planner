#include "mpc_cartesian_planner/trajectory_publisher.h"

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace mpc_cartesian_planner {

TrajectoryPublisher::TrajectoryPublisher(rclcpp::Node& node, const std::string& robotName, bool latch) {
  ee_topic_ = robotName + std::string("_mpc_target");
  base_topic_ = robotName + std::string("_base_mpc_target");
  joint_topic_ = robotName + std::string("_joint_mpc_target");
  auto qos = rclcpp::QoS(1).reliable();
  if (latch) {
    qos.transient_local();
  }
  ee_pub_ = node.create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(ee_topic_, qos);
  base_pub_ = node.create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(base_topic_, qos);
  joint_pub_ = node.create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(joint_topic_, qos);
}

void TrajectoryPublisher::publishEeTarget(const PlannedCartesianTrajectory& traj,
                                         const ocs2::SystemObservation& obs) {
  if (traj.empty() || !ee_pub_) return;

  ocs2::scalar_array_t timeTraj;
  ocs2::vector_array_t stateTraj;
  ocs2::vector_array_t inputTraj;

  timeTraj.reserve(traj.size());
  stateTraj.reserve(traj.size());

  for (std::size_t i = 0; i < traj.size(); ++i) {
    const auto& p = traj.position[i];
    const auto& q = traj.quat[i].normalized();

    ocs2::vector_t x(7);
    x << p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w();  // [p, qxyzw]

    timeTraj.push_back(traj.time[i]);
    stateTraj.push_back(std::move(x));
  }

  inputTraj.resize(timeTraj.size(), ocs2::vector_t::Zero(obs.input.size()));

  ocs2::TargetTrajectories ocs2_traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
  const auto msg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(ocs2_traj);
  ee_pub_->publish(msg);
}

void TrajectoryPublisher::publishBaseTarget(const PlannedBaseTrajectory& traj,
                                            std::size_t base_state_dim,
                                            std::size_t base_input_dim) {
  if (traj.empty() || !base_pub_) return;
  if (base_state_dim != 3 && base_state_dim != 6) return;

  ocs2::scalar_array_t timeTraj;
  ocs2::vector_array_t stateTraj;
  ocs2::vector_array_t inputTraj;

  timeTraj.reserve(traj.size());
  stateTraj.reserve(traj.size());

  for (std::size_t i = 0; i < traj.size(); ++i) {
    ocs2::vector_t x(static_cast<Eigen::Index>(base_state_dim));
    if (base_state_dim == 3) {
      x << traj.position[i].x(), traj.position[i].y(), traj.yaw[i];
    } else {
      x << traj.position[i].x(), traj.position[i].y(), 0.0, traj.yaw[i], 0.0, 0.0;
    }

    timeTraj.push_back(traj.time[i]);
    stateTraj.push_back(std::move(x));
  }

  inputTraj.resize(timeTraj.size(), ocs2::vector_t::Zero(static_cast<Eigen::Index>(base_input_dim)));

  ocs2::TargetTrajectories ocs2_traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
  const auto msg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(ocs2_traj);
  base_pub_->publish(msg);
}

void TrajectoryPublisher::publishJointTarget(const PlannedJointTrajectory& traj) {
  if (traj.empty() || !joint_pub_) return;
  if (traj.time.size() != traj.position.size()) return;

  ocs2::scalar_array_t timeTraj;
  ocs2::vector_array_t stateTraj;
  ocs2::vector_array_t inputTraj;

  timeTraj.reserve(traj.size());
  stateTraj.reserve(traj.size());

  for (std::size_t i = 0; i < traj.size(); ++i) {
    timeTraj.push_back(traj.time[i]);
    stateTraj.push_back(traj.position[i]);
  }

  inputTraj.resize(timeTraj.size(), ocs2::vector_t::Zero(0));

  ocs2::TargetTrajectories ocs2_traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
  const auto msg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(ocs2_traj);
  joint_pub_->publish(msg);
}

}  // namespace mpc_cartesian_planner
