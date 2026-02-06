#include "mpc_cartesian_planner/trajectory_publisher.h"

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace mpc_cartesian_planner {

TrajectoryPublisher::TrajectoryPublisher(rclcpp::Node& node, const std::string& robotName, bool latch) {
  topic_ = robotName + std::string("_mpc_target");
  auto qos = rclcpp::QoS(1).reliable();
  if (latch) {
    qos.transient_local();
  }
  pub_ = node.create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(topic_, qos);
}

void TrajectoryPublisher::publishEeTarget(const PlannedCartesianTrajectory& traj,
                                         const ocs2::SystemObservation& obs) {
  if (traj.empty()) return;

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
  pub_->publish(msg);
}

}  // namespace mpc_cartesian_planner
