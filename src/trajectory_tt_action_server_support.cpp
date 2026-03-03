#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include "trajectory_tt_action_server_impl.h"

using ocs2::SystemObservation;

namespace mpc_cartesian_planner {

using trajectory_tt_action_server_internal::computeEePose;
using trajectory_tt_action_server_internal::has_member_activate_asap;
using trajectory_tt_action_server_internal::has_member_activate_controllers;
using trajectory_tt_action_server_internal::has_member_deactivate_controllers;
using trajectory_tt_action_server_internal::has_member_start_asap;
using trajectory_tt_action_server_internal::has_member_start_controllers;
using trajectory_tt_action_server_internal::has_member_stop_controllers;
using trajectory_tt_action_server_internal::parseTimeScaling;
using trajectory_tt_action_server_internal::switchControllerStrictness;

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteScrewMove::Goal& goal, std::string& reason) const {
  auto finite3 = [](const std::array<double, 3>& values) {
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
  };

  if (!std::isfinite(goal.duration) || goal.duration <= 0.0) {
    reason = "duration must be > 0.";
    return false;
  }
  if (!std::isfinite(goal.dt) || goal.dt <= 0.0) {
    reason = "dt must be > 0.";
    return false;
  }
  if (!std::isfinite(goal.screw_theta)) {
    reason = "screw_theta must be finite.";
    return false;
  }
  if (!finite3(goal.screw_uhat) || !finite3(goal.screw_r)) {
    reason = "screw vectors must be finite.";
    return false;
  }

  const Eigen::Vector3d u_hat(goal.screw_uhat[0], goal.screw_uhat[1], goal.screw_uhat[2]);
  if (u_hat.norm() < 1e-9) {
    reason = "screw_uhat must be non-zero.";
    return false;
  }
  return true;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteScrewMove::Goal& goal) const {
  GoalConfig cfg;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.screw_uhat = Eigen::Vector3d(goal.screw_uhat[0], goal.screw_uhat[1], goal.screw_uhat[2]);
  cfg.screw_r = Eigen::Vector3d(goal.screw_r[0], goal.screw_r[1], goal.screw_r[2]);
  cfg.screw_theta = goal.screw_theta;
  cfg.screw_in_tool_frame = goal.screw_in_tool_frame;
  return cfg;
}

bool TrajectoryTTActionServerNode::initializeGoalIfNeeded(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle,
                                                          const GoalConfig& goal_cfg,
                                                          const SystemObservation& obs) {
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_ || activeGoal_ != goal_handle) return false;
    if (goalInitialized_) return true;
  }

  Eigen::Vector3d center_p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  if (interface_ && pin_ && mapping_) {
    std::lock_guard<std::mutex> lock(pinMtx_);
    if (!computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, center_p, q0)) {
      RCLCPP_WARN(this->get_logger(), "Initial FK failed for screw move goal. Using origin pose.");
      center_p.setZero();
      q0 = Eigen::Quaterniond::Identity();
    }
  }

  ScrewMoveParams params;
  params.horizon_T = goal_cfg.duration;
  params.dt = goal_cfg.dt;
  params.u_hat = goal_cfg.screw_uhat;
  params.r = goal_cfg.screw_r;
  params.theta = goal_cfg.screw_theta;
  params.expressed_in_body_frame = goal_cfg.screw_in_tool_frame;
  params.time_scaling = goal_cfg.time_scaling;

  auto planner = std::make_unique<ScrewMovePlanner>(center_p, q0, params);
  planner->setStartTime(obs.time);
  PlannedCartesianTrajectory nominal_traj = planner->plan(obs);
  if (nominal_traj.empty()) {
    finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "Planner returned an empty trajectory.");
    return false;
  }

  if (publishModeScheduleOnGoal_) {
    publishDefaultModeSchedule("GOAL_START");
  }
  if (activateMpcControllerOnGoal_) {
    requestSwitchMpcController(/*activate=*/true, "GOAL_START");
  }

  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_ || activeGoal_ != goal_handle) return false;
    startTime_ = obs.time;
    centerP_ = center_p;
    q0_ = q0;
    planner_ = std::move(planner);
    nominalTraj_ = std::move(nominal_traj);
    haveNominalTraj_ = true;
    goalInitialized_ = true;
    haveLastProjectionIndex_ = false;
    lastProjectionIndex_ = 0;
    warnedMissingFk_ = false;
  }

  RCLCPP_INFO(this->get_logger(), "Initialized screw move goal at t=%.3f", obs.time);
  return true;
}

void TrajectoryTTActionServerNode::onTrackingStatus(const std::string& status) {
  std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle;
  bool goal_started = false;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    monitorSnapshot_.status = status;
    if (!activeGoal_) return;
    goal_handle = activeGoal_;
    goal_started = goalPublishingStarted_;
  }
  if (!goal_started) return;

  if (status == "FINISHED") {
    if (interveneHoldOnDivergedOrFinished_) {
      publishHoldTrajectory("FINISHED");
    }
    startZeroBaseCmdBurst("FINISHED");
    requestSwitchMpcController(/*activate=*/false, "FINISHED");
    finishGoal(goal_handle, GoalTermination::Succeeded, "FINISHED", "Screw move finished.");
  } else if (status == "DIVERGED") {
    if (interveneHoldOnDivergedOrFinished_) {
      publishHoldTrajectory("DIVERGED");
    }
    startZeroBaseCmdBurst("DIVERGED");
    requestSwitchMpcController(/*activate=*/false, "DIVERGED");
    finishGoal(goal_handle, GoalTermination::Aborted, "DIVERGED", "Tracking diverged during screw move.");
  }
}

void TrajectoryTTActionServerNode::onTrackingMetric(const std_msgs::msg::Float64MultiArray& msg) {
  std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle;
  MonitorSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (msg.data.size() >= 6) {
      monitorSnapshot_.progress = msg.data[1];
      monitorSnapshot_.pos_err = msg.data[2];
      monitorSnapshot_.ori_err_deg = msg.data[3];
      monitorSnapshot_.lag_sec = msg.data[4];
      const double closest_index = std::isfinite(msg.data[5]) ? std::max(0.0, msg.data[5]) : 0.0;
      monitorSnapshot_.closest_index = static_cast<std::size_t>(closest_index);
      monitorSnapshot_.valid = true;
    }

    if (!activeGoal_ || !goalPublishingStarted_) return;
    goal_handle = activeGoal_;
    snapshot = monitorSnapshot_;
  }

  auto feedback = std::make_shared<ExecuteScrewMove::Feedback>();
  populateFeedback(*feedback, snapshot);
  goal_handle->publish_feedback(feedback);
}

void TrajectoryTTActionServerNode::publishWaitingFeedback(
    const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle,
    const std::string& status) {
  const rclcpp::Time now = this->now();
  {
    std::lock_guard<std::mutex> lock(waitingFeedbackMtx_);
    if ((now - lastWaitingFeedbackTime_).seconds() < 0.5) {
      return;
    }
    lastWaitingFeedbackTime_ = now;
  }

  auto feedback = std::make_shared<ExecuteScrewMove::Feedback>();
  feedback->tracking_status = status;
  feedback->progress = 0.0;
  feedback->pos_err = 0.0;
  feedback->ori_err_deg = 0.0;
  feedback->lag_sec = 0.0;
  feedback->closest_index = 0;
  goal_handle->publish_feedback(feedback);
}

void TrajectoryTTActionServerNode::handleCanceledGoal(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle) {
  publishHoldTrajectory("CANCELED");
  startZeroBaseCmdBurst("CANCELED");
  requestSwitchMpcController(/*activate=*/false, "CANCELED");
  finishGoal(goal_handle, GoalTermination::Canceled, "CANCELED", "Screw move canceled.");
}

void TrajectoryTTActionServerNode::finishGoal(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle,
                                              GoalTermination termination,
                                              const std::string& final_status,
                                              const std::string& message) {
  MonitorSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_ || activeGoal_ != goal_handle) return;
    snapshot = monitorSnapshot_;
    clearActiveGoalLocked();
  }

  auto result = std::make_shared<ExecuteScrewMove::Result>();
  populateResult(*result, termination == GoalTermination::Succeeded, final_status, message, snapshot);

  switch (termination) {
    case GoalTermination::Succeeded:
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Screw move goal succeeded.");
      break;
    case GoalTermination::Aborted:
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "Screw move goal aborted.");
      break;
    case GoalTermination::Canceled:
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Screw move goal canceled.");
      break;
  }
}

void TrajectoryTTActionServerNode::clearActiveGoalLocked() {
  activeGoal_.reset();
  goalInitialized_ = false;
  goalPublishingStarted_ = false;
  cancelRequested_ = false;
  planner_.reset();
  nominalTraj_ = PlannedCartesianTrajectory{};
  haveNominalTraj_ = false;
  haveLastProjectionIndex_ = false;
  lastProjectionIndex_ = 0;
  warnedMissingFk_ = false;
  centerP_ = Eigen::Vector3d::Zero();
  q0_ = Eigen::Quaterniond::Identity();
}

void TrajectoryTTActionServerNode::populateFeedback(ExecuteScrewMove::Feedback& feedback, const MonitorSnapshot& snapshot) {
  feedback.tracking_status = snapshot.status;
  feedback.progress = snapshot.progress;
  feedback.pos_err = snapshot.pos_err;
  feedback.ori_err_deg = snapshot.ori_err_deg;
  feedback.lag_sec = snapshot.lag_sec;
  feedback.closest_index = saturateClosestIndex(snapshot.closest_index);
}

void TrajectoryTTActionServerNode::populateResult(ExecuteScrewMove::Result& result,
                                                  bool success,
                                                  const std::string& final_status,
                                                  const std::string& message,
                                                  const MonitorSnapshot& snapshot) {
  result.success = success;
  result.final_status = final_status;
  result.message = message;
  result.progress = snapshot.progress;
  result.pos_err = snapshot.pos_err;
  result.ori_err_deg = snapshot.ori_err_deg;
  result.lag_sec = snapshot.lag_sec;
  result.closest_index = saturateClosestIndex(snapshot.closest_index);
}

uint32_t TrajectoryTTActionServerNode::saturateClosestIndex(std::size_t index) {
  return static_cast<uint32_t>(std::min<std::size_t>(index, std::numeric_limits<uint32_t>::max()));
}

void TrajectoryTTActionServerNode::publishHoldTrajectory(const std::string& reason) {
  if (!trajPub_) return;

  SystemObservation obs;
  bool have_obs_local = false;
  {
    std::lock_guard<std::mutex> lock(obsMtx_);
    have_obs_local = haveObs_;
    if (have_obs_local) {
      obs = latestObs_;
    }
  }
  if (!have_obs_local) {
    RCLCPP_WARN(this->get_logger(), "Hold requested (%s) but no observation is available.", reason.c_str());
    return;
  }

  Eigen::Vector3d p_hold = centerP_;
  Eigen::Quaterniond q_hold = q0_;
  bool ok = false;
  if (pin_ && mapping_ && interface_) {
    std::lock_guard<std::mutex> lock(pinMtx_);
    ok = computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p_hold, q_hold);
  }
  if (!ok) {
    q_hold.normalize();
  }

  PlannedCartesianTrajectory hold;
  hold.time = {obs.time, obs.time + 1.0};
  hold.position = {p_hold, p_hold};
  hold.quat = {q_hold, q_hold};
  trajPub_->publishEeTarget(hold, obs);

  RCLCPP_INFO(this->get_logger(), "Published HOLD target (reason=%s).", reason.c_str());
}

void TrajectoryTTActionServerNode::startZeroBaseCmdBurst(const std::string& reason) {
  if (!publishZeroBaseCmdOnIntervention_) return;
  if (baseCmdTopic_.empty()) return;
  if (zeroBaseCmdBurstCount_ <= 0) return;

  if (!baseCmdPub_) {
    baseCmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>(baseCmdTopic_, rclcpp::SystemDefaultsQoS());
  }

  zeroBaseCmdRemaining_ = zeroBaseCmdBurstCount_;
  const double rate = std::max(1e-3, zeroBaseCmdBurstRate_);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));

  if (!zeroBaseCmdTimer_) {
    zeroBaseCmdTimer_ = this->create_wall_timer(period, [this]() {
      if (zeroBaseCmdRemaining_ <= 0) {
        if (zeroBaseCmdTimer_) zeroBaseCmdTimer_->cancel();
        return;
      }
      geometry_msgs::msg::Twist z;
      baseCmdPub_->publish(z);
      --zeroBaseCmdRemaining_;
      if (zeroBaseCmdRemaining_ <= 0 && zeroBaseCmdTimer_) {
        zeroBaseCmdTimer_->cancel();
      }
    });
  } else {
    zeroBaseCmdTimer_->reset();
  }

  geometry_msgs::msg::Twist z;
  baseCmdPub_->publish(z);
  --zeroBaseCmdRemaining_;

  RCLCPP_INFO(this->get_logger(), "Publishing zero base cmd_vel burst (reason=%s).", reason.c_str());
}

void TrajectoryTTActionServerNode::requestSwitchMpcController(bool activate, const std::string& reason) {
  const bool should_switch = activate ? activateMpcControllerOnGoal_ : switchMpcControllerOnIntervention_;
  if (!should_switch) return;
  if (!switchControllerClient_ || mpcControllerName_.empty()) return;

  if (!switchControllerClient_->wait_for_service(std::chrono::seconds(0))) {
    RCLCPP_WARN(this->get_logger(),
                "controller_manager switch service '%s' not available; cannot switch '%s' (activate=%d, reason=%s).",
                controllerManagerSwitchService_.c_str(),
                mpcControllerName_.c_str(),
                static_cast<int>(activate),
                reason.c_str());
    return;
  }

  using Request = controller_manager_msgs::srv::SwitchController::Request;
  auto req = std::make_shared<Request>();

  constexpr bool kUseActivateFields =
      has_member_activate_controllers<Request>::value || has_member_deactivate_controllers<Request>::value;
  if constexpr (kUseActivateFields) {
    if constexpr (has_member_activate_controllers<Request>::value) {
      if (activate) req->activate_controllers = {mpcControllerName_};
    }
    if constexpr (has_member_deactivate_controllers<Request>::value) {
      if (!activate) req->deactivate_controllers = {mpcControllerName_};
    }
  } else {
    if constexpr (has_member_start_controllers<Request>::value) {
      if (activate) req->start_controllers = {mpcControllerName_};
    }
    if constexpr (has_member_stop_controllers<Request>::value) {
      if (!activate) req->stop_controllers = {mpcControllerName_};
    }
  }

  req->strictness = switchControllerStrictness();
  if constexpr (has_member_activate_asap<Request>::value) {
    req->activate_asap = true;
  } else if constexpr (has_member_start_asap<Request>::value) {
    req->start_asap = true;
  }
  req->timeout.sec = 0;
  req->timeout.nanosec = 0;

  using Future = rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture;
  auto cb = [this, activate, reason](Future future) {
    const auto resp = future.get();
    if (!resp->ok) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to switch mpc controller '%s' (activate=%d, reason=%s).",
                  mpcControllerName_.c_str(),
                  static_cast<int>(activate),
                  reason.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Switched mpc controller '%s' (activate=%d, reason=%s).",
                mpcControllerName_.c_str(),
                static_cast<int>(activate),
                reason.c_str());
  };
  (void)switchControllerClient_->async_send_request(req, cb);
}

void TrajectoryTTActionServerNode::publishDefaultModeSchedule(const std::string& reason) {
  if (!publishModeScheduleOnGoal_) return;
  if (!modeSchedulePub_) return;

  const int clamped_mode = std::clamp(modeScheduleMode_, -128, 127);
  ocs2_msgs::msg::ModeSchedule msg;
  msg.event_times = {};
  msg.mode_sequence = {static_cast<int8_t>(clamped_mode)};
  modeSchedulePub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Published default mode schedule (mode=%d, reason=%s).",
              clamped_mode,
              reason.c_str());
}

void TrajectoryTTActionServerNode::publishCommandTf(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = globalFrame_;
  tf.child_frame_id = commandFrameId_;
  tf.transform.translation.x = p.x();
  tf.transform.translation.y = p.y();
  tf.transform.translation.z = p.z();
  tf.transform.rotation = ocs2::ros_msg_helpers::getOrientationMsg(q);
  tfBroadcaster_->sendTransform(tf);
}

void TrajectoryTTActionServerNode::publishViz(const PlannedCartesianTrajectory& traj, double now_time) {
  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = globalFrame_;
  pose_array.header.stamp = this->now();

  marker_array.markers.reserve(traj.size());
  pose_array.poses.reserve(traj.size());

  const auto it = std::upper_bound(traj.time.begin(), traj.time.end(), now_time);
  const std::size_t passed = static_cast<std::size_t>(it - traj.time.begin());

  for (std::size_t i = 0; i < traj.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = globalFrame_;
    marker.header.stamp = this->now();
    marker.ns = "target_state";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
    marker.color.r = 0.4660f;
    marker.color.g = 0.6740f;
    marker.color.b = 0.1880f;
    marker.color.a = (i < passed) ? 0.1f : 1.0f;
    marker.pose.position = ocs2::ros_msg_helpers::getPointMsg(traj.position[i]);
    marker.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(traj.quat[i]);
    marker_array.markers.push_back(marker);

    if (i >= passed) {
      geometry_msgs::msg::Pose pose;
      pose.position = marker.pose.position;
      pose.orientation = marker.pose.orientation;
      pose_array.poses.push_back(pose);
    }
  }

  markerPub_->publish(marker_array);
  posePub_->publish(pose_array);
}

}  // namespace mpc_cartesian_planner
