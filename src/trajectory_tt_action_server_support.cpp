#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include "robot_math_utils/robot_math_utils_v1_18.hpp"
#include "trajectory_tt_action_server_impl.h"

using ocs2::SystemObservation;

namespace mpc_cartesian_planner {

using trajectory_tt_action_server_internal::computeEePose;
using trajectory_tt_action_server_internal::channelModeName;
using trajectory_tt_action_server_internal::GoalTrajectoryType;
using trajectory_tt_action_server_internal::GoalChannelMode;
using trajectory_tt_action_server_internal::goalTypeName;
using trajectory_tt_action_server_internal::has_member_activate_asap;
using trajectory_tt_action_server_internal::has_member_activate_controllers;
using trajectory_tt_action_server_internal::has_member_deactivate_controllers;
using trajectory_tt_action_server_internal::has_member_start_asap;
using trajectory_tt_action_server_internal::has_member_start_controllers;
using trajectory_tt_action_server_internal::has_member_stop_controllers;
using trajectory_tt_action_server_internal::normalizeAngle;
using trajectory_tt_action_server_internal::parseTimeScaling;
using trajectory_tt_action_server_internal::sampleTrajectoryAtTime;
using trajectory_tt_action_server_internal::switchControllerStrictness;
using trajectory_tt_action_server_internal::yawToQuaternion;

namespace {

bool validateFiniteTiming(double duration, double dt, std::string& reason) {
  if (!std::isfinite(duration) || duration <= 0.0) {
    reason = "duration must be > 0.";
    return false;
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    reason = "dt must be > 0.";
    return false;
  }
  return true;
}

template <typename Container>
bool allFinite(const Container& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

template <typename ActionT>
void populateFeedbackCommon(typename ActionT::Feedback& feedback,
                            const TrajectoryTTActionServerNode::MonitorSnapshot& snapshot) {
  if constexpr (std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedScrewMove> ||
                std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedLinearMove> ||
                std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedTargetPose>) {
    feedback.ee_tracking_status = snapshot.ee.status;
    feedback.ee_progress = snapshot.ee.progress;
    feedback.ee_pos_err = snapshot.ee.pos_err;
    feedback.ee_ori_err_deg = snapshot.ee.ori_err_deg;
    feedback.ee_lag_sec = snapshot.ee.lag_sec;
    feedback.ee_closest_index =
        static_cast<uint32_t>(std::min<std::size_t>(snapshot.ee.closest_index, std::numeric_limits<uint32_t>::max()));
    feedback.base_tracking_status = snapshot.base.status;
    feedback.base_progress = snapshot.base.progress;
    feedback.base_pos_err = snapshot.base.pos_err;
    feedback.base_ori_err_deg = snapshot.base.ori_err_deg;
    feedback.base_lag_sec = snapshot.base.lag_sec;
    feedback.base_closest_index =
        static_cast<uint32_t>(std::min<std::size_t>(snapshot.base.closest_index, std::numeric_limits<uint32_t>::max()));
  } else {
    feedback.tracking_status = snapshot.status;
    feedback.progress = snapshot.progress;
    feedback.pos_err = snapshot.pos_err;
    feedback.ori_err_deg = snapshot.ori_err_deg;
    feedback.lag_sec = snapshot.lag_sec;
    feedback.closest_index =
        static_cast<uint32_t>(std::min<std::size_t>(snapshot.closest_index, std::numeric_limits<uint32_t>::max()));
  }
}

template <typename ActionT>
void populateResultCommon(typename ActionT::Result& result,
                          bool success,
                          const std::string& final_status,
                          const std::string& message,
                          const TrajectoryTTActionServerNode::MonitorSnapshot& snapshot) {
  result.success = success;
  result.final_status = final_status;
  result.message = message;
  if constexpr (std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedScrewMove> ||
                std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedLinearMove> ||
                std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedTargetPose>) {
    result.ee_tracking_status = snapshot.ee.status;
    result.ee_progress = snapshot.ee.progress;
    result.ee_pos_err = snapshot.ee.pos_err;
    result.ee_ori_err_deg = snapshot.ee.ori_err_deg;
    result.ee_lag_sec = snapshot.ee.lag_sec;
    result.ee_closest_index =
        static_cast<uint32_t>(std::min<std::size_t>(snapshot.ee.closest_index, std::numeric_limits<uint32_t>::max()));
    result.base_tracking_status = snapshot.base.status;
    result.base_progress = snapshot.base.progress;
    result.base_pos_err = snapshot.base.pos_err;
    result.base_ori_err_deg = snapshot.base.ori_err_deg;
    result.base_lag_sec = snapshot.base.lag_sec;
    result.base_closest_index =
        static_cast<uint32_t>(std::min<std::size_t>(snapshot.base.closest_index, std::numeric_limits<uint32_t>::max()));
  } else {
    result.progress = snapshot.progress;
    result.pos_err = snapshot.pos_err;
    result.ori_err_deg = snapshot.ori_err_deg;
    result.lag_sec = snapshot.lag_sec;
    result.closest_index =
        static_cast<uint32_t>(std::min<std::size_t>(snapshot.closest_index, std::numeric_limits<uint32_t>::max()));
  }
}

template <typename ActionT>
class ActiveGoal final : public ActiveGoalBase {
 public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  ActiveGoal(std::shared_ptr<GoalHandle> goal_handle, TrajectoryTTActionServerNode::GoalConfig goal_config)
      : goal_handle_(std::move(goal_handle)), goal_config_(std::move(goal_config)) {}

  const TrajectoryTTActionServerNode::GoalConfig& goalConfig() const override { return goal_config_; }

  bool isCanceling() const override { return goal_handle_->is_canceling(); }

  bool matchesHandle(const void* native_handle) const override { return goal_handle_.get() == native_handle; }

  void publishWaitingFeedback(const std::string& status) override {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    if constexpr (std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedScrewMove> ||
                  std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedLinearMove> ||
                  std::is_same_v<ActionT, TrajectoryTTActionServerNode::ExecuteCombinedTargetPose>) {
      feedback->ee_tracking_status = status;
      feedback->ee_progress = 0.0;
      feedback->ee_pos_err = 0.0;
      feedback->ee_ori_err_deg = 0.0;
      feedback->ee_lag_sec = 0.0;
      feedback->ee_closest_index = 0;
      feedback->base_tracking_status = status;
      feedback->base_progress = 0.0;
      feedback->base_pos_err = 0.0;
      feedback->base_ori_err_deg = 0.0;
      feedback->base_lag_sec = 0.0;
      feedback->base_closest_index = 0;
    } else {
      feedback->tracking_status = status;
      feedback->progress = 0.0;
      feedback->pos_err = 0.0;
      feedback->ori_err_deg = 0.0;
      feedback->lag_sec = 0.0;
      feedback->closest_index = 0;
    }
    goal_handle_->publish_feedback(feedback);
  }

  void publishFeedback(const TrajectoryTTActionServerNode::MonitorSnapshot& snapshot) override {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    populateFeedbackCommon<ActionT>(*feedback, snapshot);
    goal_handle_->publish_feedback(feedback);
  }

  void finish(TrajectoryTTActionServerNode::GoalTermination termination,
              const std::string& final_status,
              const std::string& message,
              const TrajectoryTTActionServerNode::MonitorSnapshot& snapshot) override {
    auto result = std::make_shared<typename ActionT::Result>();
    populateResultCommon<ActionT>(*result, termination == TrajectoryTTActionServerNode::GoalTermination::Succeeded,
                                  final_status, message, snapshot);

    switch (termination) {
      case TrajectoryTTActionServerNode::GoalTermination::Succeeded:
        goal_handle_->succeed(result);
        break;
      case TrajectoryTTActionServerNode::GoalTermination::Aborted:
        goal_handle_->abort(result);
        break;
      case TrajectoryTTActionServerNode::GoalTermination::Canceled:
        goal_handle_->canceled(result);
        break;
    }
  }

 private:
  std::shared_ptr<GoalHandle> goal_handle_;
  TrajectoryTTActionServerNode::GoalConfig goal_config_;
};

template <typename ActionT>
TrajectoryTTActionServerNode::ActiveGoalPtr makeActiveGoal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
    TrajectoryTTActionServerNode::GoalConfig goal_config) {
  return std::make_shared<ActiveGoal<ActionT>>(std::move(goal_handle), std::move(goal_config));
}

std::string formatVec3(const Eigen::Vector3d& value) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "[" << value.x() << ", " << value.y() << ", " << value.z() << "]";
  return oss.str();
}

std::string describeGoal(const TrajectoryTTActionServerNode::GoalConfig& goal_cfg) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "duration=" << goal_cfg.duration
      << " dt=" << goal_cfg.dt
      << " ee_mode=" << channelModeName(goal_cfg.ee_mode)
      << " base_mode=" << channelModeName(goal_cfg.base_mode);
  switch (goal_cfg.trajectory_type) {
    case GoalTrajectoryType::ScrewMove:
      oss << " theta=" << goal_cfg.screw_theta
          << " tool_frame=" << static_cast<int>(goal_cfg.screw_in_tool_frame);
      break;
    case GoalTrajectoryType::LinearMove:
      oss << " offset=" << formatVec3(goal_cfg.linear_offset)
          << " tool_frame=" << static_cast<int>(goal_cfg.linear_in_tool_frame);
      break;
    case GoalTrajectoryType::TargetPose:
      oss << " target=" << formatVec3(goal_cfg.target_p)
          << " use_target_orientation=" << static_cast<int>(goal_cfg.use_target_orientation);
      break;
    case GoalTrajectoryType::FigureEight:
      oss << " amplitude=" << goal_cfg.figure_eight_amplitude
          << " frequency=" << goal_cfg.figure_eight_frequency;
      break;
  }
  return oss.str();
}

std::string goalMessage(const TrajectoryTTActionServerNode::GoalConfig& goal_cfg, const std::string& suffix) {
  return std::string(goalTypeName(goal_cfg.trajectory_type)) + " " + suffix;
}

}  // namespace

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteScrewMove::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (!std::isfinite(goal.screw_theta)) {
    reason = "screw_theta must be finite.";
    return false;
  }
  if (!allFinite(goal.screw_uhat) || !allFinite(goal.screw_r)) {
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

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteLinearMove::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (goal.linear_move_offset.size() != 3 && goal.linear_move_offset.size() != 6) {
    reason = "linear_move_offset must have length 3 or 6.";
    return false;
  }
  if (!allFinite(goal.linear_move_offset)) {
    reason = "linear_move_offset must be finite.";
    return false;
  }
  return true;
}

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteTargetPose::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (goal.target_pose.size() != 3 && goal.target_pose.size() != 6 && goal.target_pose.size() != 7) {
    reason = "target_pose must have length 3, 6, or 7.";
    return false;
  }
  if (!allFinite(goal.target_pose)) {
    reason = "target_pose must be finite.";
    return false;
  }
  if (goal.target_pose.size() == 7) {
    const Eigen::Quaterniond q(goal.target_pose[3], goal.target_pose[4], goal.target_pose[5], goal.target_pose[6]);
    if (q.norm() < 1e-12) {
      reason = "target_pose quaternion must be non-zero.";
      return false;
    }
  }
  return true;
}

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteFigureEight::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (!std::isfinite(goal.amplitude) || goal.amplitude < 0.0) {
    reason = "amplitude must be finite and >= 0.";
    return false;
  }
  if (!std::isfinite(goal.frequency) || goal.frequency <= 0.0) {
    reason = "frequency must be > 0.";
    return false;
  }
  if (!allFinite(goal.plane_axis)) {
    reason = "plane_axis must be finite.";
    return false;
  }
  const Eigen::Vector3d plane_axis(goal.plane_axis[0], goal.plane_axis[1], goal.plane_axis[2]);
  if (plane_axis.norm() < 1e-9) {
    reason = "plane_axis must be non-zero.";
    return false;
  }
  return true;
}

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteCombinedScrewMove::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (!goal.enable_ee && !goal.enable_base) {
    reason = "At least one of enable_ee or enable_base must be true.";
    return false;
  }
  if (goal.enable_ee && !goal.hold_ee) {
    if (!std::isfinite(goal.ee_screw_theta)) {
      reason = "ee_screw_theta must be finite.";
      return false;
    }
    if (!allFinite(goal.ee_screw_uhat) || !allFinite(goal.ee_screw_r)) {
      reason = "ee screw parameters must be finite.";
      return false;
    }
    const Eigen::Vector3d u_hat(goal.ee_screw_uhat[0], goal.ee_screw_uhat[1], goal.ee_screw_uhat[2]);
    if (u_hat.norm() < 1e-9) {
      reason = "ee_screw_uhat must be non-zero.";
      return false;
    }
  }
  if (goal.enable_base && !goal.hold_base) {
    if (!std::isfinite(goal.base_screw_theta) || !allFinite(goal.base_screw_r)) {
      reason = "base screw parameters must be finite.";
      return false;
    }
  }
  if (!goal.enable_ee && goal.hold_ee) {
    reason = "hold_ee requires enable_ee=true.";
    return false;
  }
  if (!goal.enable_base && goal.hold_base) {
    reason = "hold_base requires enable_base=true.";
    return false;
  }
  return true;
}

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteCombinedLinearMove::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (!goal.enable_ee && !goal.enable_base) {
    reason = "At least one of enable_ee or enable_base must be true.";
    return false;
  }
  if (goal.enable_ee && !goal.hold_ee) {
    if (goal.ee_linear_move_offset.size() != 3 && goal.ee_linear_move_offset.size() != 6) {
      reason = "ee_linear_move_offset must have length 3 or 6.";
      return false;
    }
    if (!allFinite(goal.ee_linear_move_offset)) {
      reason = "ee_linear_move_offset must be finite.";
      return false;
    }
  }
  if (goal.enable_base && !goal.hold_base) {
    if (!allFinite(goal.base_linear_move_offset)) {
      reason = "base_linear_move_offset must be finite.";
      return false;
    }
  }
  if (!goal.enable_ee && goal.hold_ee) {
    reason = "hold_ee requires enable_ee=true.";
    return false;
  }
  if (!goal.enable_base && goal.hold_base) {
    reason = "hold_base requires enable_base=true.";
    return false;
  }
  return true;
}

bool TrajectoryTTActionServerNode::validateGoal(const ExecuteCombinedTargetPose::Goal& goal, std::string& reason) const {
  if (!validateFiniteTiming(goal.duration, goal.dt, reason)) {
    return false;
  }
  if (!goal.enable_ee && !goal.enable_base) {
    reason = "At least one of enable_ee or enable_base must be true.";
    return false;
  }
  if (goal.enable_ee && !goal.hold_ee) {
    if (goal.ee_target_pose.size() != 3 && goal.ee_target_pose.size() != 6 && goal.ee_target_pose.size() != 7) {
      reason = "ee_target_pose must have length 3, 6, or 7.";
      return false;
    }
    if (!allFinite(goal.ee_target_pose)) {
      reason = "ee_target_pose must be finite.";
      return false;
    }
    if (goal.ee_target_pose.size() == 7) {
      const Eigen::Quaterniond q(goal.ee_target_pose[3], goal.ee_target_pose[4], goal.ee_target_pose[5], goal.ee_target_pose[6]);
      if (q.norm() < 1e-12) {
        reason = "ee_target_pose quaternion must be non-zero.";
        return false;
      }
    }
  }
  if (goal.enable_base && !goal.hold_base) {
    if (goal.base_target_pose.size() != 3) {
      reason = "base_target_pose must have length 3 [x, y, yaw].";
      return false;
    }
    if (!allFinite(goal.base_target_pose)) {
      reason = "base_target_pose must be finite.";
      return false;
    }
  }
  if (!goal.enable_ee && goal.hold_ee) {
    reason = "hold_ee requires enable_ee=true.";
    return false;
  }
  if (!goal.enable_base && goal.hold_base) {
    reason = "hold_base requires enable_base=true.";
    return false;
  }
  return true;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteScrewMove::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::ScrewMove;
  cfg.ee_mode = GoalChannelMode::Track;
  cfg.base_mode = GoalChannelMode::Disabled;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.screw_uhat = Eigen::Vector3d(goal.screw_uhat[0], goal.screw_uhat[1], goal.screw_uhat[2]);
  cfg.screw_r = Eigen::Vector3d(goal.screw_r[0], goal.screw_r[1], goal.screw_r[2]);
  cfg.screw_theta = goal.screw_theta;
  cfg.screw_in_tool_frame = goal.screw_in_tool_frame;
  return cfg;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteLinearMove::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::LinearMove;
  cfg.ee_mode = GoalChannelMode::Track;
  cfg.base_mode = GoalChannelMode::Disabled;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.linear_offset = Eigen::Vector3d(goal.linear_move_offset[0], goal.linear_move_offset[1], goal.linear_move_offset[2]);
  if (goal.linear_move_offset.size() == 6) {
    cfg.linear_euler_zyx =
        Eigen::Vector3d(goal.linear_move_offset[3], goal.linear_move_offset[4], goal.linear_move_offset[5]);
  }
  cfg.linear_in_tool_frame = goal.linear_move_in_tool_frame;
  return cfg;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteTargetPose::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::TargetPose;
  cfg.ee_mode = GoalChannelMode::Track;
  cfg.base_mode = GoalChannelMode::Disabled;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.target_p = Eigen::Vector3d(goal.target_pose[0], goal.target_pose[1], goal.target_pose[2]);
  if (goal.target_pose.size() == 6) {
    const Eigen::Vector3d euler_zyx(goal.target_pose[3], goal.target_pose[4], goal.target_pose[5]);
    cfg.target_q = RMUtils::zyxEuler2Quat(euler_zyx, /*rad=*/true).normalized();
    cfg.use_target_orientation = true;
  } else if (goal.target_pose.size() == 7) {
    cfg.target_q = Eigen::Quaterniond(goal.target_pose[3], goal.target_pose[4], goal.target_pose[5], goal.target_pose[6]).normalized();
    cfg.use_target_orientation = true;
  }
  return cfg;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteFigureEight::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::FigureEight;
  cfg.ee_mode = GoalChannelMode::Track;
  cfg.base_mode = GoalChannelMode::Disabled;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.figure_eight_amplitude = goal.amplitude;
  cfg.figure_eight_frequency = goal.frequency;
  cfg.figure_eight_plane_axis = Eigen::Vector3d(goal.plane_axis[0], goal.plane_axis[1], goal.plane_axis[2]);
  return cfg;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteCombinedScrewMove::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::ScrewMove;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.ee_mode = goal.enable_ee ? (goal.hold_ee ? GoalChannelMode::HoldCurrent : GoalChannelMode::Track)
                               : GoalChannelMode::Disabled;
  cfg.base_mode = goal.enable_base ? (goal.hold_base ? GoalChannelMode::HoldCurrent : GoalChannelMode::Track)
                                   : GoalChannelMode::Disabled;
  cfg.screw_uhat = Eigen::Vector3d(goal.ee_screw_uhat[0], goal.ee_screw_uhat[1], goal.ee_screw_uhat[2]);
  cfg.screw_r = Eigen::Vector3d(goal.ee_screw_r[0], goal.ee_screw_r[1], goal.ee_screw_r[2]);
  cfg.screw_theta = goal.ee_screw_theta;
  cfg.screw_in_tool_frame = goal.ee_screw_in_tool_frame;
  cfg.base_screw_r = Eigen::Vector2d(goal.base_screw_r[0], goal.base_screw_r[1]);
  cfg.base_screw_theta = goal.base_screw_theta;
  cfg.base_screw_in_body_frame = goal.base_screw_in_body_frame;
  return cfg;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteCombinedLinearMove::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::LinearMove;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.ee_mode = goal.enable_ee ? (goal.hold_ee ? GoalChannelMode::HoldCurrent : GoalChannelMode::Track)
                               : GoalChannelMode::Disabled;
  cfg.base_mode = goal.enable_base ? (goal.hold_base ? GoalChannelMode::HoldCurrent : GoalChannelMode::Track)
                                   : GoalChannelMode::Disabled;
  if (!goal.ee_linear_move_offset.empty()) {
    cfg.linear_offset = Eigen::Vector3d(goal.ee_linear_move_offset[0], goal.ee_linear_move_offset[1], goal.ee_linear_move_offset[2]);
    if (goal.ee_linear_move_offset.size() == 6) {
      cfg.linear_euler_zyx =
          Eigen::Vector3d(goal.ee_linear_move_offset[3], goal.ee_linear_move_offset[4], goal.ee_linear_move_offset[5]);
    }
  }
  cfg.linear_in_tool_frame = goal.ee_linear_move_in_tool_frame;
  cfg.base_linear_offset =
      Eigen::Vector3d(goal.base_linear_move_offset[0], goal.base_linear_move_offset[1], goal.base_linear_move_offset[2]);
  cfg.base_linear_in_body_frame = goal.base_linear_move_in_body_frame;
  return cfg;
}

TrajectoryTTActionServerNode::GoalConfig TrajectoryTTActionServerNode::goalToConfig(const ExecuteCombinedTargetPose::Goal& goal) const {
  GoalConfig cfg;
  cfg.trajectory_type = GoalTrajectoryType::TargetPose;
  cfg.duration = goal.duration;
  cfg.dt = goal.dt;
  cfg.time_scaling = parseTimeScaling(goal.time_scaling);
  cfg.ee_mode = goal.enable_ee ? (goal.hold_ee ? GoalChannelMode::HoldCurrent : GoalChannelMode::Track)
                               : GoalChannelMode::Disabled;
  cfg.base_mode = goal.enable_base ? (goal.hold_base ? GoalChannelMode::HoldCurrent : GoalChannelMode::Track)
                                   : GoalChannelMode::Disabled;
  if (!goal.ee_target_pose.empty()) {
    cfg.target_p = Eigen::Vector3d(goal.ee_target_pose[0], goal.ee_target_pose[1], goal.ee_target_pose[2]);
    if (goal.ee_target_pose.size() == 6) {
      const Eigen::Vector3d euler_zyx(goal.ee_target_pose[3], goal.ee_target_pose[4], goal.ee_target_pose[5]);
      cfg.target_q = RMUtils::zyxEuler2Quat(euler_zyx, /*rad=*/true).normalized();
      cfg.use_target_orientation = true;
    } else if (goal.ee_target_pose.size() == 7) {
      cfg.target_q = Eigen::Quaterniond(goal.ee_target_pose[3], goal.ee_target_pose[4], goal.ee_target_pose[5], goal.ee_target_pose[6]).normalized();
      cfg.use_target_orientation = true;
    }
  }
  if (goal.base_target_pose.size() == 3) {
    cfg.base_target_pose = Eigen::Vector3d(goal.base_target_pose[0], goal.base_target_pose[1], goal.base_target_pose[2]);
  }
  return cfg;
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::acceptGoalRequest(const std::string& goal_type) {
  std::lock_guard<std::mutex> lock(stateMtx_);
  if (activeGoal_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting %s goal: another goal is already active.", goal_type.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleCancelCommon(const void* native_handle) {
  std::lock_guard<std::mutex> lock(stateMtx_);
  if (!activeGoal_ || !activeGoal_->matchesHandle(native_handle)) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  cancelRequested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryTTActionServerNode::activateGoal(const ActiveGoalPtr& goal_handle) {
  const GoalConfig& goal_cfg = goal_handle->goalConfig();
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    activeGoal_ = goal_handle;
    goalInitialized_ = false;
    goalPublishingStarted_ = false;
    cancelRequested_ = false;
    eePlanner_.reset();
    basePlanner_.reset();
    eeMonitor_.reset();
    baseMonitor_.reset();
    nominalEeTraj_ = PlannedCartesianTrajectory{};
    nominalBaseTraj_ = PlannedBaseTrajectory{};
    haveNominalEeTraj_ = false;
    haveNominalBaseTraj_ = false;
    haveLastEeProjectionIndex_ = false;
    haveLastBaseProjectionIndex_ = false;
    lastEeProjectionIndex_ = 0;
    lastBaseProjectionIndex_ = 0;
    warnedMissingFk_ = false;
    warnedMissingBaseState_ = false;
    centerP_ = Eigen::Vector3d::Zero();
    q0_ = Eigen::Quaterniond::Identity();
    baseStartP_.setZero();
    baseStartYaw_ = 0.0;
    monitorSnapshot_ = MonitorSnapshot{};
  }
  {
    std::lock_guard<std::mutex> lock(waitingFeedbackMtx_);
    lastWaitingFeedbackTime_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  publishWaitingFeedback(goal_handle, "WAITING_FOR_OBSERVATION");
  RCLCPP_INFO(this->get_logger(), "Accepted %s goal: %s", goalTypeName(goal_cfg.trajectory_type), describeGoal(goal_cfg).c_str());
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleScrewGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteScrewMove::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting screw_move goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("screw_move");
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleLinearGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteLinearMove::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting linear_move goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("linear_move");
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleTargetPoseGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteTargetPose::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting target_pose goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("target_pose");
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleFigureEightGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteFigureEight::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting figure_eight goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("figure_eight");
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleCombinedScrewGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteCombinedScrewMove::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting combined_screw_move goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("combined_screw_move");
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleCombinedLinearGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteCombinedLinearMove::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting combined_linear_move goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("combined_linear_move");
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleCombinedTargetPoseGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteCombinedTargetPose::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting combined_target_pose goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return acceptGoalRequest("combined_target_pose");
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleScrewCancel(
    const std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleLinearCancel(
    const std::shared_ptr<GoalHandleExecuteLinearMove> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleTargetPoseCancel(
    const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleFigureEightCancel(
    const std::shared_ptr<GoalHandleExecuteFigureEight> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleCombinedScrewCancel(
    const std::shared_ptr<GoalHandleExecuteCombinedScrewMove> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleCombinedLinearCancel(
    const std::shared_ptr<GoalHandleExecuteCombinedLinearMove> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleCombinedTargetPoseCancel(
    const std::shared_ptr<GoalHandleExecuteCombinedTargetPose> goal_handle) {
  return handleCancelCommon(goal_handle.get());
}

void TrajectoryTTActionServerNode::handleScrewAccepted(const std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteScrewMove>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

void TrajectoryTTActionServerNode::handleLinearAccepted(const std::shared_ptr<GoalHandleExecuteLinearMove> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteLinearMove>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

void TrajectoryTTActionServerNode::handleTargetPoseAccepted(
    const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteTargetPose>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

void TrajectoryTTActionServerNode::handleFigureEightAccepted(
    const std::shared_ptr<GoalHandleExecuteFigureEight> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteFigureEight>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

void TrajectoryTTActionServerNode::handleCombinedScrewAccepted(
    const std::shared_ptr<GoalHandleExecuteCombinedScrewMove> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteCombinedScrewMove>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

void TrajectoryTTActionServerNode::handleCombinedLinearAccepted(
    const std::shared_ptr<GoalHandleExecuteCombinedLinearMove> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteCombinedLinearMove>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

void TrajectoryTTActionServerNode::handleCombinedTargetPoseAccepted(
    const std::shared_ptr<GoalHandleExecuteCombinedTargetPose> goal_handle) {
  activateGoal(makeActiveGoal<ExecuteCombinedTargetPose>(goal_handle, goalToConfig(*goal_handle->get_goal())));
}

bool TrajectoryTTActionServerNode::initializeGoalIfNeeded(const ActiveGoalPtr& goal_handle,
                                                          const GoalConfig& goal_cfg,
                                                          const SystemObservation& obs) {
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_ || activeGoal_ != goal_handle) return false;
    if (goalInitialized_) return true;
  }

  Eigen::Vector3d center_p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  if (goal_cfg.ee_mode != GoalChannelMode::Disabled && interface_ && pin_ && mapping_) {
    std::lock_guard<std::mutex> lock(pinMtx_);
    if (!computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, center_p, q0)) {
      RCLCPP_WARN(this->get_logger(), "Initial FK failed for %s goal. Using origin pose.",
                  goalTypeName(goal_cfg.trajectory_type));
      center_p.setZero();
      q0 = Eigen::Quaterniond::Identity();
    }
  }

  if (!updateBaseInfo(obs)) {
    finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "Failed to determine base dimensions.");
    return false;
  }

  Eigen::Vector2d base_p = Eigen::Vector2d::Zero();
  double base_yaw = 0.0;
  if (goal_cfg.base_mode != GoalChannelMode::Disabled && !getCurrentBasePose(obs, base_p, base_yaw)) {
    finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "Current base state is unavailable.");
    return false;
  }

  std::unique_ptr<CartesianTrajectoryPlanner> ee_planner;
  PlannedCartesianTrajectory nominal_ee_traj;
  if (goal_cfg.ee_mode == GoalChannelMode::Track) {
    switch (goal_cfg.trajectory_type) {
      case GoalTrajectoryType::ScrewMove: {
        ScrewMoveParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.u_hat = goal_cfg.screw_uhat;
        params.r = goal_cfg.screw_r;
        params.theta = goal_cfg.screw_theta;
        params.expressed_in_body_frame = goal_cfg.screw_in_tool_frame;
        params.time_scaling = goal_cfg.time_scaling;
        ee_planner = std::make_unique<ScrewMovePlanner>(center_p, q0, params);
        break;
      }
      case GoalTrajectoryType::LinearMove: {
        LinearMoveParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.offset = goal_cfg.linear_offset;
        params.euler_zyx = goal_cfg.linear_euler_zyx;
        params.offset_in_body_frame = goal_cfg.linear_in_tool_frame;
        params.time_scaling = goal_cfg.time_scaling;
        ee_planner = std::make_unique<LinearMovePlanner>(center_p, q0, params);
        break;
      }
      case GoalTrajectoryType::TargetPose: {
        TargetPoseParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.time_scaling = goal_cfg.time_scaling;
        params.target_p = goal_cfg.target_p;
        params.target_q = goal_cfg.use_target_orientation ? goal_cfg.target_q : q0;
        params.use_target_orientation = goal_cfg.use_target_orientation;
        ee_planner = std::make_unique<TargetPosePlanner>(center_p, q0, params);
        break;
      }
      case GoalTrajectoryType::FigureEight: {
        FigureEightParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.amplitude = goal_cfg.figure_eight_amplitude;
        params.frequency_hz = goal_cfg.figure_eight_frequency;
        params.plane_axis = goal_cfg.figure_eight_plane_axis;
        ee_planner = std::make_unique<FigureEightPlanner>(center_p, q0, params);
        break;
      }
    }
    if (!ee_planner) {
      finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "Failed to construct an EE trajectory planner.");
      return false;
    }
    ee_planner->setStartTime(obs.time);
    nominal_ee_traj = ee_planner->plan(obs);
    if (nominal_ee_traj.empty()) {
      finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "EE planner returned an empty trajectory.");
      return false;
    }
  } else if (goal_cfg.ee_mode == GoalChannelMode::HoldCurrent) {
    nominal_ee_traj = makeConstantCartesianTrajectory(center_p, q0, obs.time, goal_cfg.duration, goal_cfg.dt);
  }

  std::unique_ptr<BaseTrajectoryPlanner> base_planner;
  PlannedBaseTrajectory nominal_base_traj;
  PlannedJointTrajectory nominal_joint_traj;
  if (goal_cfg.base_mode != GoalChannelMode::Disabled && basePoseDim_ != 3) {
    finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED",
               "Base trajectory tracking currently supports SE(2) mobile bases only.");
    return false;
  }
  if (goal_cfg.base_mode == GoalChannelMode::Track) {
    switch (goal_cfg.trajectory_type) {
      case GoalTrajectoryType::ScrewMove: {
        BaseScrewMoveParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.r = goal_cfg.base_screw_r;
        params.theta = goal_cfg.base_screw_theta;
        params.expressed_in_body_frame = goal_cfg.base_screw_in_body_frame;
        params.time_scaling = goal_cfg.time_scaling;
        base_planner = std::make_unique<BaseScrewMovePlanner>(base_p, base_yaw, params);
        break;
      }
      case GoalTrajectoryType::LinearMove: {
        BaseLinearMoveParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.offset = goal_cfg.base_linear_offset;
        params.offset_in_body_frame = goal_cfg.base_linear_in_body_frame;
        params.time_scaling = goal_cfg.time_scaling;
        base_planner = std::make_unique<BaseLinearMovePlanner>(base_p, base_yaw, params);
        break;
      }
      case GoalTrajectoryType::TargetPose: {
        BaseTargetPoseParams params;
        params.horizon_T = goal_cfg.duration;
        params.dt = goal_cfg.dt;
        params.target_pose = goal_cfg.base_target_pose;
        params.time_scaling = goal_cfg.time_scaling;
        base_planner = std::make_unique<BaseTargetPosePlanner>(base_p, base_yaw, params);
        break;
      }
      case GoalTrajectoryType::FigureEight:
        break;
    }
    if (goal_cfg.base_mode == GoalChannelMode::Track && !base_planner) {
      finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "Failed to construct a base trajectory planner.");
      return false;
    }
    if (base_planner) {
      base_planner->setStartTime(obs.time);
      nominal_base_traj = base_planner->plan(obs);
      if (nominal_base_traj.empty()) {
        finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED", "Base planner returned an empty trajectory.");
        return false;
      }
    }
  } else if (goal_cfg.base_mode == GoalChannelMode::HoldCurrent) {
    nominal_base_traj = makeConstantBaseTrajectory(base_p, base_yaw, obs.time, goal_cfg.duration, goal_cfg.dt);
  }

  if (shouldAutoHoldJointsForGoal(goal_cfg)) {
    ocs2::vector_t q_arm;
    if (getCurrentArmJoints(obs, q_arm)) {
      nominal_joint_traj = makeConstantJointTrajectory(q_arm, obs.time, goal_cfg.duration, goal_cfg.dt);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Pure base goal could not capture current arm joints. Falling back to base-only mode.");
    }
  }

  if (goal_cfg.ee_mode != GoalChannelMode::Track && goal_cfg.base_mode != GoalChannelMode::Track) {
    finishGoal(goal_handle, GoalTermination::Aborted, "ABORTED",
               "At least one channel must be in TRACK mode.");
    return false;
  }

  if (publishModeScheduleOnGoal_) {
    publishDefaultModeSchedule(modeScheduleModeForGoal(goal_cfg, !nominal_joint_traj.empty()), "GOAL_START");
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
    baseStartP_ = base_p;
    baseStartYaw_ = base_yaw;
    eePlanner_ = std::move(ee_planner);
    basePlanner_ = std::move(base_planner);
    nominalEeTraj_ = std::move(nominal_ee_traj);
    nominalBaseTraj_ = std::move(nominal_base_traj);
    nominalJointTraj_ = std::move(nominal_joint_traj);
    haveNominalEeTraj_ = !nominalEeTraj_.empty();
    haveNominalBaseTraj_ = !nominalBaseTraj_.empty();
    haveNominalJointTraj_ = !nominalJointTraj_.empty();
    eeMonitor_.reset();
    baseMonitor_.reset();
    if (haveNominalEeTraj_ && goal_cfg.ee_mode == GoalChannelMode::Track) {
      MonitorParams mp;
      mp.window = projectionWindow_;
      mp.max_backtrack = projectionMaxBacktrack_;
      mp.update_dt = 1.0 / std::max(1e-6, publishRate_);
      eeMonitor_ = std::make_unique<TrajectoryMonitor>(mp);
      eeMonitor_->setTrajectory(nominalEeTraj_);
    }
    if (haveNominalBaseTraj_ && goal_cfg.base_mode == GoalChannelMode::Track) {
      MonitorParams mp;
      mp.window = projectionWindow_;
      mp.max_backtrack = projectionMaxBacktrack_;
      mp.update_dt = 1.0 / std::max(1e-6, publishRate_);
      baseMonitor_ = std::make_unique<BaseTrajectoryMonitor>(mp);
      baseMonitor_->setTrajectory(nominalBaseTraj_);
    }
    goalInitialized_ = true;
    haveLastEeProjectionIndex_ = false;
    haveLastBaseProjectionIndex_ = false;
    lastEeProjectionIndex_ = 0;
    lastBaseProjectionIndex_ = 0;
    warnedMissingFk_ = false;
    warnedMissingBaseState_ = false;
  }

  RCLCPP_INFO(this->get_logger(), "Initialized %s goal at t=%.3f", goalTypeName(goal_cfg.trajectory_type), obs.time);
  return true;
}

bool TrajectoryTTActionServerNode::updateBaseInfo(const SystemObservation& obs) {
  if (basePoseDim_ == 3 || basePoseDim_ == 6) {
    return true;
  }
  if (interface_) {
    switch (modelInfo_.manipulatorModelType) {
      case ocs2::mobile_manipulator_mpc::ManipulatorModelType::WheelBasedMobileManipulator:
        basePoseDim_ = 3;
        break;
      case ocs2::mobile_manipulator_mpc::ManipulatorModelType::FloatingArmManipulator:
      case ocs2::mobile_manipulator_mpc::ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        basePoseDim_ = 6;
        break;
      default:
        basePoseDim_ = 0;
        break;
    }
    baseInputDim_ = modelInfo_.inputDim >= modelInfo_.armDim ? (modelInfo_.inputDim - modelInfo_.armDim) : 0;
    return basePoseDim_ > 0 || modelInfo_.manipulatorModelType == ocs2::mobile_manipulator_mpc::ManipulatorModelType::DefaultManipulator;
  }

  if (obs.state.size() >= 3) {
    basePoseDim_ = 3;
    baseInputDim_ = obs.input.size() >= 3 ? 3 : (obs.input.size() >= 2 ? 2 : 0);
    return true;
  }
  return false;
}

bool TrajectoryTTActionServerNode::shouldAutoHoldJointsForGoal(const GoalConfig& goal_cfg) const {
  const bool ee_active = goal_cfg.ee_mode != GoalChannelMode::Disabled;
  const bool base_track = goal_cfg.base_mode == GoalChannelMode::Track;
  return !ee_active && base_track;
}

bool TrajectoryTTActionServerNode::getCurrentBasePose(const SystemObservation& obs,
                                                      Eigen::Vector2d& p,
                                                      double& yaw) const {
  if (obs.state.size() < 3) {
    return false;
  }
  p.x() = obs.state(0);
  p.y() = obs.state(1);
  yaw = obs.state(2);
  return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(yaw);
}

bool TrajectoryTTActionServerNode::getCurrentArmJoints(const SystemObservation& obs, ocs2::vector_t& q_arm) const {
  if (!interface_ || modelInfo_.armDim == 0 || modelInfo_.stateDim < modelInfo_.armDim) {
    return false;
  }

  const std::size_t base_state_dim = modelInfo_.stateDim - modelInfo_.armDim;
  if (obs.state.size() < static_cast<Eigen::Index>(base_state_dim + modelInfo_.armDim)) {
    return false;
  }

  q_arm = obs.state.segment(static_cast<Eigen::Index>(base_state_dim), static_cast<Eigen::Index>(modelInfo_.armDim));
  return q_arm.allFinite();
}

TrajectoryTTActionServerNode::MonitorSnapshot TrajectoryTTActionServerNode::computeMonitorSnapshot(
    const GoalConfig& goal_cfg,
    const SystemObservation& obs,
    const PlannedCartesianTrajectory* ee_traj,
    const PlannedBaseTrajectory* base_traj) {
  MonitorSnapshot snapshot;
  snapshot.ee.status = channelModeName(goal_cfg.ee_mode);
  snapshot.base.status = channelModeName(goal_cfg.base_mode);

  const bool ee_track = goal_cfg.ee_mode == GoalChannelMode::Track && ee_traj && !ee_traj->empty() && eeMonitor_;
  if (ee_track) {
    Eigen::Vector3d p_now = centerP_;
    Eigen::Quaterniond q_now = q0_;
    bool have_fk = false;
    if (pin_ && mapping_) {
      std::lock_guard<std::mutex> lock(pinMtx_);
      have_fk = computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p_now, q_now);
    }
    if (have_fk) {
      const TrackingMetrics met = eeMonitor_->update(obs.time, p_now, q_now);
      snapshot.ee.status = toString(met.status);
      snapshot.ee.progress = met.progress;
      snapshot.ee.pos_err = met.pos_err;
      snapshot.ee.ori_err_deg = met.ori_err_deg;
      snapshot.ee.lag_sec = met.lag_sec;
      snapshot.ee.closest_index = met.closest_index;
      snapshot.ee.valid = true;
    }
  }

  const bool base_track = goal_cfg.base_mode == GoalChannelMode::Track && base_traj && !base_traj->empty() && baseMonitor_;
  if (base_track) {
    Eigen::Vector2d p_now = Eigen::Vector2d::Zero();
    double yaw_now = 0.0;
    if (getCurrentBasePose(obs, p_now, yaw_now)) {
      const TrackingMetrics met = baseMonitor_->update(obs.time, p_now, yaw_now);
      snapshot.base.status = toString(met.status);
      snapshot.base.progress = met.progress;
      snapshot.base.pos_err = met.pos_err;
      snapshot.base.ori_err_deg = met.ori_err_deg;
      snapshot.base.lag_sec = met.lag_sec;
      snapshot.base.closest_index = met.closest_index;
      snapshot.base.valid = true;
    }
  }

  std::vector<TrackingStatus> active_statuses;
  if (snapshot.ee.valid) {
    if (snapshot.ee.status == "FINISHED") active_statuses.push_back(TrackingStatus::FINISHED);
    else if (snapshot.ee.status == "DIVERGED") active_statuses.push_back(TrackingStatus::DIVERGED);
    else if (snapshot.ee.status == "ON_TRACK") active_statuses.push_back(TrackingStatus::ON_TRACK);
    else if (snapshot.ee.status == "LAGGING") active_statuses.push_back(TrackingStatus::LAGGING);
    else active_statuses.push_back(TrackingStatus::NOT_READY);
  }
  if (snapshot.base.valid) {
    if (snapshot.base.status == "FINISHED") active_statuses.push_back(TrackingStatus::FINISHED);
    else if (snapshot.base.status == "DIVERGED") active_statuses.push_back(TrackingStatus::DIVERGED);
    else if (snapshot.base.status == "ON_TRACK") active_statuses.push_back(TrackingStatus::ON_TRACK);
    else if (snapshot.base.status == "LAGGING") active_statuses.push_back(TrackingStatus::LAGGING);
    else active_statuses.push_back(TrackingStatus::NOT_READY);
  }

  if (active_statuses.empty()) {
    snapshot.status = "NOT_READY";
    return snapshot;
  }

  if (std::any_of(active_statuses.begin(), active_statuses.end(),
                  [](TrackingStatus s) { return s == TrackingStatus::DIVERGED; })) {
    snapshot.status = "DIVERGED";
  } else if (std::all_of(active_statuses.begin(), active_statuses.end(),
                         [](TrackingStatus s) { return s == TrackingStatus::FINISHED; })) {
    snapshot.status = "FINISHED";
  } else if (std::all_of(active_statuses.begin(), active_statuses.end(),
                         [](TrackingStatus s) { return s == TrackingStatus::ON_TRACK || s == TrackingStatus::FINISHED; })) {
    snapshot.status = "ON_TRACK";
  } else {
    snapshot.status = "LAGGING";
  }

  const auto& primary = snapshot.ee.valid ? snapshot.ee : snapshot.base;
  snapshot.progress = primary.progress;
  snapshot.pos_err = primary.pos_err;
  snapshot.ori_err_deg = primary.ori_err_deg;
  snapshot.lag_sec = primary.lag_sec;
  snapshot.closest_index = primary.closest_index;
  snapshot.valid = primary.valid;
  return snapshot;
}

void TrajectoryTTActionServerNode::publishWaitingFeedback(const ActiveGoalPtr& goal_handle, const std::string& status) {
  const rclcpp::Time now = this->now();
  {
    std::lock_guard<std::mutex> lock(waitingFeedbackMtx_);
    if ((now - lastWaitingFeedbackTime_).seconds() < 0.5) {
      return;
    }
    lastWaitingFeedbackTime_ = now;
  }

  goal_handle->publishWaitingFeedback(status);
}

void TrajectoryTTActionServerNode::handleCanceledGoal(const ActiveGoalPtr& goal_handle) {
  const GoalConfig goal_cfg = goal_handle->goalConfig();
  publishHoldTrajectory("CANCELED");
  startZeroBaseCmdBurst("CANCELED");
  requestSwitchMpcController(/*activate=*/false, "CANCELED");
  finishGoal(goal_handle, GoalTermination::Canceled, "CANCELED", goalMessage(goal_cfg, "canceled."));
}

void TrajectoryTTActionServerNode::finishGoal(const ActiveGoalPtr& goal_handle,
                                              GoalTermination termination,
                                              const std::string& final_status,
                                              const std::string& message) {
  MonitorSnapshot snapshot;
  GoalConfig goal_cfg;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_ || activeGoal_ != goal_handle) return;
    snapshot = monitorSnapshot_;
    goal_cfg = goal_handle->goalConfig();
    clearActiveGoalLocked();
  }

  goal_handle->finish(termination, final_status, message, snapshot);

  switch (termination) {
    case GoalTermination::Succeeded:
      RCLCPP_INFO(this->get_logger(), "%s goal succeeded.", goalTypeName(goal_cfg.trajectory_type));
      break;
    case GoalTermination::Aborted:
      RCLCPP_WARN(this->get_logger(), "%s goal aborted.", goalTypeName(goal_cfg.trajectory_type));
      break;
    case GoalTermination::Canceled:
      RCLCPP_INFO(this->get_logger(), "%s goal canceled.", goalTypeName(goal_cfg.trajectory_type));
      break;
  }
}

void TrajectoryTTActionServerNode::clearActiveGoalLocked() {
  activeGoal_.reset();
  goalInitialized_ = false;
  goalPublishingStarted_ = false;
  cancelRequested_ = false;
  eePlanner_.reset();
  basePlanner_.reset();
  eeMonitor_.reset();
  baseMonitor_.reset();
  nominalEeTraj_ = PlannedCartesianTrajectory{};
  nominalBaseTraj_ = PlannedBaseTrajectory{};
  nominalJointTraj_ = PlannedJointTrajectory{};
  haveNominalEeTraj_ = false;
  haveNominalBaseTraj_ = false;
  haveNominalJointTraj_ = false;
  haveLastEeProjectionIndex_ = false;
  haveLastBaseProjectionIndex_ = false;
  lastEeProjectionIndex_ = 0;
  lastBaseProjectionIndex_ = 0;
  warnedMissingFk_ = false;
  warnedMissingBaseState_ = false;
  centerP_ = Eigen::Vector3d::Zero();
  q0_ = Eigen::Quaterniond::Identity();
  baseStartP_.setZero();
  baseStartYaw_ = 0.0;
}

bool TrajectoryTTActionServerNode::getDeltaPose(Eigen::Vector3d& dp, Eigen::Quaterniond& dq, const rclcpp::Time& now) const {
  std::lock_guard<std::mutex> lock(deltaPoseMtx_);
  if (!haveDeltaPose_) return false;
  if (deltaPoseTimeoutSec_ > 0.0) {
    const double age = (now - deltaPoseStamp_).seconds();
    if (age > deltaPoseTimeoutSec_) return false;
  }
  dp = deltaP_;
  dq = deltaQ_;
  return true;
}

void TrajectoryTTActionServerNode::applyDeltaPoseToTrajectory(PlannedCartesianTrajectory& traj) const {
  if (traj.empty()) return;

  Eigen::Vector3d dp;
  Eigen::Quaterniond dq;
  if (!getDeltaPose(dp, dq, this->now())) return;

  if (deltaPoseInToolFrame_) {
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Eigen::Quaterniond q_nom = traj.quat[i].normalized();
      traj.position[i] = traj.position[i] + q_nom.toRotationMatrix() * dp;
      traj.quat[i] = (q_nom * dq).normalized();
    }
  } else {
    // Delta is expressed in world frame.
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Eigen::Quaterniond q_nom = traj.quat[i].normalized();
      traj.position[i] = traj.position[i] + dp;
      traj.quat[i] = (dq * q_nom).normalized();
    }
  }
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

  GoalConfig goal_cfg;
  bool joint_hold_active = false;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_) return;
    goal_cfg = activeGoal_->goalConfig();
    joint_hold_active = haveNominalJointTraj_;
  }

  if (goal_cfg.ee_mode != GoalChannelMode::Disabled) {
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

    const PlannedCartesianTrajectory hold = makeConstantCartesianTrajectory(p_hold, q_hold, obs.time, 1.0, 1.0);
    trajPub_->publishEeTarget(hold, obs);
  }

  if (goal_cfg.base_mode != GoalChannelMode::Disabled && updateBaseInfo(obs)) {
    Eigen::Vector2d base_p = baseStartP_;
    double base_yaw = baseStartYaw_;
    if (getCurrentBasePose(obs, base_p, base_yaw)) {
      const PlannedBaseTrajectory hold = makeConstantBaseTrajectory(base_p, base_yaw, obs.time, 1.0, 1.0);
      trajPub_->publishBaseTarget(hold, basePoseDim_, baseInputDim_);
    }
  }

  if (joint_hold_active) {
    ocs2::vector_t q_arm;
    if (getCurrentArmJoints(obs, q_arm)) {
      const PlannedJointTrajectory hold = makeConstantJointTrajectory(q_arm, obs.time, 1.0, 1.0);
      trajPub_->publishJointTarget(hold);
    }
  }

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

int TrajectoryTTActionServerNode::modeScheduleModeForGoal(const GoalConfig& goal_cfg, bool joint_hold_active) const {
  const bool ee_active = goal_cfg.ee_mode != GoalChannelMode::Disabled;
  const bool base_active = goal_cfg.base_mode != GoalChannelMode::Disabled;
  if (ee_active && base_active) return 4;
  if (base_active) return joint_hold_active ? 5 : 3;
  if (ee_active) return modeScheduleMode_;
  return modeScheduleMode_;
}

void TrajectoryTTActionServerNode::publishDefaultModeSchedule(int mode, const std::string& reason) {
  if (!publishModeScheduleOnGoal_) return;
  if (!modeSchedulePub_) return;

  const int clamped_mode = std::clamp(mode, -128, 127);
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

void TrajectoryTTActionServerNode::publishBaseViz(const PlannedBaseTrajectory& traj, double now_time) {
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
    marker.ns = "base_target_state";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.850f;
    marker.color.g = 0.325f;
    marker.color.b = 0.098f;
    marker.color.a = (i < passed) ? 0.15f : 1.0f;
    marker.pose.position.x = traj.position[i].x();
    marker.pose.position.y = traj.position[i].y();
    marker.pose.position.z = 0.02;
    marker.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(yawToQuaternion(traj.yaw[i]));
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
