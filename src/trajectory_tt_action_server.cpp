/**
 * trajectory_tt_action_server
 *
 * Screw-move action server for Cartesian TT. It publishes the same
 * <robotName>_mpc_target reference stream consumed by MPC and uses the
 * existing trajectory progress monitor for feedback/result status.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "mpc_cartesian_planner/trajectory_tt_action_server.h"
#include "trajectory_tt_action_server_impl.h"

using ocs2::SystemObservation;

namespace mpc_cartesian_planner {

using trajectory_tt_action_server_internal::closestWaypointIndexWindow;
using trajectory_tt_action_server_internal::computeEePose;
using trajectory_tt_action_server_internal::sampleTrajectoryAtTime;
using trajectory_tt_action_server_internal::toLowerCopy;

TrajectoryTTActionServerNode::TrajectoryTTActionServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("trajectory_tt_action_server", options) {
  this->declare_parameter<std::string>("taskFile", "");
  this->declare_parameter<std::string>("urdfFile", "");
  this->declare_parameter<std::string>("libFolder", "");
  this->declare_parameter<std::string>("robotName", std::string("mobile_manipulator"));
  this->declare_parameter<std::string>("actionName", std::string(""));

  this->declare_parameter<double>("publishRate", 20.0);
  this->declare_parameter<std::string>("referenceTiming", std::string("time"));
  this->declare_parameter<int>("projectionWindow", 20);
  this->declare_parameter<int>("projectionMaxBacktrack", 5);
  this->declare_parameter<std::string>("trajectoryGlobalFrame", std::string("world"));
  this->declare_parameter<bool>("publishViz", true);
  this->declare_parameter<bool>("publishTF", true);
  this->declare_parameter<std::string>("commandFrameId", std::string("command"));
  this->declare_parameter<bool>("interveneHoldOnDivergedOrFinished", false);
  this->declare_parameter<bool>("publishZeroBaseCmdOnIntervention", false);
  this->declare_parameter<std::string>("baseCmdTopic", std::string("/cmd_vel"));
  this->declare_parameter<int>("zeroBaseCmdBurstCount", 10);
  this->declare_parameter<double>("zeroBaseCmdBurstRate", 50.0);
  this->declare_parameter<bool>("switchMpcControllerOnIntervention", false);
  this->declare_parameter<bool>("activateMpcControllerOnGoal", false);
  this->declare_parameter<std::string>("controllerManagerSwitchService", std::string("/controller_manager/switch_controller"));
  this->declare_parameter<std::string>("mpcControllerName", std::string("mpc_controller"));
  this->declare_parameter<bool>("publishModeScheduleOnGoal", true);
  this->declare_parameter<int>("modeScheduleMode", 1);

  taskFile_ = this->get_parameter("taskFile").as_string();
  urdfFile_ = this->get_parameter("urdfFile").as_string();
  libFolder_ = this->get_parameter("libFolder").as_string();
  robotName_ = this->get_parameter("robotName").as_string();

  actionName_ = this->get_parameter("actionName").as_string();
  if (actionName_.empty()) {
    actionName_ = robotName_ + "/trajectory_tracking/execute_screw_move";
  }

  publishRate_ = this->get_parameter("publishRate").as_double();
  referenceTiming_ = this->get_parameter("referenceTiming").as_string();
  projectionWindow_ = static_cast<int>(this->get_parameter("projectionWindow").as_int());
  projectionMaxBacktrack_ = static_cast<int>(this->get_parameter("projectionMaxBacktrack").as_int());
  projectionWindow_ = std::max(1, projectionWindow_);
  projectionMaxBacktrack_ = std::max(0, projectionMaxBacktrack_);
  globalFrame_ = this->get_parameter("trajectoryGlobalFrame").as_string();
  publishViz_ = this->get_parameter("publishViz").as_bool();
  publishTF_ = this->get_parameter("publishTF").as_bool();
  commandFrameId_ = this->get_parameter("commandFrameId").as_string();
  if (commandFrameId_.empty()) commandFrameId_ = "command";
  interveneHoldOnDivergedOrFinished_ = this->get_parameter("interveneHoldOnDivergedOrFinished").as_bool();
  publishZeroBaseCmdOnIntervention_ = this->get_parameter("publishZeroBaseCmdOnIntervention").as_bool();
  baseCmdTopic_ = this->get_parameter("baseCmdTopic").as_string();
  zeroBaseCmdBurstCount_ = static_cast<int>(this->get_parameter("zeroBaseCmdBurstCount").as_int());
  zeroBaseCmdBurstRate_ = this->get_parameter("zeroBaseCmdBurstRate").as_double();
  switchMpcControllerOnIntervention_ = this->get_parameter("switchMpcControllerOnIntervention").as_bool();
  activateMpcControllerOnGoal_ = this->get_parameter("activateMpcControllerOnGoal").as_bool();
  controllerManagerSwitchService_ = this->get_parameter("controllerManagerSwitchService").as_string();
  mpcControllerName_ = this->get_parameter("mpcControllerName").as_string();
  publishModeScheduleOnGoal_ = this->get_parameter("publishModeScheduleOnGoal").as_bool();
  modeScheduleMode_ = static_cast<int>(this->get_parameter("modeScheduleMode").as_int());

  {
    const std::string mode = toLowerCopy(referenceTiming_);
    if (mode == "time" || mode == "time_indexed" || mode == "time-indexed") {
      useSpatialProjectionRetime_ = false;
    } else if (mode == "spatial_projection" || mode == "spatial" || mode == "path" || mode == "path_based" ||
               mode == "path-based" || mode == "pathbased") {
      useSpatialProjectionRetime_ = true;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown referenceTiming='%s'. Falling back to 'time'.",
                  referenceTiming_.c_str());
      useSpatialProjectionRetime_ = false;
    }
  }

  if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
    try {
      interface_ =
          std::make_unique<ocs2::mobile_manipulator_mpc::MobileManipulatorInterface>(taskFile_, libFolder_, urdfFile_);
      pin_ = std::make_unique<ocs2::PinocchioInterface>(interface_->getPinocchioInterface());
      modelInfo_ = interface_->getManipulatorModelInfo();
      mapping_ = std::make_unique<ocs2::mobile_manipulator_mpc::MobileManipulatorPinocchioMapping>(modelInfo_);
      eeFrameId_ = pin_->getModel().getFrameId(modelInfo_.eeFrame);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
                "taskFile/urdfFile/libFolder not set. Goals will fall back to origin pose if FK is unavailable.");
  }

  trajPub_ = std::make_unique<TrajectoryPublisher>(*this, robotName_, /*latch=*/false);
  modeSchedulePub_ = this->create_publisher<ocs2_msgs::msg::ModeSchedule>(
      robotName_ + std::string("_mode_schedule"), rclcpp::QoS(1).reliable());

  if (publishViz_) {
    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/mobile_manipulator/targetStateTrajectory", 1);
    posePub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mobile_manipulator/targetPoseTrajectory", 1);
  }
  if (publishTF_) {
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  if ((switchMpcControllerOnIntervention_ || activateMpcControllerOnGoal_) && !controllerManagerSwitchService_.empty()) {
    switchControllerClient_ =
        this->create_client<controller_manager_msgs::srv::SwitchController>(controllerManagerSwitchService_);
  }

  obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
      robotName_ + std::string("_mpc_observation"), rclcpp::QoS(1).reliable(),
      [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(obsMtx_);
        latestObs_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
        haveObs_ = true;
      });

  trackingStatusSub_ = this->create_subscription<std_msgs::msg::String>(
      robotName_ + "/trajectory_tracking/tracking_status", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::ConstSharedPtr msg) { onTrackingStatus(msg->data); });

  trackingMetricSub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      robotName_ + "/trajectory_tracking/tracking_metric", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) { onTrackingMetric(*msg); });

  actionServer_ = rclcpp_action::create_server<ExecuteScrewMove>(
      this,
      actionName_,
      std::bind(&TrajectoryTTActionServerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTTActionServerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryTTActionServerNode::handleAccepted, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, publishRate_));
  timer_ = this->create_wall_timer(period, std::bind(&TrajectoryTTActionServerNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "TT action server ready on %s", actionName_.c_str());
}

rclcpp_action::GoalResponse TrajectoryTTActionServerNode::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteScrewMove::Goal> goal) {
  std::string reject_reason;
  if (!validateGoal(*goal, reject_reason)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting screw move goal: %s", reject_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (activeGoal_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting screw move goal: another goal is already active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryTTActionServerNode::handleCancel(
    const std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle) {
  std::lock_guard<std::mutex> lock(stateMtx_);
  if (!activeGoal_ || activeGoal_ != goal_handle) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  cancelRequested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryTTActionServerNode::handleAccepted(const std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle) {
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    activeGoal_ = goal_handle;
    activeGoalConfig_ = goalToConfig(*goal_handle->get_goal());
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
    monitorSnapshot_ = MonitorSnapshot{};
  }
  {
    std::lock_guard<std::mutex> lock(waitingFeedbackMtx_);
    lastWaitingFeedbackTime_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  publishWaitingFeedback(goal_handle, "WAITING_FOR_OBSERVATION");
  RCLCPP_INFO(this->get_logger(),
              "Accepted screw move goal: duration=%.3f dt=%.3f theta=%.3f tool_frame=%d",
              goal_handle->get_goal()->duration,
              goal_handle->get_goal()->dt,
              goal_handle->get_goal()->screw_theta,
              static_cast<int>(goal_handle->get_goal()->screw_in_tool_frame));
}

void TrajectoryTTActionServerNode::onTimer() {
  std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle;
  GoalConfig goal_cfg;
  bool cancel_requested = false;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_) return;
    goal_handle = activeGoal_;
    goal_cfg = activeGoalConfig_;
    cancel_requested = cancelRequested_ || activeGoal_->is_canceling();
  }

  if (cancel_requested) {
    handleCanceledGoal(goal_handle);
    return;
  }

  SystemObservation obs;
  {
    std::lock_guard<std::mutex> lock(obsMtx_);
    if (!haveObs_) {
      publishWaitingFeedback(goal_handle, "WAITING_FOR_OBSERVATION");
      return;
    }
    obs = latestObs_;
  }

  if (!initializeGoalIfNeeded(goal_handle, goal_cfg, obs)) {
    return;
  }

  PlannedCartesianTrajectory nominal_traj;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_ || activeGoal_ != goal_handle || !haveNominalTraj_) return;
    nominal_traj = nominalTraj_;
  }

  const PlannedCartesianTrajectory* traj_to_pub = &nominal_traj;
  PlannedCartesianTrajectory retimed_traj;
  if (useSpatialProjectionRetime_) {
    Eigen::Vector3d p_now;
    Eigen::Quaterniond q_now;
    bool have_fk = false;
    if (pin_ && mapping_) {
      std::lock_guard<std::mutex> lock(pinMtx_);
      have_fk = computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p_now, q_now);
    }
    if (!have_fk) {
      bool warn_now = false;
      {
        std::lock_guard<std::mutex> lock(stateMtx_);
        if (!warnedMissingFk_) {
          warnedMissingFk_ = true;
          warn_now = true;
        }
      }
      if (warn_now) {
        RCLCPP_WARN(this->get_logger(),
                    "referenceTiming=spatial_projection requested but Pinocchio FK is not available. "
                    "Falling back to time-indexed reference for this goal.");
      }
    } else {
      std::size_t closest_idx = 0;
      {
        std::lock_guard<std::mutex> lock(stateMtx_);
        closest_idx = closestWaypointIndexWindow(
            nominal_traj, p_now, lastProjectionIndex_, haveLastProjectionIndex_, projectionWindow_, projectionMaxBacktrack_);
        lastProjectionIndex_ = closest_idx;
        haveLastProjectionIndex_ = true;
      }

      const double shift = obs.time - nominal_traj.time[closest_idx];
      if (std::isfinite(shift)) {
        retimed_traj = nominal_traj;
        for (double& t : retimed_traj.time) {
          t += shift;
        }
        traj_to_pub = &retimed_traj;
      }
    }
  }

  if (publishTF_ && tfBroadcaster_) {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    if (sampleTrajectoryAtTime(*traj_to_pub, obs.time, p, q)) {
      publishCommandTf(p, q);
    }
  }

  trajPub_->publishEeTarget(*traj_to_pub, obs);
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (activeGoal_ == goal_handle) {
      goalPublishingStarted_ = true;
    }
  }

  if (publishViz_ && markerPub_ && posePub_) {
    publishViz(*traj_to_pub, obs.time);
  }
}

std::shared_ptr<rclcpp::Node> createTrajectoryTTActionServerNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<TrajectoryTTActionServerNode>(options);
}

}  // namespace mpc_cartesian_planner
