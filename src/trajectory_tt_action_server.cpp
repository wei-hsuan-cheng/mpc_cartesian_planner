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
  this->declare_parameter<std::string>("screwActionName", std::string(""));
  this->declare_parameter<std::string>("linearActionName", std::string(""));
  this->declare_parameter<std::string>("targetPoseActionName", std::string(""));
  this->declare_parameter<std::string>("figureEightActionName", std::string(""));

  this->declare_parameter<double>("publishRate", 20.0);
  this->declare_parameter<std::string>("referenceTiming", std::string("time"));
  this->declare_parameter<int>("projectionWindow", 20);
  this->declare_parameter<int>("projectionMaxBacktrack", 5);
  this->declare_parameter<std::string>("trajectoryGlobalFrame", std::string("world"));
  this->declare_parameter<bool>("publishViz", true);
  this->declare_parameter<bool>("publishTF", true);
  this->declare_parameter<std::string>("commandFrameId", std::string("command"));
  this->declare_parameter<std::string>("deltaPoseTopic", std::string("/delta_pose"));
  this->declare_parameter<bool>("deltaPoseInToolFrame", true);
  this->declare_parameter<double>("deltaPoseTimeout", 0.0);
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

  const std::string legacyActionName = this->get_parameter("actionName").as_string();
  screwActionName_ = this->get_parameter("screwActionName").as_string();
  linearActionName_ = this->get_parameter("linearActionName").as_string();
  targetPoseActionName_ = this->get_parameter("targetPoseActionName").as_string();
  figureEightActionName_ = this->get_parameter("figureEightActionName").as_string();
  if (screwActionName_.empty()) {
    screwActionName_ = legacyActionName.empty() ? robotName_ + "/trajectory_tracking/execute_screw_move" : legacyActionName;
  }
  if (linearActionName_.empty()) {
    linearActionName_ = robotName_ + "/trajectory_tracking/execute_linear_move";
  }
  if (targetPoseActionName_.empty()) {
    targetPoseActionName_ = robotName_ + "/trajectory_tracking/execute_target_pose";
  }
  if (figureEightActionName_.empty()) {
    figureEightActionName_ = robotName_ + "/trajectory_tracking/execute_figure_eight";
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
  deltaPoseTopic_ = this->get_parameter("deltaPoseTopic").as_string();
  deltaPoseInToolFrame_ = this->get_parameter("deltaPoseInToolFrame").as_bool();
  deltaPoseTimeoutSec_ = this->get_parameter("deltaPoseTimeout").as_double();
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

  if (!deltaPoseTopic_.empty()) {
    deltaPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        deltaPoseTopic_, rclcpp::QoS(1).best_effort(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          const Eigen::Vector3d dp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
          const Eigen::Quaterniond dq(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                      msg->pose.orientation.z);
          const Eigen::Quaterniond dq_norm = (dq.norm() < 1e-12) ? Eigen::Quaterniond::Identity() : dq.normalized();

          rclcpp::Time stamp(msg->header.stamp);
          if (stamp.nanoseconds() == 0) {
            stamp = this->now();
          }

          std::lock_guard<std::mutex> lock(deltaPoseMtx_);
          deltaP_ = dp;
          deltaQ_ = dq_norm;
          deltaPoseStamp_ = stamp;
          haveDeltaPose_ = true;
        });

    RCLCPP_INFO(this->get_logger(),
                "Delta pose subscriber enabled: topic=%s (in_tool_frame=%d timeout=%.3f sec)",
                deltaPoseTopic_.c_str(), static_cast<int>(deltaPoseInToolFrame_), deltaPoseTimeoutSec_);
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

  screwActionServer_ = rclcpp_action::create_server<ExecuteScrewMove>(
      this,
      screwActionName_,
      std::bind(&TrajectoryTTActionServerNode::handleScrewGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTTActionServerNode::handleScrewCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryTTActionServerNode::handleScrewAccepted, this, std::placeholders::_1));
  linearActionServer_ = rclcpp_action::create_server<ExecuteLinearMove>(
      this,
      linearActionName_,
      std::bind(&TrajectoryTTActionServerNode::handleLinearGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTTActionServerNode::handleLinearCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryTTActionServerNode::handleLinearAccepted, this, std::placeholders::_1));
  targetPoseActionServer_ = rclcpp_action::create_server<ExecuteTargetPose>(
      this,
      targetPoseActionName_,
      std::bind(&TrajectoryTTActionServerNode::handleTargetPoseGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTTActionServerNode::handleTargetPoseCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryTTActionServerNode::handleTargetPoseAccepted, this, std::placeholders::_1));
  figureEightActionServer_ = rclcpp_action::create_server<ExecuteFigureEight>(
      this,
      figureEightActionName_,
      std::bind(&TrajectoryTTActionServerNode::handleFigureEightGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryTTActionServerNode::handleFigureEightCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryTTActionServerNode::handleFigureEightAccepted, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, publishRate_));
  timer_ = this->create_wall_timer(period, std::bind(&TrajectoryTTActionServerNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(),
              "TT action servers ready on %s, %s, %s, %s",
              screwActionName_.c_str(),
              linearActionName_.c_str(),
              targetPoseActionName_.c_str(),
              figureEightActionName_.c_str());
}

void TrajectoryTTActionServerNode::onTimer() {
  ActiveGoalPtr goal_handle;
  GoalConfig goal_cfg;
  bool cancel_requested = false;
  {
    std::lock_guard<std::mutex> lock(stateMtx_);
    if (!activeGoal_) return;
    goal_handle = activeGoal_;
    goal_cfg = activeGoal_->goalConfig();
    cancel_requested = cancelRequested_ || activeGoal_->isCanceling();
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

  PlannedCartesianTrajectory traj_with_delta = nominal_traj;
  applyDeltaPoseToTrajectory(traj_with_delta);

  const PlannedCartesianTrajectory* traj_to_pub = &traj_with_delta;
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
            traj_with_delta, p_now, lastProjectionIndex_, haveLastProjectionIndex_, projectionWindow_, projectionMaxBacktrack_);
        lastProjectionIndex_ = closest_idx;
        haveLastProjectionIndex_ = true;
      }

      const double shift = obs.time - traj_with_delta.time[closest_idx];
      if (std::isfinite(shift)) {
        retimed_traj = traj_with_delta;
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
