#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/MobileManipulatorPinocchioMapping.h>
#include <mobile_manipulator_mpc/ManipulatorModelInfo.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "mpc_cartesian_planner/action/execute_screw_move.hpp"
#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"
#include "mpc_cartesian_planner/trajectory_publisher.h"
#include "trajectory_tt_action_server_internal.h"

namespace mpc_cartesian_planner {

class TrajectoryTTActionServerNode final : public rclcpp::Node {
 public:
  using ExecuteScrewMove = mpc_cartesian_planner::action::ExecuteScrewMove;
  using GoalHandleExecuteScrewMove = rclcpp_action::ServerGoalHandle<ExecuteScrewMove>;
  using GoalConfig = trajectory_tt_action_server_internal::GoalConfig;
  using GoalTermination = trajectory_tt_action_server_internal::GoalTermination;
  using MonitorSnapshot = trajectory_tt_action_server_internal::MonitorSnapshot;

  explicit TrajectoryTTActionServerNode(const rclcpp::NodeOptions& options);

 private:
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteScrewMove::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleExecuteScrewMove> goal_handle);
  bool validateGoal(const ExecuteScrewMove::Goal& goal, std::string& reason) const;
  GoalConfig goalToConfig(const ExecuteScrewMove::Goal& goal) const;
  void onTimer();
  bool initializeGoalIfNeeded(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle,
                              const GoalConfig& goal_cfg,
                              const ocs2::SystemObservation& obs);
  void onTrackingStatus(const std::string& status);
  void onTrackingMetric(const std_msgs::msg::Float64MultiArray& msg);
  void publishWaitingFeedback(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle, const std::string& status);
  void handleCanceledGoal(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle);
  void finishGoal(const std::shared_ptr<GoalHandleExecuteScrewMove>& goal_handle,
                  GoalTermination termination,
                  const std::string& final_status,
                  const std::string& message);
  void clearActiveGoalLocked();
  static void populateFeedback(ExecuteScrewMove::Feedback& feedback, const MonitorSnapshot& snapshot);
  static void populateResult(ExecuteScrewMove::Result& result,
                             bool success,
                             const std::string& final_status,
                             const std::string& message,
                             const MonitorSnapshot& snapshot);
  static uint32_t saturateClosestIndex(std::size_t index);
  bool getDeltaPose(Eigen::Vector3d& dp, Eigen::Quaterniond& dq, const rclcpp::Time& now) const;
  void applyDeltaPoseToTrajectory(PlannedCartesianTrajectory& traj) const;
  void publishHoldTrajectory(const std::string& reason);
  void startZeroBaseCmdBurst(const std::string& reason);
  void requestSwitchMpcController(bool activate, const std::string& reason);
  void publishDefaultModeSchedule(const std::string& reason);
  void publishCommandTf(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
  void publishViz(const PlannedCartesianTrajectory& traj, double now_time);

  std::string taskFile_;
  std::string urdfFile_;
  std::string libFolder_;
  std::string robotName_{"mobile_manipulator"};
  std::string actionName_;
  std::string globalFrame_{"world"};

  double publishRate_{20.0};
  std::string referenceTiming_{"time"};
  bool useSpatialProjectionRetime_{false};
  int projectionWindow_{20};
  int projectionMaxBacktrack_{5};
  bool publishViz_{true};
  bool publishTF_{true};
  std::string commandFrameId_{"command"};
  std::string deltaPoseTopic_{"/delta_pose"};
  bool deltaPoseInToolFrame_{true};
  double deltaPoseTimeoutSec_{0.0};
  bool interveneHoldOnDivergedOrFinished_{false};
  bool publishZeroBaseCmdOnIntervention_{false};
  std::string baseCmdTopic_{"/cmd_vel"};
  int zeroBaseCmdBurstCount_{10};
  double zeroBaseCmdBurstRate_{50.0};
  int zeroBaseCmdRemaining_{0};
  bool switchMpcControllerOnIntervention_{false};
  bool activateMpcControllerOnGoal_{false};
  std::string controllerManagerSwitchService_{"/controller_manager/switch_controller"};
  std::string mpcControllerName_{"mpc_controller"};
  bool publishModeScheduleOnGoal_{true};
  int modeScheduleMode_{1};

  std::unique_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::PinocchioInterface> pin_;
  ocs2::mobile_manipulator_mpc::ManipulatorModelInfo modelInfo_;
  std::unique_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorPinocchioMapping> mapping_;
  std::size_t eeFrameId_{0};

  std::unique_ptr<TrajectoryPublisher> trajPub_;
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr modeSchedulePub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr baseCmdPub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trackingStatusSub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trackingMetricSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr deltaPoseSub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switchControllerClient_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr zeroBaseCmdTimer_;
  rclcpp_action::Server<ExecuteScrewMove>::SharedPtr actionServer_;

  std::mutex obsMtx_;
  ocs2::SystemObservation latestObs_;
  bool haveObs_{false};

  std::mutex stateMtx_;
  std::shared_ptr<GoalHandleExecuteScrewMove> activeGoal_;
  GoalConfig activeGoalConfig_;
  bool goalInitialized_{false};
  bool goalPublishingStarted_{false};
  bool cancelRequested_{false};
  std::unique_ptr<CartesianTrajectoryPlanner> planner_;
  PlannedCartesianTrajectory nominalTraj_;
  bool haveNominalTraj_{false};
  double startTime_{0.0};
  Eigen::Vector3d centerP_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q0_{Eigen::Quaterniond::Identity()};
  std::size_t lastProjectionIndex_{0};
  bool haveLastProjectionIndex_{false};
  bool warnedMissingFk_{false};
  MonitorSnapshot monitorSnapshot_;

  std::mutex pinMtx_;
  mutable std::mutex deltaPoseMtx_;
  bool haveDeltaPose_{false};
  Eigen::Vector3d deltaP_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond deltaQ_{Eigen::Quaterniond::Identity()};
  rclcpp::Time deltaPoseStamp_{0, 0, RCL_ROS_TIME};
  std::mutex waitingFeedbackMtx_;
  rclcpp::Time lastWaitingFeedbackTime_{0, 0, RCL_ROS_TIME};
};

}  // namespace mpc_cartesian_planner
