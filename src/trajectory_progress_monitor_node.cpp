/**
 * trajectory_progress_monitor_node
 *
 * Window-search progress monitor for Cartesian TT.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/MobileManipulatorPinocchioMapping.h>
#include <mobile_manipulator_mpc/ManipulatorModelInfo.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "mpc_cartesian_planner/trajectory_monitor.h"

using ocs2::SystemObservation;

namespace mpc_cartesian_planner {

namespace {

// ===== FK helper ============================================================
inline bool computeEePose(ocs2::PinocchioInterface& pin,
                          const ocs2::mobile_manipulator_mpc::MobileManipulatorPinocchioMapping& mapping,
                          std::size_t eeFrameId,
                          const ocs2::vector_t& state,
                          Eigen::Vector3d& p_out,
                          Eigen::Quaterniond& q_out) {
  const auto qPinRaw = mapping.getPinocchioJointPosition(state);

  const auto& model = pin.getModel();
  auto& data = pin.getData();

  pinocchio::Model::ConfigVectorType q = pinocchio::neutral(model);
  q = qPinRaw;

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  if (eeFrameId >= data.oMf.size()) {
    return false;
  }

  const auto& oMf = data.oMf[eeFrameId];
  p_out = oMf.translation();
  q_out = Eigen::Quaterniond(oMf.rotation());
  q_out.normalize();
  return true;
}

// ===== Target msg → PlannedCartesianTrajectory ==============================
inline PlannedCartesianTrajectory
fromTargetMsg(const ocs2_msgs::msg::MpcTargetTrajectories& msg) {
  const auto target = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(msg);

  PlannedCartesianTrajectory out;
  const std::size_t N = target.stateTrajectory.size();

  out.time.reserve(N);
  out.position.reserve(N);
  out.quat.reserve(N);

  for (std::size_t i = 0; i < N; ++i) {
    const auto& x = target.stateTrajectory[i];  // [px py pz qx qy qz qw]

    out.time.push_back(target.timeTrajectory[i]);
    out.position.emplace_back(x(0), x(1), x(2));
    out.quat.emplace_back(x(6), x(3), x(4), x(5));  // w, x, y, z
  }
  return out;
}

inline std::string statusToString(const TrackingMetrics& m) {
  // Keep the status mapping simple and consistent with TrajectoryMonitor.
  return toString(m.status);
}

}  // namespace

// ============================================================================

class TrajectoryProgressMonitorNode final : public rclcpp::Node {
 public:
  explicit TrajectoryProgressMonitorNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("trajectory_progress_monitor", options) {

    // --- Parameters ---------------------------------------------------------
    this->declare_parameter<std::string>("taskFile", "");
    this->declare_parameter<std::string>("urdfFile", "");
    this->declare_parameter<std::string>("libFolder", "");
    this->declare_parameter<std::string>("robotName", "mobile_manipulator");

    this->declare_parameter<double>("monitorRate", 50.0);

    // Monitor params (match config/tt_params.yaml)
    this->declare_parameter<int>("window", 20);
    this->declare_parameter<int>("maxBacktrack", 5);
    this->declare_parameter<double>("onTrackPos", 0.005);
    this->declare_parameter<double>("onTrackOriDeg", 5.0);
    this->declare_parameter<double>("divergedPos", 0.03);
    this->declare_parameter<double>("divergedHoldSec", 0.5);
    this->declare_parameter<double>("finishProgress", 0.98);
    this->declare_parameter<double>("finishPos", 0.005);

    taskFile_ = this->get_parameter("taskFile").as_string();
    urdfFile_ = this->get_parameter("urdfFile").as_string();
    libFolder_ = this->get_parameter("libFolder").as_string();
    robotName_ = this->get_parameter("robotName").as_string();

    const double rate = this->get_parameter("monitorRate").as_double();
    monitorPeriod_ = std::chrono::duration<double>(1.0 / std::max(1e-6, rate));

    MonitorParams mp;
    mp.window = this->get_parameter("window").as_int();
    mp.max_backtrack = this->get_parameter("maxBacktrack").as_int();
    mp.on_track_pos = this->get_parameter("onTrackPos").as_double();
    mp.on_track_ori_deg = this->get_parameter("onTrackOriDeg").as_double();
    mp.diverged_pos = this->get_parameter("divergedPos").as_double();
    mp.diverged_hold_sec = this->get_parameter("divergedHoldSec").as_double();
    mp.finish_progress = this->get_parameter("finishProgress").as_double();
    mp.finish_pos = this->get_parameter("finishPos").as_double();
    monitor_ = std::make_unique<TrajectoryMonitor>(mp);

    // --- Model / FK init ----------------------------------------------------
    if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
      interface_ = std::make_unique<
          ocs2::mobile_manipulator_mpc::MobileManipulatorInterface>(
          taskFile_, libFolder_, urdfFile_);

      pin_ = std::make_unique<ocs2::PinocchioInterface>(
          interface_->getPinocchioInterface());

      modelInfo_ = interface_->getManipulatorModelInfo();
      mapping_ = std::make_unique<
          ocs2::mobile_manipulator_mpc::MobileManipulatorPinocchioMapping>(modelInfo_);

      eeFrameId_ = pin_->getModel().getFrameId(modelInfo_.eeFrame);
    }

    // --- Publishers ---------------------------------------------------------
    statusPub_ =
        this->create_publisher<std_msgs::msg::String>("tracking_status", 10);
    metricsPub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("tracking_metrics", 10);

    // --- Subscriptions ------------------------------------------------------
    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + "_mpc_observation", rclcpp::QoS(1).reliable(),
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latestObs_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    targetSub_ = this->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
        robotName_ + "_mpc_target", rclcpp::QoS(1).reliable(),
        [&](const ocs2_msgs::msg::MpcTargetTrajectories::ConstSharedPtr msg) {
          const auto traj = fromTargetMsg(*msg);
          std::lock_guard<std::mutex> lock(mtx_);
          latestRef_ = traj;
          haveRef_ =
              !traj.empty() &&
              traj.time.size() == traj.position.size() &&
              traj.time.size() == traj.quat.size();
          if (haveRef_) {
            monitor_->setTrajectory(latestRef_);
          }
        });

    timer_ = this->create_wall_timer(
        monitorPeriod_,
        std::bind(&TrajectoryProgressMonitorNode::onTimer, this));
  }

 private:
  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!haveObs_ || !haveRef_) return;
      obs = latestObs_;
    }

    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    if (!computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p, q)) {
      return;
    }

    const TrackingMetrics met = monitor_->update(obs.time, p, q);
    const std::string status = statusToString(met);

    std_msgs::msg::String s;
    s.data = status;
    statusPub_->publish(s);

    std_msgs::msg::Float64MultiArray m;
    // [t_now, progress(0..1), pos_err(m), ori_err(deg), lag_sec, closest_index]
    m.data = {
        obs.time,
        met.progress,
        met.pos_err,
        met.ori_err_deg,
        met.lag_sec,
        static_cast<double>(met.closest_index)};
    metricsPub_->publish(m);
  }

 private:
  std::string taskFile_, urdfFile_, libFolder_, robotName_;

  std::chrono::duration<double> monitorPeriod_{0.02};

  std::unique_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::PinocchioInterface> pin_;
  ocs2::mobile_manipulator_mpc::ManipulatorModelInfo modelInfo_;
  std::unique_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorPinocchioMapping> mapping_;
  std::size_t eeFrameId_{0};

  std::unique_ptr<TrajectoryMonitor> monitor_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metricsPub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  SystemObservation latestObs_;
  PlannedCartesianTrajectory latestRef_;
  bool haveObs_{false}, haveRef_{false};
};

}  // namespace mpc_cartesian_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<
      mpc_cartesian_planner::TrajectoryProgressMonitorNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
