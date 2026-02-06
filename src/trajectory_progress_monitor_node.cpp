/**
 * trajectory_progress_monitor_node
 *
 * Window-search progress monitor for Cartesian TT.
 */

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "mpc_cartesian_planner/trajectory_monitor.h"

using ocs2::SystemObservation;

namespace mpc_cartesian_planner {

namespace {

// ===== FK helper ============================================================
inline bool computeEePose(ocs2::PinocchioInterface& pin,
                          const ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping& mapping,
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

inline bool isHoldTrajectory(const PlannedCartesianTrajectory& traj) {
  if (traj.time.size() != 2 || traj.position.size() != 2 || traj.quat.size() != 2) {
    return false;
  }
  constexpr double kPosEps = 1e-9;
  constexpr double kAngEps = 1e-9;

  const Eigen::Vector3d dp = traj.position[0] - traj.position[1];
  if (dp.norm() > kPosEps) {
    return false;
  }

  Eigen::Quaterniond q0 = traj.quat[0].normalized();
  Eigen::Quaterniond q1 = traj.quat[1].normalized();
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }
  const double dot = std::clamp(std::abs(q0.dot(q1)), 0.0, 1.0);
  const double ang = 2.0 * std::acos(dot);
  return ang <= kAngEps;
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
    this->declare_parameter<std::string>("trajectoryGlobalFrame", std::string("world"));
    this->declare_parameter<bool>("publishViz", true);
    this->declare_parameter<bool>("publishTF", false);
    this->declare_parameter<std::string>("closestFrameId", std::string("command_closest"));

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

    globalFrame_ = this->get_parameter("trajectoryGlobalFrame").as_string();
    publishViz_ = this->get_parameter("publishViz").as_bool();
    publishTF_ = this->get_parameter("publishTF").as_bool();
    closestFrameId_ = this->get_parameter("closestFrameId").as_string();
    if (closestFrameId_.empty()) closestFrameId_ = "command_closest";
    if (publishTF_) {
      tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    MonitorParams mp;
    mp.window = this->get_parameter("window").as_int();
    mp.max_backtrack = this->get_parameter("maxBacktrack").as_int();
    mp.update_dt = monitorPeriod_.count();
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
          ocs2::mobile_manipulator::MobileManipulatorInterface>(
          taskFile_, libFolder_, urdfFile_);

      pin_ = std::make_unique<ocs2::PinocchioInterface>(
          interface_->getPinocchioInterface());

      modelInfo_ = interface_->getManipulatorModelInfo();
      mapping_ = std::make_unique<
          ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping>(modelInfo_);

      eeFrameId_ = pin_->getModel().getFrameId(modelInfo_.eeFrame);
    }

    // --- Publishers ---------------------------------------------------------
    statusPub_ =
        this->create_publisher<std_msgs::msg::String>(
            robotName_ + "/trajectory_tracking/tracking_status", 10);
    metricsPub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            robotName_ + "/trajectory_tracking/tracking_metric", 10);
    if (publishViz_) {
      closestWaypointPub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              "/mobile_manipulator/closestTargetWaypoint", 1);
    }

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
          const bool is_hold = isHoldTrajectory(traj);
          bool wake = false;
          {
            std::lock_guard<std::mutex> lock(mtx_);
            latestRef_ = traj;
            haveRef_ =
                !traj.empty() &&
                traj.time.size() == traj.position.size() &&
                traj.time.size() == traj.quat.size();
            if (haveRef_) {
              monitor_->setTrajectory(latestRef_);
            }
            // If we entered idle mode due to FINISHED/DIVERGED, only wake up when a new "real"
            // trajectory arrives (ignore the 2-point HOLD trajectory published on intervention).
            if (idle_ && haveRef_ && !is_hold) {
              idle_ = false;
              wake = true;
            }
          }
          if (wake && timer_) {
            timer_->reset();
            RCLCPP_INFO(this->get_logger(), "New trajectory received. Progress monitor resumed.");
          }
        });

    timer_ = this->create_wall_timer(
        monitorPeriod_,
        std::bind(&TrajectoryProgressMonitorNode::onTimer, this));
  }

 private:
  void publishClosestWaypointMarker(const PlannedCartesianTrajectory& traj, std::size_t index) {
    if (!publishViz_ || !closestWaypointPub_) return;
    if (traj.empty() || traj.position.size() != traj.time.size() || traj.quat.size() != traj.time.size()) return;
    if (index >= traj.size()) return;

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.reserve(1);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = globalFrame_;
    marker.header.stamp = this->now();
    marker.ns = "closest_target_waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    const auto& p = traj.position[index];
    const auto q = traj.quat[index].normalized();
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    markerArray.markers.push_back(std::move(marker));
    closestWaypointPub_->publish(markerArray);
    closestWaypointPublished_ = true;
  }

  void clearClosestWaypointMarker() {
    if (!closestWaypointPublished_ || !closestWaypointPub_) return;
    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.reserve(1);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = globalFrame_;
    marker.header.stamp = this->now();
    marker.ns = "closest_target_waypoint";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;

    markerArray.markers.push_back(std::move(marker));
    closestWaypointPub_->publish(markerArray);
    closestWaypointPublished_ = false;
  }

  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!haveObs_ || !haveRef_) {
        clearClosestWaypointMarker();
        return;
      }
      obs = latestObs_;
    }

    if (!pin_ || !mapping_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Pinocchio not initialized (set taskFile/urdfFile/libFolder).");
      clearClosestWaypointMarker();
      return;
    }

    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    if (!computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p, q)) {
      clearClosestWaypointMarker();
      return;
    }

    TrackingMetrics met;
    PlannedCartesianTrajectory traj;
    bool enter_idle = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!haveRef_) {
        clearClosestWaypointMarker();
        return;
      }
      met = monitor_->update(obs.time, p, q);
      traj = latestRef_;

      const bool terminal = (met.status == TrackingStatus::FINISHED) || (met.status == TrackingStatus::DIVERGED);
      if (terminal && !idle_) {
        idle_ = true;
        enter_idle = true;
      }
    }
    if (enter_idle && timer_) {
      timer_->cancel();
    }
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

    if (publishTF_ && tfBroadcaster_ && !traj.empty() && met.closest_index < traj.size()) {
      const auto& p_closest = traj.position[met.closest_index];
      const auto q_closest = traj.quat[met.closest_index].normalized();

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = this->now();
      tf.header.frame_id = globalFrame_;
      tf.child_frame_id = closestFrameId_;
      tf.transform.translation.x = p_closest.x();
      tf.transform.translation.y = p_closest.y();
      tf.transform.translation.z = p_closest.z();
      tf.transform.rotation.x = q_closest.x();
      tf.transform.rotation.y = q_closest.y();
      tf.transform.rotation.z = q_closest.z();
      tf.transform.rotation.w = q_closest.w();
      tfBroadcaster_->sendTransform(tf);
    }

    publishClosestWaypointMarker(traj, met.closest_index);

    if (enter_idle) {
      if (met.status == TrackingStatus::FINISHED) {
        RCLCPP_INFO(this->get_logger(), "Tracking finished. Progress monitor entering idle mode.");
      } else if (met.status == TrackingStatus::DIVERGED) {
        RCLCPP_WARN(this->get_logger(), "Tracking diverged. Progress monitor entering idle mode.");
      }
    }
  }

 private:
  std::string taskFile_, urdfFile_, libFolder_, robotName_;
  std::string globalFrame_{"world"};
  bool publishViz_{true};
  bool publishTF_{false};
  std::string closestFrameId_{"command_closest"};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::chrono::duration<double> monitorPeriod_{0.02};

  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::PinocchioInterface> pin_;
  ocs2::mobile_manipulator::ManipulatorModelInfo modelInfo_;
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping> mapping_;
  std::size_t eeFrameId_{0};

  std::unique_ptr<TrajectoryMonitor> monitor_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metricsPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr closestWaypointPub_;
  bool closestWaypointPublished_{false};
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  SystemObservation latestObs_;
  PlannedCartesianTrajectory latestRef_;
  bool haveObs_{false}, haveRef_{false};
  bool idle_{false};
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
