/**
 * trajectory_tt_publisher_node
 *
 * Cartesian TT (trajectory tracking) target publisher for OCS2 mobile manipulator MPC.
 *
 * Subscribes:
 *   - <robotName>_mpc_observation   (ocs2_msgs/msg/MpcObservation)
 * Publishes:
 *   - <robotName>_mpc_target    (ocs2_msgs/msg/MpcTargetTrajectories)
 *
 * Optional visualization:
 *   - /mobile_manipulator/targetStateTrajectory (visualization_msgs/msg/MarkerArray)
 *   - /mobile_manipulator/targetPoseTrajectory  (geometry_msgs/msg/PoseArray)
 *   - TF: <trajectoryGlobalFrame> -> command
 *
 * Notes
 * -----
 * - We compute the initial end-effector pose via Pinocchio FK (from the current MPC observation).
 * - The planned trajectory is time-anchored to the first observation time.
 * - This node only publishes an EE target. Mode scheduling / blending is handled by your
 *   MobileManipulatorRosReferenceManager in mpc_controller.
 */

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"
#include "mpc_cartesian_planner/trajectory_publisher.h"

using ocs2::SystemObservation;

namespace mpc_cartesian_planner {

namespace {

inline Eigen::Vector3d readAxis(rclcpp::Node& node) {
  Eigen::Vector3d axis;
  axis.x() = node.get_parameter("axisX").as_double();
  axis.y() = node.get_parameter("axisY").as_double();
  axis.z() = node.get_parameter("axisZ").as_double();
  if (axis.norm() < 1e-9) axis = Eigen::Vector3d::UnitZ();
  axis.normalize();
  return axis;
}

// Compute EE pose via Pinocchio FK using OCS2 pinocchio mapping.
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

inline bool sampleTrajectoryAtTime(const PlannedCartesianTrajectory& traj,
                                   double t,
                                   Eigen::Vector3d& p_out,
                                   Eigen::Quaterniond& q_out) {
  if (traj.empty() ||
      traj.time.size() != traj.position.size() ||
      traj.time.size() != traj.quat.size()) {
    return false;
  }

  if (t <= traj.time.front()) {
    p_out = traj.position.front();
    q_out = traj.quat.front();
    return true;
  }
  if (t >= traj.time.back()) {
    p_out = traj.position.back();
    q_out = traj.quat.back();
    return true;
  }

  const auto it = std::upper_bound(traj.time.begin(), traj.time.end(), t);
  if (it == traj.time.begin() || it == traj.time.end()) {
    return false;
  }
  const std::size_t i1 = static_cast<std::size_t>(it - traj.time.begin());
  const std::size_t i0 = i1 - 1;

  const double t0 = traj.time[i0];
  const double t1 = traj.time[i1];
  const double denom = t1 - t0;
  const double alpha = (std::abs(denom) < 1e-12) ? 0.0 : (t - t0) / denom;

  p_out = (1.0 - alpha) * traj.position[i0] + alpha * traj.position[i1];

  Eigen::Quaterniond q0 = traj.quat[i0].normalized();
  Eigen::Quaterniond q1 = traj.quat[i1].normalized();
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }
  q_out = q0.slerp(alpha, q1).normalized();
  return true;
}

}  // namespace

class TrajectoryTTPublisherNode final : public rclcpp::Node {
 public:
  explicit TrajectoryTTPublisherNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("trajectory_tt_publisher", options) {
    // --- Parameters (match config/tt_params.yaml) ---
    this->declare_parameter<std::string>("taskFile", "");
    this->declare_parameter<std::string>("urdfFile", "");
    this->declare_parameter<std::string>("libFolder", "");
    this->declare_parameter<std::string>("robotName", std::string("mobile_manipulator"));

    this->declare_parameter<double>("publishRate", 20.0);
    this->declare_parameter<double>("horizon", 2.0);
    this->declare_parameter<double>("dt", 0.05);
    this->declare_parameter<double>("amplitude", 0.2);
    this->declare_parameter<double>("frequency", 0.1);
    this->declare_parameter<double>("axisX", 0.0);
    this->declare_parameter<double>("axisY", 0.0);
    this->declare_parameter<double>("axisZ", 1.0);
    this->declare_parameter<std::string>("trajectoryGlobalFrame", std::string("world"));
    this->declare_parameter<bool>("publishViz", true);
    this->declare_parameter<bool>("publishTF", true);
    this->declare_parameter<bool>("publishOnce", false);
    this->declare_parameter<std::string>("commandFrameId", std::string("command"));

    taskFile_ = this->get_parameter("taskFile").as_string();
    urdfFile_ = this->get_parameter("urdfFile").as_string();
    libFolder_ = this->get_parameter("libFolder").as_string();
    robotName_ = this->get_parameter("robotName").as_string();

    publishRate_ = this->get_parameter("publishRate").as_double();
    horizon_ = this->get_parameter("horizon").as_double();
    dt_ = this->get_parameter("dt").as_double();
    amplitude_ = this->get_parameter("amplitude").as_double();
    freqHz_ = this->get_parameter("frequency").as_double();
    axis_ = readAxis(*this);
    globalFrame_ = this->get_parameter("trajectoryGlobalFrame").as_string();
    publishViz_ = this->get_parameter("publishViz").as_bool();
    publishTF_ = this->get_parameter("publishTF").as_bool();
    publishOnce_ = this->get_parameter("publishOnce").as_bool();
    commandFrameId_ = this->get_parameter("commandFrameId").as_string();
    if (commandFrameId_.empty()) commandFrameId_ = "command";

    if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
      try {
        interface_ = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorInterface>(taskFile_, libFolder_, urdfFile_);
        // PinocchioInterface has no default ctor; store a copy behind a pointer.
        pin_ = std::make_unique<ocs2::PinocchioInterface>(interface_->getPinocchioInterface());
        modelInfo_ = interface_->getManipulatorModelInfo();
        mapping_ = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping>(modelInfo_);
        eeFrameId_ = pin_->getModel().getFrameId(modelInfo_.eeFrame);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "taskFile/urdfFile/libFolder not set. Will publish around origin (0,0,0) with identity orientation.");
    }

    // Publishers
    // In one-shot mode we latch the last target so late-joining subscribers can still receive it.
    trajPub_ = std::make_unique<TrajectoryPublisher>(*this, robotName_, /*latch=*/publishOnce_);

    if (publishViz_) {
      markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/mobile_manipulator/targetStateTrajectory", 1);
      posePub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
          "/mobile_manipulator/targetPoseTrajectory", 1);
    }
    if (publishTF_) {
      tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    // Observation subscription
    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + std::string("_mpc_observation"), rclcpp::QoS(1).reliable(),
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latestObs_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, publishRate_));
    timer_ = this->create_wall_timer(period, std::bind(&TrajectoryTTPublisherNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "TT publisher started. Publishing to %s at %.1f Hz",
                trajPub_->topic().c_str(), publishRate_);
    if (publishOnce_) {
      RCLCPP_INFO(this->get_logger(),
                  "publishOnce=true: will publish the target trajectory once (use 'horizon' as total duration) and then stop publishing.");
    }
  }

 private:
  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!haveObs_) return;
      obs = latestObs_;
    }

    if (!initialized_) {
      startTime_ = obs.time;
      // Compute initial center and orientation via FK if possible.
      if (interface_ && pin_ && mapping_) {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        const bool ok = computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p, q);
        if (ok) {
          centerP_ = p;
          q0_ = q;
        } else {
          RCLCPP_WARN(this->get_logger(), "Initial FK failed. Using origin pose.");
        }
      }

      FigureEightParams p;
      p.horizon_T = horizon_;
      p.dt = dt_;
      p.amplitude = amplitude_;
      p.frequency_hz = freqHz_;
      p.plane_axis = axis_;
      planner_ = std::make_unique<FigureEightPlanner>(centerP_, q0_, p);
      planner_->setStartTime(startTime_);

      initialized_ = true;
    }

    if (!planner_) return;

    PlannedCartesianTrajectory traj;
    if (!publishOnce_ || !publishedOnce_) {
      traj = planner_->plan(obs);

      // Publish EE target
      trajPub_->publishEeTarget(traj, obs);

      if (publishViz_ && markerPub_ && posePub_) {
        publishViz(traj);
      }

      if (publishOnce_ && !publishedOnce_) {
        publishedTraj_ = traj;
        havePublishedTraj_ = !publishedTraj_.empty();
        publishedOnce_ = true;
        RCLCPP_INFO(this->get_logger(), "One-shot trajectory published (%zu samples).", traj.size());
      }
    }

    if (publishTF_ && tfBroadcaster_) {
      if (publishOnce_ && publishedOnce_ && havePublishedTraj_) {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        if (sampleTrajectoryAtTime(publishedTraj_, obs.time, p, q)) {
          publishCommandTf(p, q);
        }
      } else if (!traj.empty()) {
        publishCommandTf(traj.position.front(), traj.quat.front());
      }
    }
  }

  void publishCommandTf(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
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

  void publishViz(const PlannedCartesianTrajectory& traj) {
    visualization_msgs::msg::MarkerArray markerArray;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = globalFrame_;
    poseArray.header.stamp = this->now();

    markerArray.markers.reserve(traj.size());
    poseArray.poses.reserve(traj.size());

    for (std::size_t i = 0; i < traj.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = globalFrame_;
      marker.header.stamp = this->now();
      marker.ns = "target_state";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
      marker.color.r = 0.4660f;
      marker.color.g = 0.6740f;
      marker.color.b = 0.1880f;
      marker.color.a = 1.0f;

      marker.pose.position = ocs2::ros_msg_helpers::getPointMsg(traj.position[i]);
      marker.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(traj.quat[i]);
      markerArray.markers.push_back(marker);

      geometry_msgs::msg::Pose pose;
      pose.position = marker.pose.position;
      pose.orientation = marker.pose.orientation;
      poseArray.poses.push_back(pose);
    }

    markerPub_->publish(markerArray);
    posePub_->publish(poseArray);
  }

 private:
  // Params
  std::string taskFile_;
  std::string urdfFile_;
  std::string libFolder_;
  std::string robotName_{"mobile_manipulator"};
  std::string globalFrame_{"world"};

  double publishRate_{20.0};
  double horizon_{2.0};
  double dt_{0.05};
  double amplitude_{0.2};
  double freqHz_{0.1};
  Eigen::Vector3d axis_{0, 0, 1};
  bool publishViz_{true};
  bool publishTF_{true};
  bool publishOnce_{true};
  bool publishedOnce_{false};
  std::string commandFrameId_{"command"};
  PlannedCartesianTrajectory publishedTraj_;
  bool havePublishedTraj_{false};

  // OCS2 / Pinocchio
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::PinocchioInterface> pin_;
  ocs2::mobile_manipulator::ManipulatorModelInfo modelInfo_;
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping> mapping_;
  std::size_t eeFrameId_{0};

  // Planner
  bool initialized_{false};
  double startTime_{0.0};
  Eigen::Vector3d centerP_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q0_{Eigen::Quaterniond::Identity()};
  std::unique_ptr<FigureEightPlanner> planner_;

  // ROS I/O
  std::unique_ptr<TrajectoryPublisher> trajPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  SystemObservation latestObs_;
  bool haveObs_{false};
};

}  // namespace mpc_cartesian_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(false);

  auto node = std::make_shared<mpc_cartesian_planner::TrajectoryTTPublisherNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
