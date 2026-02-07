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
#include <cctype>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mode_schedule.hpp>
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
#include "robot_math_utils/robot_math_utils_v1_18.hpp"

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

inline std::string toLowerCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

inline TimeScalingType parseTimeScaling(const std::string& s) {
  const std::string v = toLowerCopy(s);
  if (v == "linear") return TimeScalingType::Linear;
  if (v == "min_jerk" || v == "minjerk" || v == "min-jerk") return TimeScalingType::MinJerk;
  return TimeScalingType::MinJerk;
}

template <typename T, typename = void>
struct has_member_activate_controllers : std::false_type {};
template <typename T>
struct has_member_activate_controllers<T, std::void_t<decltype(std::declval<T>().activate_controllers)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_member_deactivate_controllers : std::false_type {};
template <typename T>
struct has_member_deactivate_controllers<T, std::void_t<decltype(std::declval<T>().deactivate_controllers)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_member_start_controllers : std::false_type {};
template <typename T>
struct has_member_start_controllers<T, std::void_t<decltype(std::declval<T>().start_controllers)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_member_stop_controllers : std::false_type {};
template <typename T>
struct has_member_stop_controllers<T, std::void_t<decltype(std::declval<T>().stop_controllers)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_member_activate_asap : std::false_type {};
template <typename T>
struct has_member_activate_asap<T, std::void_t<decltype(std::declval<T>().activate_asap)>> : std::true_type {};

template <typename T, typename = void>
struct has_member_start_asap : std::false_type {};
template <typename T>
struct has_member_start_asap<T, std::void_t<decltype(std::declval<T>().start_asap)>> : std::true_type {};

inline int32_t switchControllerStrictness() {
  // controller_manager_msgs/srv/SwitchController: STRICT=2, BEST_EFFORT=1 (ROS 2 control convention)
  return 2;
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

    this->declare_parameter<std::string>("trajectoryType", std::string("figure_eight"));
    this->declare_parameter<std::vector<double>>("linearMoveOffset", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    this->declare_parameter<bool>("linearMoveOffsetInToolFrame", false);
    this->declare_parameter<std::string>("linearMoveTimeScaling", std::string("min_jerk"));
    this->declare_parameter<std::vector<double>>("poseMoveTarget", std::vector<double>{});
    this->declare_parameter<std::string>("poseMoveTimeScaling", std::string("min_jerk"));
    this->declare_parameter<std::vector<double>>("screwMoveUhat", std::vector<double>({0.0, 1.0, 0.0}));
    this->declare_parameter<std::vector<double>>("screwMoveR", std::vector<double>({0.1, 0.0, 0.0}));
    this->declare_parameter<double>("screwMoveTheta", 1.5707963267948966);  // pi/2
    this->declare_parameter<bool>("screwMoveInToolFrame", true);
    this->declare_parameter<std::string>("screwMoveTimeScaling", std::string("min_jerk"));
    this->declare_parameter<bool>("interveneHoldOnDivergedOrFinished", false);
    this->declare_parameter<bool>("switchMpcControllerOnIntervention", false);
    this->declare_parameter<std::string>("controllerManagerSwitchService", std::string("/controller_manager/switch_controller"));
    this->declare_parameter<std::string>("mpcControllerName", std::string("mpc_controller"));
    this->declare_parameter<bool>("publishModeScheduleOnEnable", true);
    this->declare_parameter<int>("modeScheduleMode", 1);

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

    trajectoryType_ = this->get_parameter("trajectoryType").as_string();
    if (trajectoryType_.empty()) trajectoryType_ = "figure_eight";
    {
      const auto offset = this->get_parameter("linearMoveOffset").as_double_array();
      if (offset.size() == 3) {
        linearMoveOffset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
        linearMoveEulerZyx_ = Eigen::Vector3d::Zero();
      } else if (offset.size() == 6) {
        linearMoveOffset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
        // ZYX Euler order: [thz, thy, thx] in radians
        linearMoveEulerZyx_ = Eigen::Vector3d(offset[3], offset[4], offset[5]);
      } else {
        RCLCPP_WARN(this->get_logger(), "linearMoveOffset must be a double[3] or double[6]. Got size=%zu. Using zeros.", offset.size());
        linearMoveOffset_ = Eigen::Vector3d::Zero();
        linearMoveEulerZyx_ = Eigen::Vector3d::Zero();
      }
    }
    linearMoveOffsetInToolFrame_ = this->get_parameter("linearMoveOffsetInToolFrame").as_bool();
    linearMoveTimeScaling_ = parseTimeScaling(this->get_parameter("linearMoveTimeScaling").as_string());
    interveneHoldOnDivergedOrFinished_ = this->get_parameter("interveneHoldOnDivergedOrFinished").as_bool();
    switchMpcControllerOnIntervention_ = this->get_parameter("switchMpcControllerOnIntervention").as_bool();
    controllerManagerSwitchService_ = this->get_parameter("controllerManagerSwitchService").as_string();
    mpcControllerName_ = this->get_parameter("mpcControllerName").as_string();
    publishModeScheduleOnEnable_ = this->get_parameter("publishModeScheduleOnEnable").as_bool();
    modeScheduleMode_ = static_cast<int>(this->get_parameter("modeScheduleMode").as_int());

    {
      const auto pose = this->get_parameter("poseMoveTarget").as_double_array();
      if (!pose.empty()) {
        if (pose.size() == 3) {
          poseMoveHasTarget_ = true;
          poseMoveHasOrientation_ = false;
          poseMoveTargetP_ = Eigen::Vector3d(pose[0], pose[1], pose[2]);
          poseMoveTargetQ_ = Eigen::Quaterniond::Identity();
        } else if (pose.size() == 6) {
          poseMoveHasTarget_ = true;
          poseMoveHasOrientation_ = true;
          poseMoveTargetP_ = Eigen::Vector3d(pose[0], pose[1], pose[2]);
          // ZYX Euler order: [thz, thy, thx] in radians
          const Eigen::Vector3d euler_zyx(pose[3], pose[4], pose[5]);
          poseMoveTargetQ_ = RMUtils::zyxEuler2Quat(euler_zyx, /*rad=*/true).normalized();
        } else if (pose.size() == 7) {
          poseMoveHasTarget_ = true;
          poseMoveHasOrientation_ = true;
          poseMoveTargetP_ = Eigen::Vector3d(pose[0], pose[1], pose[2]);
          const Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]);  // [x y z qw qx qy qz]
          if (q.norm() < 1e-12) {
            RCLCPP_WARN(this->get_logger(), "poseMoveTarget quaternion has near-zero norm. Ignoring orientation.");
            poseMoveHasOrientation_ = false;
            poseMoveTargetQ_ = Eigen::Quaterniond::Identity();
          } else {
            poseMoveTargetQ_ = q.normalized();
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "poseMoveTarget must be a double[3], double[6], or double[7]. Got size=%zu. Ignoring.",
                      pose.size());
          poseMoveHasTarget_ = false;
          poseMoveHasOrientation_ = false;
          poseMoveTargetP_ = Eigen::Vector3d::Zero();
          poseMoveTargetQ_ = Eigen::Quaterniond::Identity();
        }
      }
    }
    poseMoveTimeScaling_ = parseTimeScaling(this->get_parameter("poseMoveTimeScaling").as_string());

    {
      const auto uhat = this->get_parameter("screwMoveUhat").as_double_array();
      if (uhat.size() == 3) {
        screwMoveUhat_ = Eigen::Vector3d(uhat[0], uhat[1], uhat[2]);
      } else {
        RCLCPP_WARN(this->get_logger(), "screwMoveUhat must be a double[3]. Got size=%zu. Using [0,1,0].", uhat.size());
        screwMoveUhat_ = Eigen::Vector3d(0.0, 1.0, 0.0);
      }
    }
    {
      const auto r = this->get_parameter("screwMoveR").as_double_array();
      if (r.size() == 3) {
        screwMoveR_ = Eigen::Vector3d(r[0], r[1], r[2]);
      } else {
        RCLCPP_WARN(this->get_logger(), "screwMoveR must be a double[3]. Got size=%zu. Using [0.1,0,0].", r.size());
        screwMoveR_ = Eigen::Vector3d(0.1, 0.0, 0.0);
      }
    }
    screwMoveTheta_ = this->get_parameter("screwMoveTheta").as_double();
    screwMoveInToolFrame_ = this->get_parameter("screwMoveInToolFrame").as_bool();
    screwMoveTimeScaling_ = parseTimeScaling(this->get_parameter("screwMoveTimeScaling").as_string());

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
    modeSchedulePub_ = this->create_publisher<ocs2_msgs::msg::ModeSchedule>(
        robotName_ + std::string("_mode_schedule"), rclcpp::QoS(1).reliable());
    // Publish an initial ModeSchedule once at startup so the controller has a mode even before activation.
    modeScheduleStartupTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          publishDefaultModeSchedule("STARTUP");
          if (modeScheduleStartupTimer_) {
            modeScheduleStartupTimer_->cancel();
          }
        });

    if (publishViz_) {
      markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/mobile_manipulator/targetStateTrajectory", 1);
      posePub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
          "/mobile_manipulator/targetPoseTrajectory", 1);
    }
    if (publishTF_) {
      tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    if (switchMpcControllerOnIntervention_ && !controllerManagerSwitchService_.empty()) {
      switchControllerClient_ =
          this->create_client<controller_manager_msgs::srv::SwitchController>(controllerManagerSwitchService_);
    }

    toggleTrackingSrv_ = this->create_service<std_srvs::srv::SetBool>(
        robotName_ + "/trajectory_tracking/toggle_tt_publisher",
        [this](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
               const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          const bool enable = req->data;
          const bool wasPaused = trackingFinished_;

          if (!enable) {
            if (wasPaused) {
              res->success = true;
              res->message = "TT publisher already paused.";
              return;
            }

            // Pause: publish a HOLD target at the current EE pose (if possible) and stop the timer.
            SystemObservation obs;
            bool haveObsLocal = false;
            {
              std::lock_guard<std::mutex> lock(mtx_);
              haveObsLocal = haveObs_;
              if (haveObsLocal) {
                obs = latestObs_;
              }
            }

            if (haveObsLocal) {
              publishHoldTrajectory(obs, "MANUAL_PAUSE");
            } else {
              RCLCPP_WARN(this->get_logger(), "Pause requested but no observation received yet; not publishing hold target.");
            }

            trackingFinished_ = true;
            if (timer_) timer_->cancel();
            requestSwitchMpcController(/*activate=*/false, "MANUAL_PAUSE");

            res->success = true;
            res->message = "TT publisher paused (hold target published).";
            return;
          }

          // Enable (start/resume). If previously paused, reset the planner so the next run starts from current EE.
          if (wasPaused) {
            trackingFinished_ = false;
            initialized_ = false;
            planner_.reset();
            publishedOnce_ = false;
          }

          publishDefaultModeSchedule(wasPaused ? "MANUAL_RESUME" : "MANUAL_ENABLE");
          requestSwitchMpcController(/*activate=*/true, "MANUAL_RESUME");
          if (timer_) timer_->reset();

          res->success = true;
          res->message = wasPaused ? "TT publisher resumed." : "TT publisher enabled.";
        });

    // Observation subscription
    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + std::string("_mpc_observation"), rclcpp::QoS(1).reliable(),
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latestObs_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    // Tracking status subscription (to stop when the monitor reports FINISHED).
    trackingStatusSub_ = this->create_subscription<std_msgs::msg::String>(
        robotName_ + "/trajectory_tracking/tracking_status", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          const bool finished = (msg->data == "FINISHED");
          const bool diverged = (msg->data == "DIVERGED");
          if (!finished && !diverged) return;

          if (trackingFinished_) return;

          if (interveneHoldOnDivergedOrFinished_) {
            SystemObservation obs;
            bool haveObsLocal = false;
            {
              std::lock_guard<std::mutex> lock(mtx_);
              haveObsLocal = haveObs_;
              if (!haveObsLocal) {
                RCLCPP_WARN(this->get_logger(),
                            "Intervention requested (%s) but no observation received yet; skipping hold publish.",
                            msg->data.c_str());
              } else {
                obs = latestObs_;
              }
            }

            if (haveObsLocal) {
              publishHoldTrajectory(obs, msg->data);
            }
          }

          trackingFinished_ = true;
          requestSwitchMpcController(/*activate=*/false, msg->data);
          if (finished) {
            RCLCPP_INFO(this->get_logger(), "Tracking finished. TT publisher entering idle mode.");
          } else {
            RCLCPP_WARN(this->get_logger(), "Tracking diverged. TT publisher entering idle mode.");
          }
          if (timer_) timer_->cancel();
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
  void publishHoldTrajectory(const SystemObservation& obs, const std::string& reason) {
    if (!trajPub_) return;

    Eigen::Vector3d p_hold = centerP_;
    Eigen::Quaterniond q_hold = q0_;

    bool ok = false;
    if (pin_ && mapping_ && interface_) {
      std::lock_guard<std::mutex> lock(pinMtx_);
      ok = computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p_hold, q_hold);
    }
    if (!ok) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to compute current EE pose for hold (reason=%s). Using last known pose instead.",
                  reason.c_str());
      q_hold.normalize();
    }

    PlannedCartesianTrajectory hold;
    const double t0 = obs.time;
    const double T = std::max(1e-3, horizon_);
    hold.time = {t0, t0 + T};
    hold.position = {p_hold, p_hold};
    hold.quat = {q_hold, q_hold};

    trajPub_->publishEeTarget(hold, obs);

    publishedTraj_ = hold;
    havePublishedTraj_ = true;
    publishedOnce_ = true;

    RCLCPP_INFO(this->get_logger(), "Published HOLD target (reason=%s) at t=%.3f", reason.c_str(), t0);
  }

  void requestSwitchMpcController(bool activate, const std::string& reason) {
    if (!switchMpcControllerOnIntervention_) return;
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

    // Prefer activate/deactivate fields (stop/start are deprecated on newer ros2_control).
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

  void publishDefaultModeSchedule(const std::string& reason) {
    if (!publishModeScheduleOnEnable_) return;
    if (!modeSchedulePub_) return;

    const int clamped_mode = std::clamp(modeScheduleMode_, -128, 127);
    if (clamped_mode != modeScheduleMode_) {
      RCLCPP_WARN(this->get_logger(),
                  "modeScheduleMode=%d is out of int8 range; clamping to %d.",
                  modeScheduleMode_,
                  clamped_mode);
    }

    ocs2_msgs::msg::ModeSchedule msg;
    msg.event_times = {};
    msg.mode_sequence = {static_cast<int8_t>(clamped_mode)};
    modeSchedulePub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
                "Published default mode schedule (mode=%d, reason=%s).",
                clamped_mode,
                reason.c_str());
  }

  void onTimer() {
    if (trackingFinished_) {
      return;
    }

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
        std::lock_guard<std::mutex> lock(pinMtx_);
        const bool ok = computeEePose(*pin_, *mapping_, eeFrameId_, obs.state, p, q);
        if (ok) {
          centerP_ = p;
          q0_ = q;
        } else {
          RCLCPP_WARN(this->get_logger(), "Initial FK failed. Using origin pose.");
        }
      }

      const std::string type = toLowerCopy(trajectoryType_);
      if (type == "figure_eight" || type == "figure8" || type == "figure-eight") {
        FigureEightParams p;
        p.horizon_T = horizon_;
        p.dt = dt_;
        p.amplitude = amplitude_;
        p.frequency_hz = freqHz_;
        p.plane_axis = axis_;
        planner_ = std::make_unique<FigureEightPlanner>(centerP_, q0_, p);
      } else if (type == "linear_move" || type == "linearmove" || type == "linear-move" || type == "linear") {
        LinearMoveParams p;
        p.horizon_T = horizon_;
        p.dt = dt_;
        p.offset = linearMoveOffset_;
        p.euler_zyx = linearMoveEulerZyx_;
        p.offset_in_body_frame = linearMoveOffsetInToolFrame_;
        p.time_scaling = linearMoveTimeScaling_;
        planner_ = std::make_unique<LinearMovePlanner>(centerP_, q0_, p);
      } else if (type == "screw_move" || type == "screw-move" || type == "screw" || type == "screw_motion" || type == "screwmotion") {
        ScrewMoveParams p;
        p.horizon_T = horizon_;
        p.dt = dt_;
        p.u_hat = screwMoveUhat_;
        p.r = screwMoveR_;
        p.theta = screwMoveTheta_;
        p.expressed_in_body_frame = screwMoveInToolFrame_;
        p.time_scaling = screwMoveTimeScaling_;
        planner_ = std::make_unique<ScrewMovePlanner>(centerP_, q0_, p);
      } else if (type == "target_pose" || type == "pose_target" || type == "goal_pose" || type == "pose") {
        TargetPoseParams p;
        p.horizon_T = horizon_;
        p.dt = dt_;
        p.time_scaling = poseMoveTimeScaling_;
        p.target_p = poseMoveHasTarget_ ? poseMoveTargetP_ : centerP_;
        if (poseMoveHasOrientation_) {
          p.target_q = poseMoveTargetQ_;
          p.use_target_orientation = true;
        } else {
          p.target_q = q0_;
          p.use_target_orientation = false;
        }
        planner_ = std::make_unique<TargetPosePlanner>(centerP_, q0_, p);
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown trajectoryType='%s'. Falling back to figure_eight.", trajectoryType_.c_str());
        FigureEightParams p;
        p.horizon_T = horizon_;
        p.dt = dt_;
        p.amplitude = amplitude_;
        p.frequency_hz = freqHz_;
        p.plane_axis = axis_;
        planner_ = std::make_unique<FigureEightPlanner>(centerP_, q0_, p);
      }
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
  bool publishOnce_{false};
  bool publishedOnce_{false};
  bool trackingFinished_{false};
  std::string commandFrameId_{"command"};
  std::string trajectoryType_{"figure_eight"};
  Eigen::Vector3d linearMoveOffset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linearMoveEulerZyx_{Eigen::Vector3d::Zero()};
  bool linearMoveOffsetInToolFrame_{false};
  TimeScalingType linearMoveTimeScaling_{TimeScalingType::MinJerk};
  bool poseMoveHasTarget_{false};
  bool poseMoveHasOrientation_{false};
  Eigen::Vector3d poseMoveTargetP_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond poseMoveTargetQ_{Eigen::Quaterniond::Identity()};
  TimeScalingType poseMoveTimeScaling_{TimeScalingType::MinJerk};
  Eigen::Vector3d screwMoveUhat_{0.0, 1.0, 0.0};
  Eigen::Vector3d screwMoveR_{0.1, 0.0, 0.0};
  double screwMoveTheta_{1.5707963267948966};
  bool screwMoveInToolFrame_{true};
  TimeScalingType screwMoveTimeScaling_{TimeScalingType::MinJerk};
  bool interveneHoldOnDivergedOrFinished_{false};
  bool switchMpcControllerOnIntervention_{false};
  std::string controllerManagerSwitchService_{"/controller_manager/switch_controller"};
  std::string mpcControllerName_{"mpc_controller"};
  bool publishModeScheduleOnEnable_{true};
  int modeScheduleMode_{1};
  PlannedCartesianTrajectory publishedTraj_;
  bool havePublishedTraj_{false};

  // OCS2 / Pinocchio
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::PinocchioInterface> pin_;
  ocs2::mobile_manipulator::ManipulatorModelInfo modelInfo_;
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping> mapping_;
  std::size_t eeFrameId_{0};
  std::mutex pinMtx_;

  // Planner
  bool initialized_{false};
  double startTime_{0.0};
  Eigen::Vector3d centerP_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q0_{Eigen::Quaterniond::Identity()};
  std::unique_ptr<CartesianTrajectoryPlanner> planner_;

  // ROS I/O
  std::unique_ptr<TrajectoryPublisher> trajPub_;
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr modeSchedulePub_;
  rclcpp::TimerBase::SharedPtr modeScheduleStartupTimer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggleTrackingSrv_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switchControllerClient_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trackingStatusSub_;
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
