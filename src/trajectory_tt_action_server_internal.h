#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ocs2_mpc/SystemObservation.h>

#include <mobile_manipulator_mpc/MobileManipulatorPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"

namespace mpc_cartesian_planner::trajectory_tt_action_server_internal {

enum class GoalTrajectoryType {
  ScrewMove = 0,
  LinearMove,
  TargetPose,
  FigureEight
};

inline const char* goalTypeName(GoalTrajectoryType type) {
  switch (type) {
    case GoalTrajectoryType::ScrewMove:
      return "screw_move";
    case GoalTrajectoryType::LinearMove:
      return "linear_move";
    case GoalTrajectoryType::TargetPose:
      return "target_pose";
    case GoalTrajectoryType::FigureEight:
      return "figure_eight";
  }
  return "unknown";
}

inline std::string toLowerCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

inline TimeScalingType parseTimeScaling(const std::string& s) {
  const std::string v = toLowerCopy(s);
  if (v == "linear") return TimeScalingType::Linear;
  if (v == "min_jerk" || v == "minjerk" || v == "min-jerk" || v.empty()) return TimeScalingType::MinJerk;
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
  return 2;
}

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

inline std::size_t closestWaypointIndexWindow(const PlannedCartesianTrajectory& traj,
                                              const Eigen::Vector3d& p_now,
                                              std::size_t last_best,
                                              bool have_last_best,
                                              int window,
                                              int max_backtrack) {
  if (traj.empty() || traj.position.size() != traj.time.size() || traj.position.size() != traj.quat.size()) {
    return 0;
  }

  const int N = static_cast<int>(traj.size());
  const int k0 = have_last_best ? static_cast<int>(std::min(last_best, traj.size() - 1)) : 0;
  const int W = std::max(1, window);

  const int k_min = have_last_best ? std::max(0, k0 - W) : 0;
  const int k_max = have_last_best ? std::min(N - 1, k0 + W) : (N - 1);

  double best_d2 = std::numeric_limits<double>::infinity();
  int best_k = k0;
  for (int k = k_min; k <= k_max; ++k) {
    const auto& pd = traj.position[static_cast<std::size_t>(k)];
    const double d2 = (p_now - pd).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best_k = k;
    }
  }

  if (have_last_best) {
    const int mb = std::max(0, max_backtrack);
    if (best_k + mb < k0) {
      best_k = k0;
    }
  }
  return static_cast<std::size_t>(best_k);
}

inline bool sampleTrajectoryAtTime(const PlannedCartesianTrajectory& traj,
                                   double t,
                                   Eigen::Vector3d& p_out,
                                   Eigen::Quaterniond& q_out) {
  if (traj.empty() || traj.time.size() != traj.position.size() || traj.time.size() != traj.quat.size()) {
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

struct GoalConfig {
  GoalTrajectoryType trajectory_type{GoalTrajectoryType::ScrewMove};
  double duration{0.0};
  double dt{0.0};
  TimeScalingType time_scaling{TimeScalingType::MinJerk};

  Eigen::Vector3d linear_offset{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_euler_zyx{Eigen::Vector3d::Zero()};
  bool linear_in_tool_frame{false};

  Eigen::Vector3d target_p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_q{Eigen::Quaterniond::Identity()};
  bool use_target_orientation{false};

  Eigen::Vector3d screw_uhat{0.0, 1.0, 0.0};
  Eigen::Vector3d screw_r{0.0, 0.0, 0.0};
  double screw_theta{0.0};
  bool screw_in_tool_frame{true};

  double figure_eight_amplitude{0.2};
  double figure_eight_frequency{0.1};
  Eigen::Vector3d figure_eight_plane_axis{Eigen::Vector3d::UnitZ()};
};

struct MonitorSnapshot {
  std::string status{"NOT_READY"};
  double progress{0.0};
  double pos_err{0.0};
  double ori_err_deg{0.0};
  double lag_sec{0.0};
  std::size_t closest_index{0};
  bool valid{false};
};

enum class GoalTermination {
  Succeeded = 0,
  Aborted,
  Canceled
};

}  // namespace mpc_cartesian_planner::trajectory_tt_action_server_internal
