#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"

#include <algorithm>
#include <cmath>

#include "robot_math_utils/robot_math_utils_v1_18.hpp"

namespace mpc_cartesian_planner {

namespace {

inline double evalTimeScaling(TimeScalingType type, double tau01) {
  const double t = std::clamp(tau01, 0.0, 1.0);
  switch (type) {
    case TimeScalingType::Linear: return t;
    case TimeScalingType::MinJerk: {
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double t4 = t3 * t;
      const double t5 = t4 * t;
      return 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
    }
    default: return t;
  }
}

}  // namespace

FigureEightPlanner::FigureEightPlanner(const Eigen::Vector3d& center_p,
                                       const Eigen::Quaterniond& q0,
                                       FigureEightParams params)
    : center_p_(center_p), q0_(q0), params_(std::move(params)) {
  if (params_.plane_axis.norm() < 1e-9) {
    params_.plane_axis = Eigen::Vector3d::UnitZ();
  }
  params_.plane_axis.normalize();
}

void FigureEightPlanner::orthonormalBasisFromAxis(const Eigen::Vector3d& axis_unit,
                                                  Eigen::Vector3d& u,
                                                  Eigen::Vector3d& v) {
  Eigen::Vector3d ref =
      (std::abs(axis_unit.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  u = axis_unit.cross(ref);
  double n = u.norm();
  if (n < 1e-9) {
    ref = Eigen::Vector3d::UnitY();
    u = axis_unit.cross(ref);
    n = u.norm();
  }
  u /= n;
  v = axis_unit.cross(u);
}

PlannedCartesianTrajectory FigureEightPlanner::plan(const ocs2::SystemObservation& obs) {
  const double t0 = obs.time;
  if (!have_start_time_) {
    start_time_ = t0;
    have_start_time_ = true;
  }

  const int N = std::max(1, static_cast<int>(std::ceil(params_.horizon_T / std::max(1e-9, params_.dt))));
  PlannedCartesianTrajectory out;
  out.time.reserve(N + 1);
  out.position.reserve(N + 1);
  out.quat.reserve(N + 1);

  const double omega = 2.0 * M_PI * params_.frequency_hz;
  Eigen::Vector3d u, v;
  orthonormalBasisFromAxis(params_.plane_axis, u, v);

  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * params_.dt;
    const double tau = tk - start_time_;
    const double x = params_.amplitude * std::sin(omega * tau);
    const double y = 0.5 * params_.amplitude * std::sin(2.0 * omega * tau);
    const Eigen::Vector3d p = center_p_ + u * x + v * y;

    out.time.push_back(tk);
    out.position.push_back(p);
    out.quat.push_back(q0_.normalized());  // constant orientation
  }
  return out;
}

LinearMovePlanner::LinearMovePlanner(const Eigen::Vector3d& start_p,
                                     const Eigen::Quaterniond& start_q,
                                     LinearMoveParams params)
    : p0_(start_p), q0_(start_q.normalized()), params_(std::move(params)) {
  // Build the relative command pose e->e_cmd and convert it to (pos, so3) coordinates:
  //   pos_so3 = [p; log(R)]
  PosQuat pos_quat_b_e(p0_, q0_);

  const Eigen::Quaterniond dq =
      RMUtils::zyxEuler2Quat(params_.euler_zyx, /*rad=*/true).normalized();

  PosQuat pos_quat_e_e_cmd;
  if (params_.offset_in_body_frame) {
    // Interpret the offset as a relative pose expressed in the initial EE/tool frame.
    pos_quat_e_e_cmd = PosQuat(params_.offset, dq);
  } else {
    // Interpret the offset as a delta in the global frame (translation in base, rotation about base axes).
    const Eigen::Vector3d p_goal = p0_ + params_.offset;
    const Eigen::Quaterniond q_goal = (dq * q0_).normalized();
    const PosQuat pos_quat_b_e_goal(p_goal, q_goal);
    pos_quat_e_e_cmd = RMUtils::PosQuats2RelativePosQuat(pos_quat_b_e, pos_quat_b_e_goal);
  }

  pos_so3_e_e_cmd_ = RMUtils::PosQuat2Posso3(pos_quat_e_e_cmd);
}

PlannedCartesianTrajectory LinearMovePlanner::plan(const ocs2::SystemObservation& obs) {
  const double t0 = obs.time;
  if (!have_start_time_) {
    start_time_ = t0;
    have_start_time_ = true;
  }

  const double dt = std::max(1e-9, params_.dt);
  const double T = std::max(1e-9, params_.horizon_T);
  const int N = std::max(1, static_cast<int>(std::ceil(T / dt)));

  PlannedCartesianTrajectory out;
  out.time.reserve(N + 1);
  out.position.reserve(N + 1);
  out.quat.reserve(N + 1);

  const PosQuat pos_quat_b_e(p0_, q0_);

  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * dt;
    const double tau01 = (tk - start_time_) / T;
    const double s = evalTimeScaling(params_.time_scaling, tau01);

    const Eigen::Matrix<double, 6, 1> pos_so3_e_e_cmd_i = s * pos_so3_e_e_cmd_;
    const PosQuat pos_quat_e_e_cmd_i = RMUtils::Posso32PosQuat(pos_so3_e_e_cmd_i);
    const PosQuat pos_quat_b_e_d_i = RMUtils::TransformPosQuats({pos_quat_b_e, pos_quat_e_e_cmd_i});

    out.time.push_back(tk);
    out.position.push_back(pos_quat_b_e_d_i.pos);
    out.quat.push_back(pos_quat_b_e_d_i.quat.normalized());
  }
  return out;
}

TargetPosePlanner::TargetPosePlanner(const Eigen::Vector3d& start_p,
                                     const Eigen::Quaterniond& start_q,
                                     TargetPoseParams params)
    : p0_(start_p), q0_(start_q.normalized()), params_(std::move(params)) {
  if (params_.target_q.norm() < 1e-12) {
    params_.target_q = Eigen::Quaterniond::Identity();
  } else {
    params_.target_q.normalize();
  }
}

PlannedCartesianTrajectory TargetPosePlanner::plan(const ocs2::SystemObservation& obs) {
  const double t0 = obs.time;
  if (!have_start_time_) {
    start_time_ = t0;
    have_start_time_ = true;
  }

  const double dt = std::max(1e-9, params_.dt);
  const double T = std::max(1e-9, params_.horizon_T);
  const int N = std::max(1, static_cast<int>(std::ceil(T / dt)));

  PlannedCartesianTrajectory out;
  out.time.reserve(N + 1);
  out.position.reserve(N + 1);
  out.quat.reserve(N + 1);

  const Eigen::Vector3d dp = params_.target_p - p0_;

  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * dt;
    const double tau01 = (tk - start_time_) / T;
    const double s = evalTimeScaling(params_.time_scaling, tau01);

    const Eigen::Vector3d p = p0_ + s * dp;
    const Eigen::Quaterniond q = params_.use_target_orientation
                                     ? q0_.slerp(s, params_.target_q).normalized()
                                     : q0_;

    out.time.push_back(tk);
    out.position.push_back(p);
    out.quat.push_back(q);
  }
  return out;
}

ScrewMovePlanner::ScrewMovePlanner(const Eigen::Vector3d& start_p,
                                   const Eigen::Quaterniond& start_q,
                                   ScrewMoveParams params)
    : p0_(start_p), q0_(start_q.normalized()), params_(std::move(params)) {
  if (params_.u_hat.norm() < 1e-12) {
    params_.u_hat = Eigen::Vector3d::UnitZ();
  } else {
    params_.u_hat.normalize();
  }

  const Eigen::Vector3d v = params_.u_hat.cross(params_.r);
  se3_cmd_.head<3>() = v * params_.theta;
  se3_cmd_.tail<3>() = params_.u_hat * params_.theta;
}

PlannedCartesianTrajectory ScrewMovePlanner::plan(const ocs2::SystemObservation& obs) {
  const double t0 = obs.time;
  if (!have_start_time_) {
    start_time_ = t0;
    have_start_time_ = true;
  }

  const double dt = std::max(1e-9, params_.dt);
  const double T = std::max(1e-9, params_.horizon_T);
  const int N = std::max(1, static_cast<int>(std::ceil(T / dt)));

  PlannedCartesianTrajectory out;
  out.time.reserve(N + 1);
  out.position.reserve(N + 1);
  out.quat.reserve(N + 1);

  const PosQuat pos_quat_b_e0(p0_, q0_);
  const Eigen::Matrix4d T_b_e0 = RMUtils::PosQuat2TMat(pos_quat_b_e0);

  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * dt;
    const double tau01 = (tk - start_time_) / T;
    const double s = evalTimeScaling(params_.time_scaling, tau01);

    const Eigen::Matrix<double, 6, 1> se3_cmd_i = s * se3_cmd_;
    const Eigen::Matrix4d T_delta = RMUtils::MatrixExp6(RMUtils::R6Vec2se3Mat(se3_cmd_i));
    const Eigen::Matrix4d T_b_ed = params_.expressed_in_body_frame ? (T_b_e0 * T_delta) : (T_delta * T_b_e0);
    const PosQuat pos_quat_b_ed = RMUtils::TMat2PosQuat(T_b_ed);

    out.time.push_back(tk);
    out.position.push_back(pos_quat_b_ed.pos);
    out.quat.push_back(pos_quat_b_ed.quat.normalized());
  }
  return out;
}

}  // namespace mpc_cartesian_planner
