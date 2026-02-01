#include "mpc_cartesian_planner/cartesian_path_planner.h"

#include <cmath>

namespace mpc_cartesian_planner {

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

}  // namespace mpc_cartesian_planner
