#include "mpc_cartesian_planner/trajectory_monitor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mpc_cartesian_planner {

TrajectoryMonitor::TrajectoryMonitor(MonitorParams params) : params_(params) {}

void TrajectoryMonitor::setTrajectory(PlannedCartesianTrajectory traj) {
  traj_ = std::move(traj);
  last_best_ = 0;
  diverged_accum_ = 0.0;
  have_last_update_time_ = false;
}

void TrajectoryMonitor::updateTrajectory(PlannedCartesianTrajectory traj) {
  const std::size_t old_last_best = last_best_;
  traj_ = std::move(traj);
  if (traj_.empty()) {
    last_best_ = 0;
    diverged_accum_ = 0.0;
    have_last_update_time_ = false;
    return;
  }
  last_best_ = std::min(old_last_best, traj_.size() - 1);
}

double TrajectoryMonitor::orientationErrorDeg(const Eigen::Quaterniond& q_des,
                                              const Eigen::Quaterniond& q) {
  Eigen::Quaterniond dq = q_des.normalized().conjugate() * q.normalized();
  dq.normalize();
  const double angle = 2.0 * std::acos(std::clamp(std::abs(dq.w()), 0.0, 1.0));
  return angle * 180.0 / M_PI;
}

TrackingMetrics TrajectoryMonitor::update(double t_now,
                                         const Eigen::Vector3d& p_now,
                                         const Eigen::Quaterniond& q_now) {
  TrackingMetrics m;
  if (traj_.empty()) {
    m.status = TrackingStatus::NOT_READY;
    have_last_update_time_ = false;
    return m;
  }

  double dt_update = std::max(1e-9, params_.update_dt);
  if (have_last_update_time_) {
    const double dt_meas = t_now - last_update_time_;
    if (std::isfinite(dt_meas) && dt_meas > 0.0) {
      dt_update = dt_meas;
    }
  }
  last_update_time_ = t_now;
  have_last_update_time_ = true;

  const int N = static_cast<int>(traj_.size());
  const int k0 = static_cast<int>(last_best_);
  const int W = std::max(1, params_.window);

  const int k_min = std::max(0, k0 - W);
  const int k_max = std::min(N - 1, k0 + W);

  // Window search: nearest sample in the local window
  double best_d2 = std::numeric_limits<double>::infinity();
  int best_k = k0;

  for (int k = k_min; k <= k_max; ++k) {
    const Eigen::Vector3d& pd = traj_.position[static_cast<std::size_t>(k)];
    const double d2 = (p_now - pd).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best_k = k;
    }
  }

  // Monotonic guard: avoid jumping to another loop at self-intersection
  if (best_k + params_.max_backtrack < k0) {
    best_k = k0;
  }
  last_best_ = static_cast<std::size_t>(best_k);

  const Eigen::Vector3d& p_des = traj_.position[last_best_];
  const Eigen::Quaterniond& q_des = traj_.quat[last_best_];

  m.closest_index = last_best_;
  m.progress = (N <= 1) ? 1.0 : static_cast<double>(last_best_) / static_cast<double>(N - 1);
  m.pos_err = (p_now - p_des).norm();
  m.ori_err_deg = orientationErrorDeg(q_des, q_now);
  m.lag_sec = t_now - traj_.time[last_best_];

  const bool finished = (m.progress >= params_.finish_progress) && (m.pos_err <= params_.finish_pos);
  if (finished) {
    m.status = TrackingStatus::FINISHED;
    diverged_accum_ = 0.0;
    return m;
  }

  const bool on_track = (m.pos_err <= params_.on_track_pos) && (m.ori_err_deg <= params_.on_track_ori_deg);
  if (on_track) {
    m.status = TrackingStatus::ON_TRACK;
    diverged_accum_ = 0.0;
    return m;
  }

  // Divergence: error large for some duration (simple accumulator)
  if (m.pos_err >= params_.diverged_pos) {
    diverged_accum_ += dt_update;
    if (diverged_accum_ >= params_.diverged_hold_sec) {
      m.status = TrackingStatus::DIVERGED;
      return m;
    }
  } else {
    diverged_accum_ = 0.0;
  }

  m.status = TrackingStatus::LAGGING;
  return m;
}

std::string toString(TrackingStatus s) {
  switch (s) {
    case TrackingStatus::NOT_READY: return "NOT_READY";
    case TrackingStatus::ON_TRACK:  return "ON_TRACK";
    case TrackingStatus::LAGGING:   return "LAGGING";
    case TrackingStatus::DIVERGED:  return "DIVERGED";
    case TrackingStatus::FINISHED:  return "FINISHED";
    default: return "UNKNOWN";
  }
}

}  // namespace mpc_cartesian_planner
