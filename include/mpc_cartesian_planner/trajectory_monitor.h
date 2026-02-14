#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"

namespace mpc_cartesian_planner {

enum class TrackingStatus {
  NOT_READY = 0,
  ON_TRACK,
  LAGGING,
  DIVERGED,
  FINISHED
};

struct TrackingMetrics {
  double progress = 0.0;     // 0..1
  double pos_err = 0.0;      // meters
  double ori_err_deg = 0.0;  // degrees
  double lag_sec = 0.0;      // t_now - t_closest
  std::size_t closest_index = 0;
  TrackingStatus status = TrackingStatus::NOT_READY;
};

struct MonitorParams {
  int window = 20;                 // +/- samples for window search
  int max_backtrack = 5;           // prevent big backward jumps at self-intersection
  double update_dt = 0.05;         // monitor update period (seconds)
  double on_track_pos = 0.005;     // 5 mm
  double on_track_ori_deg = 5.0;   // deg
  double diverged_pos = 0.03;      // 3 cm
  double diverged_hold_sec = 0.5;  // must be above diverged_pos for this long
  double finish_progress = 0.98;
  double finish_pos = 0.005;
};

class TrajectoryMonitor {
 public:
  explicit TrajectoryMonitor(MonitorParams params);

  void setTrajectory(PlannedCartesianTrajectory traj);

  // Update the stored trajectory without resetting the window-search state.
  // Useful when the sample poses are unchanged but their timestamps are retimed.
  void updateTrajectory(PlannedCartesianTrajectory traj);

  TrackingMetrics update(double t_now,
                         const Eigen::Vector3d& p_now,
                         const Eigen::Quaterniond& q_now);

  bool hasTrajectory() const { return !traj_.empty(); }

 private:
  static double orientationErrorDeg(const Eigen::Quaterniond& q_des,
                                    const Eigen::Quaterniond& q);

 private:
  MonitorParams params_;
  PlannedCartesianTrajectory traj_;
  std::size_t last_best_{0};
  double diverged_accum_{0.0};
  double last_update_time_{0.0};
  bool have_last_update_time_{false};
};

std::string toString(TrackingStatus s);

}  // namespace mpc_cartesian_planner
