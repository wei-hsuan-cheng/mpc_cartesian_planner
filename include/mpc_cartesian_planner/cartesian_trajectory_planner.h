#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <ocs2_mpc/SystemObservation.h>

namespace mpc_cartesian_planner {

struct PlannedCartesianTrajectory {
  std::vector<double> time;                 // seconds (OCS2 time)
  std::vector<Eigen::Vector3d> position;    // world frame
  std::vector<Eigen::Quaterniond> quat;     // world frame
  bool empty() const { return time.empty(); }
  std::size_t size() const { return time.size(); }
};

struct FigureEightParams {
  double horizon_T = 2.0;
  double dt = 0.05;
  double amplitude = 0.2;      // meters
  double frequency_hz = 0.1;   // Hz
  Eigen::Vector3d plane_axis = Eigen::Vector3d::UnitZ();  // unit vector normal to plane
};

class CartesianTrajectoryPlanner {
 public:
  virtual ~CartesianTrajectoryPlanner() = default;
  virtual PlannedCartesianTrajectory plan(const ocs2::SystemObservation& obs) = 0;
};

class FigureEightPlanner final : public CartesianTrajectoryPlanner {
 public:
  FigureEightPlanner(const Eigen::Vector3d& center_p,
                     const Eigen::Quaterniond& q0,
                     FigureEightParams params);

  PlannedCartesianTrajectory plan(const ocs2::SystemObservation& obs) override;

  void setCenter(const Eigen::Vector3d& p) { center_p_ = p; }
  void setOrientation(const Eigen::Quaterniond& q) { q0_ = q; }
  void setStartTime(double t_start) { start_time_ = t_start; have_start_time_ = true; }

 private:
  static void orthonormalBasisFromAxis(const Eigen::Vector3d& axis_unit,
                                       Eigen::Vector3d& u,
                                       Eigen::Vector3d& v);

 private:
  Eigen::Vector3d center_p_;
  Eigen::Quaterniond q0_;
  FigureEightParams params_;
  double start_time_{0.0};
  bool have_start_time_{false};
};

}  // namespace mpc_cartesian_planner
