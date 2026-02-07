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

enum class TimeScalingType {
  Linear = 0,
  MinJerk
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
  virtual void setStartTime(double /*t_start*/) {}
};

class FigureEightPlanner final : public CartesianTrajectoryPlanner {
 public:
  FigureEightPlanner(const Eigen::Vector3d& center_p,
                     const Eigen::Quaterniond& q0,
                     FigureEightParams params);

  PlannedCartesianTrajectory plan(const ocs2::SystemObservation& obs) override;

  void setCenter(const Eigen::Vector3d& p) { center_p_ = p; }
  void setOrientation(const Eigen::Quaterniond& q) { q0_ = q; }
  void setStartTime(double t_start) override { start_time_ = t_start; have_start_time_ = true; }

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

struct LinearMoveParams {
  double horizon_T = 2.0;
  double dt = 0.05;
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();     // translation (meters)
  Eigen::Vector3d euler_zyx = Eigen::Vector3d::Zero();  // rotation offset [thz, thy, thx] (radians)
  bool offset_in_body_frame = false;                      // if true, interpret offset+euler_zyx in the initial EE/tool frame
  TimeScalingType time_scaling = TimeScalingType::MinJerk;
};

class LinearMovePlanner final : public CartesianTrajectoryPlanner {
 public:
  LinearMovePlanner(const Eigen::Vector3d& start_p,
                    const Eigen::Quaterniond& start_q,
                    LinearMoveParams params);

  PlannedCartesianTrajectory plan(const ocs2::SystemObservation& obs) override;
  void setStartTime(double t_start) override { start_time_ = t_start; have_start_time_ = true; }

 private:
  Eigen::Vector3d p0_;
  Eigen::Quaterniond q0_;
  Eigen::Matrix<double, 6, 1> pos_so3_e_e_cmd_{Eigen::Matrix<double, 6, 1>::Zero()};
  LinearMoveParams params_;
  double start_time_{0.0};
  bool have_start_time_{false};
};

struct TargetPoseParams {
  double horizon_T = 2.0;
  double dt = 0.05;
  Eigen::Vector3d target_p = Eigen::Vector3d::Zero();               // world frame
  Eigen::Quaterniond target_q = Eigen::Quaterniond::Identity();     // world frame
  bool use_target_orientation = true;                               // if false, keep the initial orientation
  TimeScalingType time_scaling = TimeScalingType::MinJerk;
};

class TargetPosePlanner final : public CartesianTrajectoryPlanner {
 public:
  TargetPosePlanner(const Eigen::Vector3d& start_p,
                    const Eigen::Quaterniond& start_q,
                    TargetPoseParams params);

  PlannedCartesianTrajectory plan(const ocs2::SystemObservation& obs) override;
  void setStartTime(double t_start) override { start_time_ = t_start; have_start_time_ = true; }

 private:
  Eigen::Vector3d p0_;
  Eigen::Quaterniond q0_;
  TargetPoseParams params_;
  double start_time_{0.0};
  bool have_start_time_{false};
};

struct ScrewMoveParams {
  double horizon_T = 2.0;
  double dt = 0.05;
  Eigen::Vector3d u_hat = Eigen::Vector3d::UnitY();  // screw axis direction
  Eigen::Vector3d r = Eigen::Vector3d::Zero();       // axis->TCP vector (||r|| is the radius)
  double theta = 0.0;                                // total rotation about axis (radians)
  bool expressed_in_body_frame = true;               // true: (u_hat, r) in initial EE/tool frame; false: in world frame
  TimeScalingType time_scaling = TimeScalingType::MinJerk;
};

class ScrewMovePlanner final : public CartesianTrajectoryPlanner {
 public:
  ScrewMovePlanner(const Eigen::Vector3d& start_p,
                   const Eigen::Quaterniond& start_q,
                   ScrewMoveParams params);

  PlannedCartesianTrajectory plan(const ocs2::SystemObservation& obs) override;
  void setStartTime(double t_start) override { start_time_ = t_start; have_start_time_ = true; }

 private:
  Eigen::Vector3d p0_;
  Eigen::Quaterniond q0_;
  Eigen::Matrix<double, 6, 1> se3_cmd_{Eigen::Matrix<double, 6, 1>::Zero()};  // [v*theta; w*theta]
  ScrewMoveParams params_;
  double start_time_{0.0};
  bool have_start_time_{false};
};

}  // namespace mpc_cartesian_planner
