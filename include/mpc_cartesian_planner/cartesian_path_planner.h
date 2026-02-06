#pragma once

// Backward-compatibility header. Prefer including:
//   mpc_cartesian_planner/cartesian_trajectory_planner.h

#include "mpc_cartesian_planner/cartesian_trajectory_planner.h"

namespace mpc_cartesian_planner {

using CartesianPathPlanner = CartesianTrajectoryPlanner;

}  // namespace mpc_cartesian_planner
