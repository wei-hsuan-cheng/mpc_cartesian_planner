# mpc_cartesian_planner

Cartesian trajectory/path planner for OCS2 MPC.

## Prerequisites

Install ROS 2 Humble (or a newer distro with the same APIs).

## Build and Run Demo

```bash
# Clone the related repositories
git clone \
  --recursive https://github.com/wei-hsuan-cheng/ocs2_ros2.git \
  -b humble_stable

git clone \
  https://github.com/wei-hsuan-cheng/mobile_manipulator_mpc.git \
  -b humble_stable

git clone \
  https://github.com/wei-hsuan-cheng/mpc_controller.git \
  -b humble_stable

git clone \
  https://github.com/wei-hsuan-cheng/mpc_cartesian_planner.git \
  -b humble_stable

# rosdep install
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and install
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-up-to mpc_controller mpc_cartesian_planner \
  --parallel-workers 4 --executor sequential \
  && . install/setup.bash
```

## Trajectory Types

`trajectory_tt_publisher` supports these `trajectoryType` values (see `config/tt_params.yaml`):

- `figure_eight`: Periodic figure-eight around the initial EE pose.
- `linear_move`: Apply a relative offset (global frame or tool frame).
- `target_pose`: Move in a straight line (global frame) from the initial EE pose to an absolute target pose.
- `screw_move`: Relative 6D screw motion starting from the current EE pose.

Finite trajectories (`linear_move`, `target_pose`, `screw_move`) use `timeScaling` (`linear` / `min_jerk`).

## Pause / Resume

`trajectory_tt_publisher` provides a pause/resume service:

```bash
# Pause (publish HOLD at current EE pose)
ros2 service call /<robotName>/trajectory_tracking/toggle_tt_publisher std_srvs/srv/SetBool "{data: false}"

# Resume
ros2 service call /<robotName>/trajectory_tracking/toggle_tt_publisher std_srvs/srv/SetBool "{data: true}"
```

If `publishModeScheduleOnEnable: true` (default), the TT publisher also publishes `/<robotName>_mode_schedule` (default: mode `1`) on enable/resume so you don't have to manually publish a `ModeSchedule` before activating `mpc_controller`.
