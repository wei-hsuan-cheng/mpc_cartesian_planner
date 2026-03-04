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

## Path-based timing (spatial projection)

By default, the TT reference is time-indexed (the target at time `t` is fixed once planned). If your low-level position controller introduces phase lag, you can retime the published target so `t_now` aligns to the closest spatial waypoint:

- Set `referenceTiming: spatial_projection` in `config/tt_params.yaml`.
- Tune `projectionWindow` / `projectionMaxBacktrack` (windowed nearest search + monotonic guard).

This mode needs `taskFile` / `urdfFile` / `libFolder` so the TT publisher can compute the current end-effector pose via FK.

## Reactive Delta Pose

You can subscribe a `geometry_msgs/msg/PoseStamped` delta pose and apply it to the nominal TT before publishing:

- `deltaPoseTopic`: source topic (set empty string to disable)
- `deltaPoseInToolFrame`: if `true`, delta is interpreted in each waypoint's tool frame
- `deltaPoseTimeout`: if `> 0`, stale deltas older than this timeout are ignored

The defaults are in `config/tt_params.yaml` (example topic: `/admittance_controller/delta_pose`).

## Pause / Resume

`trajectory_tt_publisher` provides a pause/resume service:

```bash
# Start/resume
ros2 service call /<robotName>/trajectory_tracking/toggle_tt_publisher std_srvs/srv/SetBool "{data: true}"
# Pause (publish HOLD at current EE pose)
ros2 service call /<robotName>/trajectory_tracking/toggle_tt_publisher std_srvs/srv/SetBool "{data: false}"

# E.g., for mobile_manipulator
ros2 service call /mobile_manipulator/trajectory_tracking/toggle_tt_publisher std_srvs/srv/SetBool "{data: true}"
ros2 service call /mobile_manipulator/trajectory_tracking/toggle_tt_publisher std_srvs/srv/SetBool "{data: false}"
```

If `publishModeScheduleOnEnable: true` (default), the TT publisher also publishes `/<robotName>_mode_schedule` (default: mode `1`) on enable/resume so you don't have to manually publish a `ModeSchedule` before activating `mpc_controller`.

## EE trajectory tracking action examples:

```bash
# Screw move
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_screw_move \
  mpc_cartesian_planner/action/ExecuteScrewMove \
  "{
    duration: 5.0, 
    dt: 0.02, 
    time_scaling: 'min_jerk', 
    screw_uhat: [0.0, 1.0, 0.0], 
    screw_r: [-0.2, 0.0, 0.0], 
    screw_theta: 0.7854, 
    screw_in_tool_frame: true
  }"

# Linear move: [dx, dy, dz, thz, thy, thx]
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_linear_move \
  mpc_cartesian_planner/action/ExecuteLinearMove \
  "{
    duration: 5.0, 
    dt: 0.02, 
    time_scaling: 'min_jerk', 
    linear_move_offset: [0.0, 0.0, 0.1, 0.0, 0.7854, 0.0],
    linear_move_in_tool_frame: true
  }"

# Target pose: [x, y, z, qw, qx, qy, qz]
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_target_pose \
  mpc_cartesian_planner/action/ExecuteTargetPose \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: 'min_jerk',
    target_pose: [0.45, 0.2, 0.7, 0.707, 0.0, 0.0, -0.707]
  }"

# Figure eight
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_figure_eight \
  mpc_cartesian_planner/action/ExecuteFigureEight \
  "{
    duration: 5.0,
    dt: 0.02,
    amplitude: 0.20,
    frequency: 0.05,
    plane_axis: [1.0, 0.0, 1.0]
  }"
```

## Combined EE/base trajectory tracking action examples:

```bash
# 1) Pure EE TT
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_target_pose \
  mpc_cartesian_planner/action/ExecuteTargetPose \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: min_jerk,
    target_pose: [0.45, 0.20, 1.0, 0.707, 0.0, 0.0, -0.707]
  }"

# 2) Pure base TT: absolute SE(2) target pose
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_target_pose \
  mpc_cartesian_planner/action/ExecuteCombinedTargetPose \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: false, hold_ee: false, ee_target_pose: [],
    enable_base: true, hold_base: false,
    base_target_pose: [1.0, 0.4, 1.5708]
  }"

# 3) Pure base TT: relative linear move in base frame [dx, dy, dyaw]
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_linear_move \
  mpc_cartesian_planner/action/ExecuteCombinedLinearMove \
  "{
    duration: 4.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: false, hold_ee: false, ee_linear_move_offset: [],
    ee_linear_move_in_tool_frame: true,
    enable_base: true, hold_base: false,
    base_linear_move_offset: [0.5, 0.0, 0.0],
    base_linear_move_in_body_frame: true
  }"

# 4) Fixed EE in space while base moves
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_linear_move \
  mpc_cartesian_planner/action/ExecuteCombinedLinearMove \
  "{
    duration: 4.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: true, hold_ee: true, ee_linear_move_offset: [],
    ee_linear_move_in_tool_frame: true,
    enable_base: true, hold_base: false,
    base_linear_move_offset: [0.4, 0.0, 0.5236],
    base_linear_move_in_body_frame: true
  }"

# 5) EE + base combined target pose
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_target_pose \
  mpc_cartesian_planner/action/ExecuteCombinedTargetPose \
  "{
    duration: 6.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: true, hold_ee: false,
    ee_target_pose: [0.55, -0.05, 0.68, 0.707, 0.0, 0.0, -0.707],
    enable_base: true, hold_base: false,
    base_target_pose: [0.8, -0.2, 0.7854]
  }"

# 6) Pure base screw/arc move
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_screw_move \
  mpc_cartesian_planner/action/ExecuteCombinedScrewMove \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: false, hold_ee: false,
    ee_screw_uhat: [0.0, 0.0, 1.0],
    ee_screw_r: [0.0, 0.0, 0.0],
    ee_screw_theta: 0.0,
    ee_screw_in_tool_frame: true,
    enable_base: true, hold_base: false,
    base_screw_r: [0.0, -0.5],
    base_screw_theta: 1.5708,
    base_screw_in_body_frame: true
  }"
```
