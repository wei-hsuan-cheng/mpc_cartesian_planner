# TT (Trajectory Tracking) and Monitoring

This package provides two ROS 2 nodes:

- `trajectory_tt_publisher_node`: generates a **Cartesian end-effector target trajectory** (TT) and publishes it as an OCS2 `MpcTargetTrajectories` message. See [`src/trajectory_tt_publisher_node.cpp#L104`](../src/trajectory_tt_publisher_node.cpp#L104).
- `trajectory_progress_monitor_node`: consumes the published target trajectory and estimates **tracking progress / error** using windowed nearest-neighbor matching. See [`src/trajectory_progress_monitor_node.cpp#L103`](../src/trajectory_progress_monitor_node.cpp#L103).

Launch + parameters:
- Demo launch file: [`launch/tt_demo.launch.py#L8`](../launch/tt_demo.launch.py#L8)
- Default params: [`config/tt_params.yaml#L1`](../config/tt_params.yaml#L1)

## 1) What “TT” means here

OCS2-based MPC expects a target trajectory message (`MpcTargetTrajectories`) on `<robotName>_mpc_target`. The TT publisher node produces such a target in *Cartesian pose space*:

- State layout used in this repo: **`[px, py, pz, qx, qy, qz, qw]`**. See [`src/trajectory_publisher.cpp#L24`](../src/trajectory_publisher.cpp#L24).
- The TT publisher *only publishes the reference*; the actual tracking is done by your controller (*e.g.*, in `mpc_controller`).

## 2) TT publisher: code path and logic

### 2.1 Inputs / outputs

- Subscribes: `<robotName>_mpc_observation` (`ocs2_msgs/msg/MpcObservation`). See [`src/trajectory_tt_publisher_node.cpp#L169`](../src/trajectory_tt_publisher_node.cpp#L169).
- Publishes: `<robotName>_mpc_target` (`ocs2_msgs/msg/MpcTargetTrajectories`) via `TrajectoryPublisher`. See [`src/trajectory_publisher.cpp#L8`](../src/trajectory_publisher.cpp#L8).
- Optional visualization:
  - `/mobile_manipulator/targetStateTrajectory` (`MarkerArray`). See [`src/trajectory_tt_publisher_node.cpp#L159`](../src/trajectory_tt_publisher_node.cpp#L159).
  - `/mobile_manipulator/targetPoseTrajectory` (`PoseArray`). See [`src/trajectory_tt_publisher_node.cpp#L159`](../src/trajectory_tt_publisher_node.cpp#L159).
  - TF: `<trajectoryGlobalFrame> -> <commandFrameId>` (default: `command`). See [`src/trajectory_tt_publisher_node.cpp#L311`](../src/trajectory_tt_publisher_node.cpp#L311).

### 2.2 One-time initialization (first observation)

On the first timer tick where an observation is available, the node:

1) Stores the start time `startTime_ = obs.time`. See [`src/trajectory_tt_publisher_node.cpp#L195`](../src/trajectory_tt_publisher_node.cpp#L195).
2) If `taskFile/urdfFile/libFolder` are provided, it computes the initial end-effector pose using Pinocchio FK:
   - FK helper: [`src/trajectory_tt_publisher_node.cpp#L77`](../src/trajectory_tt_publisher_node.cpp#L77)
   - Initialization call site: [`src/trajectory_tt_publisher_node.cpp#L198`](../src/trajectory_tt_publisher_node.cpp#L198)
3) Constructs a `FigureEightPlanner` with `(centerP_, q0_)` and parameters from ROS params. See [`src/trajectory_tt_publisher_node.cpp#L210`](../src/trajectory_tt_publisher_node.cpp#L210).
4) Anchors the planner phase by setting `planner_->setStartTime(startTime_)`. See [`src/trajectory_tt_publisher_node.cpp#L217`](../src/trajectory_tt_publisher_node.cpp#L217).

If the model paths are not set, the node falls back to `centerP_ = 0` and `q0 = Identity` (and still publishes a trajectory). See [`src/trajectory_tt_publisher_node.cpp#L152`](../src/trajectory_tt_publisher_node.cpp#L152).

### 2.3 Periodic loop (each timer tick)

Each tick:

1) Grabs the latest observation. See [`src/trajectory_tt_publisher_node.cpp#L187`](../src/trajectory_tt_publisher_node.cpp#L187).
2) Plans a fresh horizon starting at the current observation time `t0 = obs.time`. See [`src/cartesian_trajectory_planner.cpp#L33`](../src/cartesian_trajectory_planner.cpp#L33).
3) Publishes the EE target trajectory as `MpcTargetTrajectories`. See [`src/trajectory_tt_publisher_node.cpp#L223`](../src/trajectory_tt_publisher_node.cpp#L223) and [`src/trajectory_publisher.cpp#L13`](../src/trajectory_publisher.cpp#L13).
4) Optionally publishes TF + visualization. See [`src/trajectory_tt_publisher_node.cpp#L228`](../src/trajectory_tt_publisher_node.cpp#L228).

Note: in the default *receding-horizon* behavior (publishing a new horizon each tick), the monitor’s `progress` (normalized waypoint index) will typically stay near zero, since the closest waypoint is usually near the start of the latest horizon.

If you want a **finite trajectory** with meaningful `progress` and a terminal `FINISHED` state, set `publishOnce: true` in the TT publisher params. In this mode the node publishes the target trajectory once (with `horizon` as the total duration) and then stops publishing.

## 3) Planner math (Figure Eight)

The planner interface and output container are:
- `PlannedCartesianTrajectory`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L11`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L11)
- `FigureEightPlanner`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L33`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L33)

### 3.1 Time discretization

Given:
- horizon length \(T\) (`horizon_T`)
- step \( \Delta t \) (`dt`)
- current time \(t_0 = \texttt{obs.time}\)

The code chooses:

$$
N = \left\lceil \frac{T}{\Delta t} \right\rceil,\quad
t_k = t_0 + k\Delta t,\quad k = 0,1,\dots,N
$$

See [`src/cartesian_trajectory_planner.cpp#L40`](../src/cartesian_trajectory_planner.cpp#L40) and [`src/cartesian_trajectory_planner.cpp#L50`](../src/cartesian_trajectory_planner.cpp#L50).

### 3.2 Plane basis from an axis

The trajectory is drawn in a plane whose normal is the user-provided axis \(n\) (`plane_axis`), normalized. The code constructs an orthonormal basis \((u,v)\) spanning the plane using cross products:

$$
u = \frac{n \times r}{\|n \times r\|}, \quad v = n \times u
$$

where \(r\) is a “reference” axis chosen to not be parallel to \(n\). See [`src/cartesian_trajectory_planner.cpp#L17`](../src/cartesian_trajectory_planner.cpp#L17).

### 3.3 Figure-eight parametric curve

Let:
- amplitude \(A\) (`amplitude`)
- frequency \(f\) (`frequency_hz`)
- angular frequency \(\omega = 2\pi f\)
- phase time \(\tau = t_k - t_{\text{start}}\) (anchored once at startup)

The code uses:

$$
x(\tau) = A\sin(\omega\tau), \quad
y(\tau) = \frac{A}{2}\sin(2\omega\tau)
$$

and maps to 3D:

$$
p(t_k) = p_c + u\,x(\tau) + v\,y(\tau)
$$

See [`src/cartesian_trajectory_planner.cpp#L46`](../src/cartesian_trajectory_planner.cpp#L46) and [`src/cartesian_trajectory_planner.cpp#L50`](../src/cartesian_trajectory_planner.cpp#L50).

Orientation is constant (the initial FK orientation):

$$
q(t_k) = q_0
$$

See [`src/cartesian_trajectory_planner.cpp#L59`](../src/cartesian_trajectory_planner.cpp#L59).

## 4) Publishing the target trajectory (OCS2 message)

`TrajectoryPublisher` converts `PlannedCartesianTrajectory` into an OCS2 `TargetTrajectories`, then to `MpcTargetTrajectories`.

- Topic is `<robotName>_mpc_target`. See [`src/trajectory_publisher.cpp#L8`](../src/trajectory_publisher.cpp#L8).
- Each sample is serialized as:

$$
x_k = [p_x, p_y, p_z, q_x, q_y, q_z, q_w]^\top
$$

See [`src/trajectory_publisher.cpp#L28`](../src/trajectory_publisher.cpp#L28).

- The input trajectory is filled with zeros with dimension `obs.input.size()` (so it matches the controller’s expected input size). See [`src/trajectory_publisher.cpp#L35`](../src/trajectory_publisher.cpp#L35).

## 5) Monitoring: code path and logic

### 5.1 Inputs / outputs

The monitor node:

- Subscribes:
  - `<robotName>_mpc_observation` for the current state/time. See [`src/trajectory_progress_monitor_node.cpp#L178`](../src/trajectory_progress_monitor_node.cpp#L178).
  - `<robotName>_mpc_target` for the reference trajectory. See [`src/trajectory_progress_monitor_node.cpp#L186`](../src/trajectory_progress_monitor_node.cpp#L186).
- Publishes:
  - `<robotName>/trajectory_tracking/tracking_status` (`std_msgs/String`). See [`src/trajectory_progress_monitor_node.cpp#L178`](../src/trajectory_progress_monitor_node.cpp#L178).
  - `<robotName>/trajectory_tracking/tracking_metric` (`std_msgs/Float64MultiArray`). See [`src/trajectory_progress_monitor_node.cpp#L181`](../src/trajectory_progress_monitor_node.cpp#L181).
  - `/mobile_manipulator/closestTargetWaypoint` (`MarkerArray`, red sphere at the currently matched waypoint index). See [`src/trajectory_progress_monitor_node.cpp#L184`](../src/trajectory_progress_monitor_node.cpp#L184) and [`src/trajectory_progress_monitor_node.cpp#L218`](../src/trajectory_progress_monitor_node.cpp#L218).

The reference message is converted back into `PlannedCartesianTrajectory` (including the quaternion layout conversion). See [`src/trajectory_progress_monitor_node.cpp#L73`](../src/trajectory_progress_monitor_node.cpp#L73).

### 5.2 EE pose estimation (FK)

To evaluate tracking error, the monitor computes the current end-effector pose from the latest observation using Pinocchio FK. See [`src/trajectory_progress_monitor_node.cpp#L44`](../src/trajectory_progress_monitor_node.cpp#L44) and its use in the timer callback [`src/trajectory_progress_monitor_node.cpp#L260`](../src/trajectory_progress_monitor_node.cpp#L260).

Note: the monitor needs the robot model to compute FK; if `taskFile` / `urdfFile` / `libFolder` are not set, it will warn and skip publishing metrics/viz. See [`src/trajectory_progress_monitor_node.cpp#L150`](../src/trajectory_progress_monitor_node.cpp#L150) and [`src/trajectory_progress_monitor_node.cpp#L273`](../src/trajectory_progress_monitor_node.cpp#L273).

## 6) Monitoring math (TrajectoryMonitor)

The core logic is in `TrajectoryMonitor::update()`:
- Params and outputs: [`include/mpc_cartesian_planner/trajectory_monitor.h#L19`](../include/mpc_cartesian_planner/trajectory_monitor.h#L19)
- Implementation: [`src/trajectory_monitor.cpp#L25`](../src/trajectory_monitor.cpp#L25)

### 6.1 Windowed nearest-neighbor matching

Let the reference samples be \(\{(t_k, p_k, q_k)\}_{k=0}^{N-1}\). The monitor keeps a previous best index \(k_0\) (`last_best_`) and searches only a window of size \(W\):

$$
\mathcal{K} = [\max(0,k_0-W),\; \min(N-1,k_0+W)]
$$

It selects:

$$
k^* = \arg\min_{k \in \mathcal{K}} \|p_{\text{now}} - p_k\|^2
$$

See [`src/trajectory_monitor.cpp#L34`](../src/trajectory_monitor.cpp#L34) and [`src/trajectory_monitor.cpp#L45`](../src/trajectory_monitor.cpp#L45).

### 6.2 Self-intersection guard (monotonic-ish progress)

Because a figure-eight can pass near the same point twice, the monitor prevents large backward jumps:

$$
\text{if } k^* + b < k_0 \text{ then } k_{\text{best}} = k_0 \text{ else } k_{\text{best}} = k^*
$$

where \(b\) is `max_backtrack`. See [`src/trajectory_monitor.cpp#L54`](../src/trajectory_monitor.cpp#L54).

### 6.3 Metrics: progress, errors, lag

With \(k_{\text{best}}\) selected:

Progress (normalized index):
$$
\text{progress} =
\begin{cases}
1, & N \le 1 \\
\frac{k_{\text{best}}}{N-1}, & N > 1
\end{cases}
$$

Position error:
$$
e_p = \|p_{\text{now}} - p_{k_{\text{best}}}\|
$$

Lag:
$$
\text{lag\_sec} = t_{\text{now}} - t_{k_{\text{best}}}
$$

See [`src/trajectory_monitor.cpp#L63`](../src/trajectory_monitor.cpp#L63).

### 6.4 Orientation error (quaternion)

The monitor computes the relative rotation:

$$
\delta q = q_{\text{des}}^{-1} \otimes q
$$

and converts it to an angle:

$$
\theta = 2\cos^{-1}(|\delta q_w|)
$$

then reports degrees:

$$
e_q = \theta \cdot \frac{180}{\pi}
$$

See [`src/trajectory_monitor.cpp#L17`](../src/trajectory_monitor.cpp#L17).

### 6.5 Status logic

The monitor emits one of:
- `FINISHED` if progress is high enough and position error is small. See [`src/trajectory_monitor.cpp#L81`](../src/trajectory_monitor.cpp#L81).
- `ON_TRACK` if both position and orientation errors are within thresholds. See [`src/trajectory_monitor.cpp#L88`](../src/trajectory_monitor.cpp#L88).
- `DIVERGED` if position error stays above `diverged_pos` for long enough. See [`src/trajectory_monitor.cpp#L95`](../src/trajectory_monitor.cpp#L95).
- Otherwise `LAGGING`. See [`src/trajectory_monitor.cpp#L106`](../src/trajectory_monitor.cpp#L106).

Thresholds are defined by `MonitorParams`. See [`include/mpc_cartesian_planner/trajectory_monitor.h#L28`](../include/mpc_cartesian_planner/trajectory_monitor.h#L28).

Implementation detail: divergence integrates elapsed time between monitor updates (derived from observation timestamps, falling back to `1 / monitorRate`). See [`src/trajectory_monitor.cpp#L95`](../src/trajectory_monitor.cpp#L95).

When the monitor reaches `FINISHED`, it publishes the final status/metrics and then cancels its internal timer (stops publishing). The TT publisher also cancels its timer when it sees `FINISHED` on the status topic.

## 7) Parameters and tuning

The demo config is in [`config/tt_params.yaml#L1`](../config/tt_params.yaml#L1).

TT publisher parameters:
- `publishRate`: publishing frequency (Hz). See [`config/tt_params.yaml#L3`](../config/tt_params.yaml#L3).
- `publishOnce`: if `true`, publish the target trajectory once and stop publishing (useful for finite tracks + `FINISHED`). See [`config/tt_params.yaml#L4`](../config/tt_params.yaml#L4).
- `commandFrameId`: child frame id for the published command TF. See [`config/tt_params.yaml#L5`](../config/tt_params.yaml#L5).
- `horizon`: horizon length \(T\) (seconds). In `publishOnce=true`, this is the total trajectory duration. See [`config/tt_params.yaml#L6`](../config/tt_params.yaml#L6).
- `dt`: discretization step \(\Delta t\) (seconds). See [`config/tt_params.yaml#L7`](../config/tt_params.yaml#L7).
- `amplitude`: figure-eight amplitude \(A\) (meters). See [`config/tt_params.yaml#L8`](../config/tt_params.yaml#L8).
- `frequency`: sinusoid frequency \(f\) (Hz). See [`config/tt_params.yaml#L9`](../config/tt_params.yaml#L9).
- `axisX/Y/Z`: normal vector for the motion plane. See [`config/tt_params.yaml#L11`](../config/tt_params.yaml#L11).

Monitor parameters:
- `monitorRate`: monitor loop rate (Hz). See [`config/tt_params.yaml#L17`](../config/tt_params.yaml#L17).
- `publishTF`: if `true`, publish a TF for the closest reference waypoint. See [`config/tt_params.yaml#L18`](../config/tt_params.yaml#L18).
- `closestFrameId`: child frame id for the closest-waypoint TF. See [`config/tt_params.yaml#L19`](../config/tt_params.yaml#L19).
- `window`: half-window size \(W\) for local search. See [`config/tt_params.yaml#L20`](../config/tt_params.yaml#L20).
- `maxBacktrack`: backtrack guard \(b\). See [`config/tt_params.yaml#L21`](../config/tt_params.yaml#L21).
- `onTrackPos`, `onTrackOriDeg`: “on track” thresholds. See [`config/tt_params.yaml#L23`](../config/tt_params.yaml#L23).
- `divergedPos`, `divergedHoldSec`: divergence thresholds. See [`config/tt_params.yaml#L26`](../config/tt_params.yaml#L26).
- `finishProgress`, `finishPos`: finish condition thresholds. See [`config/tt_params.yaml#L29`](../config/tt_params.yaml#L29).
