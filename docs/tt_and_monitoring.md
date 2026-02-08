# TT (Trajectory Tracking) and Monitoring

This package provides two ROS 2 nodes:

- `trajectory_tt_publisher_node`: generates a **Cartesian end-effector target trajectory** (TT) and publishes it as an OCS2 `MpcTargetTrajectories` message. See [`src/trajectory_tt_publisher_node.cpp#L104`](../src/trajectory_tt_publisher_node.cpp#L104).
- `trajectory_progress_monitor_node`: consumes the published target trajectory and estimates **tracking progress / error** using windowed nearest-neighbor matching. See [`src/trajectory_progress_monitor_node.cpp#L103`](../src/trajectory_progress_monitor_node.cpp#L103).

Launch + parameters:
- Demo launch file: [`launch/tt_demo.launch.py#L8`](../launch/tt_demo.launch.py#L8)
- Default params: [`config/tt_params.yaml#L1`](../config/tt_params.yaml#L1)

Runtime control:
- Service (pause/resume TT publishing): **`<robotName>/trajectory_tracking/toggle_tt_publisher`** (`std_srvs/srv/SetBool`)

## 1) What “TT” means here

OCS2-based MPC expects a target trajectory message (`MpcTargetTrajectories`) on `<robotName>_mpc_target`. The TT publisher node produces such a target in *Cartesian pose space*:

- State layout used in this repo: **`[px, py, pz, qx, qy, qz, qw]`**. See [`src/trajectory_publisher.cpp#L24`](../src/trajectory_publisher.cpp#L24).
- The TT publisher *only publishes the reference*; the actual tracking is done by your controller (*e.g.*, in `mpc_controller`).

## 2) TT publisher: code path and logic

### 2.1 Inputs / outputs

- Subscribes: `<robotName>_mpc_observation` (`ocs2_msgs/msg/MpcObservation`). See [`src/trajectory_tt_publisher_node.cpp#L169`](../src/trajectory_tt_publisher_node.cpp#L169).
- Publishes: `<robotName>_mpc_target` (`ocs2_msgs/msg/MpcTargetTrajectories`) via `TrajectoryPublisher`. See [`src/trajectory_publisher.cpp#L8`](../src/trajectory_publisher.cpp#L8).
- Provides: `<robotName>/trajectory_tracking/toggle_tt_publisher` (`std_srvs/srv/SetBool`) to pause (publish HOLD at current EE pose) / resume. See [`src/trajectory_tt_publisher_node.cpp#L309`](../src/trajectory_tt_publisher_node.cpp#L309).
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
3) Constructs a planner selected by `trajectoryType` (e.g., `FigureEightPlanner`, `LinearMovePlanner`, `TargetPosePlanner`, or `ScrewMovePlanner`) using `(centerP_, q0_)` and parameters from ROS params.
4) Anchors the planner phase by setting `planner_->setStartTime(startTime_)`. See [`src/trajectory_tt_publisher_node.cpp#L217`](../src/trajectory_tt_publisher_node.cpp#L217).

If the model paths are not set, the node falls back to `centerP_ = 0` and `q0 = Identity` (and still publishes a trajectory). See [`src/trajectory_tt_publisher_node.cpp#L152`](../src/trajectory_tt_publisher_node.cpp#L152).

### 2.3 Periodic loop (each timer tick)

Each tick:

1) Grabs the latest observation. See [`src/trajectory_tt_publisher_node.cpp#L187`](../src/trajectory_tt_publisher_node.cpp#L187).
2) Plans a fresh horizon starting at the current observation time `t0 = obs.time`. See [`src/cartesian_trajectory_planner.cpp#L33`](../src/cartesian_trajectory_planner.cpp#L33).
3) Publishes the EE target trajectory as `MpcTargetTrajectories`. See [`src/trajectory_tt_publisher_node.cpp#L223`](../src/trajectory_tt_publisher_node.cpp#L223) and [`src/trajectory_publisher.cpp#L13`](../src/trajectory_publisher.cpp#L13).
4) Optionally publishes TF + visualization. See [`src/trajectory_tt_publisher_node.cpp#L228`](../src/trajectory_tt_publisher_node.cpp#L228).

Note: in the default *receding-horizon* behavior (publishing a new horizon each tick), the monitor’s `progress` (normalized waypoint index) will typically stay near zero, since the closest waypoint is usually near the start of the latest horizon.

If you want a **finite trajectory** with meaningful `progress` and a terminal `FINISHED` state, set `publishOnce: true` in the TT publisher params. In this mode the node publishes the target trajectory once (with `duration` as the total duration) and then stops publishing.

## 3) Planner math (Trajectory Types)

The planner interface and output container are:
- `CartesianTrajectoryPlanner`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L32`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L32)
- `PlannedCartesianTrajectory`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L11`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L11)
- `FigureEightPlanner`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L39`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L39)
- `LinearMovePlanner`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L73`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L73)
- `TargetPosePlanner`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L100`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L100)
- `ScrewMovePlanner`: [`include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L127`](../include/mpc_cartesian_planner/cartesian_trajectory_planner.h#L127)

The TT publisher selects the planner via the `trajectoryType` parameter (see `config/tt_params.yaml`).

### 3.1 Time discretization

Given:
- trajectory duration \(T\) (`duration` in `config/tt_params.yaml`, stored as `horizon_T`)
- step \( \Delta t \) (`dt`)
- current time \(t_0 = \texttt{obs.time}\)

The code chooses:

$$
N = \left\lceil \frac{T}{\Delta t} \right\rceil,\quad
t_k = t_0 + k\Delta t,\quad k = 0,1,\dots,N
$$

See [`src/cartesian_trajectory_planner.cpp#L62`](../src/cartesian_trajectory_planner.cpp#L62) and [`src/cartesian_trajectory_planner.cpp#L73`](../src/cartesian_trajectory_planner.cpp#L73).

### 3.2 Plane basis from an axis

The trajectory is drawn in a plane whose normal is the user-provided axis \(n\) (`plane_axis`), normalized. The code constructs an orthonormal basis \((u,v)\) spanning the plane using cross products:

$$
u = \frac{n \times r}{\|n \times r\|}, \quad v = n \times u
$$

where \(r\) is a “reference” axis chosen to not be parallel to \(n\). See [`src/cartesian_trajectory_planner.cpp#L39`](../src/cartesian_trajectory_planner.cpp#L39).

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

See [`src/cartesian_trajectory_planner.cpp#L75`](../src/cartesian_trajectory_planner.cpp#L75) and [`src/cartesian_trajectory_planner.cpp#L77`](../src/cartesian_trajectory_planner.cpp#L77).

Orientation is constant (the initial FK orientation):

$$
q(t_k) = q_0
$$

See [`src/cartesian_trajectory_planner.cpp#L81`](../src/cartesian_trajectory_planner.cpp#L81).

### 3.4 Linear move (time scaling + so(3) interpolation)

`LinearMovePlanner` generates a straight-line translation and a smooth orientation change from the initial end-effector pose.

The command is provided as a 6D vector:

$$
[\Delta x,\;\Delta y,\;\Delta z,\;\theta_z,\;\theta_y,\;\theta_x]
$$

where \((\theta_z,\theta_y,\theta_x)\) is **ZYX Euler** (applied as \(R_z(\theta_z)R_y(\theta_y)R_x(\theta_x)\)) in radians.

For backward compatibility, `linearMoveOffset` may also be provided as `[dx, dy, dz]` (rotation assumed zero).

If `linearMoveOffsetInToolFrame: true`, the 6D offset is interpreted as a relative pose in the **initial EE/tool frame**.
If `linearMoveOffsetInToolFrame: false`, translation is in the **global/base frame** and the rotation is applied about **global/base axes**.

Time is normalized over the total duration \(T\) (`horizon_T`) as:

$$
\tau = \mathrm{clamp}\left(\frac{t - t_{\text{start}}}{T},\, 0,\, 1\right)
$$

and uses a time-scaling \(s(\tau)\) (selected by `timeScaling`):

$$
s(\tau)=
\begin{cases}
\tau, & \texttt{linear} \\
10\tau^3-15\tau^4+6\tau^5, & \texttt{min\_jerk}
\end{cases}
$$

Implementation note: the code first builds a **relative command pose** \(T_{e\rightarrow e_{\text{cmd}}}\) (from the offset parameters, either in tool frame directly or by converting a world-frame goal to a relative pose). It then interpolates in **pos + so(3)** coordinates:

$$
p_{\text{rel}}(t)=s(\tau)\,\Delta p,\quad
\phi(t)=s(\tau)\,\phi_{\text{cmd}},\quad
R_{\text{rel}}(t)=\exp\!\left([\phi(t)]_\times\right)
$$

where \(\phi_{\text{cmd}}=\log(R_{e\rightarrow e_{\text{cmd}}})\) is the rotation-vector log, and then composes back to an **absolute** target:

$$
T_{b\rightarrow e}(t)=T_{b\rightarrow e_0}\,T_{\text{rel}}(t)
$$

See [`src/cartesian_trajectory_planner.cpp#L86`](../src/cartesian_trajectory_planner.cpp#L86) and [`src/cartesian_trajectory_planner.cpp#L112`](../src/cartesian_trajectory_planner.cpp#L112).

### 3.5 Target pose (time scaling + SLERP)

`TargetPosePlanner` moves from the initial pose \((p_0, q_0)\) to an **absolute** goal pose \((p_1, q_1)\) in the world frame.

With the same \(\tau\) and \(s(\tau)\) as above:

$$
p(t)=p_0+s(\tau)\,(p_1-p_0)
$$

If `poseMoveTarget` provides an orientation, it uses quaternion SLERP:

$$
q(t)=\mathrm{slerp}(q_0,\;q_1;\;s(\tau))
$$

Otherwise it holds the initial orientation:

$$
q(t)=q_0
$$

See [`src/cartesian_trajectory_planner.cpp#L146`](../src/cartesian_trajectory_planner.cpp#L146) and [`src/cartesian_trajectory_planner.cpp#L157`](../src/cartesian_trajectory_planner.cpp#L157).

### 3.6 Screw move (SE(3) screw motion)

`ScrewMovePlanner` generates a relative **screw motion** starting from the current EE pose.

Inputs:
- axis direction \( \hat{u} \in \mathbb{R}^3 \) (`screwMoveUhat`, normalized)
- axis-to-EE vector \( r \in \mathbb{R}^3 \) (`screwMoveR`, radius \( \|r\| \))
- total rotation \( \theta \) (`screwMoveTheta`, radians)

The screw axis (twist) is defined as:

$$
S = (v,\;\omega),\quad
\omega=\hat{u},\quad
v=\hat{u}\times r
$$

Using the se(3) “hat” operator:

$$
[S]=
\begin{bmatrix}
[\omega]_\times & v\\
0 & 0
\end{bmatrix}
$$

and a time-scaled rotation \(\theta(t)=s(\tau)\,\theta\), the relative transform is:

$$
T_{\Delta}(t)=\exp([S]\;\theta(t))
$$

Finally, it converts the relative motion to an **absolute** target pose:

$$
T_{b\rightarrow e}(t)=
\begin{cases}
T_{b\rightarrow e_0}\,T_{\Delta}(t), & \texttt{screwMoveInToolFrame}=\texttt{true} \\
T_{\Delta}(t)\,T_{b\rightarrow e_0}, & \texttt{screwMoveInToolFrame}=\texttt{false}
\end{cases}
$$

See [`src/cartesian_trajectory_planner.cpp#L192`](../src/cartesian_trajectory_planner.cpp#L192) and [`src/cartesian_trajectory_planner.cpp#L207`](../src/cartesian_trajectory_planner.cpp#L207).

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

Terminal behavior:
- When `TrackingStatus` becomes `FINISHED` or `DIVERGED`, the progress monitor enters **idle mode** (stops its timer) so the status does not immediately flip back to `ON_TRACK` due to the optional HOLD trajectory.
- It automatically resumes when a new non-HOLD target trajectory is received.

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
- `trajectoryType`: trajectory generator (`figure_eight`, `linear_move`, `target_pose`, or `screw_move`). See [`config/tt_params.yaml#L9`](../config/tt_params.yaml#L9).
- `duration`: trajectory duration \(T\) (seconds). In `publishOnce=true`, this is the total trajectory duration. See [`config/tt_params.yaml#L10`](../config/tt_params.yaml#L10).
- `dt`: discretization step \(\Delta t\) (seconds). See [`config/tt_params.yaml#L11`](../config/tt_params.yaml#L11).
- `timeScaling`: time scaling for finite trajectories (`target_pose`, `linear_move`, `screw_move`): `min_jerk` or `linear`. See [`config/tt_params.yaml#L12`](../config/tt_params.yaml#L12).
- `poseMoveTarget`: absolute target pose for `target_pose` (`[x,y,z]`, `[x,y,z,thz,thy,thx]`, or `[x,y,z,qw,qx,qy,qz]`). See [`config/tt_params.yaml#L17`](../config/tt_params.yaml#L17).
- `linearMoveOffset`: 6D offset `[dx, dy, dz, thz, thy, thx]` (meters, radians) for `linear_move`. See [`config/tt_params.yaml#L25`](../config/tt_params.yaml#L25).
- `linearMoveOffsetInToolFrame`: if `true`, interpret `linearMoveOffset` in the initial EE/tool frame. See [`config/tt_params.yaml#L26`](../config/tt_params.yaml#L26).
- `screwMoveUhat`: screw axis direction `u_hat` for `screw_move` (3D vector). See [`config/tt_params.yaml#L34`](../config/tt_params.yaml#L34).
- `screwMoveR`: axis->EE vector `r` for `screw_move` (3D vector; `||r||` is radius). See [`config/tt_params.yaml#L35`](../config/tt_params.yaml#L35).
- `screwMoveTheta`: total rotation `theta` for `screw_move` (radians). See [`config/tt_params.yaml#L38`](../config/tt_params.yaml#L38).
- `screwMoveInToolFrame`: if `true`, interpret `screwMoveUhat`/`screwMoveR` in the initial EE frame (body-screw). See [`config/tt_params.yaml#L39`](../config/tt_params.yaml#L39).
- `amplitude`: figure-eight amplitude \(A\) (meters). See [`config/tt_params.yaml#L43`](../config/tt_params.yaml#L43).
- `frequency`: sinusoid frequency \(f\) (Hz). See [`config/tt_params.yaml#L44`](../config/tt_params.yaml#L44).
- `axisX/Y/Z`: normal vector for the motion plane. See [`config/tt_params.yaml#L45`](../config/tt_params.yaml#L45).
- `deltaPoseTopic`: reactive delta pose `PoseStamped` topic to add to the nominal planned EE trajectory (e.g. from an admittance controller). See [`config/tt_params.yaml#L56`](../config/tt_params.yaml#L56).
- `deltaPoseInToolFrame`: if `true` (default), interpret delta pose in each waypoint’s EE/tool frame and apply it to all waypoints. See [`config/tt_params.yaml#L57`](../config/tt_params.yaml#L57).
- `deltaPoseTimeout`: if > 0, ignore stale delta poses older than this timeout. See [`config/tt_params.yaml#L58`](../config/tt_params.yaml#L58).
- `publishZeroBaseCmdOnIntervention`: if `true`, publish a short burst of zero `cmd_vel` when tracking finishes/diverges or when you manually pause TT. See [`config/tt_params.yaml#L61`](../config/tt_params.yaml#L61).
- `baseCmdTopic`: base `cmd_vel` topic name for the zero-command burst. See [`config/tt_params.yaml#L62`](../config/tt_params.yaml#L62).
- `zeroBaseCmdBurstCount`, `zeroBaseCmdBurstRate`: how many zero messages to publish and at what rate. See [`config/tt_params.yaml#L63`](../config/tt_params.yaml#L63).

Monitor parameters:
- `monitorRate`: monitor loop rate (Hz). See [`config/tt_params.yaml#L77`](../config/tt_params.yaml#L77).
- `publishTF`: if `true`, publish a TF for the closest reference waypoint. See [`config/tt_params.yaml#L78`](../config/tt_params.yaml#L78).
- `closestFrameId`: child frame id for the closest-waypoint TF. See [`config/tt_params.yaml#L79`](../config/tt_params.yaml#L79).
- `window`: half-window size \(W\) for local search. See [`config/tt_params.yaml#L80`](../config/tt_params.yaml#L80).
- `maxBacktrack`: backtrack guard \(b\). See [`config/tt_params.yaml#L81`](../config/tt_params.yaml#L81).
- `onTrackPos`, `onTrackOriDeg`: “on track” thresholds. See [`config/tt_params.yaml#L83`](../config/tt_params.yaml#L83).
- `divergedPos`, `divergedHoldSec`: divergence thresholds. See [`config/tt_params.yaml#L86`](../config/tt_params.yaml#L86).
- `finishProgress`, `finishPos`: finish condition thresholds. See [`config/tt_params.yaml#L89`](../config/tt_params.yaml#L89).
