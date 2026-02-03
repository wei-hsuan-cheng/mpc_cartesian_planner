# mpc_cartesian_planner

Cartesian trajectory/path planner for OCS2 MPC.

## Prerequisites

Install ROS 2 Humble (or a newer distro with the same APIs).

## Build and Run Demo

```bash
# Clone the related repositories
git clone \
  https://github.com/wei-hsuan-cheng/ocs2_ros2.git

git clone \
  https://github.com/wei-hsuan-cheng/mobile_manipulator_mpc.git

git clone \
  https://github.com/wei-hsuan-cheng/mpc_cartesian_planner.git

# rosdep install
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and install
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-up-to mpc_cartesian_planner \
  --parallel-workers 4 --executor sequential \
  && . install/setup.bash
```