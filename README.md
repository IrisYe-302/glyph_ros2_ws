# `ros2_ws` System Overview

## Active Roots

Top-level directories that matter for the current robot and simulation workflows:

- `~/ros2_ws`
  - Main ROS 2 workspace for the bridge, navigation, custom nodes, and active launch files.
- `~/unitree_ros2`
  - Official Unitree ROS 2 environment used for the real robot DDS connection.
- `~/unitree_sdk2`
  - Kept as the main Unitree SDK tree used by the official stack and its build environment.
- `~/rl_sar`
  - External RL-sar MuJoCo simulator and policy/runtime stack used for the active simulation workflow.

Older roots that were intentionally archived and are no longer part of the intended workflow:

- `~/archive_obsolete_2026-03-17`
  - contains `cyclonedds`, `cyclonedds-python`, `go2_python_sdk`, `go2_robot_upstream`, `unitree_mujoco`, `unitree_mujoco_venv`, `unitree_rl_gym`, `unitree_rl_lab`, and `unitree_sdk2_python`

## Environment Entry Points

The supported workflows are exposed through shell helpers in `~/.bashrc`.

Base environment functions:

- `robot_mode`
  - sources:
    - `~/unitree_ros2/setup_go2_humble.sh`
    - `~/unitree_ros2/example/install/setup.bash`
    - `~/ros2_ws/install/setup.bash`
  - intended for the real robot
- `sim_mode`
  - uses CycloneDDS and only sources `~/ros2_ws/install/setup.bash`
  - mostly a helper for generic sim testing

Current supported command helpers:

- `go2_bridge`
  - real robot bridge only
- `go2_nav_robot`
  - real robot mapping / live-SLAM Nav2
- `go2_nav_robot_localize`
  - real robot localization on a saved map
- `go2_rlsar_sim`
  - RL-sar simulator bridge only
- `go2_rlsar_test`
  - RL-sar bridge smoke test
- `go2_nav_rlsar`
  - RL-sar mapping / live-SLAM Nav2
- `go2_nav_rlsar_localize`
  - RL-sar localization on a saved map
- `go2_nav_rlsar_localize_mppi`
  - explicit RL-sar localization using the MPPI config
- `go2_nav_rlsar_localize_dwb`
  - explicit RL-sar localization using the older DWB config
- `go2_nav_rlsar_test`
  - RL-sar Nav2 smoke test
- `go2_rlsar_reset`
  - calls the RL-sar reset ROS service from any terminal

## Real Robot Stack

The real robot path depends on all of the following:

1. `~/unitree_ros2`
   - provides the official Unitree ROS 2 setup and DDS connection scripts
   - the important setup file is `setup_go2_humble.sh`
2. `~/unitree_sdk2`
   - provides the Unitree SDK used by the official environment
3. `~/ros2_ws/src/go2_unitree_bridge`
   - converts the official Unitree-side topics into standard ROS topics used by the rest of the system
4. `~/ros2_ws/src/go2_navigation`
   - launches SLAM, localization, Nav2, and helper nodes
5. `~/ros2_ws/src/go2_robot_sdk`
   - still supplies important robot-side Nav2 and SLAM parameter files used by the active robot launches
6. `~/ros2_ws/src/unitree_go2/unitree_go2_description`
   - provides the URDF/xacro used for robot visualization and TF structure

Normal real robot launches:

- `go2_bridge`
  - starts `src/go2_unitree_bridge/launch/go2_unitree_bridge.launch.py`
  - exposes `/odom`, `/imu/data`, `/joint_states`, `/tf`, `/robot_description`, and Foxglove topics
- `go2_nav_robot`
  - starts `src/go2_navigation/launch/go2_nav_robot.launch.py`
  - intended for mapping with live SLAM
- `go2_nav_robot_localize map:=...`
  - starts `src/go2_navigation/launch/go2_nav_robot_localization.launch.py`
  - intended for localization and Nav2 on a saved map

## RL-sar Simulation Stack

The RL-sar path depends on all of the following:

1. `~/rl_sar`
   - external simulator root
   - contains the MuJoCo-based RL-sar runtime, policies, robot descriptions, and the `rl_sim_mujoco` binary
2. `~/ros2_ws/src/go2_unitree_bridge`
   - starts the ROS integration around RL-sar
   - publishes synthetic `/scan`, TF, robot description, and obstacle markers for the active scene
3. `~/ros2_ws/src/go2_navigation`
   - launches mapping, localization, Nav2, target-following, goal tolerance markers, and sim fall recovery
4. `~/ros2_ws/src/unitree_go2/unitree_go2_description`
   - provides the robot model used by visualization and `robot_state_publisher`

Normal RL-sar launches:

- `go2_rlsar_sim`
  - starts `src/go2_unitree_bridge/launch/go2_rlsar_sim.launch.py`
  - launches the RL-sar bridge, scan node, Foxglove bridge, robot state publisher, robot description publisher, and obstacle markers
- `go2_nav_rlsar`
  - starts `src/go2_navigation/launch/go2_nav_rlsar.launch.py`
  - intended for map creation with live SLAM
- `go2_nav_rlsar_localize map:=...`
  - starts `src/go2_navigation/launch/go2_nav_rlsar_localization.launch.py`
  - intended for localization and Nav2 on a saved static map

Important RL-sar-specific behavior that has been added:

- a ROS reset service `/go2_rlsar_reset`
- a shell helper `go2_rlsar_reset`
- auto initial-pose publication on localization bringup
- goal publication from `/target_location`
- goal-tolerance visualization on `/target_location_tolerance`
- sim fall recovery that can reset and republish the last target

## Main Active Packages Inside `ros2_ws`

### `src/go2_unitree_bridge`

- active bridge layer for both the real robot and the RL-sar simulator

Main responsibilities:

- real robot:
  - bridge official Unitree ROS 2 topics into a standard ROS nav stack shape
- RL-sar sim:
  - launch RL-sar support nodes
  - publish synthetic `/scan`
  - publish scene obstacle markers
  - expose Foxglove topics and robot description

Important files:

- `src/go2_unitree_bridge/launch/go2_unitree_bridge.launch.py`
- `src/go2_unitree_bridge/launch/go2_rlsar_sim.launch.py`
- `src/go2_unitree_bridge/go2_unitree_bridge/bridge_node.py`
- `src/go2_unitree_bridge/go2_unitree_bridge/rlsar_scan_node.py`
- `src/go2_unitree_bridge/go2_unitree_bridge/rlsar_obstacle_markers.py`

Status:

- active and required

### `src/go2_navigation`

- active high-level navigation layer for both robot and simulator

Main responsibilities:

- mapping launches
- localization launches
- Nav2 configuration
- target goal forwarding from `/target_location`
- default initial-pose publication
- tolerance-marker visualization
- RL-sar sim fall recovery

Important files:

- launches:
  - `src/go2_navigation/launch/go2_nav_robot.launch.py`
  - `src/go2_navigation/launch/go2_nav_robot_localization.launch.py`
  - `src/go2_navigation/launch/go2_nav_rlsar.launch.py`
  - `src/go2_navigation/launch/go2_nav_rlsar_localization.launch.py`
- nodes:
  - `src/go2_navigation/go2_navigation/location_subscriber.py`
  - `src/go2_navigation/go2_navigation/initial_pose_publisher.py`
  - `src/go2_navigation/go2_navigation/goal_tolerance_marker.py`
  - `src/go2_navigation/go2_navigation/sim_fall_recovery.py`
- configs:
  - `src/go2_navigation/config/rlsar_nav2_localization.yaml`
  - `src/go2_navigation/config/rlsar_nav2_localization_mppi.yaml`
  - `src/go2_navigation/config/robot_nav2_localization.yaml`

Status:

- active and required

### `src/go2_robot_sdk`

- older Go2 ROS package tree that still contributes important configs to the active robot path

Still actively used:

- robot-side Nav2 parameters
- SLAM parameters
- some lidar and pointcloud configuration used by the robot navigation path

Not part of the main supported path anymore:

- older presentation and experimental launch paths
- older direct SDK bridging ideas that were superseded by `go2_unitree_bridge`

Status:

- partially active

### `src/go2_interfaces`

- custom message definitions used by the active stack

Status:

- active dependency

### `src/unitree_go2`

- mixed legacy subtree, but still important because the description package is active

Still actively used:

- `src/unitree_go2/unitree_go2_description`
  - URDF/xacro for visualization and TF
  - Foxglove-compatible robot description path

Mostly legacy:

- `champ`
- `champ_base`
- `champ_msgs`
- the old Gazebo sim path under `unitree_go2_sim`

There is still local modification in this tree, so treat it as shared state, not dead code.

Status:

- `unitree_go2_description` is active
- most of the rest is legacy

## Maps and Navigation Modes

Saved maps live in:

- `~/ros2_ws/src/go2_navigation/maps`

Current RL-sar map files:

- `rlsar_scene.yaml`
- `rlsar_scene.pgm`
- `rlsar_scene.pre_obstacles.pgm`

Mapping workflow:

- mapping launches:
  - use SLAM and create or update a map
- localization launches:
  - load a saved static map and run Nav2 on top of it

## Foxglove Notes

Active launches expose Foxglove-compatible topics for:

- `/map`
- `/scan`
- `/tf`
- `/tf_static`
- `/odom`
- `/robot_description`
- `/obstacle_markers`
- `/target_location_tolerance`

The robot model path has been adjusted so that:

- mesh-based visuals can be used when Foxglove accepts them
- a simpler fallback path exists for compatibility
- the old Velodyne subtree is disabled by default in the active flows

## Deprecated Paths

Not part of the intended workflow anymore:

- the old direct MuJoCo bridge helpers (`go2_mj_*`)
- the old CHAMP/Gazebo locomotion path
- archived SDK trees under `~/archive_obsolete_2026-03-17`

## Generated Directories

Generated build outputs, not source:

- `~/ros2_ws/build`
- `~/ros2_ws/install`
- `~/ros2_ws/log`
- `~/rl_sar/build`
- `~/rl_sar/install`
- `~/rl_sar/log`
- `~/rl_sar/cmake_build`

## TLDR

- real robot:
  - `unitree_ros2` + `unitree_sdk2` + `ros2_ws`
- simulation:
  - `rl_sar` + `ros2_ws`
- shared source of truth for active ROS launches:
  - `go2_unitree_bridge`
  - `go2_navigation`
  - `unitree_go2_description`
