# `ros2_ws` Overview

## Purpose

`ros2_ws` is the ROS 2 workspace that ties together:

- the real Go2 robot bridge
- the RL-sar simulation bridge
- navigation, mapping, and localization
- shared robot description assets

The active workspace is under `~/ros2_ws/src/`.

## Active Packages

- `go2_unitree_bridge`
  - bridge layer for the real robot and RL-sar simulation
- `go2_navigation`
  - mapping, localization, Nav2 bringup, and helper nodes
- `go2_robot_sdk`
  - robot-side driver/config package used by the real robot launch path
- `go2_interfaces`
  - custom message definitions
- `lidar_processor`
  - Python LiDAR processing used by the default robot launch path
- `unitree_go2_description`
  - URDF/xacro, meshes, and world assets for visualization and TF

Optional packages still present in the workspace:

- `coco_detector`
  - object detection package, not part of the default navigation stack
- `speech_processor`
  - TTS/audio helpers, optional at runtime

## Source Layout

Top-level workspace directories:

- `~/ros2_ws/src/go2_unitree_bridge`
- `~/ros2_ws/src/go2_navigation`
- `~/ros2_ws/src/go2_robot_sdk`
- `~/ros2_ws/src/go2_interfaces`
- `~/ros2_ws/src/lidar_processor`
- `~/ros2_ws/src/unitree_go2_description`
- `~/ros2_ws/src/coco_detector`
- `~/ros2_ws/src/speech_processor`

Non-package workspace directories:

- `~/ros2_ws/src/docker`
- `~/ros2_ws/build`
- `~/ros2_ws/install`
- `~/ros2_ws/log`

## Supported Entry Points

- `go2_bridge`
  - starts the real robot bridge from `~/ros2_ws/src/go2_unitree_bridge/launch/go2_unitree_bridge.launch.py`
- `go2_nav_robot`
  - starts mapping/live SLAM for the real robot from `~/ros2_ws/src/go2_navigation/launch/go2_nav_robot.launch.py`
- `go2_nav_robot_localize map:=...`
  - starts localization/Nav2 for the real robot from `~/ros2_ws/src/go2_navigation/launch/go2_nav_robot_localization.launch.py`
- `go2_rlsar_sim`
  - starts the RL-sar bridge from `~/ros2_ws/src/go2_unitree_bridge/launch/go2_rlsar_sim.launch.py`
- `go2_nav_rlsar`
  - starts mapping/live SLAM for RL-sar from `~/ros2_ws/src/go2_navigation/launch/go2_nav_rlsar.launch.py`
- `go2_nav_rlsar_localize map:=...`
  - starts localization/Nav2 for RL-sar from `~/ros2_ws/src/go2_navigation/launch/go2_nav_rlsar_localization.launch.py`

## Dependency Notes

Real robot path:

- `go2_unitree_bridge`
- `go2_navigation`
- `go2_robot_sdk`
- `lidar_processor`
- `unitree_go2_description`

RL-sar path:

- `go2_unitree_bridge`
- `go2_navigation`
- `unitree_go2_description`

## Maps

Saved maps live in `~/ros2_ws/src/go2_navigation/maps`.
