# Unified Launch Files

This directory contains combined launch files that integrate multiple packages.

## unified_launch.py

Combines Gazebo simulation with COCO object detection.

### Features
- Gazebo simulation with Unitree Go2 robot
- CHAMP quadruped controller
- Robot state publisher
- EKF localization
- COCO object detection (person, dog, cat, chair, etc.)

### Usage

```bash
# Launch with all features (simulation + coco detector)
ros2 launch launch unified_launch.py

# Launch without coco detector
ros2 launch launch unified_launch.py coco_detector:=false

# Launch without rviz
ros2 launch launch unified_launch.py rviz:=false
```

### Arguments
- `use_sim_time`: Use simulation clock (default: true)
- `rviz`: Launch RViz2 (default: true)
- `coco_detector`: Launch COCO object detector (default: true)
- `robot_name`: Robot name in Gazebo (default: go2)
- `world_init_x`, `world_init_y`, `world_init_z`, `world_init_heading`: Robot spawn position
