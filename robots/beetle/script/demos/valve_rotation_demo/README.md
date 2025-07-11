# Valve Manipulation Trajectory Generation System

## Overview

This is a valve manipulation trajectory generation system designed for beetle drones, supporting precise dual-claw grasping and rotation operations.

## Core Features

✅ **Optimized Grasp Strategy**: Grasp beams 2&3 to avoid interference  
✅ **URDF Precise Parameters**: Based on accurate physical parameters from beetle.urdf.xacro  
✅ **Dual-Claw Coordination**: Considers 0.16876m claw separation distance  
✅ **Smooth Insertion**: Rotation direction-aware positioning for smooth claw insertion  
✅ **Valve Vector Consideration**: Default anticlockwise rotation (-1) for typical valve operation  
✅ **Complete State Machine**: Integrates align, grasp, and rotation workflow  
✅ **Mocap Compatible**: Supports motion capture system data input  

## Quick Start

```bash
# Launch complete valve manipulation system
roslaunch beetle valve_manipulation_state_machine.launch

# Custom parameters
roslaunch beetle valve_manipulation_state_machine.launch \
  module_ids:="1,2" \
  valve_x:=3.0 \
  valve_y:=0.0 \
  valve_yaw:=0.0 \
  rotation_direction:=-1
```

## Core Files

| File | Function |
|------|----------|
| `trajectory.py` | Trajectory generator (align + rotation) |
| `valve_rotation_smach_test.py` | Complete state machine |
| `base_UAV_state.py` | UAV state management base class |
| `motion_controller.py` | Motion controller |
| `valve_manipulation_state_machine.launch` | Launch file |

## Main API

```python
# Create align trajectory
align_traj = AlignToGraspTrajectory(
    approach_duration=3.0,
    valve_center=(3.0, 0.0, 0.8),
    valve_pose_yaw=0.0,
    grasp_height=0.85,
    rotation_direction=-1,  # -1 for anticlockwise (typical valve operation), 1 for clockwise
    insertion_offset=0.02  # 2cm offset for smooth insertion
)

# Set start position
align_traj.set_start_position(start_pos)

# Get trajectory point
pos, yaw = align_traj.get_next_position_and_yaw()

# Create rotation trajectory
rotation_traj = ValveRotationTrajectory.create_from_mocap_data(
    rotation_duration=8.0,
    valve_center=valve_center,
    body_cog_position=final_pos,
    body_yaw=final_yaw,
    rotation_direction=-1  # Same direction as grasp (anticlockwise for typical valve operation)
)
```

## Key Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| End-effector X offset | 0.246m | beetle.urdf.xacro |
| End-effector Z offset | 0.0743823m | beetle.urdf.xacro |
| Claw separation | 0.16876m | beetle.urdf.xacro |
| Valve radius | 0.1225m | Specification |
| Grasp target | Beams 2&3 | Optimization strategy |
| Rotation direction | -1 (default) | Anticlockwise typical for valve operation |
| Insertion offset | 0.02m | Smooth claw insertion |
| Insertion offset | 0.02m | Smooth insertion |

## More Information

For detailed usage, please see [USAGE.md](USAGE.md)
