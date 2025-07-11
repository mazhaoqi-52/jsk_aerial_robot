# 阀门操作轨迹生成系统

## 简介

这是一个专为beetle无人机设计的阀门操作轨迹生成系统，支持精确的双爪抓取和旋转操作。

## 核心特性

✅ **优化抓取策略**: 抓取第2、3根横梁，避免干涉  
✅ **URDF精确参数**: 基于beetle.urdf.xacro的准确物理参数  
✅ **双爪协调**: 考虑0.16876m爪子分离距离  
✅ **完整状态机**: 集成对准、抓取、旋转流程  
✅ **Mocap兼容**: 支持动捕系统数据输入  

## 快速启动

```bash
# 启动完整的阀门操作系统
roslaunch beetle valve_manipulation_state_machine.launch

# 自定义参数
roslaunch beetle valve_manipulation_state_machine.launch \
  module_ids:="1,2" \
  valve_x:=3.0 \
  valve_y:=0.0 \
  valve_yaw:=0.0
```

## 核心文件

| 文件 | 功能 |
|------|------|
| `trajectory.py` | 轨迹生成器（对准+旋转） |
| `valve_rotation_smach_test.py` | 完整状态机 |
| `base_UAV_state.py` | UAV状态管理基类 |
| `motion_controller.py` | 运动控制器 |
| `valve_manipulation_state_machine.launch` | 启动文件 |

## 主要API

```python
# 创建对准轨迹
align_traj = AlignToGraspTrajectory(
    approach_duration=3.0,
    valve_center=(3.0, 0.0, 0.8),
    valve_pose_yaw=0.0,
    grasp_height=0.85
)

# 设置起始位置
align_traj.set_start_position(start_pos)

# 获取轨迹点
pos, yaw = align_traj.get_next_position_and_yaw()

# 创建旋转轨迹
rotation_traj = ValveRotationTrajectory.create_from_mocap_data(
    rotation_duration=8.0,
    valve_center=valve_center,
    body_cog_position=final_pos,
    body_yaw=final_yaw
)
```

## 关键参数

| 参数 | 值 | 来源 |
|------|----|----- |
| 末端执行器X偏移 | 0.246m | beetle.urdf.xacro |
| 末端执行器Z偏移 | 0.0743823m | beetle.urdf.xacro |
| 爪子分离距离 | 0.16876m | beetle.urdf.xacro |
| 阀门半径 | 0.1225m | 规格参数 |
| 抓取目标 | 第2、3根横梁 | 优化策略 |

## 更多信息

详细使用方法请参见 [USAGE.md](USAGE.md)
