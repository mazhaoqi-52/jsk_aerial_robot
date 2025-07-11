# Valve Manipulation Trajectory Generator Usage Guide

## Overview

This system provides complete valve manipulation trajectory generation and control functionality, including:

1. **AlignToGraspTrajectory**: Trajectory generator for aligning to grasp point (**grasp beams 2&3**)
2. **ValveRotationTrajectory**: Trajectory generator for rotating around valve center
3. **Complete state machine**: Integrates align, grasp, and rotation workflow

## Main Features

### Optimized Grasp Strategy
- **Grasp target**: Beams 2&3 (120° and 240° positions relative to valve orientation)
- **Geometrically precise**: Based on accurate parameters from beetle URDF file
- **Dual-claw configuration**: Considers dual-claw structure and separation distance of end-effector
- **Valve vector consideration**: Default anticlockwise rotation (-1) for typical valve operation
- **Smooth insertion**: Insertion offset based on rotation direction for optimal claw positioning
- **Smooth insertion**: Rotation direction-aware positioning with configurable insertion offset

### Accurate Physical Parameters (Based on URDF)
- **End-effector offset**: X=0.246m, Y=0.0m, Z=0.0743823m
- **Claw separation distance**: 0.16876m (±0.08438m)
- **Claw center vs body COG**: No Y offset, matches actual mechanical structure
- **Insertion offset**: Default 0.02m for smooth claw insertion

## Core Files

### 1. trajectory.py
Main trajectory generator, containing:
- `AlignToGraspTrajectory`: Align to grasp trajectory
- `ValveRotationTrajectory`: Rotation trajectory
- `PolynomialTrajectory`: Underlying polynomial trajectory generation

### 2. valve_rotation_smach_test.py
完整的状态机实现，包含：
- 分离状态下的移动
- 组装状态
- 阀门操作状态
- 拆解状态

### 3. base_UAV_state.py 和 motion_controller.py
提供基础的UAV状态管理和运动控制功能。

## 使用方法

### 1. 启动系统
```bash
# 启动阀门操作状态机
roslaunch beetle valve_manipulation_state_machine.launch

# 参数选项
roslaunch beetle valve_manipulation_state_machine.launch \
  module_ids:="1,2" \
  real_machine:=true \
  simulation:=false \
  valve_x:=3.0 \
  valve_y:=0.0 \
  valve_yaw:=0.0
```

### 2. 直接使用轨迹生成器
```python
#!/usr/bin/env python
import rospy
from valve_rotation_demo.trajectory import AlignToGraspTrajectory, ValveRotationTrajectory
from math import pi

# 初始化节点
rospy.init_node("valve_manipulation_test")

# 创建对准轨迹
align_traj = AlignToGraspTrajectory(
    approach_duration=3.0,
    valve_center=(3.0, 0.0, 0.8),
    valve_pose_yaw=0.0,
    grasp_height=0.85,
    valve_radius=0.1225,
    valve_beam_width=0.05,
    end_effector_offset_x=0.246,      # 从URDF获取
    end_effector_offset_y=0.0,
    end_effector_offset_z=0.0743823,  # 从URDF获取
    claw_separation=0.16876           # 从URDF获取
)

# 设置起始位置
start_pos = (2.0, 0.0, 0.8)
align_traj.set_start_position(start_pos)

# 执行轨迹
rate = rospy.Rate(20)
while not rospy.is_shutdown() and not align_traj.is_complete():
    result = align_traj.get_next_position_and_yaw()
    if result is None:
        break
    pos, yaw = result
    # 发布控制命令到你的控制器
    # publish_control_command(pos, yaw)
    rate.sleep()

# 获取最终状态用于旋转轨迹
target_info = align_traj.get_target_info()
final_pos = target_info['body_position']
final_yaw = target_info['body_yaw']

# 创建旋转轨迹
rotation_traj = ValveRotationTrajectory.create_from_mocap_data(
    rotation_duration=8.0,
    valve_center=(3.0, 0.0, 0.8),
    body_cog_position=final_pos,
    body_yaw=final_yaw,
    rotation_angle=2*pi
)

# 执行旋转
rotation_traj.start_rotation()
while not rospy.is_shutdown() and not rotation_traj.is_complete():
    result = rotation_traj.get_next_position_and_yaw()
    if result is None:
        break
    pos, yaw = result
    # 发布控制命令
    # publish_control_command(pos, yaw)
    rate.sleep()
```

## 重要方法

### AlignToGraspTrajectory
- `set_start_position(start_pos)`: 设置起始位置并生成轨迹
- `get_next_position_and_yaw()`: 获取下一个机体位置和yaw角度
- `get_target_info()`: 获取目标信息
- `get_claw_positions_in_global_frame(body_pos, body_yaw)`: 计算爪子全局位置
- `is_complete()`: 检查轨迹是否完成

### ValveRotationTrajectory
- `create_from_mocap_data()`: 从mocap数据创建轨迹（静态方法）
- `start_rotation()`: 开始旋转轨迹
- `get_next_position_and_yaw()`: 获取下一个机体位置和yaw角度
- `is_complete()`: 检查轨迹是否完成

## 配置参数

### 阀门参数
- `valve_radius`: 阀门半径 (默认: 0.1225m)
- `valve_beam_width`: 横梁宽度 (默认: 0.05m)
- `valve_center`: 阀门中心位置 (x, y, z)
- `valve_pose_yaw`: 阀门朝向角度 (弧度)

### 机械参数（基于URDF）
- `end_effector_offset_x`: 0.246m (末端执行器前向偏移)
- `end_effector_offset_y`: 0.0m (无侧向偏移)
- `end_effector_offset_z`: 0.0743823m (末端执行器上向偏移)
- `claw_separation`: 0.16876m (爪子分离距离)

### 轨迹参数
- `approach_duration`: 对准持续时间 (默认: 3.0s)
- `rotation_duration`: 旋转持续时间 (默认: 8.0s)
- `rotation_angle`: 旋转角度 (默认: 2π rad)

## 调试和验证

### 日志输出
系统会输出详细的调试信息：
- 三根横梁的位置
- 选择的目标横梁（第2、3根）
- 爪子目标位置
- 末端执行器中心位置
- 机体目标位置和姿态

### 关键验证点
1. **横梁选择**: 确认选择第2、3根横梁
2. **爪子间距**: 验证爪子分离距离为0.16876m
3. **坐标变换**: 确认末端执行器偏移正确应用
4. **轨迹连续性**: 对准结束位置与旋转开始位置一致

## 集成到现有系统

本轨迹生成器已完全集成到现有的valve_rotation_smach_test.py状态机中，可以直接使用launch文件启动：

```bash
roslaunch beetle valve_manipulation_state_machine.launch
```

状态机将自动处理：
1. 分离状态下的移动到阀门位置
2. 组装状态
3. 使用优化轨迹的阀门操作
4. 拆解和离开

## 注意事项

1. **坐标系**: 所有位置都在全局坐标系中
2. **单位**: 位置单位为米，角度单位为弧度
3. **频率**: 建议控制频率为20Hz
4. **容错**: 系统包含位置和姿态容错检查
5. **URDF一致性**: 所有参数都与beetle.urdf.xacro保持一致

## 故障排除

### 常见问题
1. **轨迹不连续**: 检查末端执行器偏移参数
2. **抓取位置不准**: 验证阀门中心和朝向角度
3. **爪子碰撞**: 确认爪子分离距离设置正确
4. **旋转半径错误**: 检查create_from_mocap_data的输入参数

### 调试建议
1. 启用详细日志输出
2. 可视化轨迹点
3. 验证几何计算结果
4. 检查URDF参数一致性

### 2. ValveRotationTrajectory

负责生成绕阀门中心旋转的轨迹。

#### 初始化参数
```python
ValveRotationTrajectory(
    rotation_duration,         # 旋转持续时间 (秒)
    valve_center,             # 阀门中心位置 (x, y, z)
    rotation_radius,          # 旋转半径
    start_angle,              # 起始角度 (弧度)
    rotation_angle=2*pi,      # 旋转角度，默认一圈
    grasp_height=None,        # 抓取高度
    end_effector_offset_x=0.0,# 末端执行器X偏移
    end_effector_offset_y=0.0,# 末端执行器Y偏移
    end_effector_offset_z=0.0 # 末端执行器Z偏移
)
```

#### 主要方法
- `start_rotation()`: 开始旋转轨迹
- `get_next_position_and_yaw()`: 获取下一个机体位置和yaw角度
- `get_next_position()`: 获取下一个机体位置（兼容接口）
- `is_complete()`: 检查轨迹是否完成
- `create_from_mocap_data()`: 从mocap数据创建旋转轨迹（静态方法）

## 使用示例

### 基本使用流程

```python
import rospy
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory
from math import pi

# 初始化ROS节点
rospy.init_node("valve_manipulation")
rate = rospy.Rate(50)

# 参数设置
valve_center = (1.0, 0.5, 0.8)
valve_pose_yaw = pi/4  # 45度
start_pos = (0.5, 0.2, 0.8)
grasp_height = 0.85

# 末端执行器偏移
end_effector_offset_x = 0.15
end_effector_offset_y = 0.0
end_effector_offset_z = -0.05

# 阶段1：对准抓取点
align_traj = AlignToGraspTrajectory(
    approach_duration=3.0,
    valve_center=valve_center,
    valve_pose_yaw=valve_pose_yaw,
    grasp_height=grasp_height,
    end_effector_offset_x=end_effector_offset_x,
    end_effector_offset_y=end_effector_offset_y,
    end_effector_offset_z=end_effector_offset_z
)

align_traj.set_start_position(start_pos)

# 执行对准轨迹
while not rospy.is_shutdown() and not align_traj.is_complete():
    result = align_traj.get_next_position_and_yaw()
    if result is None:
        break
    pos, yaw = result
    print(f"对准: 位置 {pos}, yaw {yaw}")
    # 在这里发送控制命令
    rate.sleep()

# 获取对准完成后的信息
target_info = align_traj.get_target_info()
final_body_pos = target_info['body_position']

# 阶段2：绕阀门中心旋转
rotation_traj = ValveRotationTrajectory.create_from_mocap_data(
    rotation_duration=8.0,
    valve_center=valve_center,
    body_cog_position=final_body_pos,
    rotation_angle=2*pi,
    grasp_height=grasp_height,
    end_effector_offset_x=end_effector_offset_x,
    end_effector_offset_y=end_effector_offset_y,
    end_effector_offset_z=end_effector_offset_z
)

rotation_traj.start_rotation()

# 执行旋转轨迹
while not rospy.is_shutdown() and not rotation_traj.is_complete():
    result = rotation_traj.get_next_position_and_yaw()
    if result is None:
        break
    pos, yaw = result
    print(f"旋转: 位置 {pos}, yaw {yaw}")
    # 在这里发送控制命令
    rate.sleep()

print("阀门操作完成")
```

### 在状态机中使用

```python
import smach
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory

class AlignToGraspState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.align_traj = None
        
    def execute(self, userdata):
        # 从userdata获取参数
        valve_center = userdata.valve_center
        valve_yaw = userdata.valve_yaw
        current_pos = userdata.current_pos
        
        # 创建对准轨迹
        self.align_traj = AlignToGraspTrajectory(
            approach_duration=3.0,
            valve_center=valve_center,
            valve_pose_yaw=valve_yaw,
            grasp_height=valve_center[2] + 0.21,
            end_effector_offset_x=0.15,
            end_effector_offset_y=0.0,
            end_effector_offset_z=-0.05
        )
        
        self.align_traj.set_start_position(current_pos)
        
        # 执行轨迹
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.align_traj.is_complete():
            result = self.align_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            # 发送控制命令
            self.publish_control_command(pos, yaw)
            rate.sleep()
            
        # 将目标信息传递给下一个状态
        userdata.grasp_info = self.align_traj.get_target_info()
        return 'succeeded'

class RotateValveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.rotation_traj = None
        
    def execute(self, userdata):
        # 从userdata获取参数
        valve_center = userdata.valve_center
        grasp_info = userdata.grasp_info
        body_position = grasp_info['body_position']
        
        # 创建旋转轨迹
        self.rotation_traj = ValveRotationTrajectory.create_from_mocap_data(
            rotation_duration=8.0,
            valve_center=valve_center,
            body_cog_position=body_position,
            rotation_angle=2*pi,
            grasp_height=grasp_info['end_effector_position'][2],
            end_effector_offset_x=0.15,
            end_effector_offset_y=0.0,
            end_effector_offset_z=-0.05
        )
        
        self.rotation_traj.start_rotation()
        
        # 执行旋转轨迹
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.rotation_traj.is_complete():
            result = self.rotation_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            # 发送控制命令
            self.publish_control_command(pos, yaw)
            rate.sleep()
            
        return 'succeeded'
```

## 重要特性

1. **准确的机体定位**: 考虑末端执行器相对于机体重心的偏移
2. **正确的姿态控制**: 计算机体yaw角度确保末端执行器正确朝向
3. **模块化设计**: 两个独立的轨迹生成器可以分别使用
4. **Mocap数据支持**: 可以使用实时的mocap数据来初始化旋转轨迹
5. **向后兼容**: 提供兼容的接口方法

## 配置参数

### 物理参数
- `valve_radius`: 阀门半径 (默认: 0.1225m)
- `valve_beam_width`: 横梁宽度 (默认: 0.05m)
- `end_effector_offset_x/y/z`: 末端执行器相对机体重心的偏移

### 时间参数
- `approach_duration`: 对准持续时间
- `rotation_duration`: 旋转持续时间
- `rotation_angle`: 旋转角度 (默认: 2π)

### 位置参数
- `valve_center`: 阀门中心位置
- `valve_pose_yaw`: 阀门朝向角度
- `grasp_height`: 抓取高度

通过这种分离设计，您可以更灵活地控制阀门操作的每个阶段，并且可以根据实际的mocap数据进行实时调整。
