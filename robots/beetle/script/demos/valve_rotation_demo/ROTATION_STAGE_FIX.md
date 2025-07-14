# Rotation Stage Fix - Stage 4 突然上冲问题修复

## 问题分析

从日志可以看出，在Stage 4旋转阶段存在"猛地向上一冲，突然旋转"的问题。经过代码分析，发现问题的根本原因在于：

### 1. 高度计算错误
在 `RotateValveState.execute` 中，原来的代码：
```python
current_end_effector_z = current_pos[2] + end_effector_offset_z
```

然后在 `ValveRotationTrajectory` 中又计算：
```python
body_z = end_effector_z - global_offset_z
```

这导致了高度计算的重复偏移，造成突然的高度变化。

### 2. 缺乏过渡稳定
- Stage 3到Stage 4的过渡缺乏稳定期
- 旋转开始前没有预稳定阶段
- 缺乏详细的状态日志

## 修复方案

### 1. 修复高度计算逻辑
```python
# 修复前：
current_end_effector_z = current_pos[2] + end_effector_offset_z
grasp_height=current_end_effector_z

# 修复后：
current_grasp_height = current_pos[2]  # 使用当前body高度
grasp_height=current_grasp_height      # 让轨迹内部处理end-effector偏移
```

### 2. 增加稳定过渡阶段
- **Stage 4增强**：从2秒增加到3秒的位置稳定期
- **旋转前稳定**：在旋转开始前增加1秒的预稳定期
- **详细日志**：增加位置、yaw、进度的详细日志

### 3. 改进旋转执行
- 添加旋转进度日志（每2秒）
- 增加旋转参数的详细日志
- 改进紧急检测的稳定性

## 关键代码修改

### 1. 高度计算修复
```python
# 在 RotateValveState.execute 中：
current_grasp_height = current_pos[2]  # 使用当前body高度，不是end-effector高度

rotation_traj = ValveRotationTrajectory(
    # ... 其他参数 ...
    grasp_height=current_grasp_height,  # 修复高度计算
    # ... 其他参数 ...
)
```

### 2. 增加稳定过渡
```python
# Stage 4增强：
hold_duration = 3.0  # 从2.0增加到3.0秒

# 旋转前稳定：
rospy.loginfo("Pre-rotation stabilization: Holding current position for 1 second...")
stabilization_start = time.time()
while time.time() - stabilization_start < 1.0:
    # 维持当前位置
    MotionController.send_trajectory_point(self.pub, current_pos, self.current_yaw)
```

### 3. 详细日志增强
```python
# 高度计算日志：
rospy.loginfo(f"Rotation height calculation:")
rospy.loginfo(f"  Current body position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
rospy.loginfo(f"  Using grasp height: {current_grasp_height:.3f}m (body height)")

# 旋转进度日志：
if current_time - last_log_time > 2.0:
    progress = min(100.0, (elapsed_time / self.rotation_duration) * 100)
    rospy.loginfo(f"Rotation progress: {progress:.1f}% ({elapsed_time:.1f}s/{self.rotation_duration:.1f}s)")
```

## 测试建议

### 1. 日志监控重点
- 观察Stage 3到Stage 4的高度变化
- 确认旋转前后的位置连续性
- 检查旋转过程中的平滑性

### 2. 参数调整
如果仍有问题，可以调整：
- `hold_duration` (Stage 4稳定时间)
- `rotation_duration` (旋转持续时间)
- `stuck_threshold` (卡住检测阈值)

### 3. 故障排除
- 如果仍有上冲：检查 `grasp_height` 计算
- 如果旋转不平滑：增加 `rotation_duration`
- 如果误检测卡住：调整 `movement_threshold`

## 预期效果

修复后应该看到：
1. **Stage 4无突然上冲**：高度应该平滑维持
2. **旋转过渡平滑**：从插入到旋转应该无跳变
3. **详细状态日志**：便于调试和监控
4. **稳定的旋转轨迹**：圆滑的旋转路径

## 如何验证修复

1. **运行测试**：
   ```bash
   roslaunch beetle valve_rotation_single_uav.launch
   ```

2. **关键日志检查**：
   - Stage 3完成后的位置
   - Stage 4稳定后的位置
   - 旋转开始前的位置
   - 旋转过程中的平滑性

3. **成功指标**：
   - 无突然高度跳变
   - 旋转轨迹平滑
   - 无异常Force/Wrench读数
   - 任务完成无紧急停止

这次修复主要解决了高度计算的重复偏移问题，并增加了必要的稳定过渡阶段，应该能够消除"突然上冲"的问题。
