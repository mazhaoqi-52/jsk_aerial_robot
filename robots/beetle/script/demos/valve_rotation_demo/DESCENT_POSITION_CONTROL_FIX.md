# Stage 3 Descent Position Control Fix

## 问题分析

从日志分析发现Stage 3下降阶段存在以下关键问题：

1. **XY位置漂移严重**：
   - Pre-descent: [2.921, -0.149, 1.138]
   - Post-descent: [2.954, -0.190, 0.680]
   - XY漂移：约5.2cm，在精密插入任务中不可接受

2. **Yaw角度漂移严重**：
   - Pre-descent yaw: 0.580 rad
   - Post-descent yaw: 1.651 rad
   - Yaw漂移：约1.071 rad（61.4度），严重影响插入精度

3. **初始上升问题**：
   - 日志显示`descended: -0.004m`，表明UAV在开始时向上移动
   - 这可能导致后续下降控制不稳定

4. **力检测机制不可靠**：
   - 依赖外力检测来判断接触，但可能受环境因素影响
   - 在实际插入过程中，力检测可能过早或过晚触发

5. **下降过程缺乏综合位置控制**：
   - 原始实现只固定XY位置，忽略了yaw控制
   - 导致UAV在下降过程中发生不可控的姿态漂移

## 修复方案

### 1. 完整位置控制下降策略

**核心思想**：在下降过程中固定XY位置和yaw角度，只改变Z坐标，避免任何形式的漂移。

```python
# 固定XY位置和yaw，防止漂移
fixed_x = start_pos[0]
fixed_y = start_pos[1]
fixed_yaw = self.current_yaw  # 关键修复：固定yaw角度

# 发送位置控制命令
msg.target_pos_x = fixed_x      # 保持固定X
msg.target_pos_y = fixed_y      # 保持固定Y
msg.target_pos_z = target_height # 渐进式Z控制
msg.target_yaw = fixed_yaw      # 保持固定yaw
```

### 2. 初始位置保持策略

**解决上升问题**：在开始下降前，保持当前位置0.5秒，确保系统稳定。

```python
# 初始位置保持，防止上升
hold_duration = 0.5
for hold_time in range(hold_duration):
    msg.target_pos_z = start_pos[2]  # 保持起始高度
    self.pub.publish(msg)
```

### 3. 基于移动的卡住检测

**替代力检测**：使用Z方向移动量来判断是否卡住，更加可靠。

```python
# 检测参数
stuck_check_interval = 2.0  # 每2秒检查一次
min_descent_rate = 0.02     # 最小下降速率：2cm/2s

# 卡住检测逻辑
if height_change < min_descent_rate:
    rospy.logwarn("UAV appears stuck")
    return False
```

### 4. 全方位漂移监控

**实时监控**：持续监控XY位置和yaw角度漂移，超过阈值时发出警告。

```python
# XY漂移监控
xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
if xy_drift > 0.05:
    rospy.logwarn(f"XY drift detected: {xy_drift:.3f}m")

# Yaw漂移监控
yaw_drift = abs(self.current_yaw - fixed_yaw)
if yaw_drift > 0.2:  # 约11度
    rospy.logwarn(f"Yaw drift detected: {yaw_drift:.3f} rad")
```

## 关键改进点

### 1. `execute_descent_with_contact_detection`方法

**修改前**：
- 使用多项式轨迹，XY位置跟随轨迹变化
- 没有固定yaw角度，导致严重的姿态漂移
- 依赖力检测判断接触
- 容易产生XY位置漂移和yaw漂移
- 存在初始上升问题

**修改后**：
- 固定XY位置和yaw角度，只控制Z坐标
- 添加初始位置保持，防止上升问题
- 使用移动检测替代力检测
- 实时监控XY和yaw漂移
- 更可靠的卡住检测机制

### 2. `continue_descent_until_contact`方法

**修改前**：
- 使用当前位置进行下降，可能累积漂移
- 没有固定yaw角度
- 依赖力检测判断完成

**修改后**：
- 延续固定XY位置和yaw角度策略
- 基于移动的卡住检测
- 清晰的进度日志，包含yaw漂移监控

## 预期效果

1. **消除XY位置漂移**：
   - 下降过程中XY位置保持稳定
   - 提高插入精度

2. **消除yaw角度漂移**：
   - 下降过程中yaw角度保持固定
   - 避免插入方向偏差

3. **解决初始上升问题**：
   - 通过初始位置保持确保稳定开始
   - 避免意外的上升运动

4. **更可靠的下降控制**：
   - 基于实际移动量的卡住检测
   - 减少误判和过早停止

5. **改善Stage 4稳定性**：
   - 下降后位置和姿态更精确
   - 减少Stage 4的跳跃和漂移问题

6. **详细的调试信息**：
   - 实时XY和yaw漂移监控
   - 下降进度追踪
   - 卡住检测状态日志

## 测试验证

运行修复后的代码，观察以下指标：

1. **XY位置稳定性**：
   ```
   Pre-descent position: [X, Y, Z]
   Post-descent position: [X±0.01, Y±0.01, Z_target]
   ```

2. **Yaw角度稳定性**：
   ```
   Pre-descent yaw: A.BBB rad
   Post-descent yaw: A.BBB±0.05 rad
   ```

3. **初始稳定性**：
   ```
   Initial position hold: 0.5s to prevent upward movement
   Position hold completed, starting descent...
   ```

4. **下降进度**：
   ```
   Descent progress: 2.0s, descended: 0.100m, XY drift: 0.002m, yaw drift: 0.005rad
   ```

5. **卡住检测**：
   ```
   Descent progress: 0.020m in 2.0s
   ```

6. **完成状态**：
   ```
   Reached target descent height
   Total descent achieved: 0.563m
   Final yaw drift: 0.010 rad
   ```

## 后续优化

1. **参数调优**：
   - 调整卡住检测阈值
   - 优化下降速率
   - 微调XY位置控制

2. **增强监控**：
   - 添加更多传感器数据
   - 实时性能指标
   - 异常情况处理

3. **适应性改进**：
   - 根据实际环境调整参数
   - 支持不同任务场景
   - 动态阈值调整

这个修复是解决Stage 3下降问题的关键，应该能显著改善插入过程的稳定性和精度。
