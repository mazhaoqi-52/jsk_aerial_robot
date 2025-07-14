# 改进的End Effector插入策略 - 4阶段插入

## 最新优化 (2024-12-19)

### 问题修复
1. **修复yaw角度插值问题**：
   - 原始代码使用动态变化的`self.current_yaw`作为插值起点，导致角度跳跃
   - 现在使用固定的起始角度进行稳定插值
   - 添加角度规范化，确保使用最短路径旋转

2. **限制单次角度变化**：
   - 限制单次轨迹的最大yaw变化为45°（π/4弧度）
   - 圆周调整时限制最大yaw变化为30°（π/6弧度）
   - 接近阶段限制最大yaw变化为30°（π/6弧度）

3. **修复偏移过头问题**：
   - 调整偏移计算逻辑，防止多个offset参数叠加导致过度偏移
   - 圆周偏移在接近阶段减半使用：`circumferential_offset * 0.5`
   - 减少切线偏移：从0.015m降至0.010m
   - 调整默认参数：`pre_insertion_distance`从0.05m降至0.03m，`circumferential_offset`从0.03m降至0.02m

4. **更保守的默认参数**：
   - `yaw_adjustment_angle`默认值从0.2改为0.1弧度（约5.7°）
   - `pre_insertion_distance`默认值从0.05m改为0.03m
   - `circumferential_offset`默认值从0.03m改为0.02m

### 技术改进
- **角度规范化**：所有角度插值都经过规范化到[-π, π]范围
- **最短路径**：自动选择最短的旋转路径
- **平滑插值**：使用3次平滑函数确保轨迹连续性
- **详细日志**：增加角度变化的详细日志输出

## 核心改进

基于您的设想，实现了**4阶段插入策略**，完整的插入逻辑为：

1. **接近插入点** - 带圆周避让的接近
2. **插入方向调整** - Yaw方向和圆周方向调整  
3. **下降接触** - 缓慢下降建立接触
4. **旋转到位** - 最终旋转到目标位置

## 4阶段插入详解

### 阶段1: 接近插入点 (带圆周避让)
- **目标**: 避开直接路径，从圆周方向接近
- **特点**: 计算目标抓取角度，加上角度偏移进行避让接近
- **参数**: `yaw_adjustment_angle` (yaw避让角度), `circumferential_offset` (圆周偏移距离)
- **优化**: 限制最大yaw变化为30°，防止激烈旋转

### 阶段2: 插入方向调整 (圆周方向调整)
- **目标**: 沿圆周方向调整到精确插入位置和姿态
- **特点**: 平滑的圆周轨迹，同时调整yaw朝向
- **监控**: 力反馈监控防止过度接触
- **优化**: 使用角度规范化确保最短路径旋转

### 阶段3: 下降接触
- **目标**: 垂直下降建立接触
- **特点**: 保持yaw姿态，仅Z方向运动
- **监控**: 接触力检测

### 阶段4: 旋转到位
- **目标**: 最终位置微调和稳定
- **特点**: 保持接触，稳定姿态

## 主要改进点

### 1. 圆周避让策略
- **避让接近**: 计算目标抓取角度，加上 `yaw_adjustment_angle` 进行避让
- **圆周调整**: 沿圆周方向平滑调整到精确位置
- **Yaw控制**: 在圆周移动过程中同步调整yaw朝向

### 2. 新增参数
- `use_staged_insertion`: 是否使用4阶段插入 (默认: true)
- `pre_insertion_distance`: 预插入安全距离 (默认: 0.03m，从0.05m调整)
- `yaw_adjustment_angle`: Yaw避让角度 (默认: 0.1 rad ≈ 5.7°)
- `circumferential_offset`: 圆周偏移距离 (默认: 0.02m，从0.03m调整)
- `contact_force_threshold`: 接触力阈值 (默认: 2.0N)
- `descent_speed`: 下降速度 (默认: 0.05m/s)

## 使用方法

### 1. 使用4阶段插入 (推荐)
```bash
# 默认使用4阶段插入
rosrun beetle valve_rotation_fang_single.py _module_id:=1

# 自定义参数
rosrun beetle valve_rotation_fang_single.py \
  _module_id:=1 \
  _use_staged_insertion:=true \
  _yaw_adjustment_angle:=0.1 \
  _circumferential_offset:=0.03 \
  _contact_force_threshold:=2.0
```

### 2. 保守参数设置（推荐调试用）
```bash
# 保守参数，避免激烈旋转和过度偏移
rosrun beetle valve_rotation_fang_single.py \
  _module_id:=1 \
  _use_staged_insertion:=true \
  _yaw_adjustment_angle:=0.05 \
  _circumferential_offset:=0.015 \
  _pre_insertion_distance:=0.025 \
  _contact_force_threshold:=2.0 \
  _descent_speed:=0.02
```

### 3. 使用原始插入方法
```bash
# 禁用4阶段插入
rosrun beetle valve_rotation_fang_single.py \
  _module_id:=1 \
  _use_staged_insertion:=false
```

## 调试建议

### 如果遇到角度跳跃问题
1. **检查yaw_adjustment_angle**：
   - 从0.05弧度（约3°）开始测试
   - 观察日志中的角度变化信息
   - 逐步增加到0.1-0.15弧度

2. **确认无人机稳定性**：
   - 确保无人机初始悬停稳定
   - 检查valve_pos和valve_yaw是否正确

3. **观察日志输出**：
   - 关注"yaw change"和"angle change"信息
   - 查看是否有"Limiting yaw change"警告

### 参数调优顺序
1. 首先确保`yaw_adjustment_angle`较小（0.05-0.1）
2. 调整`circumferential_offset`控制接近路径
3. 根据实际情况调整`pre_insertion_distance`
4. 最后优化`descent_speed`和`contact_force_threshold`

### 常见问题及解决方案

#### 问题1: 无人机旋转过度
**解决方案**:
- 减小`yaw_adjustment_angle`至0.05弧度
- 检查valve_yaw是否正确
- 确认无人机起始姿态稳定

#### 问题2: 插入位置偏移过头
**原因**:
- 多个offset参数叠加：approach_radius = 0.1225 + pre_insertion_distance + circumferential_offset
- 额外的tangential offset: 0.015m
- 总偏移过大导致无法精确定位

**解决方案**:
- 减小`pre_insertion_distance`至0.02-0.03m
- 减小`circumferential_offset`至0.01-0.02m
- 接近阶段的circumferential_offset会自动减半
- 检查偏移计算逻辑是否合理

#### 问题3: 接触力过大
**解决方案**:
- 减小`descent_speed`
- 降低`contact_force_threshold`
- 增加`pre_insertion_distance`

## 代码变更总结

### 关键修改
1. **角度插值优化**：修复yaw插值中的角度跳跃问题
2. **角度限制**：添加单次轨迹的最大角度变化限制
3. **角度规范化**：确保所有角度计算使用最短路径
4. **参数优化**：调整默认参数使其更加保守
5. **详细日志**：增加角度变化的详细日志输出

### 代码量对比
- **原始方法**: 保持不变，作为备选方案
- **新增代码**: 约150行，主要是4阶段插入逻辑
- **总体增加**: 最小化，没有引入复杂的新类

## 预期效果

### 稳定性提升
- 避免激烈的角度跳跃
- 平滑的插入轨迹
- 更好的力控制

### 成功率提升
- 分阶段逐步接近，减少碰撞风险
- 圆周避让策略提高成功率
- 力反馈监控防止过度接触

### 适应性增强
- 可配置的参数适应不同场景
- 备选原始方法确保兼容性
- 详细日志便于调试优化

## 技术细节

### 偏移计算优化
```python
# 接近阶段的偏移计算（减半防止过度偏移）
approach_radius = 0.1225 + self.pre_insertion_distance + self.circumferential_offset * 0.5

# 切线偏移减少（从0.015m降至0.010m）
insertion_offset_x = 0.010 * tangent_x
insertion_offset_y = 0.010 * tangent_y
```

### 偏移参数说明
- **基础半径**: 0.1225m（阀门半径）
- **预插入距离**: 0.03m（默认，可调）
- **圆周偏移**: 0.02m（默认，接近时减半使用）
- **切线偏移**: 0.010m（用于精确定位）

### 总偏移量计算
```
接近阶段总偏移 = 0.1225 + 0.03 + 0.02*0.5 = 0.1625m
精确插入偏移 = 基础半径 + 切线偏移 = 0.1225 + 0.010 = 0.1325m
```

### 角度处理
```python
# 角度规范化到[-π, π]范围
while yaw_diff > math.pi:
    yaw_diff -= 2 * math.pi
while yaw_diff < -math.pi:
    yaw_diff += 2 * math.pi

# 限制最大角度变化
max_yaw_change = math.pi / 4  # 45度
if abs(yaw_diff) > max_yaw_change:
    yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
```

### 平滑插值
```python
# 3次平滑插值函数
smooth_t = 3*t**2 - 2*t**3
current_yaw = start_yaw + smooth_t * yaw_diff
```

### 力监控
```python
# 接触力检测
current_force = self.get_contact_force_magnitude()
if current_force > self.contact_force_threshold * 1.5:
    rospy.logwarn(f"High force detected: {current_force:.2f}N")
```

通过这些优化，插入过程应该更加稳定、平滑，避免了之前的角度跳跃问题。

## 最新修复 (2024-12-19 下午)

### 修复3: xy方向调整过头问题
**现象**: 从接近位置到精确插入位置的xy方向变化过大，如从`[2.838, -0.016]`到`[2.685, 0.010]`
**原因**: 
- 阶段1和阶段2的位置计算逻辑不一致
- 阶段2重新计算了完整的插入位置，而不是基于当前位置的小幅调整

**解决方案**:
1. 简化阶段2的调整逻辑，仅做小幅圆周调整
2. 添加最大XY调整距离限制（5cm）
3. 减少阶段2的角度偏移至原值的30%
4. 移除阶段2的复杂end-effector计算

### 修复4: 紧急状态userdata错误
**现象**: `InvalidUserCodeError: Reading from SMACH userdata key 'get'`
**原因**: EmergencyState中使用了`userdata.get()`方法，但userdata不是字典

**解决方案**:
1. 修改为使用`hasattr(userdata, 'start_position')`检查
2. 直接访问`userdata.start_position`属性
3. 添加默认值处理

### 修复5: UAV stuck检测过于严格
**现象**: 旋转时UAV被误判为stuck，触发emergency stop
**原因**: 检测阈值过于严格，正常的旋转停顿被误判

**解决方案**:
1. 增加stuck检测时间阈值：3.0s → 5.0s
2. 减少位置移动阈值：0.01m → 0.005m
3. 减少yaw变化阈值：0.05rad → 0.02rad
