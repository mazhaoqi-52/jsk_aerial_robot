# Enhanced Valve Rotation with Strict Alignment Control

## 概述

此增强版本的阀门旋转系统实现了严格的反馈控制，确保在拧阀门过程中：
1. **机体COG（重心）**
2. **末端执行器中心**
3. **阀门中心**

三者近乎共线，并且轨迹稳定。

## 主要特性

### 1. 严格对齐控制 (Strict Alignment Control)
- **实时监控**：25Hz频率监控COG-End_effector-Valve对齐状态
- **实时纠正**：自动计算并应用位置和姿态纠正
- **预旋转对齐**：在开始旋转前确保最佳对齐状态

### 2. 对齐检查指标
- **位置对齐**：末端执行器到阀门中心距离 < 1cm
- **姿态对齐**：偏航角误差 < 0.05弧度（~2.9°）
- **距离对齐**：UAV到阀门的距离符合几何要求
- **共线性检查**：COG-End_effector-Valve三点共线性角度 < 0.05弧度

### 3. 反馈控制系统
- **位置纠正**：基于理想几何位置的位置纠正
- **姿态纠正**：确保UAV朝向阀门中心
- **增益控制**：可配置的控制增益（0.8）
- **限制保护**：最大纠正量限制防止过度调整

## 配置参数

### 严格对齐控制参数
```yaml
strict_alignment_enabled: true          # 启用严格对齐控制
alignment_correction_enabled: true      # 启用实时对齐纠正
alignment_position_tolerance: 0.01      # 位置容差（米）
alignment_yaw_tolerance: 0.05          # 姿态容差（弧度）
alignment_control_gain: 0.8            # 控制增益
alignment_check_frequency: 25          # 检查频率（Hz）
```

### 末端执行器参数
```yaml
end_effector_offset: [0.0, 0.0, -0.246]  # 相对于COG的偏移（米）
```

### 轨迹稳定性参数
```yaml
rotation_radius_tolerance: 0.005         # 旋转半径容差（米）
rotation_height_tolerance: 0.01          # 高度容差（米）
```

## 使用方法

### 1. 基本使用
```bash
# 启动增强版阀门旋转
roslaunch beetle enhanced_valve_rotation.launch module_id:=1
```

### 2. 自定义参数
```bash
# 使用自定义对齐参数
roslaunch beetle enhanced_valve_rotation.launch \
  module_id:=1 \
  alignment_position_tolerance:=0.005 \
  alignment_control_gain:=0.9 \
  debug_alignment:=true
```

### 3. 调试模式
```bash
# 启用对齐调试信息
roslaunch beetle enhanced_valve_rotation.launch \
  module_id:=1 \
  debug_alignment:=true \
  debug_trajectory:=true
```

## 工作流程

### 阶段1：预旋转对齐
1. 检查当前COG-End_effector-Valve对齐状态
2. 计算位置和姿态纠正
3. 应用纠正直到达到对齐容差
4. 记录对齐统计信息

### 阶段2：增强旋转执行
1. **轨迹生成**：基于对齐后的位置生成旋转轨迹
2. **实时监控**：25Hz频率监控对齐状态
3. **实时纠正**：检测到偏差时自动应用纠正
4. **统计记录**：记录对齐违规次数和最大误差

### 阶段3：最终验证
1. 检查最终对齐状态
2. 报告旋转统计信息
3. 验证轨迹稳定性

## 日志输出示例

```
=== ENHANCED VALVE ROTATION WITH STRICT ALIGNMENT ===
Strict alignment enabled: true
Alignment correction: true
Position tolerance: 0.010m
Yaw tolerance: 0.050rad (2.9°)

=== PRE-ROTATION ALIGNMENT PHASE ===
Pre-rotation alignment progress:
  - End-effector to valve distance: 0.008m (tolerance: 0.010m)
  - Yaw error: 0.032rad (tolerance: 0.050rad)
  - Collinearity angle: 0.012rad
  - Status: pos_aligned=true, yaw_aligned=true, collinear=true

Enhanced rotation progress: 25.0% (2.0s/8.0s)
  Current UAV position: [2.877, 0.123, 0.550]
  Alignment status: ALIGNED
  End-effector to valve: 0.007m

=== ENHANCED VALVE ROTATION COMPLETED ===
Total rotation time: 8.2s
Alignment violations: 3
Max position error: 0.012m
Max yaw error: 0.043rad
Final alignment status: ALIGNED
```

## 技术细节

### 对齐计算
系统使用以下公式确保三点共线：
1. **末端执行器位置**：通过坐标变换从UAV位置计算
2. **理想UAV位置**：基于几何约束计算
3. **共线性检查**：使用向量夹角验证

### 控制算法
- **比例控制**：基于误差的线性纠正
- **限幅保护**：防止过度调整
- **平滑滤波**：确保稳定的控制输出

### 安全机制
- **最大纠正限制**：位置2cm，姿态0.1弧度
- **超时保护**：预对齐阶段10秒超时
- **紧急检测**：保持原有的卡住检测机制

## 故障排除

### 常见问题
1. **对齐超时**：检查末端执行器偏移参数
2. **过度振荡**：降低控制增益
3. **对齐失败**：检查容差设置

### 参数调整建议
- **保守设置**：control_gain=0.5, tolerance=0.015
- **激进设置**：control_gain=0.9, tolerance=0.008
- **调试设置**：启用debug_alignment和debug_trajectory

## 性能指标

### 期望性能
- **对齐精度**：< 1cm位置误差，< 3°姿态误差
- **响应速度**：< 2秒达到对齐状态
- **稳定性**：旋转过程中< 5次对齐违规

### 监控指标
- 对齐违规次数
- 最大位置误差
- 最大姿态误差
- 轨迹稳定性
