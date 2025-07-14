# 双牙插入策略使用说明

## 插入位置说明

### 阀门结构
- 阀门有3个横梁：beam0(0°), beam1(120°), beam2(240°)
- 角度基于阀门的 yaw 方向

### 插入位置计算

#### 第一根牙 (Fang1)
- **位置**: beam1(120°) 和 beam2(240°) 之间，靠近 beam2 一侧
- **计算**: `fang1_angle = beam2_angle - 0.1 * (2π/3)`
- **说明**: 从 beam2 向 beam1 方向偏移 12°（10% 偏移）

#### 第二根牙 (Fang2) 
- **位置**: beam0(0°) 和 beam1(120°) 之间，靠近 beam1 一侧
- **计算**: `fang2_angle = beam1_angle - 0.1 * (2π/3)`
- **说明**: 从 beam1 向 beam0 方向偏移 12°（10% 偏移）

### 当前实现
- 目前使用 fang1 作为主要插入位置
- 未来可扩展支持双牙同时插入

## 4阶段插入策略

### Stage 1: 接近插入点 (带圆周避让)
- 在目标插入点周围预设距离接近
- 添加圆周偏移避免直接碰撞
- 限制 yaw 变化幅度

### Stage 2: 插入方向调整
- 精确调整到插入角度
- 减少圆周偏移
- 逐步调整 yaw 方向

### Stage 3: 下降接触
- 垂直下降建立接触
- 监控接触力
- 达到力阈值停止

### Stage 4: 旋转到位
- 最终旋转到目标位置
- 保持接触力

## 参数配置

### 核心参数
```bash
# 预插入距离 (接近阶段与阀门的距离)
~pre_insertion_distance: 0.03  # 米

# Yaw 调整角度 (避让时的角度偏移)
~yaw_adjustment_angle: 0.1  # 弧度

# 圆周偏移 (避让时的圆周距离)
~circumferential_offset: 0.02  # 米

# 接触力阈值
~contact_force_threshold: 2.0  # 牛顿

# 下降速度
~descent_speed: 0.05  # 米/秒
```

### 使用建议
- `pre_insertion_distance`: 0.02-0.05米，调节接近距离
- `yaw_adjustment_angle`: 0.05-0.2弧度，调节避让角度
- `circumferential_offset`: 0.01-0.03米，调节圆周避让距离
- `contact_force_threshold`: 1.5-3.0牛顿，根据实际情况调节
- `descent_speed`: 0.03-0.1米/秒，慢速确保稳定

## 调试建议

### 查看插入位置计算
```bash
# 启动时会输出详细角度信息
# 检查日志中的：
# - Beam angles: 0°=xxx, 120°=xxx, 240°=xxx
# - Fang 1 angle: xxx rad (xxx°) - 期望约228°
# - Fang 2 angle: xxx rad (xxx°) - 期望约108°
```

### 参数调优顺序
1. 先调整 `pre_insertion_distance` 确保接近距离合适
2. 调整 `yaw_adjustment_angle` 避免碰撞
3. 微调 `circumferential_offset` 优化路径
4. 最后调整 `contact_force_threshold` 确保稳定接触

### 常见问题
- **插入角度不准确**: 检查阀门 yaw 角度是否正确
- **接近过程碰撞**: 增加 `pre_insertion_distance` 或 `circumferential_offset`
- **插入过程不稳定**: 降低 `descent_speed`，调整 `yaw_adjustment_angle`
- **接触力过大/过小**: 调整 `contact_force_threshold`
- **旋转过程高度变化**: 已修复，现在使用正确的end-effector高度

## 扩展性
- 当前实现支持单牙插入
- 代码结构已准备好扩展双牙同时插入
- 可通过修改 `target_grasp_angle` 选择使用 fang1 或 fang2
