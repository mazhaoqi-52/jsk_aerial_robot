# 手动插入后自动阀门旋转程序使用说明

## 概述
这个程序是为了方便实验而创建的，允许用户手动将manipulator插入阀门后，由程序自动完成后续的对齐、旋转和上升操作。

## 程序特点

### 🎯 实验友好设计
- **手动插入**: 用户可以手动精确控制manipulator的插入过程
- **自动对齐**: 程序自动完成无人机中心与阀门中心的对齐
- **自动旋转**: 程序自动执行阀门旋转操作
- **自动上升**: 完成任务后自动上升到安全高度

### 🔧 技术特性
- **增强控制器集成**: 如果可用，使用增强的对齐控制系统
- **备选算法**: 在模块不可用时自动切换到基本算法
- **手柄控制**: 使用游戏手柄控制程序启动和紧急停止
- **实时监控**: 实时显示程序状态和进度

## 文件结构

```
beetle/
├── script/demos/valve_rotation_demo/
│   ├── valve_rotation_fang_single_joy.py    # 主程序
│   ├── print_launch_params.py               # 参数打印脚本
│   └── enhanced_motion_controller.py        # 增强控制器（如果可用）
└── launch/valve_rotation_tasks/
    └── valve_rotation_joy.launch             # 启动文件
```

## 使用步骤

### 1. 启动程序
```bash
roslaunch beetle valve_rotation_joy.launch
```

### 2. 参数配置（可选）
```bash
# 修改旋转角度（90度）
roslaunch beetle valve_rotation_joy.launch rotation_angle:=1.5708

# 修改旋转时间（8秒）  
roslaunch beetle valve_rotation_joy.launch rotation_duration:=8.0

# 修改阀门位置
roslaunch beetle valve_rotation_joy.launch valve_x:=3.0 valve_y:=0.0 valve_z:=0.55

# 修改上升高度
roslaunch beetle valve_rotation_joy.launch ascent_height:=0.5
```

### 3. 操作流程
1. **手动插入**: 使用现有的手动控制方式将manipulator插入阀门
2. **位置检测**: 程序会自动检测是否接近阀门插入位置
3. **启动序列**: 当程序提示"可以按下手柄启动按钮"时，按下手柄的Start按钮（按钮7）
4. **自动执行**: 程序将自动执行以下步骤：
   - 精确对齐无人机中心与阀门中心
   - 确保manipulator中心对齐
   - 执行阀门旋转
   - 完成后自动上升

### 4. 紧急停止
- 按下手柄的Select按钮（按钮6）立即停止程序

## 手柄按钮映射

| 按钮 | 功能 |
|------|------|
| Start (按钮7) | 启动自动对齐和旋转序列 |
| Select (按钮6) | 紧急停止 |

## 程序状态

程序会显示以下状态信息：

- **WAITING_FOR_INSERTION**: 等待手动插入manipulator
- **ALIGNING**: 正在执行精确对齐
- **ROTATING**: 正在执行阀门旋转
- **ASCENDING**: 正在上升到安全高度
- **COMPLETED**: 任务完成
- **FAILED**: 任务失败

## 参数配置

### 基本参数
- `module_id`: UAV模块ID（默认: 1）
- `rotation_angle`: 旋转角度（默认: π/2 = 90°）
- `rotation_duration`: 旋转时间（默认: 8.0秒）
- `ascent_height`: 上升高度（默认: 0.5米）

### 阀门参数
- `valve_pos`: 阀门位置 [x, y, z]（默认: [3.0, 0.0, 0.55]）
- `valve_yaw`: 阀门偏航角（默认: 0.0）
- `z_offset`: Z轴偏移（默认: 0.55米）

### 对齐控制参数
- `alignment_position_tolerance`: 位置对齐容差（默认: 0.01米）
- `alignment_yaw_tolerance`: 偏航对齐容差（默认: 0.05弧度）
- `alignment_timeout`: 对齐超时时间（默认: 15.0秒）

### 增强控制器参数
- `strict_alignment_enabled`: 启用严格对齐（默认: true）
- `alignment_correction_enabled`: 启用对齐纠正（默认: true）
- `alignment_control_gain`: 对齐控制增益（默认: 0.8）
- `alignment_check_frequency`: 对齐检查频率（默认: 25Hz）

## 监控和调试

### 状态监控
程序会发布以下ROS话题：
- `/valve_rotation_status`: 任务完成状态
- `/uav1/nav`: UAV导航信息

### 日志信息
程序会输出详细的日志信息，包括：
- 当前程序状态
- 对齐进度
- 旋转进度
- 错误信息

### 调试命令
```bash
# 监控UAV位置
rostopic echo /uav1/nav

# 监控任务状态
rostopic echo /valve_rotation_status

# 查看程序日志
rosnode info valve_rotation_joy
```

## 故障排除

### 常见问题

1. **程序无法启动**
   - 检查ROS环境是否正确设置
   - 确认UAV和阀门数据是否正常发布

2. **手柄不响应**
   - 检查手柄连接: `ls /dev/input/js*`
   - 确认joy_node正常运行: `rostopic echo /joy`

3. **对齐失败**
   - 检查阀门位置参数是否正确
   - 调整对齐容差参数

4. **旋转不稳定**
   - 检查增强控制器是否可用
   - 调整旋转时间和控制增益

### 调试模式
启用调试模式查看更详细信息：
```bash
roslaunch beetle valve_rotation_joy.launch --screen
```

## 安全注意事项

1. **紧急停止**: 随时准备使用紧急停止按钮
2. **手动监控**: 即使是自动程序，也要持续监控无人机状态
3. **参数检查**: 启动前检查所有参数设置是否合理
4. **测试环境**: 在安全的测试环境中进行实验

## 扩展功能

程序设计为模块化，可以根据需要添加以下功能：
- 更多的手柄控制选项
- 视觉反馈系统
- 力反馈控制
- 多UAV协同控制

## 联系和支持

如有问题或建议，请参考原始的valve_rotation_fang_single.py文件或联系开发团队。
