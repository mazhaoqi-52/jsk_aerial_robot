# Manual Valve Rotation Controller

## 概述

这个文件 `valve_rotation_fang_single_joy.py` 专为手动插入后的阀门旋转任务设计。假设无人机的两个end effector已经通过手动操作插入到阀门中，这个脚本将完成拧阀门的任务。

## 主要特性

1. **慢速精确控制**: 旋转速度设定为0.05m/s，确保精确控制
2. **轨迹跟踪**: 无人机绕阀门中心"公转"，同时保持朝向阀门中心"自转"
3. **严格反馈控制**: 确保x、y、z、yaw角严格遵守轨迹，最小化pitch和roll偏移
4. **自动执行**: 启动后自动开始旋转，无需手动干预

## 轨迹可视化

### 一键轨迹预览（推荐）
最简单的方式，一键启动轨迹预览：

```bash
# 实际环境预览轨迹（使用MOCAP数据，无需仿真）
roslaunch beetle valve_rotation_manual.launch preview_only:=true

# 仿真环境预览轨迹（需要先启动仿真）
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true

# 自定义预览参数
roslaunch beetle valve_rotation_manual.launch preview_only:=true \
  rotation_angle:=1.5708 rotation_speed:=0.05 preview_duration:=30.0
```

**重要说明**: 
- 默认情况下，预览模式**不需要开仿真**，直接使用MOCAP系统的实际位置数据
- 只有在仿真环境中才需要设置 `simulation:=true` 参数

### 实时轨迹可视化
在运行阀门旋转时，系统会自动在RViz中发布轨迹可视化：

```bash
# 启动包含RViz的完整系统
roslaunch beetle valve_rotation_manual.launch rviz:=true
```

### 可视化内容
- **蓝色轨迹线**: 无人机的运动轨迹
- **蓝色箭头**: 轨迹上的朝向指示
- **绿色箭头**: 当前目标点（仅实际控制时）
- **红色圆柱**: 阀门位置
- **黄色球体**: UAV起始位置（仅预览模式）

### RViz话题
系统发布以下可视化话题：

**实时控制时**：
- `/beetle1/trajectory_visualization`: 实时轨迹
- `/beetle1/current_trajectory_point`: 当前目标点
- `/beetle1/valve_marker`: 阀门标记

**轨迹预览时**：
- `/beetle1/trajectory_preview`: 预览轨迹
- `/beetle1/valve_preview`: 阀门预览
- `/beetle1/uav_start_preview`: UAV起始位置

### 可视化配置
**使用默认RViz（推荐）**：
- 系统会自动发布轨迹到默认RViz中
- 在RViz中手动添加以下Display来查看轨迹：
  - **MarkerArray**: 订阅 `/beetle1/trajectory_visualization` 或 `/beetle1/trajectory_preview`
  - **Marker**: 订阅 `/beetle1/valve_marker` 或 `/beetle1/valve_preview`
  - **Marker**: 订阅 `/beetle1/current_trajectory_point`（实时控制时）

**使用专用RViz配置**：
- 自动加载的RViz配置文件：`config/valve_rotation_visualization.rviz`
- 启动方式：`roslaunch beetle valve_rotation_manual.launch rviz:=true`
- 可以根据需要调整颜色、大小等显示参数

### RViz配置说明
系统提供了灵活的RViz配置选项，您可以根据需要选择：

#### 选项1：使用默认RViz（简单）
- 使用bringup.launch自带的默认RViz
- 需要手动添加轨迹Display（仅首次）
- 适合熟悉RViz配置的用户

#### 选项2：使用专用RViz（推荐）
- 专门为阀门旋转优化的RViz配置
- 所有轨迹可视化都已预配置
- 无需手动配置Display

#### 选项3：不启动RViz
- 仅运行控制器，不显示可视化
- 适合纯命令行操作

### 使用示例

**仿真环境**：
```bash
# 1. 启动仿真环境
roslaunch beetle bringup.launch headless:=false simulation:=true real_machine:=false

# 2. 起飞无人机
rosrun beetle keyboard_command_multi.py

# 3. 导入阀门模型
roslaunch beetle valve_pos.launch simulation:=true real_machine:=false

# 4. 选择RViz配置启动阀门旋转
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true                    # 无RViz
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true rviz:=true         # 专用RViz
```

**实际环境**：
```bash
# 选择RViz配置
roslaunch beetle valve_rotation_manual.launch preview_only:=true                    # 无RViz
roslaunch beetle valve_rotation_manual.launch preview_only:=true rviz:=true         # 专用RViz
```

## 使用方法

### 方案一：专用RViz配置流程（推荐）

#### 实际环境流程：
```bash
# 步骤1：轨迹预览（自动启动专用RViz）
roslaunch beetle valve_rotation_manual.launch preview_only:=true rviz:=true

# 步骤2：确认轨迹合理后，执行实际旋转
roslaunch beetle valve_rotation_manual.launch module_id:=1 rviz:=true
```

#### 仿真环境流程：
```bash
# 步骤1：启动仿真环境（带RViz）
roslaunch beetle bringup.launch headless:=false simulation:=true real_machine:=false

# 步骤2：起飞无人机（使用键盘控制）
rosrun beetle keyboard_command_multi.py

# 步骤3：导入阀门模型
roslaunch beetle valve_pos.launch simulation:=true real_machine:=false

# 步骤4：键盘控制无人机到阀门附近位置

# 步骤5：轨迹预览（自动启动专用RViz）
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true rviz:=true

# 步骤6：执行阀门旋转（自动启动专用RViz）
roslaunch beetle valve_rotation_manual.launch module_id:=1 simulation:=true rviz:=true
```

### 详细操作说明

### 1. 准备工作
在启动系统前，请确保：
- 无人机（UAV）已通过手动方式将两个end effector 插入阀门
- 所有相关的传感器和控制接口正常工作

### 2. 实际环境使用
#### 2.1 一键轨迹预览（推荐先执行）
```bash
# 实际环境预览轨迹
roslaunch beetle valve_rotation_manual.launch preview_only:=true                    # 无RViz
roslaunch beetle valve_rotation_manual.launch preview_only:=true rviz:=true         # 专用RViz
```

#### 2.2 执行阀门旋转
```bash
# 实际执行阀门旋转
roslaunch beetle valve_rotation_manual.launch module_id:=1                    # 无RViz
roslaunch beetle valve_rotation_manual.launch module_id:=1 rviz:=true         # 专用RViz
```

### 3. 仿真环境使用
#### 3.1 启动仿真环境
```bash
# 1. 启动仿真环境（带RViz）
roslaunch beetle bringup.launch headless:=false simulation:=true real_machine:=false

# 2. 起飞无人机（使用键盘控制）
rosrun beetle keyboard_command_multi.py

# 3. 导入阀门模型
roslaunch beetle valve_pos.launch simulation:=true real_machine:=false

# 4. 使用键盘控制无人机到阀门附近位置
# 键盘控制说明：
# - 'w'/'s': 前进/后退
# - 'a'/'d': 左移/右移  
# - 'q'/'e': 上升/下降
# - 'j'/'l': 左转/右转
# - 'k': 悬停
# - 'ESC': 退出
```

#### 3.2 仿真轨迹预览
```bash
# 仿真环境预览轨迹
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true                    # 无RViz
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true rviz:=true         # 专用RViz
```

#### 3.3 仿真阀门旋转
```bash
# 仿真环境执行阀门旋转
roslaunch beetle valve_rotation_manual.launch module_id:=1 simulation:=true                    # 无RViz
roslaunch beetle valve_rotation_manual.launch module_id:=1 simulation:=true rviz:=true         # 专用RViz
```

### 4. 操作步骤

#### 实际环境：
1. 确保无人机已正确插入阀门
2. 先运行预览模式查看轨迹
3. 确认轨迹合理后，启动实际控制
4. 系统会自动执行慢速旋转，严格跟踪轨迹
5. 监控RViz中的轨迹可视化
6. 旋转完成后系统会自动停止

#### 仿真环境：
1. 启动仿真环境（可选择是否启动默认RViz）
2. 起飞无人机到合适高度
3. 导入阀门模型到仿真环境
4. 使用键盘控制无人机到阀门插入位置
5. 选择RViz配置运行轨迹预览确认路径
6. 启动仿真阀门旋转控制

## 参数配置

### 旋转参数
- `rotation_angle`: 旋转角度（弧度），默认π/2（90度）
- `rotation_speed`: 旋转速度（m/s），默认0.05
- `feedback_frequency`: 反馈控制频率（Hz），默认30

### 控制参数
- `position_tolerance`: 位置误差容忍度（m），默认0.015
- `yaw_tolerance`: 朝向误差容忍度（rad），默认0.02
- `distance_control_gain`: 距离控制增益，默认0.3
- `position_control_gain`: 位置控制增益，默认0.3
- `yaw_control_gain`: 朝向控制增益，默认0.2

## 技术特点

### 轨迹生成
- 使用 `create_constant_distance_trajectory` 创建等距轨迹
- 确保end effector与阀门中心保持恒定距离
- 支持顺时针和逆时针旋转

### 预览模式
- 一键预览轨迹，无需复杂配置
- 自动检测当前UAV和阀门位置
- 静态显示完整轨迹路径

### 反馈控制
- 30Hz高频率反馈控制
- 严格的位置和朝向跟踪
- 自动pitch和roll稳定

### 安全保护
- 实时监控UAV与阀门的距离
- 自动检测异常情况并紧急停止
- 防止碰撞和过度偏离轨迹

## 数据源说明

系统会根据 `simulation` 参数自动选择数据源：

### 实际环境（simulation=false，默认）
- UAV位置：`/beetle1/mocap/pose`
- 阀门位置：`/valve/mocap/pose`
- 需要MOCAP系统正常工作
- **预览轨迹时无需启动仿真**

### 仿真环境（simulation=true）
- UAV位置：`/beetle1/mocap/pose`（仿真中模拟）
- 阀门位置：`/valve/odom`（仿真中的阀门位置）
- 需要先启动Gazebo仿真
- 适合调试和参数优化

**仿真环境准备步骤**：
1. 启动仿真环境：`roslaunch beetle bringup.launch headless:=false simulation:=true real_machine:=false`
2. 起飞无人机：`rosrun beetle keyboard_command_multi.py`
3. 导入阀门模型：`roslaunch beetle valve_pos.launch simulation:=true real_machine:=false`
4. 使用键盘控制无人机到阀门附近位置
5. 选择RViz配置运行轨迹预览或实际控制：
   - 无RViz：不添加`rviz:=true`参数
   - 专用RViz：添加`rviz:=true`参数

## 故障排除

### 常见问题
1. **无法获取位置信息**: 检查mocap或传感器连接
2. **轨迹创建失败**: 确认UAV位置相对于阀门合理
3. **旋转不稳定**: 调整控制增益参数
4. **紧急停止触发**: 检查UAV是否偏离预期轨迹
5. **RViz中看不到轨迹**: 系统默认会自动启动专用RViz配置，如果仍看不到请检查话题发布状态

### 调试建议
1. 检查ROS话题: 
   - 实际环境: `/beetle1/mocap/pose`, `/valve/mocap/pose`
   - 仿真环境: `/beetle1/mocap/pose`, `/valve/odom`
2. 监控控制话题: `/beetle1/uav/nav`
3. 查看状态话题: `/valve_rotation_status`
4. 在默认RViz中添加轨迹可视化：
   - 点击"Add" → "MarkerArray" → Topic选择 `/beetle1/trajectory_visualization`
   - 点击"Add" → "Marker" → Topic选择 `/beetle1/valve_marker`
   - 点击"Add" → "Marker" → Topic选择 `/beetle1/current_trajectory_point`

## 注意事项

1. 确保在启动旋转前，UAV已正确插入阀门
2. 旋转过程中避免手动干预
3. 监控系统状态，如有异常立即停止
4. 调整参数时要小心，过高的增益可能导致不稳定

## 实验建议

1. 先在仿真环境中测试参数
2. 实际实验时从小角度开始
3. 逐渐增加旋转角度和速度
4. 记录关键参数用于优化
