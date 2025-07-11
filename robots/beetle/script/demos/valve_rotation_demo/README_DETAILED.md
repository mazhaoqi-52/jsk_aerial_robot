# 阀门操作状态机

这个状态机实现了双爪末端执行器的阀门抓取和旋转操作。

## 功能概述

状态机包含四个主要状态：
1. **初始化状态 (InitializationState)**: 获取无人机和阀门的位置信息
2. **对准抓取状态 (AlignToGraspState)**: 将末端执行器移动到合适的抓取位置
3. **旋转阀门状态 (RotateValveState)**: 绕阀门中心旋转一周
4. **返回起始位置状态 (ReturnHomeState)**: 返回起始位置

## 设计理念

### 抓取策略
- 阀门把手由三根互成120度的横梁组成
- 末端执行器的两个爪子分别插入相邻的两根横梁的不同侧
- 这样可以确保稳定的抓取和有效的力传递

### 轨迹规划
- 使用5阶多项式轨迹确保平滑运动
- 所有移动都考虑了安全的z轴偏移
- 旋转过程保持恒定的半径和高度

## 参数配置

### 主要参数
- `simulation`: 是否在仿真环境中运行 (默认: true)
- `module_ids`: 使用的无人机模块ID (默认: "2,3")
- `real_machine`: 是否使用真实硬件 (默认: false)

### 物理参数
- `valve_radius`: 阀门半径 (0.1225m)
- `beam_width`: 横梁宽度 (0.05m) 
- `z_offset`: z轴安全偏移 (0.21m)
- `rotation_duration`: 旋转一周的时间 (8.0s)

## 使用方法

### 仿真环境
```bash
roslaunch beetle valve_manipulation_state_machine.launch simulation:=true
```

### 真实环境
```bash
roslaunch beetle valve_manipulation_state_machine.launch simulation:=false real_machine:=true
```

### 自定义模块ID
```bash
roslaunch beetle valve_manipulation_state_machine.launch module_ids:="1,2"
```

## 话题接口

### 订阅话题
- `/beetle{id}/mocap/pose` (geometry_msgs/PoseStamped): 无人机位置
- `/valve/odom` (nav_msgs/Odometry): 阀门位置 (仿真)
- `/valve/mocap/pose` (geometry_msgs/PoseStamped): 阀门位置 (真实)

### 发布话题
- `/assembly/uav/nav` (aerial_robot_msgs/FlightNav): 导航命令

## 状态转换图

```
[START] -> INITIALIZATION -> ALIGN_TO_GRASP -> ROTATE_VALVE -> RETURN_HOME -> [TASK_COMPLETED]
             |                    |               |              |
             v                    v               v              v
        [TASK_FAILED]       [TASK_FAILED]   [TASK_FAILED]  [TASK_FAILED]
```

## 故障排除

### 常见问题
1. **初始化超时**: 检查话题名称和网络连接
2. **移动失败**: 确认导航系统正常工作
3. **位置不准确**: 调整PID参数或增加校正时间

### 调试工具
- 使用 `smach_viewer` 可视化状态机执行
- 检查 `/rosout` 获取详细日志信息
- 监控相关话题确认数据流

## 扩展和定制

### 添加新状态
1. 继承 `smach.State` 类
2. 实现 `execute` 方法
3. 在主函数中添加到状态机

### 调整轨迹参数
- 修改各状态类中的时间和速度参数
- 调整物理尺寸参数以匹配实际硬件

### 增加安全检查
- 在每个状态中添加位置验证
- 实现紧急停止机制
- 添加碰撞检测

## 依赖项

- ROS (Robot Operating System)
- smach (状态机库)
- numpy (数值计算)
- aerial_robot_msgs (无人机消息包)
- geometry_msgs, nav_msgs (几何和导航消息)

确保所有依赖项都已正确安装和配置。
