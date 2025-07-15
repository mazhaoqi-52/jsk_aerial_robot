#!/usr/bin/env python
"""
使用示例：模块化反馈控制系统
演示如何使用新的EnhancedMotionController进行对齐控制
"""
import rospy
import math
from enhanced_motion_controller import EnhancedMotionController
from trajectory import ValveRotationTrajectory

def example_standalone_alignment_control():
    """独立使用对齐控制的示例"""
    
    # 假设你有一个状态机对象
    class MockStateMachine:
        def __init__(self):
            self.current_yaw = 0.0
            self.valve_pos = (3.0, 0.0, 0.55)
            
        def get_current_position(self):
            return (2.8, 0.1, 0.8)  # 模拟当前位置
        
        def normalize_angle(self, angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle
    
    # 创建模拟状态机
    mock_sm = MockStateMachine()
    
    # 创建增强Motion Controller
    motion_controller = EnhancedMotionController(mock_sm)
    
    # 示例1: 检查对齐状态
    current_pos = mock_sm.get_current_position()
    is_aligned, alignment_report = motion_controller.check_alignment_status(
        current_pos, mock_sm.current_yaw, mock_sm.valve_pos)
    
    print("=== 对齐状态检查 ===")
    print(f"对齐状态: {'已对齐' if is_aligned else '未对齐'}")
    print(f"末端执行器到阀门距离: {alignment_report['end_effector_to_valve_distance']:.3f}m")
    print(f"偏航角误差: {alignment_report['yaw_error']:.3f}rad")
    print(f"共线性角度: {alignment_report['collinearity_angle']:.3f}rad")
    
    # 示例2: 计算对齐纠正
    if not is_aligned:
        pos_correction, yaw_correction = motion_controller.calculate_alignment_correction(alignment_report)
        print("\n=== 对齐纠正计算 ===")
        print(f"位置纠正: [{pos_correction[0]:.3f}, {pos_correction[1]:.3f}, {pos_correction[2]:.3f}]")
        print(f"偏航角纠正: {yaw_correction:.3f}rad")
    
    # 示例3: 执行预旋转对齐
    print("\n=== 预旋转对齐 ===")
    alignment_success = motion_controller.execute_pre_rotation_alignment(
        mock_sm.valve_pos, timeout=5.0)
    print(f"预旋转对齐结果: {'成功' if alignment_success else '超时'}")

def example_integrated_rotation_control():
    """集成旋转控制的示例"""
    
    # 这个示例展示如何在实际状态机中使用
    class MockRotationState:
        def __init__(self):
            self.valve_pos = (3.0, 0.0, 0.55)
            self.current_yaw = 0.0
            self.rotation_angle = math.pi / 2  # 90度
            self.rotation_duration = 8.0
            
        def get_current_position(self):
            return (2.877, 0.123, 0.550)  # 模拟对齐后的位置
            
        def normalize_angle(self, angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle
    
    # 创建模拟状态
    mock_state = MockRotationState()
    
    # 创建增强Motion Controller
    motion_controller = EnhancedMotionController(mock_state)
    
    # 创建旋转轨迹
    rotation_traj = ValveRotationTrajectory(
        rotation_duration=mock_state.rotation_duration,
        rotation_angle=mock_state.rotation_angle,
        valve_center=mock_state.valve_pos,
        valve_radius=0.1225,
        valve_yaw=0.0
    )
    
    # 设置起始位置
    current_pos = mock_state.get_current_position()
    rotation_traj.set_start_position(current_pos, mock_state.current_yaw)
    
    # 执行对齐控制的旋转
    print("=== 对齐控制旋转 ===")
    rotation_stats = motion_controller.execute_alignment_controlled_rotation(
        rotation_traj, mock_state.valve_pos)
    
    # 打印统计信息
    print(f"执行时间: {rotation_stats.get('execution_time', 0):.1f}s")
    print(f"对齐违规次数: {rotation_stats.get('alignment_violations', 0)}")
    print(f"应用纠正次数: {rotation_stats.get('corrections_applied', 0)}")
    print(f"最大位置误差: {rotation_stats.get('max_position_error', 0):.3f}m")
    print(f"最大偏航角误差: {rotation_stats.get('max_yaw_error', 0):.3f}rad")

def example_parameter_configuration():
    """参数配置示例"""
    
    # 展示如何通过ROS参数配置对齐控制
    config_examples = {
        "保守配置": {
            "strict_alignment_enabled": True,
            "alignment_correction_enabled": True,
            "alignment_position_tolerance": 0.015,  # 1.5cm
            "alignment_yaw_tolerance": 0.08,        # ~4.6度
            "alignment_control_gain": 0.5,          # 保守增益
            "alignment_check_frequency": 20         # 20Hz
        },
        
        "激进配置": {
            "strict_alignment_enabled": True,
            "alignment_correction_enabled": True,
            "alignment_position_tolerance": 0.008,  # 0.8cm
            "alignment_yaw_tolerance": 0.03,        # ~1.7度
            "alignment_control_gain": 0.9,          # 激进增益
            "alignment_check_frequency": 30         # 30Hz
        },
        
        "调试配置": {
            "strict_alignment_enabled": True,
            "alignment_correction_enabled": False,  # 只监控不纠正
            "alignment_position_tolerance": 0.02,   # 宽容度
            "alignment_yaw_tolerance": 0.1,         # 宽容度
            "alignment_control_gain": 0.3,          # 低增益
            "alignment_check_frequency": 10         # 10Hz
        }
    }
    
    print("=== 参数配置示例 ===")
    for config_name, config in config_examples.items():
        print(f"\n{config_name}:")
        for param, value in config.items():
            print(f"  {param}: {value}")
    
    # 展示launch文件中的参数设置
    print("\n=== Launch文件参数示例 ===")
    launch_example = """
<launch>
  <node pkg="beetle" type="valve_rotation_fang_single.py" name="valve_rotation">
    <!-- 对齐控制参数 -->
    <param name="strict_alignment_enabled" value="true" />
    <param name="alignment_correction_enabled" value="true" />
    <param name="alignment_position_tolerance" value="0.01" />
    <param name="alignment_yaw_tolerance" value="0.05" />
    <param name="alignment_control_gain" value="0.8" />
    <param name="alignment_check_frequency" value="25" />
    
    <!-- 末端执行器参数 -->
    <rosparam param="end_effector_offset">[0.0, 0.0, -0.246]</rosparam>
  </node>
</launch>
"""
    print(launch_example)

if __name__ == "__main__":
    print("=== 模块化反馈控制系统使用示例 ===")
    
    try:
        # 初始化ROS（如果在ROS环境中）
        rospy.init_node('alignment_control_example', anonymous=True)
        
        print("\n1. 独立对齐控制示例")
        example_standalone_alignment_control()
        
        print("\n2. 集成旋转控制示例")
        example_integrated_rotation_control()
        
        print("\n3. 参数配置示例")
        example_parameter_configuration()
        
        print("\n=== 示例完成 ===")
        print("新的模块化架构提供了:")
        print("✅ 清晰的代码组织")
        print("✅ 易于使用的接口")
        print("✅ 灵活的参数配置")
        print("✅ 强大的对齐控制功能")
        
    except rospy.ROSInterruptException:
        print("ROS中断")
    except Exception as e:
        print(f"示例执行错误: {e}")
        # 在非ROS环境中仍然可以运行部分示例
        print("在非ROS环境中运行部分示例...")
        example_parameter_configuration()
