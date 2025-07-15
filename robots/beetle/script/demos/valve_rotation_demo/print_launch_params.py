#!/usr/bin/env python
"""
Launch参数信息打印脚本
"""
import rospy

def print_launch_params():
    """打印启动参数信息"""
    rospy.init_node('param_info_printer')
    
    print("\n" + "="*60)
    print("手动插入后自动阀门旋转程序 - 参数信息")
    print("="*60)
    
    # 基本参数
    print("基本参数:")
    print(f"  模块ID: {rospy.get_param('~module_id', 1)}")
    print(f"  旋转角度: {rospy.get_param('~rotation_angle', 1.5708):.3f}rad ({rospy.get_param('~rotation_angle', 1.5708)*180/3.14159:.1f}°)")
    print(f"  旋转时间: {rospy.get_param('~rotation_duration', 8.0):.1f}s")
    print(f"  上升高度: {rospy.get_param('~ascent_height', 0.5):.1f}m")
    
    # 阀门参数
    print("\n阀门参数:")
    valve_pos = rospy.get_param('~valve_pos', [3.0, 0.0, 0.55])
    print(f"  阀门位置: [{valve_pos[0]:.1f}, {valve_pos[1]:.1f}, {valve_pos[2]:.2f}]")
    print(f"  阀门偏航: {rospy.get_param('~valve_yaw', 0.0):.3f}rad")
    print(f"  Z偏移: {rospy.get_param('~z_offset', 0.55):.2f}m")
    
    # 对齐控制参数
    print("\n对齐控制参数:")
    print(f"  位置容差: {rospy.get_param('~alignment_position_tolerance', 0.01):.3f}m")
    print(f"  偏航容差: {rospy.get_param('~alignment_yaw_tolerance', 0.05):.3f}rad")
    print(f"  对齐超时: {rospy.get_param('~alignment_timeout', 15.0):.1f}s")
    print(f"  严格对齐: {rospy.get_param('~strict_alignment_enabled', True)}")
    print(f"  对齐纠正: {rospy.get_param('~alignment_correction_enabled', True)}")
    print(f"  控制增益: {rospy.get_param('~alignment_control_gain', 0.8):.2f}")
    print(f"  检查频率: {rospy.get_param('~alignment_check_frequency', 25)}Hz")
    
    # 末端执行器参数
    print("\n末端执行器参数:")
    end_effector_offset = rospy.get_param('~end_effector_offset', [0.0, 0.0, -0.246])
    print(f"  末端执行器偏移: [{end_effector_offset[0]:.3f}, {end_effector_offset[1]:.3f}, {end_effector_offset[2]:.3f}]")
    print(f"  末端执行器长度: {rospy.get_param('~end_effector_length', 0.246):.3f}m")
    
    # 旋转轨迹稳定性参数
    print("\n旋转轨迹稳定性参数:")
    print(f"  半径容差: {rospy.get_param('~rotation_radius_tolerance', 0.005):.3f}m")
    print(f"  高度容差: {rospy.get_param('~rotation_height_tolerance', 0.01):.3f}m")
    
    # 紧急检测参数
    print("\n紧急检测参数:")
    print(f"  卡住阈值: {rospy.get_param('~stuck_threshold', 25.0):.1f}s")
    print(f"  移动阈值: {rospy.get_param('~movement_threshold', 0.02):.3f}m")
    print(f"  偏航阈值: {rospy.get_param('~yaw_threshold', 0.1):.3f}rad")
    
    # 手柄控制参数
    print("\n手柄控制参数:")
    print(f"  死区: {rospy.get_param('~joy_deadzone', 0.1):.2f}")
    
    print("\n" + "="*60)
    print("反馈控制功能 (继承自源程序):")
    print("✅ COG-末端执行器-阀门对齐控制")
    print("✅ 实时对齐检查和纠正")
    print("✅ 紧急检测和处理")
    print("✅ 旋转过程中的轨迹稳定性保证")
    print("✅ 预旋转对齐确保最佳起始位置")
    print("="*60)
    print("操作说明:")
    print("1. 手动将manipulator插入阀门")
    print("2. 接近阀门位置后程序会提示可以启动")
    print("3. 按下手柄Start按钮(按钮7)启动自动序列")
    print("4. 程序将自动执行: 对齐 -> 旋转 -> 上升")
    print("5. 紧急停止: 按下手柄Select按钮(按钮6)")
    print("="*60)
    
    # 延时后退出
    rospy.sleep(5.0)

if __name__ == '__main__':
    try:
        print_launch_params()
    except rospy.ROSInterruptException:
        pass
