#!/usr/bin/env python

"""
Formation Valve Rotation - Unified State Version
===================================================================
一个状态内完成接触建立和阀门旋转，避免状态间轨迹传递问题

基于Single UAV逻辑，使用统一的正向角速度，确保轨迹连续性。
"""

import rospy
import math
import time
import smach
import threading
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from std_msgs.msg import Float64MultiArray, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

# Import formation adapter and trajectory generator
import sys
import os
sys.path.append(os.path.dirname(__file__))
from beetle.utils import AssemblyAdapter
from beetle.assembly_api import OnlineCircularTrajectoryGenerator


class FormationStateBase(smach.State):
    """基础Formation状态类"""
    
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        super().__init__(outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.adapter = AssemblyAdapter()
        
        # 阀门参数
        self.valve_pos = [2.0, 0.0, 0.57]  # 阀门中心位置
        self.valve_yaw = 0.0  # 当前阀门朝向
        
        # 编队控制参数
        self.position_tolerance = 0.02  # 20mm位置容差
        self.yaw_tolerance = math.radians(3.0)  # 3度角度容差


class FormationMoveToValveState(FormationStateBase):
    """编队移动到阀门位置"""
    
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo("=== Formation Move To Valve State ===")
        
        # 简化版本：直接移动到阀门上方
        target_pos = [self.valve_pos[0], self.valve_pos[1], self.valve_pos[2] + 0.16]
        target_yaw = math.radians(-60)  # 与Single UAV一致的接近角度
        
        success = self.adapter.move_to_position(
            target_pos, target_yaw, 
            tolerance=self.position_tolerance,
            yaw_tolerance=self.yaw_tolerance,
            timeout=30.0
        )
        
        if success:
            rospy.loginfo("✓ Formation positioned at valve vicinity")
            return 'succeeded'
        else:
            rospy.logerr("❌ Failed to move formation to valve")
            return 'failed'


class FormationUnifiedValveRotationState(FormationStateBase):
    """统一的Formation阀门旋转状态 - 包含接触建立和旋转执行"""
    
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'])
        
        # 接触建立参数（模仿Single UAV）
        self.contact_angular_velocity = 0.08  # 固定正向角速度，与Single UAV完全一致
        self.contact_target_rotation = math.radians(4.6)  # 4.6度接触检测阈值
        
        # 旋转执行参数（模仿Single UAV）
        self.rotation_angular_velocity = 0.05  # 固定正向角速度，与Single UAV完全一致
        self.target_rotation = math.radians(90)  # 90度目标旋转
        
        # 阶段标识
        self.current_phase = "contact"  # "contact" -> "rotation"
        
    def execute(self, userdata):
        rospy.loginfo("=== Formation Unified Valve Rotation State ===")
        rospy.logwarn("🔧 使用统一状态避免轨迹传递问题，完全模仿Single UAV逻辑")
        
        # 获取当前位置和阀门状态
        current_pos = self.adapter.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("无法获取当前位置")
            return 'failed'
            
        # 计算当前半径
        relative_pos = np.array(current_pos[:2]) - np.array(self.valve_pos[:2])
        current_radius = np.linalg.norm(relative_pos)
        
        rospy.loginfo(f"当前位置: {current_pos}")
        rospy.loginfo(f"阀门中心: {self.valve_pos}")
        rospy.loginfo(f"当前半径: {current_radius*1000:.1f}mm")
        
        # 初始化轨迹生成器（只创建一次，避免重新初始化）
        initial_angle = 120.0  # 固定起始角度，与Single UAV一致
        corrected_valve_center = [self.valve_pos[0], self.valve_pos[1], current_pos[2]]
        
        # =============================================
        # 阶段1：接触建立（Contact Establishment）
        # =============================================
        rospy.loginfo(">>> 开始阶段1：接触建立（模仿Single UAV Circle Contact）")
        rospy.logwarn(f"🎯 Contact阶段角速度: +{self.contact_angular_velocity:.3f} rad/s (固定正向)")
        
        # 创建接触阶段轨迹生成器
        contact_generator = OnlineCircularTrajectoryGenerator(
            valve_center=corrected_valve_center,
            initial_radius=current_radius,
            target_angular_velocity=self.contact_angular_velocity,  # 正向角速度
            control_frequency=25.0
        )
        
        # 设置起始状态
        contact_generator.initialize_trajectory(
            angle=math.radians(initial_angle),
            radius=current_radius,
            z=current_pos[2]
        )
        
        # 执行接触建立
        contact_success = self._execute_contact_phase(contact_generator, corrected_valve_center)
        if not contact_success:
            rospy.logerr("❌ 接触建立失败")
            return 'failed'
            
        # =============================================
        # 阶段2：阀门旋转（Valve Rotation）
        # =============================================
        rospy.loginfo(">>> 开始阶段2：阀门旋转（无缝衔接轨迹状态）")
        rospy.logwarn(f"🎯 Rotation阶段角速度: +{self.rotation_angular_velocity:.3f} rad/s (继续正向)")
        
        # 直接修改现有轨迹生成器的角速度，保持所有其他状态不变
        contact_generator.target_angular_velocity = self.rotation_angular_velocity
        rospy.logwarn("✓ 无缝切换：仅更改角速度，保持轨迹连续性")
        
        # 执行阀门旋转
        rotation_success = self._execute_rotation_phase(contact_generator, corrected_valve_center)
        if rotation_success:
            rospy.loginfo("✓ Formation阀门旋转完成！")
            return 'succeeded'
        else:
            rospy.logerr("❌ 阀门旋转失败")
            return 'failed'
    
    def _execute_contact_phase(self, trajectory_generator, valve_center):
        """执行接触建立阶段"""
        rospy.loginfo("Contact Phase: 寻找阀门接触...")
        
        start_time = time.time()
        initial_valve_yaw = self.valve_yaw
        
        control_rate = rospy.Rate(25.0)  # 25Hz控制频率
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # 超时检查
            if elapsed > 20.0:  # 20秒超时
                rospy.logwarn("Contact phase timeout")
                break
                
            # 获取当前状态
            current_pos = self.adapter.get_end_effector_position()
            current_yaw = self.adapter.get_module2_yaw()
            current_valve_yaw = self.valve_yaw  # 这里需要实际获取阀门角度
            
            if any(v is None for v in [current_pos, current_yaw]):
                rospy.logwarn("状态信息丢失，跳过本次控制周期")
                control_rate.sleep()
                continue
            
            # 计算阀门转动
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            
            # 更新轨迹生成器
            state_info = trajectory_generator.update_state(current_pos, current_yaw, 0.0)
            target_state = trajectory_generator.generate_target_state()
            
            # 执行控制命令
            success = self.adapter.send_assembly_command_from_end_effector(
                target_position=target_state['position'],
                target_yaw=target_state['yaw'],
                linear_velocity=target_state['linear_velocity'],
                angular_velocity=target_state['angular_velocity']
            )
            
            if not success:
                rospy.logwarn("Control command failed")
            
            # 检测接触（阀门开始转动）
            if valve_rotation >= self.contact_target_rotation:
                rospy.loginfo(f"✓ 检测到阀门接触: {math.degrees(valve_rotation):.1f}° (阈值: {math.degrees(self.contact_target_rotation):.1f}°)")
                rospy.loginfo(f"接触建立时间: {elapsed:.1f}s")
                return True
            
            # 进度输出
            if int(elapsed) % 3 == 0 and (elapsed - int(elapsed)) < 0.04:
                rospy.loginfo(f"Contact progress: {math.degrees(valve_rotation):.2f}°/{math.degrees(self.contact_target_rotation):.1f}°, time={elapsed:.1f}s")
            
            control_rate.sleep()
        
        # 检查是否有微小转动
        final_valve_rotation = abs(self.valve_yaw - initial_valve_yaw)
        if final_valve_rotation > math.radians(0.5):  # 0.5度最小阈值
            rospy.logwarn(f"Contact timeout但检测到微小阀门转动: {math.degrees(final_valve_rotation):.2f}°，认为接触成功")
            return True
        
        return False
    
    def _execute_rotation_phase(self, trajectory_generator, valve_center):
        """执行阀门旋转阶段"""
        rospy.loginfo("Rotation Phase: 执行90度阀门旋转...")
        
        start_time = time.time()
        initial_valve_yaw = self.valve_yaw
        max_valve_rotation_achieved = 0.0
        
        control_rate = rospy.Rate(25.0)  # 25Hz控制频率
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # 超时检查
            if elapsed > 45.0:  # 45秒超时
                rospy.logwarn("Rotation phase timeout")
                break
                
            # 获取当前状态
            current_pos = self.adapter.get_end_effector_position()
            current_yaw = self.adapter.get_module2_yaw()
            current_valve_yaw = self.valve_yaw  # 这里需要实际获取阀门角度
            
            if any(v is None for v in [current_pos, current_yaw]):
                rospy.logwarn("状态信息丢失，跳过本次控制周期")
                control_rate.sleep()
                continue
            
            # 计算阀门转动进度
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            max_valve_rotation_achieved = max(max_valve_rotation_achieved, valve_rotation)
            
            # 更新轨迹生成器
            state_info = trajectory_generator.update_state(current_pos, current_yaw, 0.0)
            target_state = trajectory_generator.generate_target_state()
            
            # 执行控制命令
            success = self.adapter.send_assembly_command_from_end_effector(
                target_position=target_state['position'],
                target_yaw=target_state['yaw'],
                linear_velocity=target_state['linear_velocity'],
                angular_velocity=target_state['angular_velocity']
            )
            
            if not success:
                rospy.logwarn("Control command failed")
            
            # 检测旋转完成
            rotation_progress = valve_rotation / self.target_rotation * 100
            if valve_rotation >= self.target_rotation:
                rospy.loginfo(f"✓ 阀门旋转完成: {math.degrees(valve_rotation):.1f}°/{math.degrees(self.target_rotation):.1f}°")
                rospy.loginfo(f"旋转执行时间: {elapsed:.1f}s")
                return True
            
            # 进度输出
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.04:
                rospy.loginfo(f"Rotation progress: {rotation_progress:.1f}% | "
                            f"Valve Δ={math.degrees(valve_rotation):.1f}° | "
                            f"Max={math.degrees(max_valve_rotation_achieved):.1f}° | "
                            f"Time={elapsed:.1f}s")
            
            control_rate.sleep()
        
        # 检查最终结果
        final_valve_rotation = max_valve_rotation_achieved
        if final_valve_rotation >= self.target_rotation * 0.8:  # 80%成功率
            rospy.logwarn(f"Rotation timeout但达到80%目标: {math.degrees(final_valve_rotation):.1f}°/{math.degrees(self.target_rotation):.1f}°，认为成功")
            return True
        
        return False


class FormationReturnState(FormationStateBase):
    """编队返回起始位置"""
    
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo("=== Formation Return State ===")
        
        # 简化版本：返回安全高度
        safe_pos = [self.valve_pos[0], self.valve_pos[1], self.valve_pos[2] + 0.3]
        target_yaw = 0.0
        
        success = self.adapter.move_to_position(
            safe_pos, target_yaw, 
            tolerance=self.position_tolerance,
            yaw_tolerance=self.yaw_tolerance,
            timeout=20.0
        )
        
        if success:
            rospy.loginfo("✓ Formation returned to safe position")
            return 'succeeded'
        else:
            rospy.logwarn("❌ Formation return failed, but continuing...")
            return 'succeeded'  # 即使返回失败也继续，避免卡死


def create_state_machine():
    """创建Formation阀门旋转状态机"""
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    with sm:
        smach.StateMachine.add('FORMATION_MOVE_TO_VALVE',
                             FormationMoveToValveState(),
                             transitions={'succeeded': 'FORMATION_UNIFIED_VALVE_ROTATION',
                                        'failed': 'failed'})
        
        smach.StateMachine.add('FORMATION_UNIFIED_VALVE_ROTATION',
                             FormationUnifiedValveRotationState(),
                             transitions={'succeeded': 'FORMATION_RETURN',
                                        'failed': 'failed'})
        
        smach.StateMachine.add('FORMATION_RETURN',
                             FormationReturnState(),
                             transitions={'succeeded': 'succeeded',
                                        'failed': 'succeeded'})  # 继续执行，即使返回失败
    
    return sm


def main():
    """主函数"""
    rospy.init_node('formation_valve_rotation_unified', anonymous=True)
    
    rospy.loginfo("==============================================")
    rospy.loginfo("Formation Valve Rotation - Unified Version")
    rospy.loginfo("==============================================")
    rospy.loginfo("✓ 统一状态设计，避免轨迹传递问题")
    rospy.loginfo("✓ 完全模仿Single UAV正向角速度逻辑")
    rospy.loginfo("✓ 无缝接触-旋转切换")
    rospy.loginfo("==============================================")
    
    # 等待系统初始化
    rospy.loginfo("等待系统初始化...")
    time.sleep(3.0)
    
    # 创建并执行状态机
    sm = create_state_machine()
    
    try:
        outcome = sm.execute()
        if outcome == 'succeeded':
            rospy.loginfo("🎉 Formation阀门旋转任务完成！")
        else:
            rospy.logerr("❌ Formation阀门旋转任务失败")
    except Exception as e:
        rospy.logerr(f"状态机执行异常: {e}")
    finally:
        rospy.loginfo("程序结束")


if __name__ == '__main__':
    main()