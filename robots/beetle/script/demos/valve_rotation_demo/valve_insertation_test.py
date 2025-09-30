#!/usr/bin/env python3
"""
Valve Insertion Test - 专注于插入部分，去除阀门旋转功能
基于 valve_rotation_fang_single.py 的插入相关功能
"""

import sys
import os

# Add current directory FIRST to ensure local imports work correctly
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import smach
import smach_ros
import time
import math
import threading

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Import from valve_rotation_fang_single for reusability
from valve_rotation_fang_single import (
    SingleUAVStateBase,
    InitializeStartPositionState,
    MoveToValveState,
    DescendAndContactState,
    ReturnToStartState
)


class ValveInsertionState(SingleUAVStateBase):
    """
    专门的阀门插入状态 - Execute插入后immediate返回
    基于 DescendAndContactState 但去除阀门旋转功能
    """
    def __init__(self, module_id=1, rotation_direction=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'],
                        output_keys=['insertion_position'],
                        module_id=module_id)
        
        # 导入必要的插入组件
        from insertion_optimizer import InsertionOptimizer
        
        # 初始化插入优化器
        self.optimizer = InsertionOptimizer(
            valve_radius=0.10,
            valve_beam_width=0.035,
            safety_margin=0.002,
            module_id=module_id,
            simulation=rospy.get_param("~simulation", True)
        )
        
        self.rotation_direction = rotation_direction
        self.pre_insertion_distance = 0.035
        self.circumferential_offset = 0.005
        
        rospy.loginfo("Valve insertion state initialized - insertion only, no rotation")
    
    def execute(self, userdata):
        """
        Execute插入测试流程：
        1. 等待位置信息
        2. 计算插入策略
        3. 移动到插入点
        4. Execute插入
        5. 等待确认
        6. immediate返回
        """
        rospy.loginfo("=== VALVE INSERTION TEST - NO ROTATION ===")
        
        # Step 1: 等待必要的位置信息
        if not self.wait_for_positions():
            rospy.logerr("Failed to get required position data")
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return 'failed'
        
        # Step 2: 计算安全插入高度
        valve_z = self.valve_pos[2]
        required_uav_z = self.optimizer.calculate_insertion_uav_z(valve_z)
        
        rospy.loginfo(f"=== INSERTION HEIGHT CALCULATION ===")
        rospy.loginfo(f"Valve height: {valve_z:.6f}m")
        rospy.loginfo(f"Required UAV Z for insertion: {required_uav_z:.6f}m")
        rospy.loginfo(f"Safety margin applied: 40mm above valve")
        
        # Step 3: 计算插入策略
        insertion_strategy = self.calculate_insertion_strategy()
        if insertion_strategy is None:
            rospy.logerr("Failed to calculate insertion strategy")
            return 'failed'
        
        # Step 4: Execute插入移动
        insertion_success = self.execute_insertion_movement(insertion_strategy, required_uav_z)
        if not insertion_success:
            rospy.logerr("Failed to execute insertion movement")
            return 'failed'
        
        # Step 5: verify插入位置
        insertion_verification = self.verify_insertion_position()
        if not insertion_verification:
            rospy.logwarn("Insertion position verification failed, but continuing...")
        
        # Step 6: 保持插入位置一段时间进行观察
        rospy.loginfo("=== HOLDING INSERTION POSITION FOR OBSERVATION ===")
        rospy.loginfo("Insertion completed - holding position for 3 seconds...")
        time.sleep(3.0)
        
        # 保存插入位置用于返回状态
        userdata.insertion_position = self.get_current_position()
        
        rospy.loginfo("=== VALVE INSERTION TEST COMPLETED ===")
        rospy.loginfo("Ready to return to start position")
        
        return 'succeeded'
    
    def calculate_insertion_strategy(self):
        """计算插入策略"""
        rospy.loginfo("--- Calculating insertion strategy ---")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return None
        
        # 设置旋转方向
        self.optimizer.set_rotation_direction(self.rotation_direction)
        
        # 使用优化器计算双齿轮插入策略
        strategy = self.optimizer.calculate_dual_fang_clockwise_strategy(
            uav_pos=current_pos,
            valve_pos=self.valve_pos,
            valve_yaw=self.valve_yaw
        )
        
        if strategy is None:
            rospy.logerr("Failed to calculate dual-fang strategy")
            return None
        
        self.current_insertion_params = strategy
        
        strategy_type = strategy.get('strategy', 'unknown')
        rospy.loginfo(f"Insertion strategy: {strategy_type}")
        
        return strategy
    
    def execute_insertion_movement(self, strategy, target_z):
        """Execute插入移动"""
        rospy.loginfo("--- Executing insertion movement ---")
        
        # 根据策略类型Execute不同的插入逻辑
        if strategy.get('strategy') == 'two_phase_safe_insertion':
            return self.execute_two_phase_insertion(strategy, target_z)
        else:
            return self.execute_direct_insertion(strategy, target_z)
    
    def execute_two_phase_insertion(self, strategy, target_z):
        """Execute两阶段插入"""
        rospy.loginfo("Executing two-phase insertion strategy")
        
        # Phase 1: 移动到插入预备位置
        phase1_left = strategy['phase1_left_final']
        phase1_right = strategy['phase1_right_final']
        
        # 计算UAV中心位置
        dual_fang_center = (
            (phase1_left[0] + phase1_right[0]) / 2,
            (phase1_left[1] + phase1_right[1]) / 2,
            target_z
        )
        
        # 计算UAV朝向
        claw_vector_x = phase1_right[0] - phase1_left[0]
        claw_vector_y = phase1_right[1] - phase1_left[1]
        target_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
        target_yaw = self.normalize_angle(target_yaw)
        
        rospy.loginfo(f"Phase 1 target: {dual_fang_center}")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Execute移动到Phase 1位置
        success = self.motion_controller.execute_smooth_trajectory_with_yaw(
            self.get_current_position(),
            dual_fang_center,
            target_yaw,
            duration=8.0,
            pos_threshold=0.03,
            yaw_threshold=0.05
        )
        
        if not success:
            rospy.logerr("Failed to reach Phase 1 insertion position")
            return False
        
        time.sleep(1.0)  # 稳定时间
        
        # Phase 2: 精确插入
        phase2_left = strategy['phase2_left_final']
        phase2_right = strategy['phase2_right_final']
        
        dual_fang_center_phase2 = (
            (phase2_left[0] + phase2_right[0]) / 2,
            (phase2_left[1] + phase2_right[1]) / 2,
            target_z
        )
        
        rospy.loginfo(f"Phase 2 target: {dual_fang_center_phase2}")
        
        # Execute精确插入移动
        success = self.motion_controller.execute_smooth_trajectory_with_yaw(
            self.get_current_position(),
            dual_fang_center_phase2,
            target_yaw,
            duration=6.0,
            pos_threshold=0.02,
            yaw_threshold=0.03
        )
        
        if not success:
            rospy.logerr("Failed to reach Phase 2 insertion position")
            return False
        
        rospy.loginfo("Two-phase insertion completed successfully")
        return True
    
    def execute_direct_insertion(self, strategy, target_z):
        """Execute直接插入"""
        rospy.loginfo("Executing direct insertion strategy")
        
        dual_fang_center = strategy.get('dual_fang_center')
        if dual_fang_center is None:
            rospy.logerr("Strategy missing dual_fang_center")
            return False
        
        target_pos = (dual_fang_center[0], dual_fang_center[1], target_z)
        target_yaw = strategy.get('optimal_uav_yaw', 0.0)
        
        rospy.loginfo(f"Direct insertion target: {target_pos}")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Execute直接插入移动
        success = self.motion_controller.execute_smooth_trajectory_with_yaw(
            self.get_current_position(),
            target_pos,
            target_yaw,
            duration=10.0,
            pos_threshold=0.03,
            yaw_threshold=0.05
        )
        
        if not success:
            rospy.logerr("Failed to reach direct insertion position")
            return False
        
        rospy.loginfo("Direct insertion completed successfully")
        return True
    
    def verify_insertion_position(self):
        """verify插入位置精度"""
        rospy.loginfo("--- Verifying insertion position ---")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # 计算距离阀门中心的距离
        valve_center = (self.valve_pos[0], self.valve_pos[1])
        uav_center = (current_pos[0], current_pos[1])
        
        distance_to_valve = math.sqrt(
            (uav_center[0] - valve_center[0])**2 + 
            (uav_center[1] - valve_center[1])**2
        )
        
        height_difference = abs(current_pos[2] - (self.valve_pos[2] - self.optimizer.end_effector_offset_z))
        
        rospy.loginfo(f"Distance to valve center: {distance_to_valve*1000:.1f}mm")
        rospy.loginfo(f"Height difference from target: {height_difference*1000:.1f}mm")
        
        # verify标准(相对宽松用于测试)
        if distance_to_valve < 0.15 and height_difference < 0.05:  # 150mm xy, 50mm z
            rospy.loginfo("Insertion position verification passed")
            return True
        else:
            rospy.logwarn(f"WARNING: Insertion position verification failed")
            rospy.logwarn(f"  XY error: {distance_to_valve*1000:.1f}mm (limit: 150mm)")
            rospy.logwarn(f"  Z error: {height_difference*1000:.1f}mm (limit: 50mm)")
            return False


def create_insertion_test_state_machine(module_id=1):
    """创建插入测试状态机 - 去除阀门旋转部分"""
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    with sm:
        # 初始化起始位置
        smach.StateMachine.add('INITIALIZE_START_POSITION',
                             InitializeStartPositionState(module_id=module_id),
                             transitions={'succeeded': 'MOVE_TO_VALVE',
                                        'failed': 'failed'},
                             remapping={'start_position': 'start_position'})
        
        # 移动到阀门附近
        smach.StateMachine.add('MOVE_TO_VALVE',
                             MoveToValveState(module_id=module_id),
                             transitions={'succeeded': 'VALVE_INSERTION',
                                        'failed': 'failed'})
        
        # Execute插入测试(新状态，替代DescendAndContactState + RotateValveState)
        smach.StateMachine.add('VALVE_INSERTION',
                             ValveInsertionState(module_id=module_id),
                             transitions={'succeeded': 'RETURN_TO_START',
                                        'failed': 'failed'},
                             remapping={'insertion_position': 'insertion_position'})
        
        # 返回起始位置
        smach.StateMachine.add('RETURN_TO_START',
                             ReturnToStartState(module_id=module_id),
                             transitions={'succeeded': 'succeeded',
                                        'failed': 'failed'},
                             remapping={'start_position': 'start_position'})
    
    return sm


def main():
    """主函数 - 运行插入测试"""
    rospy.init_node('valve_insertion_test')
    
    try:
        # 获取模块ID参数
        module_id = rospy.get_param('~module_id', 1)
        rospy.loginfo(f"Starting valve insertion test for module {module_id}")
        
        # 创建并Execute状态机
        sm = create_insertion_test_state_machine(module_id=module_id)
        
        # 创建SMACH查看器(可选)
        sis = smach_ros.IntrospectionServer('valve_insertion_test', sm, '/SM_ROOT')
        sis.start()
        
        # Execute状态机
        rospy.loginfo("=== STARTING VALVE INSERTION TEST ===")
        rospy.loginfo("This test will:")
        rospy.loginfo("1. Move to valve vicinity")
        rospy.loginfo("2. Execute insertion sequence")
        rospy.loginfo("3. Hold position for observation")
        rospy.loginfo("4. Return to start position")
        rospy.loginfo("NO valve rotation will be performed")
        
        outcome = sm.execute()
        
        rospy.loginfo(f"=== VALVE INSERTION TEST COMPLETED ===")
        rospy.loginfo(f"Final outcome: {outcome}")
        
        # 停止内省服务器
        sis.stop()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
