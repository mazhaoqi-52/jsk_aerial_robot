#!/usr/bin/env python3
"""
Joy-controlled valve rotation using existing state machine
Simplified version that directly imports and reuses original components
"""

import rospy
import smach
import smach_ros
import threading
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Import the existing state machine components
from valve_rotation_fang_single import (
    InitializeStartPositionState,
    MoveToValveState,
    DescendAndContactState,
    RotateValveState
)

# Import the unified motion controller
from unified_motion_controller import UnifiedMotionController

class EnhancedRotateValveState(RotateValveState):
    """
    Enhanced rotate valve state that uses constant distance feedback controller
    """
    
    def __init__(self, module_id):
        super().__init__(module_id)
        self.feedback_controller = None
        
    def execute(self, userdata):
        """
        Execute valve rotation with constant distance feedback control
        """
        rospy.loginfo("Starting enhanced valve rotation (constant distance feedback control)")
        
        # Initialize unified motion controller if not already done
        if self.feedback_controller is None:
            self.feedback_controller = UnifiedMotionController(self.sm)
        
        # Execute the original rotation logic but with feedback control
        return self.execute_with_feedback_control(userdata)
    
    def execute_with_feedback_control(self, userdata):
        """
        Execute valve rotation with feedback control
        """
        try:
            # Get valve position and rotation parameters
            valve_pos = self.sm.get_current_position()
            if valve_pos is None:
                rospy.logerr("无法获取当前位置")
                return 'failed'
            
            valve_center = (valve_pos[0], valve_pos[1], valve_pos[2])
            
            # Calculate rotation parameters
            rotation_angle = rospy.get_param('~valve_rotation_angle', 90.0)
            rotation_steps = rospy.get_param('~valve_rotation_steps', 36)
            end_effector_distance = rospy.get_param('~end_effector_distance', 0.15)
            
            # Create constant distance trajectory
            from trajectory import create_constant_distance_trajectory
            trajectory = create_constant_distance_trajectory(
                valve_center=valve_center,
                rotation_angle=rotation_angle,
                rotation_steps=rotation_steps,
                end_effector_distance=end_effector_distance
            )
            
            # Execute rotation with feedback control
            feedback_frequency = rospy.get_param('~feedback_frequency', 50)
            alignment_check_interval = rospy.get_param('~alignment_check_interval', 0.1)
            
            rospy.loginfo(f"Executing feedback control rotation: angle={rotation_angle}°, steps={rotation_steps}, distance={end_effector_distance}m")
            
            stats = self.feedback_controller.execute_constant_distance_rotation(
                trajectory=trajectory,
                valve_center=valve_center,
                feedback_frequency=feedback_frequency,
                alignment_check_interval=alignment_check_interval
            )
            
            # Check execution success based on statistics
            if stats['max_distance_error'] < self.feedback_controller.distance_tolerance * 3:
                rospy.loginfo("增强型阀门旋转成功完成")
                return 'succeeded'
            else:
                rospy.logwarn("增强型阀门旋转完成但精度不足")
                return 'succeeded'  # Still consider as success but with warning
                
        except Exception as e:
            rospy.logerr(f"增强型阀门旋转失败: {e}")
            return 'failed'

class JoyControlledValveRotation:
    """
    Simplified joy-controlled valve rotation that reuses existing state machine
    """
    
    def __init__(self):
        rospy.init_node('joy_controlled_valve_rotation')
        
        # Get parameters
        self.module_id = rospy.get_param('~module_id', 1)
        self.auto_start = rospy.get_param('~auto_start', False)
        
        # Joy control state
        self.joy_enabled = False
        self.manual_insertion_active = False
        self.manual_insertion_complete = False
        self.auto_sequence_started = False
        
        # Joy subscriber
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        
        # Status publisher
        self.status_pub = rospy.Publisher('valve_rotation_status', Bool, queue_size=1)
        
        # Create the existing state machine
        self.create_state_machine()
        
        rospy.loginfo("Joy-controlled valve rotation initialized")
        rospy.loginfo("Press X button to enable joy control")
        rospy.loginfo("Press Square button to start manual insertion")
        rospy.loginfo("Press Triangle button to start automatic sequence")
        
    def create_state_machine(self):
        """Create the state machine using existing states"""
        # Create state machine container
        self.sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        
        # Initialize userdata
        self.sm.userdata.start_position = None
        
        # Add states from existing implementation
        with self.sm:
            smach.StateMachine.add(
                'INITIALIZE_START_POSITION',
                InitializeStartPositionState(module_id=self.module_id),
                transitions={
                    'succeeded': 'MOVE_TO_VALVE',
                    'failed': 'failed'
                }
            )
            
            smach.StateMachine.add(
                'MOVE_TO_VALVE',
                MoveToValveState(module_id=self.module_id),
                transitions={
                    'succeeded': 'DESCEND_AND_CONTACT',
                    'failed': 'failed'
                }
            )
            
            smach.StateMachine.add(
                'DESCEND_AND_CONTACT',
                DescendAndContactState(module_id=self.module_id),
                transitions={
                    'succeeded': 'ROTATE_VALVE',
                    'failed': 'failed'
                }
            )
            
            smach.StateMachine.add(
                'ROTATE_VALVE',
                EnhancedRotateValveState(module_id=self.module_id),
                transitions={
                    'succeeded': 'succeeded',
                    'failed': 'failed',
                    'emergency': 'failed'
                }
            )
    
    def joy_callback(self, msg):
        """Handle joystick input"""
        if not msg.buttons:
            return
            
        # X button (button 0) - Enable/disable joy control
        if msg.buttons[0] == 1:  # X button pressed
            self.joy_enabled = not self.joy_enabled
            rospy.loginfo(f"Joy control {'enabled' if self.joy_enabled else 'disabled'}")
        
        if not self.joy_enabled:
            return
            
        # Square button (button 2) - Start manual insertion
        if msg.buttons[2] == 1:  # Square button pressed
            if not self.manual_insertion_active and not self.auto_sequence_started:
                self.start_manual_insertion()
        
        # Triangle button (button 3) - Start automatic sequence
        if msg.buttons[3] == 1:  # Triangle button pressed
            if not self.auto_sequence_started:
                self.start_automatic_sequence()
    
    def start_manual_insertion(self):
        """Start manual insertion phase"""
        rospy.loginfo("Starting manual insertion phase...")
        self.manual_insertion_active = True
        
        # Execute initialization and move to valve
        def manual_insertion_thread():
            try:
                # Create manual insertion state machine
                manual_sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
                manual_sm.userdata.start_position = None
                
                with manual_sm:
                    smach.StateMachine.add(
                        'INITIALIZE',
                        InitializeStartPositionState(module_id=self.module_id),
                        transitions={
                            'succeeded': 'MOVE_TO_VALVE',
                            'failed': 'failed'
                        }
                    )
                    
                    smach.StateMachine.add(
                        'MOVE_TO_VALVE',
                        MoveToValveState(module_id=self.module_id),
                        transitions={
                            'succeeded': 'succeeded',
                            'failed': 'failed'
                        }
                    )
                
                # Execute manual insertion
                outcome = manual_sm.execute()
                
                if outcome == 'succeeded':
                    rospy.loginfo("Manual insertion completed successfully")
                    rospy.loginfo("Press Triangle button to start automatic sequence")
                    self.manual_insertion_complete = True
                else:
                    rospy.logerr("Manual insertion failed")
                    
            except Exception as e:
                rospy.logerr(f"Manual insertion error: {e}")
            finally:
                self.manual_insertion_active = False
        
        # Start manual insertion in separate thread
        thread = threading.Thread(target=manual_insertion_thread)
        thread.daemon = True
        thread.start()
    
    def start_automatic_sequence(self):
        """Start automatic sequence (descent, rotation)"""
        rospy.loginfo("Starting automatic sequence...")
        self.auto_sequence_started = True
        
        def automatic_sequence_thread():
            try:
                # Create automatic sequence state machine
                auto_sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
                
                # Use existing start position if available
                if hasattr(self.sm.userdata, 'start_position') and self.sm.userdata.start_position:
                    auto_sm.userdata.start_position = self.sm.userdata.start_position
                else:
                    auto_sm.userdata.start_position = None
                
                with auto_sm:
                    smach.StateMachine.add(
                        'DESCEND_AND_CONTACT',
                        DescendAndContactState(module_id=self.module_id),
                        transitions={
                            'succeeded': 'ROTATE_VALVE',
                            'failed': 'failed'
                        }
                    )
                    
                    smach.StateMachine.add(
                        'ROTATE_VALVE',
                        EnhancedRotateValveState(module_id=self.module_id),
                        transitions={
                            'succeeded': 'succeeded',
                            'failed': 'failed',
                            'emergency': 'failed'
                        }
                    )
                
                # Execute automatic sequence
                outcome = auto_sm.execute()
                
                if outcome == 'succeeded':
                    rospy.loginfo("Automatic sequence completed successfully")
                    self.status_pub.publish(Bool(data=True))
                else:
                    rospy.logerr("Automatic sequence failed")
                    self.status_pub.publish(Bool(data=False))
                    
            except Exception as e:
                rospy.logerr(f"Automatic sequence error: {e}")
                self.status_pub.publish(Bool(data=False))
            finally:
                self.auto_sequence_started = False
        
        # Start automatic sequence in separate thread
        thread = threading.Thread(target=automatic_sequence_thread)
        thread.daemon = True
        thread.start()
    
    def run(self):
        """Main execution loop"""
        if self.auto_start:
            rospy.loginfo("Auto-start enabled - starting full sequence")
            # Execute full state machine
            outcome = self.sm.execute()
            rospy.loginfo(f"State machine completed with outcome: {outcome}")
            return outcome == 'succeeded'
        else:
            rospy.loginfo("Joy control mode - waiting for user input")
            rospy.loginfo("Button mapping:")
            rospy.loginfo("  X (button 0): Toggle joy control")
            rospy.loginfo("  Square (button 2): Start manual insertion")
            rospy.loginfo("  Triangle (button 3): Start automatic sequence")
            
            # Keep node running
            rospy.spin()
            return True

def main():
    try:
        controller = JoyControlledValveRotation()
        success = controller.run()
        
        if success:
            rospy.loginfo("Joy-controlled valve rotation completed successfully")
        else:
            rospy.logerr("Joy-controlled valve rotation failed")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Joy-controlled valve rotation interrupted")
    except Exception as e:
        rospy.logerr(f"Joy-controlled valve rotation error: {e}")

if __name__ == '__main__':
    main()
