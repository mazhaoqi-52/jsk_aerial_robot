#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Unified Control and Feedforward Control Usage Example for Beetle Formation

This example demonstrates how to use the new unified control and feedforward
control features for valve rotation tasks with assembled beetle formation.
"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

class BeetleUnifiedControlExample:
    def __init__(self):
        rospy.init_node('beetle_unified_control_example')
        
        # Publishers for control commands
        self.feedforward_pub = rospy.Publisher(
            '/beetle1/controller/feedforward_wrench', 
            WrenchStamped, 
            queue_size=1
        )
        
        # Subscribers for monitoring
        self.formation_wrench_sub = rospy.Subscriber(
            '/beetle1/controller/formation_wrench_debug',
            WrenchStamped,
            self.formation_wrench_callback
        )
        
        self.valve_rotation_active = False
        
    def enable_unified_control(self):
        """Enable unified 4n-rotor control mode"""
        rospy.set_param("controller/unified_control_mode", True)
        rospy.loginfo("Unified control mode enabled")
        
    def disable_unified_control(self):
        """Disable unified control and revert to leader-follower mode"""
        rospy.set_param("controller/unified_control_mode", False)
        rospy.loginfo("Unified control mode disabled, using leader-follower mode")
        
    def set_valve_rotation_feedforward(self, valve_torque=2.0, valve_force_z=-5.0):
        """
        Set feedforward compensation for valve rotation
        
        Args:
            valve_torque: Expected torque resistance from valve (Nm)
            valve_force_z: Expected vertical force from valve (N)
        """
        # Create feedforward wrench message
        ff_wrench = WrenchStamped()
        ff_wrench.header.stamp = rospy.Time.now()
        ff_wrench.header.frame_id = "cog"
        
        # Set expected external forces/torques
        ff_wrench.wrench.force.x = 0.0
        ff_wrench.wrench.force.y = 0.0
        ff_wrench.wrench.force.z = valve_force_z  # Valve resistance force
        ff_wrench.wrench.torque.x = 0.0
        ff_wrench.wrench.torque.y = 0.0
        ff_wrench.wrench.torque.z = valve_torque  # Valve rotation resistance
        
        # Enable feedforward and publish
        rospy.set_param("controller/valve_rotation_feedforward/enabled", True)
        self.feedforward_pub.publish(ff_wrench)
        
        rospy.loginfo(f"Valve rotation feedforward set: torque={valve_torque} Nm, force_z={valve_force_z} N")
        
    def disable_feedforward(self):
        """Disable feedforward compensation"""
        rospy.set_param("controller/valve_rotation_feedforward/enabled", False)
        rospy.loginfo("Feedforward compensation disabled")
        
    def formation_wrench_callback(self, msg):
        """Monitor formation control wrench"""
        rospy.loginfo_throttle(1.0, 
            f"Formation wrench - Force: [{msg.wrench.force.x:.2f}, {msg.wrench.force.y:.2f}, {msg.wrench.force.z:.2f}] N, "
            f"Torque: [{msg.wrench.torque.x:.2f}, {msg.wrench.torque.y:.2f}, {msg.wrench.torque.z:.2f}] Nm"
        )
        
    def execute_valve_rotation_task(self):
        """
        Execute a complete valve rotation task with unified control and feedforward
        """
        rospy.loginfo("Starting valve rotation task with unified control...")
        
        try:
            # Step 1: Enable unified control mode
            self.enable_unified_control()
            rospy.sleep(2.0)  # Wait for mode switch
            
            # Step 2: Set feedforward for valve interaction
            self.set_valve_rotation_feedforward(valve_torque=2.5, valve_force_z=-6.0)
            rospy.sleep(1.0)
            
            # Step 3: Simulate valve rotation process
            rospy.loginfo("Simulating valve rotation with feedforward compensation...")
            
            # Gradually increase expected valve resistance as rotation progresses
            for progress in np.linspace(0, 1, 20):
                if rospy.is_shutdown():
                    break
                    
                # Adjust feedforward based on rotation progress
                current_torque = 2.5 + progress * 1.5  # Increasing resistance
                current_force_z = -6.0 - progress * 2.0  # Increasing downward force
                
                self.set_valve_rotation_feedforward(
                    valve_torque=current_torque,
                    valve_force_z=current_force_z
                )
                
                rospy.sleep(0.5)  # 10 seconds total rotation time
                
            rospy.loginfo("Valve rotation completed")
            
            # Step 4: Disable feedforward
            self.disable_feedforward()
            rospy.sleep(1.0)
            
            # Step 5: Optionally disable unified control
            # self.disable_unified_control()
            
            rospy.loginfo("Valve rotation task completed successfully")
            
        except Exception as e:
            rospy.logerr(f"Error during valve rotation task: {e}")
            self.disable_feedforward()
            
    def configure_advanced_parameters(self):
        """Configure advanced unified control parameters"""
        # Enable dynamic tilt optimization for better torque generation
        rospy.set_param("controller/unified_control_advanced/dynamic_tilt_optimization", True)
        
        # Set maximum rotor tilt angle (radians)
        rospy.set_param("controller/unified_control_advanced/max_rotor_tilt_angle", 0.3)
        
        # Balance between attitude stability and torque generation
        rospy.set_param("controller/unified_control_advanced/attitude_stability_weight", 10.0)
        rospy.set_param("controller/unified_control_advanced/torque_optimization_weight", 2.0)
        
        rospy.loginfo("Advanced unified control parameters configured")

def main():
    """Main function demonstrating unified control usage"""
    try:
        # Create the control example
        controller = BeetleUnifiedControlExample()
        
        # Configure advanced parameters
        controller.configure_advanced_parameters()
        
        # Wait for system to be ready
        rospy.loginfo("Waiting for beetle formation to be assembled...")
        rospy.sleep(5.0)
        
        # Execute the valve rotation task
        controller.execute_valve_rotation_task()
        
        # Keep the node running for monitoring
        rospy.loginfo("Task completed. Monitoring formation control...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Unified control example interrupted")
    except Exception as e:
        rospy.logerr(f"Error in unified control example: {e}")

if __name__ == '__main__':
    main()
