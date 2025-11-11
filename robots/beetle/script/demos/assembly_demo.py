#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
from beetle.assembly import * 

class AssemblyDemo():
    def __init__(self):
        rospy.init_node("assembly_demo")
        
        # Read parameters from launch file or use defaults
        self.follower = rospy.get_param('~follower', 'beetle1')
        self.leader = rospy.get_param('~leader', 'beetle2')
        
        # Extract robot IDs from names (assuming format: beetleX)
        self.follower_id = int(self.follower.replace('beetle', ''))
        self.leader_id = int(self.leader.replace('beetle', ''))
        
        # Determine attach direction based on IDs
        # If follower_id < leader_id: attach from left (dir = -1)
        # If follower_id > leader_id: attach from right (dir = 1)
        self.attach_dir = -1.0 if self.follower_id < self.leader_id else 1.0
        
        rospy.loginfo("Assembly Demo Configuration:")
        rospy.loginfo("  Follower: %s (ID: %d)" % (self.follower, self.follower_id))
        rospy.loginfo("  Leader: %s (ID: %d)" % (self.leader, self.leader_id))
        rospy.loginfo("  Attach Direction: %.1f" % self.attach_dir)

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded','interupted'])
        with sm_top:
            # Create a single state machine for the specified robot pair
            smach.StateMachine.add('StandbyState',
                                   StandbyState(robot_name = self.follower, 
                                               robot_id = self.follower_id, 
                                               leader = self.leader, 
                                               leader_id = self.leader_id, 
                                               attach_dir = self.attach_dir),
                                   transitions={'done':'ApproachState', 
                                               'in_process': 'StandbyState', 
                                               'emergency':'interupted'})

            smach.StateMachine.add('ApproachState',
                                   ApproachState(robot_name = self.follower, 
                                                robot_id = self.follower_id, 
                                                leader = self.leader, 
                                                leader_id = self.leader_id, 
                                                attach_dir = self.attach_dir),
                                   transitions={'done':'AssemblyState', 
                                               'in_process':'ApproachState', 
                                               'fail':'StandbyState', 
                                               'emergency':'interupted'})

            smach.StateMachine.add('AssemblyState', 
                                   AssemblyState(robot_name = self.follower, 
                                                robot_id = self.follower_id, 
                                                leader = self.leader, 
                                                leader_id = self.leader_id, 
                                                attach_dir = self.attach_dir),
                                   transitions={'done':'succeeded', 
                                               'emergency':'interupted'})

        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        sis.stop()
        rospy.loginfo("Assembly demo completed with outcome: %s" % outcome)
        
if __name__ == '__main__':
    try:
        demo = AssemblyDemo()
        demo.main()
    except rospy.ROSInterruptException: pass
