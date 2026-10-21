#!/usr/bin/env python

import rospy
import smach
import smach_ros
from beetle.assembly_api import build_assembly_pair

class AssemblyDemo():
    def __init__(self,module_ids,real_machine):
        self.module_ids = module_ids
        self.real_machine = real_machine
        rospy.loginfo(self.module_ids)
    def main(self):
        motions = []
        sm_top = smach.StateMachine(outcomes=['succeeded','interupted'])
        with sm_top:
            for i,item in enumerate(self.module_ids):
                if i == 0:
                    continue
                # IDs follow increasing X: leader male (+X), follower female (-X).
                sub_sm, motion = build_assembly_pair(self.module_ids[i-1], item, self.real_machine)
                motions.append(motion)
                next_state = 'succeeded' if i == len(self.module_ids)-1 else 'SUB'+str(i+1)
                smach.StateMachine.add('SUB'+str(i), sub_sm,
                                       transitions={'succeeded': next_state, 'interupted': 'interupted'})

        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()

        try:
            outcome = sm_top.execute()
            rospy.loginfo(f"Assembly demo state machine completed with outcome: {outcome}")
            return outcome
        except Exception as e:
            for motion in motions:
                if motion.started and motion.guidance.phase != 'complete':
                    motion.abort(reason='state machine exception: %s' % e)
            rospy.logerr(f"Error during assembly demo execution: {e}")
            return 'interupted'
        finally:
            try:
                sis.stop()
            except Exception as exc:
                rospy.logwarn("Could not stop SMACH introspection: %s", exc)
if __name__ == '__main__':
    rospy.init_node("assembly_motion")
    modules_str = rospy.get_param("~module_ids", rospy.get_param("module_ids", ""))
    rm = rospy.get_param("~real_machine", rospy.get_param("real_machine", True))
    modules = []
    if(modules_str):
        modules = [int(x) for x in modules_str.split(',')]
    else:
        rospy.loginfo("No module ID is designated!")
    try:
        assembly_motion = AssemblyDemo(module_ids = modules, real_machine=rm);
        assembly_motion.main()
    except rospy.ROSInterruptException: pass
