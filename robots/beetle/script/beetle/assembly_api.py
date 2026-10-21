#!/usr/bin/env python3
"""SMACH assembly workflow: motion and connector hardware are composed services."""
import rospy
import smach
import smach_ros
from beetle.assembly_alignment import DockingMotion
from beetle.assembly_hardware import AssemblyHardware


class MotionState(smach.State):
    def __init__(self, motion, outcomes):
        self.transitions = outcomes
        super().__init__(outcomes=sorted(set(outcomes.values()) | {'in_process', 'emergency'}))
        self.motion = motion

    def execute(self, userdata):
        phase = self.motion.tick()
        return 'emergency' if phase == 'aborted' else self.transitions.get(phase, 'in_process')


class DockingStandbyState(smach.State):
    def __init__(self, motion, hardware):
        super().__init__(outcomes=['done', 'in_process', 'emergency'])
        self.motion = motion
        self.hardware = hardware

    def execute(self, userdata):
        if self.motion.ready():
            self.motion.started = True
            if not self.hardware.prepare():
                self.motion.abort(reason='connector preparation interrupted')
                return 'emergency'
            return 'done'
        self.motion.rate.sleep()
        return 'emergency' if self.motion.aborted else 'in_process'


class AssemblyState(smach.State):
    def __init__(self, hardware):
        super().__init__(outcomes=['done', 'emergency'])
        self.hardware = hardware

    def execute(self, userdata):
        try:
            outcome = self.hardware.finish()
        except Exception:
            self.hardware.motion.abort(reason='assembly handoff exception')
            raise
        if outcome == 'emergency':
            self.hardware.motion.abort(reason='assembly handoff interrupted')
        return outcome


def build_assembly_pair(leader_id, follower_id, real_machine, direction=1):
    suffix = '%s_%s' % (follower_id, leader_id)
    motion = DockingMotion('beetle'+str(leader_id), 'beetle'+str(follower_id), direction)
    hardware = AssemblyHardware(robot_name=motion.follower, robot_id=follower_id,
                                leader=motion.leader, leader_id=leader_id,
                                real_machine=real_machine, attach_dir=direction, motion=motion)
    machine = smach.StateMachine(outcomes=['succeeded', 'interupted'])
    standby, prealign, approach, assembly = [name+suffix for name in
        ('StandbyState', 'PreAlignState', 'ApproachState', 'AssemblyState')]
    with machine:
        smach.StateMachine.add(standby, DockingStandbyState(motion, hardware),
                               transitions={'done': prealign, 'in_process': standby, 'emergency': 'interupted'})
        smach.StateMachine.add(prealign, MotionState(motion, {'approach': 'done'}),
                               transitions={'done': approach, 'in_process': prealign, 'emergency': 'interupted'})
        smach.StateMachine.add(approach, MotionState(motion, {'complete': 'done'}),
                               transitions={'done': assembly, 'in_process': approach, 'emergency': 'interupted'})
        smach.StateMachine.add(assembly, AssemblyState(hardware),
                               transitions={'done': 'succeeded', 'emergency': 'interupted'})
    return machine, motion


if __name__ == '__main__':
    rospy.init_node('assemble_demo')
    machine, motion = build_assembly_pair(2, 1, rospy.get_param('~real_machine', False), direction=-1)
    server = smach_ros.IntrospectionServer('smach_server', machine, '/SM_ROOT')
    server.start()
    try:
        machine.execute()
    except Exception:
        motion.abort(reason='state machine exception')
        raise
    finally:
        server.stop()
