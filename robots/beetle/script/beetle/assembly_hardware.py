"""Connector preparation and assembly handoff, independent of SMACH."""
import time
import rospy
from std_msgs.msg import Bool
from aerial_robot_msgs.msg import FlightNav
from diagnostic_msgs.msg import KeyValue
from beetle.dynamixel_control_api import DynamixelControl
from beetle.gazebo_link_attacher import GazeboLinkAttacher


class AssemblyHardware:
    def __init__(self,
                 robot_name = 'beetle1',
                 robot_id = 1,
                 male_servo_id = 4,
                 female_servo_id = 5,
                 real_machine = False,
                 unlock_servo_angle_male = 1820,
                 lock_servo_angle_male = 1580,
                 lock_servo_angle_female = 3000,
                 leader = 'beetle2',
                 leader_id = 2,
                 attach_dir = -1.0,
                 motion = None):

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.unlock_servo_angle_male = unlock_servo_angle_male
        self.lock_servo_angle_male = lock_servo_angle_male
        self.lock_servo_angle_female = lock_servo_angle_female
        self.leader = leader
        self.leader_id = leader_id
        self.attach_dir = attach_dir
        self.motion = motion
        if motion is None:
            raise ValueError('AssemblyHardware requires a guarded DockingMotion')
        self.prepared = False

        # publisher
        self.male_robot, self.male_id = ((leader, leader_id) if attach_dir > 0 else (robot_name, robot_id))
        self.female_robot, self.female_id = ((robot_name, robot_id) if attach_dir > 0 else (leader, leader_id))
        self.docking_pub = rospy.Publisher(self.male_robot+"/docking_cmd", Bool, queue_size=10)
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.leader_nav_pub = rospy.Publisher(self.leader+"/uav/nav", FlightNav, queue_size=10)
        self.assembly_nav_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
        self.flag_pub = rospy.Publisher('/' + self.robot_name + '/assembly_flag', KeyValue, queue_size = 1)
        self.flag_pub_leader = rospy.Publisher('/' + self.leader + '/assembly_flag', KeyValue, queue_size = 1)

    def interrupted(self):
        return rospy.is_shutdown() or not self.motion.ready()

    def _servo(self, robot, robot_id, servo_id, angle):
        if self.interrupted():
            return False
        servo = DynamixelControl(robot, robot_id, servo_id, self.real_machine)
        timeout = rospy.Time.now() + rospy.Duration(3.0)
        while servo.dx_pos_pub.get_num_connections() == 0:
            if self.interrupted():
                return False
            if rospy.Time.now() > timeout:
                self.motion.abort(reason='no servo subscriber on '+robot)
                return False
            rospy.sleep(.1)
        if self.interrupted():
            return False
        servo.sendTargetAngle(angle)
        return True

    def prepare(self):
        if self.interrupted():
            return False
        if self.prepared:
            return True
        if self.real_machine:
            if not self._servo(self.male_robot, self.male_id, self.male_servo_id, self.unlock_servo_angle_male):
                return False
            if not self._servo(self.female_robot, self.female_id, self.female_servo_id, self.lock_servo_angle_female):
                return False
        else:
            self.docking_pub.publish(Bool(data=True))
        self.prepared = True
        return True

    def finish(self):
        if self.interrupted():
            return 'emergency'

        if self.real_machine:
            # Create DynamixelControl fresh at execute time to ensure valid publisher connection
            rospy.loginfo("[AssemblyState] Creating new DynamixelControl for %s" % self.male_robot)
            self.dynamixel_servo = DynamixelControl(self.male_robot, self.male_id, self.male_servo_id, self.real_machine)

            # Wait for publisher to establish connection
            rospy.loginfo("[AssemblyState] Waiting for servo publisher to connect...")
            timeout = rospy.Time.now() + rospy.Duration(5.0)
            while self.dynamixel_servo.dx_pos_pub.get_num_connections() == 0:
                if self.interrupted():
                    return 'emergency'
                if rospy.Time.now() > timeout:
                    rospy.logwarn("[AssemblyState] Timeout waiting for servo subscriber on /%s/servo/target_states" % self.male_robot)
                    self.motion.abort(reason='no lock servo subscriber')
                    return 'emergency'
                rospy.sleep(0.1)

            # Send servo command multiple times to ensure it's received
            for i in range(5):
                if self.interrupted():
                    return 'emergency'
                rospy.loginfo("[AssemblyState] Sending lock_servo_angle_male: %d (attempt %d)" % (self.lock_servo_angle_male, i+1))
                self.dynamixel_servo.sendTargetAngle(self.lock_servo_angle_male)
                rospy.sleep(0.2)

        time.sleep(1.0)
        if not self.real_machine:
            rospy.sleep(1.0)
        if self.interrupted():
            return 'emergency'
        nav_msg = FlightNav()
        nav_msg.pos_xy_nav_mode = FlightNav.STAY_HERE_MODE
        self.follower_nav_pub.publish(nav_msg)
        self.leader_nav_pub.publish(nav_msg)
        self.assembly_nav_pub.publish(nav_msg)
        rospy.sleep(1.0)
        if self.interrupted():
            return 'emergency'
        self.flag_pub.publish(KeyValue(key=str(self.robot_id), value='1'))
        self.flag_pub_leader.publish(KeyValue(key=str(self.leader_id), value='1'))
        rospy.sleep(0.3)
        if not self.real_machine:
            try:
                link_attacher = GazeboLinkAttacher(self.robot_name, 'root', self.leader, 'root')
                link_attacher.attach_links()
            except rospy.ServiceException:
                self.motion.abort(reason='Gazebo link attachment failed')
                return 'emergency'
        rospy.sleep(2.0)

        return 'done'
