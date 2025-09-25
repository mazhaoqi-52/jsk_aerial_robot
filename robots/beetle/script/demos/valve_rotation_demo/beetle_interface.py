#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import PoseStamped, WrenchStamped, Quaternion
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Joy

# Safe import for beetle.msg with fallback
try:
    from beetle.msg import TaggedWrench
except ImportError as e:
    rospy.logwarn("Failed to import beetle.msg: {}. Creating mock TaggedWrench.".format(e))
    # Create a proper ROS message mock class
    import genpy
    
    class TaggedWrench(genpy.Message):
        _md5sum = "fake_md5"
        _type = "beetle/TaggedWrench"
        _has_header = False
        _full_text = "int32 id\ngeometry_msgs/WrenchStamped wrench"
        
        __slots__ = ['id', 'wrench']
        _slot_types = ['int32', 'geometry_msgs/WrenchStamped']
        
        def __init__(self, *args, **kwds):
            super(TaggedWrench, self).__init__()
            if args or kwds:
                super(TaggedWrench, self).__init__(*args, **kwds)
            else:
                self.id = 0
                self.wrench = WrenchStamped()
        
        def _get_types(self):
            return self._slot_types[:]


class BeetleInterface(object):
    def __init__(self, module_id=1, debug_view=False, assembly_mode=False, assembly_tf_calculator=None):
        
        # Flight states
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6
        
        self.module_id = module_id
        self.debug_view = debug_view
        self.robot_name = f"beetle{module_id}"
        
        # Assembly mode configuration
        self.assembly_mode = assembly_mode
        self.assembly_tf_calculator = assembly_tf_calculator
        if self.assembly_mode and self.assembly_tf_calculator:
            rospy.loginfo(f"BeetleInterface initialized in ASSEMBLY mode for module {module_id}")
            rospy.loginfo(f"  Leader ID: {self.assembly_tf_calculator.get_leader_id()}")
        else:
            rospy.loginfo(f"BeetleInterface initialized in SINGLE mode for module {module_id}")
        
        # Robot parameters
        self.mass = rospy.get_param('~robot_mass', 1.5)
        self.default_pos_thresh = rospy.get_param('~default_pos_thresh', 0.03)
        self.default_rot_thresh = rospy.get_param('~default_rot_thresh', 0.05)
        
        # State variables
        self.uav_odom = Odometry()
        self.assembly_odom = Odometry()  # Assembly CoG odometry
        self.valve_pose = None
        self.flight_state = self.ARM_OFF_STATE
        self.target_pos = np.array([0, 0, 0])
        self.est_wrench = None
        self.is_simulation = rospy.get_param("~simulation", True)
        
        # Publishers - choose based on mode
        if self.assembly_mode:
            # Assembly mode: control the entire formation through assembly CoG
            self.nav_pub = rospy.Publisher('/assembly/uav/nav', FlightNav, queue_size=1)
            rospy.loginfo(f"Using assembly navigation topic: /assembly/uav/nav")
        else:
            # Single mode: control individual UAV
            self.nav_pub = rospy.Publisher(f'/beetle{module_id}/uav/nav', FlightNav, queue_size=1)
            rospy.loginfo(f"Using single UAV navigation topic: /beetle{module_id}/uav/nav")
            
        self.start_pub = rospy.Publisher('teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher('teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('teleop_command/land', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher('teleop_command/halt', Empty, queue_size=1)
        
        # External wrench control publisher
        self.tagged_wrench_pub = rospy.Publisher(f'/beetle{module_id}/tagged_wrench', TaggedWrench, queue_size=1)
        
        # External wrench state
        self.external_wrench_active = False
        self.current_external_wrench = TaggedWrench()
        
        # Subscribers - setup for both individual and assembly tracking
        if self.assembly_mode:
            # Subscribe to assembly CoG position
            self.assembly_sub = rospy.Subscriber('/assemble/cog/odom', Odometry, self.assemblyCallback, queue_size=1)
            # Also subscribe to individual UAV for leader tracking (if this UAV is leader)
            if self.assembly_tf_calculator and module_id == self.assembly_tf_calculator.get_leader_id():
                self.leader_sub = rospy.Subscriber(f'/beetle{module_id}/mocap/pose', PoseStamped, self.uavCallback, queue_size=1)
                rospy.loginfo(f"Leader UAV {module_id}: subscribing to both assembly and individual topics")
            else:
                rospy.loginfo(f"Follower UAV {module_id}: subscribing to assembly topic only")
        else:
            # Single mode: individual UAV tracking only
            self.uav_sub = rospy.Subscriber(f'/beetle{module_id}/mocap/pose', PoseStamped, self.uavCallback, queue_size=1)
            
        self.wrench_sub = rospy.Subscriber(f'/beetle{module_id}/estimated_external_wrench', WrenchStamped, self.wrenchCallback, queue_size=1)
        self.flight_state_sub = rospy.Subscriber('flight_state', UInt8, self.flightStateCallback, queue_size=1)
        
        if self.is_simulation:
            self.valve_sub = rospy.Subscriber('/valve/odom', Odometry, self.valveSimCallback, queue_size=1)
        else:
            self.valve_sub = rospy.Subscriber('/valve/mocap/pose', PoseStamped, self.valveCallback, queue_size=1)
        
        # Joy control for halt/skip functionality
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback, queue_size=1)
        self.prev_joy_state = Joy()
        self.halt_task = False
        self.force_skip = False
        
    def uavCallback(self, msg):
        self.uav_odom.header = msg.header
        self.uav_odom.pose.pose.position = msg.pose.position
        self.uav_odom.pose.pose.orientation = msg.pose.orientation
        
    def assemblyCallback(self, msg):
        """Callback for assembly CoG odometry"""
        self.assembly_odom = msg
        
    def valveCallback(self, msg):
        self.valve_pose = msg.pose
        
    def valveSimCallback(self, msg):
        if hasattr(msg, 'child_frame_id') and msg.child_frame_id == "handle":
            self.valve_pose = msg.pose.pose
            
    def wrenchCallback(self, msg):
        self.est_wrench = msg.wrench
        
    def flightStateCallback(self, msg):
        self.flight_state = msg.data
        
    def joyCallback(self, msg):
        if len(msg.buttons) > 4 and (msg.buttons[4] == 1) and (self.prev_joy_state.buttons[4] == 0):
            self.halt_task = True
            rospy.loginfo('Halt Task!!')
            
        if len(msg.buttons) > 5 and (msg.buttons[5] == 1) and (self.prev_joy_state.buttons[5] == 0):
            self.force_skip = True
            rospy.loginfo('Force skip while function')
            
        self.prev_joy_state = msg
        
    # Basic flight commands
    def start(self, sleep=1.0):
        self.start_pub.publish()
        rospy.sleep(sleep)
        
    def takeoff(self):
        self.takeoff_pub.publish()
        
    def land(self):
        self.land_pub.publish()
        
    def halt(self):
        self.halt_pub.publish()
        
    # State getters
    def getUavPos(self):
        """
        Get effective UAV position for control.
        
        In single mode: returns individual UAV position
        In assembly mode: returns assembly CoG position (used for formation control)
        """
        if self.assembly_mode:
            # Return assembly CoG position for formation control
            return np.array([
                self.assembly_odom.pose.pose.position.x,
                self.assembly_odom.pose.pose.position.y,
                self.assembly_odom.pose.pose.position.z
            ])
        else:
            # Return individual UAV position
            return np.array([
                self.uav_odom.pose.pose.position.x,
                self.uav_odom.pose.pose.position.y,
                self.uav_odom.pose.pose.position.z
            ])
            
    def getAssemblyPos(self):
        """Get assembly CoG position (only available in assembly mode)"""
        if not self.assembly_mode:
            rospy.logwarn("getAssemblyPos() called in single mode - returning None")
            return None
        return np.array([
            self.assembly_odom.pose.pose.position.x,
            self.assembly_odom.pose.pose.position.y,
            self.assembly_odom.pose.pose.position.z
        ])
        
    def getIndividualUavPos(self):
        """Get individual UAV position (available in both modes if subscribed)"""
        return np.array([
            self.uav_odom.pose.pose.position.x,
            self.uav_odom.pose.pose.position.y,
            self.uav_odom.pose.pose.position.z
        ])
        
    def getUavRot(self):
        """
        Get effective UAV orientation for control.
        
        In single mode: returns individual UAV orientation
        In assembly mode: returns assembly orientation
        """
        if self.assembly_mode:
            return np.array([
                self.assembly_odom.pose.pose.orientation.x,
                self.assembly_odom.pose.pose.orientation.y,
                self.assembly_odom.pose.pose.orientation.z,
                self.assembly_odom.pose.pose.orientation.w
            ])
        else:
            return np.array([
                self.uav_odom.pose.pose.orientation.x,
                self.uav_odom.pose.pose.orientation.y,
                self.uav_odom.pose.pose.orientation.z,
                self.uav_odom.pose.pose.orientation.w
            ])
        
    def getUavRPY(self):
        return euler_from_quaternion(self.getUavRot())
        
    def getAssemblyRPY(self):
        """Get assembly orientation (only in assembly mode)"""
        if not self.assembly_mode:
            return None
        assembly_quat = np.array([
            self.assembly_odom.pose.pose.orientation.x,
            self.assembly_odom.pose.pose.orientation.y,
            self.assembly_odom.pose.pose.orientation.z,
            self.assembly_odom.pose.pose.orientation.w
        ])
        return euler_from_quaternion(assembly_quat)
    
    def getEndEffectorPos(self):
        """
        Get end-effector position in world coordinates.
        
        In single mode: calculated from individual UAV position
        In assembly mode: calculated from assembly CoG using coordinate transforms
        """
        if self.assembly_mode and self.assembly_tf_calculator:
            # Assembly mode: transform from assembly CoG to end-effector
            assembly_pos = self.getAssemblyPos()
            assembly_rpy = self.getAssemblyRPY()
            if assembly_pos is None or assembly_rpy is None:
                rospy.logwarn("Assembly position/orientation not available")
                return None
                
            assembly_yaw = assembly_rpy[2]
            return self.assembly_tf_calculator.transform_assembly_to_end_effector(
                tuple(assembly_pos), assembly_yaw
            )
        else:
            # Single mode: calculate from individual UAV position
            uav_pos = self.getIndividualUavPos()
            uav_rpy = self.getUavRPY()
            if len(uav_pos) == 0 or len(uav_rpy) == 0:
                return None
                
            uav_yaw = uav_rpy[2]
            # Use hardcoded parameters for single mode (backward compatibility)
            dual_fang_center_offset = 0.246
            end_effector_offset_z = 0.074382
            
            end_effector_x = uav_pos[0] + dual_fang_center_offset * math.cos(uav_yaw)
            end_effector_y = uav_pos[1] + dual_fang_center_offset * math.sin(uav_yaw)
            end_effector_z = uav_pos[2] + end_effector_offset_z
            
            return (end_effector_x, end_effector_y, end_effector_z)
        
    def getValvePos(self):
        if self.valve_pose is None:
            return None
        return np.array([
            self.valve_pose.position.x,
            self.valve_pose.position.y,
            self.valve_pose.position.z
        ])
        
    def getValveRot(self):
        if self.valve_pose is None:
            return None
        return np.array([
            self.valve_pose.orientation.x,
            self.valve_pose.orientation.y,
            self.valve_pose.orientation.z,
            self.valve_pose.orientation.w
        ])
        
    def getValveYaw(self):
        if self.valve_pose is None:
            return None
        return euler_from_quaternion(self.getValveRot())[2]
        
    def getFlightState(self):
        return self.flight_state
        
    def getEstimatedWrench(self):
        return self.est_wrench
        
    def getTaskHaltFlag(self):
        return self.halt_task
        
    def resetTaskHaltFlag(self):
        self.halt_task = False
        
    def getForceSkipFlag(self):
        return self.force_skip
        
    def resetForceSkipFlag(self):
        self.force_skip = False
        
    # Core control method with enhanced SE(3) support
    def targetMotion(self, pos, rot=None, linear_vel=None, angular_vel=None):
        """
        Enhanced SE(3) position-velocity control for complex trajectory execution
        
        Args:
            pos: Target position [x, y, z]
            rot: Target rotation (yaw angle or quaternion)
            linear_vel: Target linear velocity [vx, vy, vz] (enables POS_VEL_MODE)
            angular_vel: Target angular velocity [wx, wy, wz] or scalar yaw rate
        """
        nav_msg = FlightNav()
        nav_msg.control_frame = nav_msg.WORLD_FRAME  # Always use WORLD_FRAME
        nav_msg.target = FlightNav.COG               # Always use COG target
        nav_msg.header.stamp = rospy.Time.now()
        
        # Enable POS_VEL_MODE if linear or angular velocity is provided
        mode = FlightNav.POS_VEL_MODE if (linear_vel is not None or angular_vel is not None) else FlightNav.POS_MODE
        
        nav_msg.pos_xy_nav_mode = mode
        nav_msg.pos_z_nav_mode = mode
        nav_msg.target_pos_x = pos[0]
        nav_msg.target_pos_y = pos[1]
        nav_msg.target_pos_z = pos[2]
        
        # Linear velocity control
        if linear_vel is not None:
            nav_msg.target_vel_x = linear_vel[0]
            nav_msg.target_vel_y = linear_vel[1]
            nav_msg.target_vel_z = linear_vel[2]
        else:
            nav_msg.target_vel_x = 0.0
            nav_msg.target_vel_y = 0.0
            nav_msg.target_vel_z = 0.0
            
        # Angular control (position and/or velocity)
        if rot is not None:
            nav_msg.yaw_nav_mode = mode  # Use same mode as position
            if isinstance(rot, (list, tuple, np.ndarray)) and len(rot) == 4:
                yaw = euler_from_quaternion(rot)[2]
            else:
                yaw = rot
            nav_msg.target_yaw = yaw
        else:
            nav_msg.yaw_nav_mode = 0
            nav_msg.target_yaw = 0.0
            
        # Angular velocity control - supporting both 3D angular velocity and scalar yaw rate
        if angular_vel is not None:
            if isinstance(angular_vel, (list, tuple, np.ndarray)) and len(angular_vel) == 3:
                # Full 3D angular velocity (primarily using yaw component)
                nav_msg.target_omega_x = angular_vel[0]
                nav_msg.target_omega_y = angular_vel[1] 
                nav_msg.target_omega_z = angular_vel[2]
            else:
                # Scalar yaw rate
                nav_msg.target_omega_x = 0.0
                nav_msg.target_omega_y = 0.0
                nav_msg.target_omega_z = angular_vel
        else:
            nav_msg.target_omega_x = 0.0
            nav_msg.target_omega_y = 0.0
            nav_msg.target_omega_z = 0.0
            
        nav_msg.roll_nav_mode = 0
        nav_msg.pitch_nav_mode = FlightNav.POS_MODE
        nav_msg.target_roll = 0.0
        nav_msg.target_pitch = 0.0
        
        self.nav_pub.publish(nav_msg)
        self.target_pos = pos
        
        # Publish external wrench if active
        if self.external_wrench_active:
            self.tagged_wrench_pub.publish(self.current_external_wrench)
    
    # External wrench control methods for force feedforward
    def addExternalWrench(self, force, torque, frame_id="world"):
        """
        Apply external wrench for force/torque feedforward control
        
        Args:
            force: Force vector [fx, fy, fz] in Newtons
            torque: Torque vector [tx, ty, tz] in Newton-meters  
            frame_id: Reference frame (default: "world")
        """
        self.current_external_wrench.index = self.module_id
        self.current_external_wrench.wrench.header.stamp = rospy.Time.now()
        self.current_external_wrench.wrench.header.frame_id = frame_id
        
        self.current_external_wrench.wrench.wrench.force.x = force[0]
        self.current_external_wrench.wrench.wrench.force.y = force[1]
        self.current_external_wrench.wrench.wrench.force.z = force[2]
        
        self.current_external_wrench.wrench.wrench.torque.x = torque[0]
        self.current_external_wrench.wrench.wrench.torque.y = torque[1]
        self.current_external_wrench.wrench.wrench.torque.z = torque[2]
        
        self.external_wrench_active = True
        
        # Immediately publish the wrench
        self.tagged_wrench_pub.publish(self.current_external_wrench)
        
        if self.debug_view:
            rospy.loginfo(f"External wrench applied - Force: [{force[0]:.3f}, {force[1]:.3f}, {force[2]:.3f}], "
                         f"Torque: [{torque[0]:.3f}, {torque[1]:.3f}, {torque[2]:.3f}]")
    
    def clearExternalWrench(self):
        """
        Clear/stop external wrench application
        """
        if self.external_wrench_active:
            # Send zero wrench to stop force application
            zero_wrench = TaggedWrench()
            zero_wrench.index = self.module_id
            zero_wrench.wrench.header.stamp = rospy.Time.now()
            zero_wrench.wrench.header.frame_id = "world"
            
            self.tagged_wrench_pub.publish(zero_wrench)
            self.external_wrench_active = False
            
            if self.debug_view:
                rospy.loginfo("External wrench cleared")
    
    def updateExternalWrench(self, force, torque):
        """
        Update currently active external wrench (convenience method)
        
        Args:
            force: Updated force vector [fx, fy, fz]
            torque: Updated torque vector [tx, ty, tz]
        """
        if self.external_wrench_active:
            self.addExternalWrench(force, torque)
        else:
            rospy.logwarn("No active external wrench to update")
    
    def getExternalWrenchStatus(self):
        """
        Get current external wrench status and values
        
        Returns:
            dict: Status information including active state and current values
        """
        if self.external_wrench_active:
            return {
                'active': True,
                'force': [
                    self.current_external_wrench.wrench.wrench.force.x,
                    self.current_external_wrench.wrench.wrench.force.y,
                    self.current_external_wrench.wrench.wrench.force.z
                ],
                'torque': [
                    self.current_external_wrench.wrench.wrench.torque.x,
                    self.current_external_wrench.wrench.wrench.torque.y,
                    self.current_external_wrench.wrench.wrench.torque.z
                ]
            }
        else:
            return {'active': False, 'force': [0, 0, 0], 'torque': [0, 0, 0]}
        
    # Enhanced trajectory control method for Dragon-style SE(3) control
    def executeTrajectoryWithWrench(self, pos, rot, linear_vel, angular_vel, force, torque):
        """
        Combined SE(3) control with external wrench feedforward
        This is the core method for Dragon-style circular trajectory execution
        
        Args:
            pos: Target position [x, y, z] 
            rot: Target rotation (yaw angle)
            linear_vel: Target linear velocity [vx, vy, vz]
            angular_vel: Target angular velocity (yaw rate)
            force: Feedforward force [fx, fy, fz]
            torque: Feedforward torque [tx, ty, tz]
        """
        # Apply external wrench first
        self.addExternalWrench(force, torque)
        
        # Execute motion command with SE(3) control
        self.targetMotion(pos, rot, linear_vel, angular_vel)
        
        return True
        
    # Core convergence wait method
    def goPoseWaitConvergence(self, pos, rot=None, pos_thresh=None, vel_thresh=0.1, rot_thresh=None, timeout=30, check_func=None):
        if pos_thresh is None:
            pos_thresh = self.default_pos_thresh
        if rot_thresh is None:
            rot_thresh = self.default_rot_thresh
            
        self.targetMotion(pos, rot=rot)
        start_time = rospy.get_time()
        
        if check_func is None:
            check_func = self.posYawConvergenceCheck
            
        while not check_func(pos, rot, pos_thresh, vel_thresh, rot_thresh):
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout and timeout > 0:
                return False
                
            if self.force_skip:
                rospy.logwarn("Force skip go pos convergence check")
                self.force_skip = False
                return True
                
            if self.halt_task:
                rospy.logwarn("Halt the task")
                return False
                
            if rospy.is_shutdown():
                return False
                
            rospy.sleep(0.1)
            
        return True
        
    # Standard position-yaw convergence check
    def posYawConvergenceCheck(self, target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh):
        if isinstance(pos_thresh, list):
            pos_thresh = pos_thresh[2] if len(pos_thresh) == 3 else self.default_pos_thresh
            
        if isinstance(rot_thresh, list):
            rot_thresh = rot_thresh[2] if len(rot_thresh) == 3 else self.default_rot_thresh
            
        current_pos = self.getUavPos()
        current_rpy = self.getUavRPY()
        current_yaw = current_rpy[2]
        
        if target_rot is None:
            target_yaw = current_yaw
        elif isinstance(target_rot, (list, tuple, np.ndarray)) and len(target_rot) == 4:
            target_yaw = euler_from_quaternion(target_rot)[2]
        else:
            target_yaw = target_rot
            
        delta_pos = target_pos - current_pos
        delta_yaw = target_yaw - current_yaw
        
        # Normalize yaw difference
        if delta_yaw > np.pi:
            delta_yaw -= np.pi * 2
        elif delta_yaw < -np.pi:
            delta_yaw += np.pi * 2
            
        pos_error = np.linalg.norm(delta_pos)
        yaw_error = abs(delta_yaw)
        
        if self.debug_view:
            rospy.loginfo_throttle(1.0, f'Convergence check - pos: {pos_error:.4f}, yaw: {yaw_error:.4f}')
            
        return pos_error < pos_thresh and yaw_error < rot_thresh
        
    # Valve-specific convergence check for insertion phases with rotation detection
    def valveApproachConvergenceCheck(self, target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh,
                                     valve_rotation_threshold=0.035, contact_force_threshold=0.5):
        """
        专门用于阀门接近和转动检测的收敛检查函数
        
        这个函数不仅检查UAV位置收敛，更重要的是检测阀门是否开始转动
        这对于确定何时开始执行阀门转动任务至关重要
        
        Args:
            target_pos: 目标位置
            target_rot: 目标姿态
            pos_thresh: 位置阈值
            vel_thresh: 速度阈值 (兼容参数)
            rot_thresh: 旋转阈值
            valve_rotation_threshold: 阀门转动检测阈值 (默认: 2°)
            contact_force_threshold: 接触力阈值 (默认: 0.5N)
        
        Returns:
            bool: 是否收敛 (考虑阀门转动检测)
        """
        current_pos = self.getUavPos()
        current_rpy = self.getUavRPY()
        valve_pos = self.getValvePos()
        current_valve_yaw = self.getValveYaw()
        
        if valve_pos is None or current_valve_yaw is None:
            rospy.logwarn("阀门状态信息不可用，回退到标准收敛检查")
            return self.posYawConvergenceCheck(target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh)
            
        # 标准位置和姿态收敛检查
        valve_to_uav = current_pos - valve_pos
        valve_yaw = self.getValveYaw()
        
        cos_yaw = math.cos(valve_yaw)
        sin_yaw = math.sin(valve_yaw)
        
        # Transform to valve coordinate system
        xy_error_valve = math.sqrt((valve_to_uav[0] * cos_yaw + valve_to_uav[1] * sin_yaw)**2 + 
                                  (-valve_to_uav[0] * sin_yaw + valve_to_uav[1] * cos_yaw)**2)
        z_error = abs(valve_to_uav[2])
        
        # Attitude errors
        target_yaw = target_rot if not isinstance(target_rot, (list, tuple, np.ndarray)) else target_rot
        roll_error = abs(current_rpy[0])
        pitch_error = abs(current_rpy[1])
        yaw_error = abs(current_rpy[2] - target_yaw)
        
        # Normalize yaw error
        if yaw_error > np.pi:
            yaw_error = 2 * np.pi - yaw_error
            
        # 位置和姿态阈值
        xy_thresh = pos_thresh if isinstance(pos_thresh, (int, float)) else pos_thresh[0]
        z_thresh = pos_thresh if isinstance(pos_thresh, (int, float)) else pos_thresh[2]
        attitude_thresh = rot_thresh if isinstance(rot_thresh, (int, float)) else rot_thresh[2]
        
        # 基本收敛条件
        basic_converged = (xy_error_valve < xy_thresh and 
                          z_error < z_thresh and 
                          roll_error < attitude_thresh and 
                          pitch_error < attitude_thresh and 
                          yaw_error < attitude_thresh)
        
        # 阀门转动检测 - 关键新增功能
        if not hasattr(self, '_initial_valve_yaw') or not hasattr(self, '_valve_yaw_history'):
            # 初始化阀门转动检测
            self._initial_valve_yaw = current_valve_yaw
            self._valve_yaw_history = [current_valve_yaw]
            self._last_valve_check_time = rospy.Time.now().to_sec()
            self._valve_rotation_detected = False
            valve_rotation_angle = 0.0
        else:
            # 更新阀门角度历史
            current_time = rospy.Time.now().to_sec()
            if current_time - self._last_valve_check_time > 0.1:  # 每100ms更新一次
                self._valve_yaw_history.append(current_valve_yaw)
                if len(self._valve_yaw_history) > 20:  # 保持2秒历史
                    self._valve_yaw_history.pop(0)
                self._last_valve_check_time = current_time
            
            # 计算总的阀门转动角度
            valve_rotation_angle = abs(self._normalize_angle_diff(current_valve_yaw - self._initial_valve_yaw))
            
            # 检测阀门是否正在转动 (通过角度变化率)
            if len(self._valve_yaw_history) >= 5:
                recent_change = abs(self._normalize_angle_diff(
                    self._valve_yaw_history[-1] - self._valve_yaw_history[-5]))
                valve_rotation_rate = recent_change / 0.5  # 0.5秒内的变化率
                
                # 阀门转动检测：总角度超过阈值 或 转动速率超过1°/s
                self._valve_rotation_detected = (valve_rotation_angle > valve_rotation_threshold or 
                                               valve_rotation_rate > 0.017)  # 1°/s = 0.017 rad/s
            else:
                self._valve_rotation_detected = valve_rotation_angle > valve_rotation_threshold
        
        # 接触检测 (通过扭矩/力传感器)
        contact_established = False
        if hasattr(self, 'wrench_data') and self.wrench_data:
            try:
                # 检查末端执行器的力/扭矩
                force_magnitude = math.sqrt(
                    self.wrench_data.force.x**2 + 
                    self.wrench_data.force.y**2 + 
                    self.wrench_data.force.z**2
                )
                torque_magnitude = math.sqrt(
                    self.wrench_data.torque.x**2 + 
                    self.wrench_data.torque.y**2 + 
                    self.wrench_data.torque.z**2
                )
                contact_established = (force_magnitude > contact_force_threshold or 
                                     torque_magnitude > 0.1)  # 0.1 N·m扭矩阈值
            except:
                contact_established = False
        
        # 调试信息 (节流输出)
        if not hasattr(self, '_last_valve_debug_time'):
            self._last_valve_debug_time = 0
        current_time = rospy.Time.now().to_sec()
        if current_time - self._last_valve_debug_time > 2.0:  # 每2秒打印一次
            rospy.loginfo(f"阀门接近检查 - 位置误差: {xy_error_valve*1000:.1f}mm, "
                         f"Z误差: {z_error*1000:.1f}mm, 偏航误差: {math.degrees(yaw_error):.1f}°")
            rospy.loginfo(f"阀门转动: {math.degrees(valve_rotation_angle):.1f}°, "
                         f"转动检测: {self._valve_rotation_detected}, 接触: {contact_established}")
            self._last_valve_debug_time = current_time
        
        # 收敛条件：基本位置收敛 + (建立接触 或 检测到阀门转动)
        # 这样可以确保在开始检测到阀门转动时就认为已经准备好了
        converged = basic_converged and (contact_established or self._valve_rotation_detected)
        
        # 存储状态供其他函数使用
        if not hasattr(self, 'valve_check_result'):
            self.valve_check_result = {}
        
        self.valve_check_result = {
            'basic_converged': basic_converged,
            'valve_rotation_detected': self._valve_rotation_detected,
            'contact_established': contact_established,
            'valve_rotation_angle': valve_rotation_angle,
            'position_error': xy_error_valve,
            'z_error': z_error,
            'yaw_error': yaw_error
        }
        
        return converged
                
    # Geometric target calculation for different phases
    def calculateTargetCogPose(self, phase, **kwargs):
        """Calculate target CoG pose based on phase and constraints"""
        valve_pos = self.getValvePos()
        valve_yaw = self.getValveYaw()
        
        if valve_pos is None or valve_yaw is None:
            rospy.logwarn("Valve position/orientation not available for target calculation")
            return None, None
            
        if phase == 'approach':
            return self._calculateApproachPose(valve_pos, valve_yaw, **kwargs)
        elif phase == 'insertion':
            return self._calculateInsertionPose(valve_pos, valve_yaw, **kwargs)
        elif phase == 'rotation':
            return self._calculateRotationPose(valve_pos, valve_yaw, **kwargs)
        elif phase == 'withdraw':
            return self._calculateWithdrawPose(valve_pos, valve_yaw, **kwargs)
        else:
            rospy.logwarn(f"Unknown phase: {phase}")
            return None, None
            
    def _calculateApproachPose(self, valve_pos, valve_yaw, distance=0.5, height_offset=0.1):
        """Calculate approach position maintaining safe distance"""
        target_pos = np.array([
            valve_pos[0] - distance * math.cos(valve_yaw),
            valve_pos[1] - distance * math.sin(valve_yaw),
            valve_pos[2] + height_offset
        ])
        target_yaw = valve_yaw + math.pi  # Face the valve
        return target_pos, target_yaw
        
    def _calculateInsertionPose(self, valve_pos, valve_yaw, end_effector_distance=0.05, z_offset=0.074):
        """Calculate insertion pose based on geometric constraints"""
        # Use beam1 (120° from valve orientation) for symmetric insertion
        spoke_angle = valve_yaw + math.pi * 2/3  # 120°
        
        # Calculate end-effector center position
        end_effector_center = np.array([
            valve_pos[0] + end_effector_distance * math.cos(spoke_angle),
            valve_pos[1] + end_effector_distance * math.sin(spoke_angle),
            valve_pos[2] + z_offset
        ])
        
        # UAV CoG position (offset from end-effector)
        dual_fang_offset = 0.25346  # From previous analysis
        target_pos = np.array([
            end_effector_center[0] + dual_fang_offset * math.cos(spoke_angle),
            end_effector_center[1] + dual_fang_offset * math.sin(spoke_angle),
            end_effector_center[2] - z_offset
        ])
        
        # UAV faces opposite to spoke direction
        target_yaw = spoke_angle + math.pi
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
            
        return target_pos, target_yaw
        
    def _calculateRotationPose(self, valve_pos, valve_yaw, **kwargs):
        """Maintain current insertion pose during rotation"""
        current_pos = self.getUavPos()
        current_yaw = self.getUavRPY()[2]
        return current_pos, current_yaw
        
    def _calculateWithdrawPose(self, valve_pos, valve_yaw, withdraw_distance=0.3, height_offset=0.1):
        """Calculate safe withdrawal position"""
        target_pos = np.array([
            valve_pos[0] - withdraw_distance * math.cos(valve_yaw),
            valve_pos[1] - withdraw_distance * math.sin(valve_yaw),
            valve_pos[2] + height_offset
        ])
        target_yaw = valve_yaw + math.pi  # Face the valve
        return target_pos, target_yaw
    
    def _normalize_angle_diff(self, angle_diff):
        """
        将角度差值标准化到 [-π, π] 范围内
        
        Args:
            angle_diff: 角度差值 (弧度)
            
        Returns:
            float: 标准化后的角度差值
        """
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
