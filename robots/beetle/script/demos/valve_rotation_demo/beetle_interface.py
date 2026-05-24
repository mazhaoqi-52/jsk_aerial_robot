#!/usr/bin/env python
"""
Beetle UAV Interface - Core control interface for single/assembly mode operations.
Provides position control, trajectory execution, and wrench feedforward.
"""

import rospy
import numpy as np
import math
import sys
import os
from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy

# Import TaggedWrench message with fallback
try:
    from beetle.msg import TaggedWrench
except (ImportError, AttributeError):
    # Handle path conflicts between script/beetle and devel/beetle
    original_path = sys.path.copy()
    try:
        # Remove conflicting paths temporarily
        for path in [p for p in sys.path if os.path.exists(os.path.join(p, 'beetle')) 
                     and not os.path.exists(os.path.join(p, 'beetle', 'msg'))]:
            sys.path.remove(path)
        for mod in ['beetle', 'beetle.msg']:
            sys.modules.pop(mod, None)
        from beetle.msg import TaggedWrench
    except ImportError:
        sys.path = original_path
        # Create minimal mock
        import genpy
        class TaggedWrench(genpy.Message):
            __slots__ = ['index', 'wrench']
            def __init__(self):
                self.index = 0
                self.wrench = WrenchStamped()
        rospy.logwarn("Using mock TaggedWrench")


class BeetleInterface(object):
    """Interface for controlling Beetle UAV in single or assembly mode."""
    
    # Flight state constants
    ARM_OFF_STATE = 0
    START_STATE = 1
    ARM_ON_STATE = 2
    TAKEOFF_STATE = 3
    LAND_STATE = 4
    HOVER_STATE = 5
    STOP_STATE = 6
    
    def __init__(self, module_id=1, debug_view=False, assembly_mode=False, assembly_tf_calculator=None):
        self.module_id = module_id
        self.debug_view = debug_view
        self.robot_name = f"beetle{module_id}"
        self.assembly_mode = assembly_mode
        self.assembly_tf_calculator = assembly_tf_calculator
        
        # Parameters
        self.mass = rospy.get_param('~robot_mass', 1.5)
        self.default_pos_thresh = rospy.get_param('~default_pos_thresh', 0.03)
        self.default_rot_thresh = rospy.get_param('~default_rot_thresh', 0.05)
        self.is_simulation = rospy.get_param("~simulation", True)
        
        # State variables
        self.uav_odom = Odometry()
        self.assembly_odom = Odometry()
        self.valve_pose = None
        self.flight_state = self.ARM_OFF_STATE
        self.target_pos = np.array([0, 0, 0])
        self.est_wrench = None
        
        # External wrench state
        self.external_wrench_active = False
        self.current_external_wrench = TaggedWrench()

        # Task observer-prediction state (Level 2: spatial-inertia consistent).
        # `attach_module_id` is retained only as an opt-in marker; the
        # decomposition formula is uniform across modules.
        self._inter_attach_module_id = None
        self._inter_module_masses = {}
        self._inter_module_ids = []
        # Positions r_i in formation frame (will be auto-recentered to
        # formation CoG); diagonal inertias I_i (Ixx,Iyy,Izz) in module body
        # frame. Both optional — if absent, helper falls back to mass-ratio
        # (Level 1) which is OK for pure-translational tasks like towing but
        # NOT for tasks with significant torque (e.g. valve rotation).
        self._inter_module_positions = {}
        self._inter_module_inertias_diag = {}
        # Cached after setAttachModule(): centred positions, M_form^{-1}.
        self._inter_r_centered = {}
        self._inter_M_form_inv = None
        self._inter_m_total = 0.0
        self._inter_auto_publish = False
        self._est_wrench_task_pubs = {}
        # Track last published ŷ^task per module so clearExternalWrench can
        # zero them out cleanly.
        self._last_est_wrench_task = {}

        # Joy control state
        self.prev_joy_state = Joy()
        self.halt_task = False
        self.force_skip = False
        
        # Setup publishers
        nav_topic = '/assembly/uav/nav' if assembly_mode else f'/beetle{module_id}/uav/nav'
        self.nav_pub = rospy.Publisher(nav_topic, FlightNav, queue_size=1)
        self.start_pub = rospy.Publisher('teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher('teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('teleop_command/land', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher('teleop_command/halt', Empty, queue_size=1)
        self.tagged_wrench_pub = rospy.Publisher(f'/beetle{module_id}/tagged_wrench', TaggedWrench, queue_size=1)
        # Wrench must be routed to the C++ LEADER (centre-of-sorted-IDs), which may
        # differ from the Python "leader" (end-effector module = last in chain).
        # C++ beetle_navigation publishes assembly_leader_id to rosparam.
        wrench_target_id = rospy.get_param(f'/beetle{module_id}/assembly_leader_id', module_id) if assembly_mode else module_id
        self.wrench_target_id = wrench_target_id
        self.desired_ext_wrench_pub = rospy.Publisher(f'/beetle{wrench_target_id}/desired_external_wrench', WrenchStamped, queue_size=1)
        if assembly_mode:
            self.formation_wrench_pub = rospy.Publisher(f'/beetle{wrench_target_id}/formation_desired_wrench', WrenchStamped, queue_size=1)
        if assembly_mode and wrench_target_id != module_id:
            rospy.logwarn(f"[BeetleInterface] Wrench routed to C++ LEADER beetle{wrench_target_id} "
                          f"(Python EE module={module_id})")

        # ----- Per-module task observer prediction (ŷ^task) publishers -----
        # Theory: the per-module momentum observer outputs
        #     ŷ_i = c_i + d_i + b_i   (joint force + direct external + parasitic)
        # We publish a task-space prediction ŷ_i^task and the C++ controller
        # subtracts it BEFORE the joint-cut recursion in calcInteractionWrench,
        # so downstream inter_wrench_list_ is the task-subtracted residual
        # (~parasitic if the model is accurate). Two decomposition models are
        # supported:
        #   Level 1 (mass ratio): ŷ_i^task = (m_i/m_tot) * W_ext
        #     Only valid for pure-translational tasks (towing). Used when
        #     positions/inertias are not provided.
        #   Level 2 (spatial inertia): physically consistent decomposition
        #     F_i = m_i (a + α × r_i), τ_{i,Ci} = I_i α, where
        #     [a; α] = M_form^{-1} W_ext at formation CoG. Required when the
        #     task wrench has torque (e.g. valve rotation).
        # Invariant: Σ ŷ_i^task = W_ext (Newton 2nd + parallel-axis identity).
        default_ids = [1, 2]
        self._inter_module_ids = rospy.get_param(
            f'/beetle{wrench_target_id}/assembly_ids', default_ids)
        for mid in self._inter_module_ids:
            self._est_wrench_task_pubs[mid] = rospy.Publisher(
                f'/beetle{mid}/est_wrench_task', TaggedWrench, queue_size=1)
            self._last_est_wrench_task[mid] = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        
        # Setup subscribers
        if assembly_mode:
            rospy.Subscriber('/assemble/cog/odom', Odometry, self._assembly_cb, queue_size=1)
            if assembly_tf_calculator and module_id == assembly_tf_calculator.get_leader_id():
                rospy.Subscriber(f'/beetle{module_id}/mocap/pose', PoseStamped, self._uav_cb, queue_size=1)
        else:
            rospy.Subscriber(f'/beetle{module_id}/mocap/pose', PoseStamped, self._uav_cb, queue_size=1)
        
        rospy.Subscriber(f'/beetle{module_id}/estimated_external_wrench', WrenchStamped, self._wrench_cb, queue_size=1)
        rospy.Subscriber('flight_state', UInt8, self._flight_state_cb, queue_size=1)
        rospy.Subscriber('joy', Joy, self._joy_cb, queue_size=1)
        
        valve_topic = '/valve/odom' if self.is_simulation else '/valve/mocap/pose'
        if self.is_simulation:
            rospy.Subscriber(valve_topic, Odometry, self._valve_sim_cb, queue_size=1)
        else:
            rospy.Subscriber(valve_topic, PoseStamped, self._valve_cb, queue_size=1)
        
        mode_str = "ASSEMBLY" if assembly_mode else "SINGLE"
        rospy.loginfo(f"BeetleInterface[{module_id}] initialized in {mode_str} mode")
    
    # Callbacks
    def _uav_cb(self, msg):
        self.uav_odom.header = msg.header
        self.uav_odom.pose.pose.position = msg.pose.position
        self.uav_odom.pose.pose.orientation = msg.pose.orientation
    
    def _assembly_cb(self, msg):
        self.assembly_odom = msg
    
    def _valve_cb(self, msg):
        self.valve_pose = msg.pose
    
    def _valve_sim_cb(self, msg):
        if hasattr(msg, 'child_frame_id') and msg.child_frame_id == "handle":
            self.valve_pose = msg.pose.pose
    
    def _wrench_cb(self, msg):
        self.est_wrench = msg.wrench
    
    def _flight_state_cb(self, msg):
        self.flight_state = msg.data
    
    def _joy_cb(self, msg):
        if len(msg.buttons) > 4 and msg.buttons[4] == 1 and self.prev_joy_state.buttons[4] == 0:
            self.halt_task = True
            rospy.loginfo('Halt Task!')
        if len(msg.buttons) > 5 and msg.buttons[5] == 1 and self.prev_joy_state.buttons[5] == 0:
            self.force_skip = True
            rospy.loginfo('Force skip')
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
    
    # Position getters
    def getUavPos(self):
        """Get effective position (assembly CoG in assembly mode, UAV pos otherwise)."""
        odom = self.assembly_odom if self.assembly_mode else self.uav_odom
        p = odom.pose.pose.position
        return np.array([p.x, p.y, p.z])
    
    def getAssemblyPos(self):
        """Get assembly CoG position (assembly mode only)."""
        if not self.assembly_mode:
            return None
        p = self.assembly_odom.pose.pose.position
        return np.array([p.x, p.y, p.z])
    
    def getIndividualUavPos(self):
        """Get individual UAV position."""
        p = self.uav_odom.pose.pose.position
        return np.array([p.x, p.y, p.z])
    
    def getUavRot(self):
        """Get effective orientation quaternion."""
        odom = self.assembly_odom if self.assembly_mode else self.uav_odom
        o = odom.pose.pose.orientation
        return np.array([o.x, o.y, o.z, o.w])
    
    def getUavRPY(self):
        """Get effective orientation as roll, pitch, yaw."""
        return euler_from_quaternion(self.getUavRot())
    
    def getAssemblyRPY(self):
        """Get assembly orientation as RPY (assembly mode only)."""
        if not self.assembly_mode:
            return None
        o = self.assembly_odom.pose.pose.orientation
        return euler_from_quaternion([o.x, o.y, o.z, o.w])

    def _worldToFormationBodyWrench(self, force, torque, yaw_only=False):
        """Rotate a world-frame wrench into the formation body frame."""
        rpy = self.getAssemblyRPY()
        if rpy is None:
            return force, torque
        roll, pitch, yaw = rpy
        if yaw_only:
            roll = 0.0
            pitch = 0.0
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        body_to_world = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        world_to_body = body_to_world.T
        return ((world_to_body @ np.asarray(force, dtype=float)).tolist(),
                (world_to_body @ np.asarray(torque, dtype=float)).tolist())
    
    def getEndEffectorPos(self):
        """Get end-effector position in world coordinates (pitch-aware)."""
        if self.assembly_mode and self.assembly_tf_calculator:
            assembly_pos = self.getAssemblyPos()
            assembly_rpy = self.getAssemblyRPY()
            if assembly_pos is None or assembly_rpy is None:
                return None
            return self.assembly_tf_calculator.transform_assembly_to_end_effector(
                tuple(assembly_pos), assembly_rpy[2], assembly_rpy[1])
        else:
            # Single mode: calculate from UAV position
            uav_pos = self.getIndividualUavPos()
            uav_rpy = self.getUavRPY()
            if len(uav_pos) == 0 or len(uav_rpy) == 0:
                return None
            yaw = uav_rpy[2]
            offset_x, offset_z = 0.246, 0.074382  # End-effector offsets
            return (uav_pos[0] + offset_x * math.cos(yaw),
                    uav_pos[1] + offset_x * math.sin(yaw),
                    uav_pos[2] + offset_z)
    
    def getValvePos(self):
        """Get valve position."""
        if self.valve_pose is None:
            return None
        p = self.valve_pose.position
        return np.array([p.x, p.y, p.z])
    
    def getValveRot(self):
        """Get valve orientation quaternion."""
        if self.valve_pose is None:
            return None
        o = self.valve_pose.orientation
        return np.array([o.x, o.y, o.z, o.w])
    
    def getValveYaw(self):
        """Get valve yaw angle."""
        rot = self.getValveRot()
        return euler_from_quaternion(rot)[2] if rot is not None else None
    
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
    
    def _to_list3(self, data):
        """Convert various data formats to [x, y, z] list."""
        if data is None:
            return None
        try:
            return [float(data[0]), float(data[1]), float(data[2])]
        except (IndexError, TypeError, ValueError):
            return [0.0, 0.0, 0.0]
    
    def targetMotion(self, pos, rot=None, linear_vel=None, angular_vel=None):
        """
        SE(3) position-velocity control.
        
        Args:
            pos: Target position [x, y, z]
            rot: Target yaw angle or quaternion
            linear_vel: Target linear velocity [vx, vy, vz]
            angular_vel: Target angular velocity [wx, wy, wz] or scalar yaw rate
        """
        pos_list = self._to_list3(pos)
        vel_list = self._to_list3(linear_vel)
        
        nav_msg = FlightNav()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG
        nav_msg.header.stamp = rospy.Time.now()
        
        # Use POS_VEL_MODE if velocity is provided
        mode = FlightNav.POS_VEL_MODE if (vel_list or angular_vel) else FlightNav.POS_MODE
        
        nav_msg.pos_xy_nav_mode = mode
        nav_msg.pos_z_nav_mode = mode
        nav_msg.target_pos_x = pos_list[0]
        nav_msg.target_pos_y = pos_list[1]
        nav_msg.target_pos_z = pos_list[2]
        
        # Linear velocity
        if vel_list:
            nav_msg.target_vel_x, nav_msg.target_vel_y, nav_msg.target_vel_z = vel_list
        else:
            nav_msg.target_vel_x = nav_msg.target_vel_y = nav_msg.target_vel_z = 0.0
        
        # Yaw control
        if rot is not None:
            nav_msg.yaw_nav_mode = mode
            if isinstance(rot, (list, tuple, np.ndarray)) and len(rot) == 4:
                nav_msg.target_yaw = euler_from_quaternion(rot)[2]
            else:
                nav_msg.target_yaw = rot
        else:
            nav_msg.yaw_nav_mode = 0
            nav_msg.target_yaw = 0.0
        
        # Angular velocity
        if angular_vel is not None:
            if isinstance(angular_vel, (list, tuple, np.ndarray)) and len(angular_vel) == 3:
                nav_msg.target_omega_x, nav_msg.target_omega_y, nav_msg.target_omega_z = angular_vel
            else:
                nav_msg.target_omega_x = nav_msg.target_omega_y = 0.0
                nav_msg.target_omega_z = angular_vel
        else:
            nav_msg.target_omega_x = nav_msg.target_omega_y = nav_msg.target_omega_z = 0.0
        
        nav_msg.roll_nav_mode = 0
        nav_msg.pitch_nav_mode = FlightNav.POS_MODE
        nav_msg.target_roll = nav_msg.target_pitch = 0.0
        
        self.nav_pub.publish(nav_msg)
        self.target_pos = pos
    
    def isUnifiedMode(self):
        """Query C++ runtime: is unified_control_mode currently active on the leader?"""
        leader_id = self.wrench_target_id if hasattr(self, 'wrench_target_id') else self.module_id
        return rospy.get_param(f'/beetle{leader_id}/controller/unified_control_mode', False)

    def addExternalWrench(self, force, torque, frame_id="world"):
        """Apply desired external wrench.
        
        In assembly_mode, frame_id="world" rotates full RPY world-frame input
        to formation body frame (fc). Use frame_id="world_yaw" for the legacy
        yaw-only conversion, or frame_id="fc" if the wrench is already in the
        formation body frame.
        Then routes to:
          - formation_desired_wrench  when isUnifiedMode() is True  (unified allocation)
          - desired_external_wrench   when isUnifiedMode() is False (lead-follower wrench_comp)
        Outside assembly_mode: publishes to desired_external_wrench as-is.
        """
        force_list = self._to_list3(force) or [0.0, 0.0, 0.0]
        torque_list = self._to_list3(torque) or [0.0, 0.0, 0.0]
        
        # Both unified and lead-follower paths need formation-body data.
        frame_key = str(frame_id).lower()
        if self.assembly_mode and frame_key in ("world", "map", "odom", "world_yaw"):
            force_list, torque_list = self._worldToFormationBodyWrench(
                force_list, torque_list, yaw_only=(frame_key == "world_yaw"))
            frame_id = "fc"
        elif frame_key in ("fc", "body", "formation", "formation_body"):
            frame_id = "fc"
        
        ff_msg = WrenchStamped()
        ff_msg.header.stamp = rospy.Time.now()
        ff_msg.header.frame_id = frame_id
        ff_msg.wrench.force.x = force_list[0]
        ff_msg.wrench.force.y = force_list[1]
        ff_msg.wrench.force.z = force_list[2]
        ff_msg.wrench.torque.x = torque_list[0]
        ff_msg.wrench.torque.y = torque_list[1]
        ff_msg.wrench.torque.z = torque_list[2]
        
        self.external_wrench_active = True
        self.current_ff_force = force_list
        self.current_ff_torque = torque_list
        if self.assembly_mode and hasattr(self, 'formation_wrench_pub') and self.isUnifiedMode():
            self.formation_wrench_pub.publish(ff_msg)
        else:
            self.desired_ext_wrench_pub.publish(ff_msg)

        # ----- Also drive per-module ŷ^task in lockstep, if enabled -----
        # The body-frame force/torque is reused (same frame as the C++
        # est_wrench_task_list_).
        if self._inter_auto_publish:
            self._publishInternalWrenchFromExternal(force_list, torque_list, frame_id)
    
    def clearExternalWrench(self):
        """Clear external wrench application."""
        if self.external_wrench_active:
            zero_msg = WrenchStamped()
            zero_msg.header.stamp = rospy.Time.now()
            zero_msg.header.frame_id = "fc"
            if self.assembly_mode and hasattr(self, 'formation_wrench_pub') and self.isUnifiedMode():
                self.formation_wrench_pub.publish(zero_msg)
            else:
                self.desired_ext_wrench_pub.publish(zero_msg)
            self.external_wrench_active = False
        # Also zero per-module ŷ^task so the controller's residual
        # subtraction no longer subtracts a stale task expectation.
        if self._inter_auto_publish:
            self._publishInternalWrenchRaw(
                {mid: ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                 for mid in self._inter_module_ids})

    # ------------------------------------------------------------------
    # Task internal wrench API
    # ------------------------------------------------------------------
    def setAttachModule(self, module_id, module_masses=None,
                        module_positions=None, module_inertias_diag=None):
        """Declare an external load is bolted to one module and enable
        per-module ŷ^task auto-publishing.

        Decomposition model is selected by the data provided:
          * mass-ratio (Level 1) if positions+inertias are both missing;
          * spatial-inertia (Level 2) otherwise (use this whenever the task
            wrench has non-trivial torque, e.g. valve rotation).

        All vectors/tensors are in the formation body frame `fc` at
        formation CoG. Module body frames are assumed axis-aligned with the
        formation frame (R_i = I); update _decomposeTaskWrench to apply R_i
        if a non-aligned assembly is introduced.

        Parameters
        ----------
        module_id : int or None
            Opt-in marker only (uniform formula across modules). Pass None
            to disable auto-publishing and broadcast a final zero.
        module_masses : dict[int, float] or None
            Per-module mass in kg. Defaults to uniform `self.mass`.
        module_positions : dict[int, list[float]] or None
            Per-module CoG position [x,y,z] (m) in the formation frame.
            Will be auto-recentered to the mass-weighted centroid so the
            caller may use any consistent origin.
        module_inertias_diag : dict[int, list[float]] or None
            Per-module diagonal inertia [Ixx,Iyy,Izz] (kg·m^2) about its own
            CoG, expressed in the module body frame.
        """
        if module_id is None:
            self._inter_attach_module_id = None
            self._inter_auto_publish = False
            # Best-effort clear so a previously running auto-publish
            # doesn't leave stale ŷ^task on the bus.
            self._publishInternalWrenchRaw(
                {mid: ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                 for mid in self._inter_module_ids})
            rospy.loginfo("[BeetleInterface] est_wrench_task auto-publish DISABLED")
            return
        if module_id not in self._inter_module_ids:
            rospy.logwarn(
                "[BeetleInterface] setAttachModule(%d) but module not in "
                "assembly_ids=%s — est_wrench_task publish skipped",
                module_id, self._inter_module_ids)
            return
        self._inter_attach_module_id = int(module_id)
        if module_masses:
            self._inter_module_masses = {int(k): float(v) for k, v in module_masses.items()}
        else:
            # Default: uniform self.mass for every known module.
            self._inter_module_masses = {mid: float(self.mass)
                                         for mid in self._inter_module_ids}
        # Optional geometry for Level 2 spatial-inertia decomposition.
        self._inter_module_positions = (
            {int(k): [float(c) for c in v] for k, v in module_positions.items()}
            if module_positions else {})
        self._inter_module_inertias_diag = (
            {int(k): [float(c) for c in v] for k, v in module_inertias_diag.items()}
            if module_inertias_diag else {})
        self._precomputeTaskDecomposition()
        self._inter_auto_publish = True
        level = 2 if (self._inter_module_positions and
                      self._inter_module_inertias_diag) else 1
        rospy.loginfo(
            "[BeetleInterface] est_wrench_task auto-publish ENABLED "
            "(level=%d, attach_module=%d, masses=%s)",
            level, self._inter_attach_module_id, self._inter_module_masses)

    def setInternalWrenchPerModule(self, per_module, frame_id="fc"):
        """Explicit advanced API: directly publish ŷ^task for each module.

        Parameters
        ----------
        per_module : dict[int, (force3, torque3)]
            Body-frame 6D wrench to assign to each module.
        frame_id : str
            Header frame id (defaults to formation body 'fc').
        """
        norm = {}
        for mid, ft in per_module.items():
            f = self._to_list3(ft[0]) or [0.0, 0.0, 0.0]
            t = self._to_list3(ft[1]) or [0.0, 0.0, 0.0]
            norm[int(mid)] = (f, t)
        self._publishInternalWrenchRaw(norm, frame_id=frame_id)

    # --- internal helpers -------------------------------------------------
    def _precomputeTaskDecomposition(self):
        """Cache m_total, mass-centred positions and M_form^{-1} (block-diag
        translation/rotation inertia at formation CoG) so per-call work in
        addExternalWrench is just two 3x3 matrix-vector multiplies."""
        self._inter_m_total = float(sum(self._inter_module_masses.values()))
        self._inter_r_centered = {}
        self._inter_M_form_inv = None
        if self._inter_m_total <= 1e-6:
            return
        if not (self._inter_module_positions and self._inter_module_inertias_diag):
            return  # Level-1 path; no further caching needed.
        # Mass-weighted centroid; recenter so Σ m_i r_i = 0.
        ids = self._inter_module_ids
        r_cog = np.zeros(3)
        for mid in ids:
            m_i = self._inter_module_masses.get(mid, 0.0)
            r_i = np.array(self._inter_module_positions.get(mid, [0.0, 0.0, 0.0]))
            r_cog += m_i * r_i
        r_cog /= self._inter_m_total
        # Build I_form at formation CoG (parallel-axis on diagonals; assumes
        # module body frames axis-aligned with formation frame).
        I_form = np.zeros((3, 3))
        for mid in ids:
            m_i = self._inter_module_masses.get(mid, 0.0)
            r_i = np.array(self._inter_module_positions.get(mid, [0.0, 0.0, 0.0])) - r_cog
            self._inter_r_centered[mid] = r_i
            I_i = np.diag(self._inter_module_inertias_diag.get(mid, [0.0, 0.0, 0.0]))
            I_form += I_i + m_i * (np.dot(r_i, r_i) * np.eye(3) - np.outer(r_i, r_i))
        try:
            self._inter_M_form_inv = np.linalg.inv(I_form)
        except np.linalg.LinAlgError:
            rospy.logwarn("[BeetleInterface] I_form singular; falling back to mass-ratio")
            self._inter_M_form_inv = None

    def _decomposeTaskWrench(self, force, torque):
        """Return {mid: (F_i_body, τ_i_body)} for the body-frame external
        wrench (F, τ) applied at formation CoG.

        Level 2 (spatial inertia) used when geometry is cached; else falls
        back to Level 1 (mass ratio). Module body frames are assumed
        axis-aligned with the formation frame.
        """
        if self._inter_m_total <= 1e-6:
            return {}
        F = np.asarray(force, dtype=float)
        tau = np.asarray(torque, dtype=float)
        per_module = {}
        if self._inter_M_form_inv is None:
            # Level 1: mass-ratio split (force AND torque). OK only for tasks
            # without torque — retained for backward compatibility / towing.
            if np.linalg.norm(tau) > 1e-6:
                rospy.logwarn_throttle(
                    2.0,
                    "[BeetleInterface] task wrench has torque but Level-2 "
                    "module geometry is unavailable; mass-ratio torque split is approximate")
            for mid in self._inter_module_ids:
                share = self._inter_module_masses.get(mid, 0.0) / self._inter_m_total
                per_module[mid] = ((share * F).tolist(), (share * tau).tolist())
            return per_module
        # Level 2: physically consistent spatial-inertia decomposition.
        a = F / self._inter_m_total
        alpha = self._inter_M_form_inv @ tau
        for mid in self._inter_module_ids:
            m_i = self._inter_module_masses.get(mid, 0.0)
            r_i = self._inter_r_centered.get(mid, np.zeros(3))
            I_i = np.array(self._inter_module_inertias_diag.get(mid, [0.0, 0.0, 0.0]))
            F_i = m_i * (a + np.cross(alpha, r_i))
            tau_i_Ci = I_i * alpha  # diagonal I_i times alpha (componentwise)
            per_module[mid] = (F_i.tolist(), tau_i_Ci.tolist())
        return per_module

    def _publishInternalWrenchFromExternal(self, force_body, torque_body, frame_id):
        """Decompose the body-frame external wrench at formation CoG into
        per-module observer task prediction ŷ^task and publish."""
        per_module = self._decomposeTaskWrench(force_body, torque_body)
        if not per_module:
            return
        # Optional self-check: Σ W_i^{task,F} == W_ext^F at formation CoG.
        # Only runs at DEBUG verbosity; cheap (constant work per call).
        if rospy.get_param('~debug_task_wrench_check', False):
            F_sum = np.zeros(3)
            tau_sum = np.zeros(3)
            for mid, (f, t) in per_module.items():
                r_i = self._inter_r_centered.get(mid, np.zeros(3))
                F_sum += np.asarray(f)
                tau_sum += np.cross(r_i, np.asarray(f)) + np.asarray(t)
            err = np.concatenate([F_sum - np.asarray(force_body),
                                  tau_sum - np.asarray(torque_body)])
            if np.linalg.norm(err) > 1e-6:
                rospy.logwarn_throttle(2.0,
                    "[BeetleInterface] ΣW_i^task != W_ext, residual=%s", err)
        self._publishInternalWrenchRaw(per_module, frame_id=frame_id)

    def _publishInternalWrenchRaw(self, per_module, frame_id="fc"):
        """Publish raw per-module ŷ^task and cache last values."""
        stamp = rospy.Time.now()
        for mid, (f, t) in per_module.items():
            pub = self._est_wrench_task_pubs.get(mid)
            if pub is None:
                continue
            msg = TaggedWrench()
            msg.index = int(mid)
            msg.wrench.header.stamp = stamp
            msg.wrench.header.frame_id = frame_id
            msg.wrench.wrench.force.x = f[0]
            msg.wrench.wrench.force.y = f[1]
            msg.wrench.wrench.force.z = f[2]
            msg.wrench.wrench.torque.x = t[0]
            msg.wrench.wrench.torque.y = t[1]
            msg.wrench.wrench.torque.z = t[2]
            pub.publish(msg)
            self._last_est_wrench_task[mid] = (list(f), list(t))

    
    def executeTrajectoryWithWrench(self, pos, rot, linear_vel, angular_vel, force, torque):
        """Combined SE(3) control with external wrench feedforward."""
        self.addExternalWrench(force, torque)
        self.targetMotion(pos, rot, linear_vel, angular_vel)
        return True
    
    def goPoseWaitConvergence(self, pos, rot=None, pos_thresh=None, vel_thresh=0.1, 
                               rot_thresh=None, timeout=30, check_func=None):
        """Move to position and wait for convergence."""
        pos_thresh = pos_thresh or self.default_pos_thresh
        rot_thresh = rot_thresh or self.default_rot_thresh
        check_func = check_func or self.posYawConvergenceCheck
        
        self.targetMotion(pos, rot=rot)
        start_time = rospy.get_time()
        
        while not check_func(pos, rot, pos_thresh, vel_thresh, rot_thresh):
            if timeout > 0 and (rospy.get_time() - start_time) > timeout:
                return False
            if self.force_skip:
                self.force_skip = False
                return True
            if self.halt_task or rospy.is_shutdown():
                return False
            rospy.sleep(0.1)
        return True
    
    def posYawConvergenceCheck(self, target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh):
        """Check if position and yaw have converged."""
        if isinstance(pos_thresh, list):
            pos_thresh = pos_thresh[2] if len(pos_thresh) >= 3 else self.default_pos_thresh
        if isinstance(rot_thresh, list):
            rot_thresh = rot_thresh[2] if len(rot_thresh) >= 3 else self.default_rot_thresh
        
        current_pos = self.getUavPos()
        current_yaw = self.getUavRPY()[2]
        
        if target_rot is None:
            target_yaw = current_yaw
        elif isinstance(target_rot, (list, tuple, np.ndarray)) and len(target_rot) == 4:
            target_yaw = euler_from_quaternion(target_rot)[2]
        else:
            target_yaw = target_rot
        
        delta_pos = target_pos - current_pos
        delta_yaw = target_yaw - current_yaw
        
        # Normalize yaw to [-pi, pi]
        while delta_yaw > np.pi:
            delta_yaw -= 2 * np.pi
        while delta_yaw < -np.pi:
            delta_yaw += 2 * np.pi
        
        pos_error = np.linalg.norm(delta_pos)
        yaw_error = abs(delta_yaw)
        
        if self.debug_view:
            rospy.loginfo_throttle(1.0, f'Convergence: pos={pos_error:.4f}, yaw={yaw_error:.4f}')
        
        return pos_error < pos_thresh and yaw_error < rot_thresh
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
