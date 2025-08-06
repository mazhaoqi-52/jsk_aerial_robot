#!/usr/bin/env python
"""
Joy-controlled valve rotation for manual insertion scenario
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import rospy
import smach
import smach_ros
import threading
import time
import math
from std_msgs.msg import Bool
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import numpy as np
# Import GazeboLinkAttacher for simulation UAV-valve handle attachment
from beetle.gazebo_link_attacher import GazeboLinkAttacher

# Import trajectory generation and control components
from trajectory import create_constant_distance_trajectory, ValveRotationTrajectory
from unified_motion_controller import UnifiedMotionController


class UAVStateBase(smach.State):
    """Base class for UAV state management"""
    
    def __init__(self, outcomes, input_keys=None, output_keys=None, module_id=1):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.module_id = module_id
        
        # Publishers and subscribers
        self.pub = rospy.Publisher(f"/beetle{module_id}/uav/nav", FlightNav, queue_size=1)
        self.motion_controller = UnifiedMotionController(self)
        
        # Trajectory visualization publishers
        self.trajectory_pub = rospy.Publisher(f"/beetle{module_id}/trajectory_visualization", MarkerArray, queue_size=1)
        self.current_point_pub = rospy.Publisher(f"/beetle{module_id}/current_trajectory_point", Marker, queue_size=1)
        self.valve_marker_pub = rospy.Publisher(f"/beetle{module_id}/valve_marker", Marker, queue_size=1)
        
        # Position and orientation data
        self.uav_pos = None
        self.current_yaw = 0.0
        self.valve_pos = None
        self.valve_yaw = 0.0
        
        # Thread events for data synchronization
        self.uav_received = threading.Event()
        self.valve_received = threading.Event()
        
        # Get simulation and real_machine parameters
        self.simulation = rospy.get_param("~simulation", False)

        # In simulation mode, automatically attach UAV to valve handle using GazeboLinkAttacher
        if self.simulation:
            try:
                from beetle.gazebo_link_attacher import GazeboLinkAttacher
                attacher = GazeboLinkAttacher()
                # UAV模型名和link名需根据实际仿真模型调整
                uav_model = f"beetle{module_id}"  # 假定无人机模型名
                uav_link = "beetle_fang_link"     # 假定无人机主link
                valve_model = "valve"              # 阀门模型名
                valve_link = "handle"              # 阀门handle link
                rospy.loginfo(f"Attaching {uav_model}:{uav_link} to {valve_model}:{valve_link} via GazeboLinkAttacher...")
                attacher.attach(uav_model, uav_link, valve_model, valve_link)
                rospy.loginfo("Gazebo link attacher: UAV and valve handle attached.")
            except Exception as e:
                rospy.logwarn(f"Gazebo link attacher failed: {e}")
        
        # Setup subscribers
        rospy.loginfo(f"Subscribing to UAV pose: /beetle{module_id}/mocap/pose")
        self.uav_sub = rospy.Subscriber(f"/beetle{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        
        # Setup valve pose subscriber based on mode
        if self.simulation:
            rospy.loginfo("Subscribing to valve pose (simulation): /valve/odom")
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            rospy.loginfo("Subscribing to valve pose (real): /valve/mocap/pose")
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        # Control parameters
        self.position_threshold = 0.02  # Stricter position control
        self.yaw_threshold = 0.03      # Stricter orientation control
        self.timeout = 60.0
        
        rospy.loginfo(f"Initialized UAV{module_id} state with strict control parameters")
    
    def create_valve_marker(self):
        """Create valve position marker for visualization"""
        if self.valve_pos is None:
            return None
        
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"valve_{self.module_id}"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Valve position
        marker.pose.position.x = self.valve_pos[0]
        marker.pose.position.y = self.valve_pos[1]
        marker.pose.position.z = self.valve_pos[2]
        
        # Valve orientation (if available)
        if hasattr(self, 'valve_yaw') and self.valve_yaw is not None:
            from tf.transformations import quaternion_from_euler
            q = quaternion_from_euler(0, 0, self.valve_yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
        else:
            marker.pose.orientation.w = 1.0
        
        # Valve size (typical valve dimensions)
        marker.scale.x = 0.245  # Diameter
        marker.scale.y = 0.245  # Diameter
        marker.scale.z = 0.037  # Height
        
        # Red color for valve
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        return marker
    
    def create_current_point_marker(self, position, target_yaw):
        """Create current trajectory point marker"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"current_point_{self.module_id}"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Current position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        # Current orientation
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, target_yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Arrow size
        marker.scale.x = 0.1   # Length
        marker.scale.y = 0.02  # Width
        marker.scale.z = 0.02  # Height
        
        # Green color for current point
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker
    
    def uav_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.uav_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw
        
        self.uav_received.set()
    
    def valve_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.valve_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.valve_yaw = yaw
        
        self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.valve_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.valve_yaw = yaw
        
        self.valve_received.set()
    
    def wait_for_positions(self, timeout=10.0):
        """Wait for UAV and valve positions"""
        rospy.loginfo("Waiting for UAV and valve positions...")
        start_time = time.time()
        
        while not rospy.is_shutdown():
            if self.uav_received.is_set() and self.valve_received.is_set():
                rospy.loginfo("Received both UAV and valve positions")
                return True
            
            if time.time() - start_time > timeout:
                rospy.logerr("Timeout waiting for positions")
                return False
            
            time.sleep(0.1)
        
        return False
    
    def get_current_position(self):
        """Get current UAV position"""
        return self.uav_pos
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def publish_trajectory_visualization(self):
        """Publish trajectory visualization markers"""
        if not self.trajectory_points:
            return
        
        marker_array = MarkerArray()
        
        # Create trajectory path as LINE_STRIP
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = f"trajectory_path_{self.module_id}"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        # Path line properties
        path_marker.scale.x = 0.01  # Line width
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 1.0
        path_marker.color.a = 0.8
        
        # Add trajectory points to path
        for i, (pos, yaw) in enumerate(self.trajectory_points):
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = pos[2]
            path_marker.points.append(point)
        
        marker_array.markers.append(path_marker)
        
        # Create orientation arrows at key points
        arrow_interval = max(1, len(self.trajectory_points) // 20)  # Show ~20 arrows
        for i in range(0, len(self.trajectory_points), arrow_interval):
            pos, yaw = self.trajectory_points[i]
            
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "world"
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = f"trajectory_arrows_{self.module_id}"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Arrow position
            arrow_marker.pose.position.x = pos[0]
            arrow_marker.pose.position.y = pos[1]
            arrow_marker.pose.position.z = pos[2]
            
            # Arrow orientation
            from tf.transformations import quaternion_from_euler
            q = quaternion_from_euler(0, 0, yaw)
            arrow_marker.pose.orientation.x = q[0]
            arrow_marker.pose.orientation.y = q[1]
            arrow_marker.pose.orientation.z = q[2]
            arrow_marker.pose.orientation.w = q[3]
            
            # Arrow size
            arrow_marker.scale.x = 0.05  # Length
            arrow_marker.scale.y = 0.01  # Width
            arrow_marker.scale.z = 0.01  # Height
            
            # Blue color for trajectory arrows
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.5
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 0.6
            
            marker_array.markers.append(arrow_marker)
        
        # Publish trajectory visualization
        self.trajectory_pub.publish(marker_array)


class TrajectoryPreviewState(UAVStateBase):
    """
    Trajectory preview state - only shows trajectory visualization without actual control
    """
    
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], module_id=module_id)
        
        # Preview parameters
        self.rotation_angle = rospy.get_param('~rotation_angle', math.pi/2)  # 90 degrees default
        self.rotation_speed = rospy.get_param('~rotation_speed', 0.05)  # 0.05 m/s
        self.preview_duration = rospy.get_param('~preview_duration', 30.0)  # 30 seconds preview
        
        # Override trajectory publishers with preview topics
        self.trajectory_pub = rospy.Publisher(f"/beetle{module_id}/trajectory_preview", MarkerArray, queue_size=1)
        self.valve_marker_pub = rospy.Publisher(f"/beetle{module_id}/valve_preview", Marker, queue_size=1)
        self.uav_start_pub = rospy.Publisher(f"/beetle{module_id}/uav_start_preview", Marker, queue_size=1)
        
        # Calculate rotation duration based on speed
        typical_radius = 0.15
        circumference = 2 * math.pi * typical_radius
        rotation_fraction = abs(self.rotation_angle) / (2 * math.pi)
        self.rotation_duration = (circumference * rotation_fraction) / self.rotation_speed
        
        rospy.loginfo(f"Trajectory preview parameters:")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f}rad ({self.rotation_angle*180/math.pi:.1f}°)")
        rospy.loginfo(f"  Rotation speed: {self.rotation_speed:.3f}m/s")
        rospy.loginfo(f"  Preview duration: {self.preview_duration:.1f}s")
        
    def execute(self, userdata):
        """Execute trajectory preview - show visualization only"""
        rospy.loginfo("Starting trajectory preview...")
        
        # Wait for position data
        if not self.wait_for_positions(timeout=5.0):
            rospy.logerr("Failed to get position data for preview")
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Unable to get current UAV position for preview")
            return 'failed'
        
        rospy.loginfo(f"UAV start position: {current_pos}")
        rospy.loginfo(f"UAV start yaw: {self.current_yaw:.3f}rad ({self.current_yaw*180/math.pi:.1f}°)")
        rospy.loginfo(f"Valve position: {self.valve_pos}")
        
        # Create rotation trajectory
        rotation_trajectory = create_constant_distance_trajectory(
            current_uav_pos=current_pos,
            current_uav_yaw=self.current_yaw,
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.246,
            end_effector_offset_y=0.0,
            end_effector_offset_z=0.0743823,
            rotation_direction=1
        )
        
        if rotation_trajectory is None:
            rospy.logerr("Failed to create rotation trajectory for preview")
            return 'failed'
        
        # Generate all trajectory points for preview
        rotation_trajectory.start_trajectory()
        self.trajectory_points = []
        num_points = 2000  # number of trajectory points
        duration = rotation_trajectory.rotation_duration
        time_steps = np.linspace(0, duration, num_points)
        start_time = rospy.Time.now().to_sec()
        for t in time_steps:
            rotation_trajectory._angle_trajectory.start_time = start_time - t
            result = rotation_trajectory.get_next_position_and_yaw()
            if result is None:
                break
            target_pos, target_yaw = result
            self.trajectory_points.append((target_pos, target_yaw))
            if len(self.trajectory_points) > 2000:
                break
        rospy.loginfo(f"Generated {len(self.trajectory_points)} trajectory points for preview")
        # Publish static visualizations
        self.publish_preview_visualization(current_pos)
        
        # Keep publishing for preview duration
        rate = rospy.Rate(1)  # 1Hz
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < self.preview_duration:
            self.publish_preview_visualization(current_pos)
            rate.sleep()
        
        rospy.loginfo("Trajectory preview completed")
        return 'succeeded'
    
    def publish_preview_visualization(self, start_pos):
        """Publish all preview visualizations"""
        # Publish trajectory
        self.publish_trajectory_visualization()
        
        # Publish valve marker
        valve_marker = self.create_valve_marker()
        if valve_marker:
            self.valve_marker_pub.publish(valve_marker)
        
        # Publish UAV start position marker
        start_marker = self.create_uav_start_marker(start_pos)
        if start_marker:
            self.uav_start_pub.publish(start_marker)
    
    def create_uav_start_marker(self, start_pos):
        """Create UAV start position marker"""
        if start_pos is None:
            return None
        
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"uav_start_{self.module_id}"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Start position
        marker.pose.position.x = start_pos[0]
        marker.pose.position.y = start_pos[1]
        marker.pose.position.z = start_pos[2]
        
        # Start orientation
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.current_yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Size
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Yellow color for start position
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker


class SlowValveRotationState(UAVStateBase):
    """
    Slow valve rotation state for precise control
    UAV revolves around valve center while maintaining orientation towards valve center
    """
    
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], module_id=module_id)
        
        # Rotation parameters - optimized for slow, precise control
        self.rotation_angle = rospy.get_param('~rotation_angle', math.pi/2)  # 90 degrees default
        self.rotation_speed = rospy.get_param('~rotation_speed', 0.05)  # 0.05 m/s as requested
        
        # Calculate rotation duration based on speed
        # Assuming circular motion with typical radius of ~0.15m
        typical_radius = 0.15
        circumference = 2 * math.pi * typical_radius
        rotation_fraction = abs(self.rotation_angle) / (2 * math.pi)
        self.rotation_duration = (circumference * rotation_fraction) / self.rotation_speed
        
        rospy.loginfo(f"Slow valve rotation parameters:")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f}rad ({self.rotation_angle*180/math.pi:.1f}°)")
        rospy.loginfo(f"  Rotation speed: {self.rotation_speed:.3f}m/s")
        rospy.loginfo(f"  Estimated duration: {self.rotation_duration:.1f}s")
        
        # Feedback control parameters for strict trajectory following
        self.feedback_frequency = rospy.get_param('~feedback_frequency', 30)  # 30Hz for smooth control
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.015)  # 1.5cm tolerance
        self.yaw_tolerance = rospy.get_param('~yaw_tolerance', 0.02)  # ~1.2 degree tolerance
        
        # Emergency detection parameters
        self.emergency_stop = threading.Event()
        self.max_position_error = 0.08  # Maximum allowed position error
        self.max_yaw_error = 0.15       # Maximum allowed yaw error
        
        # Trajectory visualization parameters
        self.trajectory_points = []  # Store trajectory points for visualization
        self.max_trajectory_points = 1000  # Maximum points to store
        self.publish_trajectory_interval = 10  # Publish every N points
        
    def execute(self, userdata):
        """Execute slow valve rotation with strict feedback control"""
        rospy.loginfo("Starting slow valve rotation with strict feedback control...")
        
        # Wait for position data
        if not self.wait_for_positions(timeout=5.0):
            rospy.logerr("Failed to get position data")
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Unable to get current UAV position")
            return 'failed'
        
        rospy.loginfo(f"Current UAV position: {current_pos}")
        rospy.loginfo(f"Current UAV yaw: {self.current_yaw:.3f}rad ({self.current_yaw*180/math.pi:.1f}°)")
        rospy.loginfo(f"Valve position: {self.valve_pos}")
        
        # Create rotation trajectory
        rotation_trajectory = create_constant_distance_trajectory(
            current_uav_pos=current_pos,
            current_uav_yaw=self.current_yaw,
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.246,
            end_effector_offset_y=0.0,
            end_effector_offset_z=0.0743823,
            rotation_direction=1
        )
        
        if rotation_trajectory is None:
            rospy.logerr("Failed to create rotation trajectory")
            return 'failed'
        
        # Start trajectory execution
        rotation_trajectory.start_trajectory()
        
        # Clear previous trajectory points
        self.trajectory_points.clear()
        
        # Publish valve marker
        valve_marker = self.create_valve_marker()
        if valve_marker:
            self.valve_marker_pub.publish(valve_marker)
        
        # Start emergency monitoring
        emergency_thread = threading.Thread(target=self._emergency_monitor)
        emergency_thread.daemon = True
        emergency_thread.start()
        
        # Execute rotation with strict feedback control
        try:
            success = self.execute_strict_rotation(rotation_trajectory)
            
            # Stop emergency monitoring
            self.emergency_stop.set()
            
            if self.emergency_stop.is_set():
                rospy.logwarn("Emergency stop triggered during rotation")
                return 'failed'
            
            if success:
                rospy.loginfo("Slow valve rotation completed successfully")
                return 'succeeded'
            else:
                rospy.logerr("Slow valve rotation failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during rotation: {e}")
            self.emergency_stop.set()
            return 'failed'
    
    def execute_strict_rotation(self, trajectory):
        """Execute rotation with real-time trajectory adjustment, relaxing yaw error threshold for initial steps"""
        rate = rospy.Rate(self.feedback_frequency)
        start_time = time.time()
        point_counter = 0
        rospy.loginfo("Starting strict rotation trajectory execution with real-time adjustment...")

        # Calculate the angle for each rotation step
        total_steps = int(self.rotation_duration * self.feedback_frequency)
        if total_steps < 1:
            total_steps = 1
        angle_step = self.rotation_angle / total_steps
        duration_step = self.rotation_duration / total_steps
        step_count = 0

        # Relax yaw error threshold for initial steps
        initial_relax_steps = max(20, int(0.05 * total_steps))  # Relax for first 5 steps or 5% of total
        relaxed_yaw_error = max(0.5, self.max_yaw_error * 2)   # Use a larger threshold for initial alignment
        strict_yaw_error = self.max_yaw_error

        while not rospy.is_shutdown() and not self.emergency_stop.is_set():
            # Get current UAV position and yaw
            current_pos = self.get_current_position()
            current_yaw = self.current_yaw
            if current_pos is None:
                rospy.logwarn("Unable to get current position")
                rate.sleep()
                continue

            # Dynamically generate the next target point (rotate by angle_step each time)
            temp_traj = create_constant_distance_trajectory(
                current_uav_pos=current_pos,
                current_uav_yaw=current_yaw,
                valve_center=self.valve_pos,
                rotation_angle=angle_step,
                rotation_duration=duration_step,
                end_effector_offset_x=0.246,
                end_effector_offset_y=0.0,
                end_effector_offset_z=0.0743823,
                rotation_direction=1
            )
            if temp_traj is None:
                rospy.logerr("Failed to create real-time trajectory step")
                self.emergency_stop.set()
                break
            temp_traj.start_trajectory()
            result = temp_traj.get_next_position_and_yaw()
            if result is None:
                rospy.loginfo("Trajectory completed")
                break
            target_pos, target_yaw = result

            # Store trajectory point for visualization
            self.trajectory_points.append((target_pos, target_yaw))
            if len(self.trajectory_points) > self.max_trajectory_points:
                self.trajectory_points.pop(0)

            # Visualization
            if point_counter % self.publish_trajectory_interval == 0:
                self.publish_trajectory_visualization()
            current_marker = self.create_current_point_marker(target_pos, target_yaw)
            self.current_point_pub.publish(current_marker)

            # Error checking
            pos_error = math.sqrt(
                (target_pos[0] - current_pos[0])**2 +
                (target_pos[1] - current_pos[1])**2 +
                (target_pos[2] - current_pos[2])**2
            )
            yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))

            # Use relaxed yaw error for initial steps, strict for later
            if step_count < initial_relax_steps:
                yaw_error_threshold = relaxed_yaw_error
            else:
                yaw_error_threshold = strict_yaw_error

            if pos_error > self.max_position_error:
                rospy.logerr(f"Position error too large: {pos_error:.3f}m > {self.max_position_error:.3f}m")
                self.emergency_stop.set()
                break
            if yaw_error > yaw_error_threshold:
                rospy.logerr(f"Yaw error too large: {yaw_error:.3f}rad > {yaw_error_threshold:.3f}rad (step {step_count})")
                self.emergency_stop.set()
                break

            # Command UAV to track the target point
            self.motion_controller.send_trajectory_point(target_pos, target_yaw)

            # Logging
            if int(time.time() - start_time) % 2 == 0:
                rospy.loginfo(f"Rotation progress: pos_error={pos_error:.3f}m, yaw_error={yaw_error:.3f}rad, points={len(self.trajectory_points)}, step={step_count}")

            point_counter += 1
            step_count += 1
            if step_count >= total_steps:
                rospy.loginfo("All rotation steps completed")
                break
            rate.sleep()

        # Publish final trajectory visualization at the end
        self.publish_trajectory_visualization()
        return True
    
    def _emergency_monitor(self):
        """Monitor for emergency conditions"""
        rate = rospy.Rate(2)  # 2Hz monitoring
        
        while not self.emergency_stop.is_set() and not rospy.is_shutdown():
            current_pos = self.get_current_position()
            if current_pos is None:
                rate.sleep()
                continue
            
            # Check if UAV is too far from valve (collision avoidance)
            if self.valve_pos is not None:
                distance_to_valve = math.sqrt(
                    (current_pos[0] - self.valve_pos[0])**2 + 
                    (current_pos[1] - self.valve_pos[1])**2 + 
                    (current_pos[2] - self.valve_pos[2])**2
                )
                
                # Emergency if too close or too far
                if distance_to_valve < 0.05:  # Too close
                    rospy.logerr(f"UAV too close to valve: {distance_to_valve:.3f}m")
                    self.emergency_stop.set()
                elif distance_to_valve > 0.5:  # Too far
                    rospy.logerr(f"UAV too far from valve: {distance_to_valve:.3f}m")
                    self.emergency_stop.set()
            
            rate.sleep()


class ValveRotationController:
    """
    Simplified valve rotation controller for manual insertion scenario
    Designed for post-insertion valve rotation, joystick control removed, focused on core rotation functionality
    Supports preview mode: preview_only=true only shows trajectory visualization, no actual control
    """
    
    def __init__(self):
        rospy.init_node('valve_rotation_controller')
        
        # Get parameters
        self.module_id = rospy.get_param('~module_id', 1)
        self.auto_start = rospy.get_param('~auto_start', True)  # 改为默认自动开始
        self.preview_only = rospy.get_param('~preview_only', False)  # 预览模式
        
        # Status publisher
        self.status_pub = rospy.Publisher('valve_rotation_status', Bool, queue_size=1)
        
        # Create state machine
        if self.preview_only:
            self.create_preview_state_machine()
        else:
            self.create_state_machine()
        
        if self.preview_only:
            rospy.loginfo("Valve rotation controller initialized in PREVIEW mode")
            rospy.loginfo("Will only display trajectory visualization without actual control")
        else:
            rospy.loginfo("Valve rotation controller initialized")
            rospy.loginfo("Ready to start valve rotation")
        
    def create_state_machine(self):
        """Create simplified state machine for valve rotation only"""
        self.sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        
        with self.sm:
            smach.StateMachine.add(
                'ROTATE_VALVE',
                SlowValveRotationState(module_id=self.module_id),
                transitions={
                    'succeeded': 'succeeded',
                    'failed': 'failed'
                }
            )
    
    def create_preview_state_machine(self):
        """Create preview state machine that only shows trajectory visualization"""
        self.sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        
        with self.sm:
            smach.StateMachine.add(
                'PREVIEW_TRAJECTORY',
                TrajectoryPreviewState(module_id=self.module_id),
                transitions={
                    'succeeded': 'succeeded',
                    'failed': 'failed'
                }
            )
    
    def run(self):
        """Main execution loop"""
        if self.preview_only:
            rospy.loginfo("Starting trajectory preview...")
            rospy.loginfo("Will display trajectory visualization for preview")
        else:
            rospy.loginfo("Starting valve rotation...")
            rospy.loginfo("Make sure the UAV is properly inserted into the valve before starting")
        
        # 直接执行状态机
        outcome = self.sm.execute()
        
        if outcome == 'succeeded':
            if self.preview_only:
                rospy.loginfo("Trajectory preview completed successfully")
            else:
                rospy.loginfo("Valve rotation completed successfully")
            self.status_pub.publish(Bool(data=True))
            return True
        else:
            if self.preview_only:
                rospy.logerr("Trajectory preview failed")
            else:
                rospy.logerr("Valve rotation failed")
            self.status_pub.publish(Bool(data=False))
            return False


def main():
    try:
        controller = ValveRotationController()
        success = controller.run()
        
        if success:
            rospy.loginfo("Valve rotation completed successfully")
        else:
            rospy.logerr("Valve rotation failed")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Valve rotation interrupted")
    except Exception as e:
        rospy.logerr(f"Valve rotation error: {e}")


if __name__ == '__main__':
    main()
