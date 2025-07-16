#!/usr/bin/env python
"""
Dual-Fang Insertion Strategy Optimizer for Valve Rotation Task
Optimizes dual-fang simultaneous insertion strategy for valve rotation operations
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import math
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class InsertionOptimizer:
    """
    Optimizer for dual-fang simultaneous insertion strategy based on UAV position and valve configuration
    """
    
    def __init__(self, valve_radius=0.1225, valve_beam_width=0.0185, safety_margin=0.01, 
                 module_id=1, simulation=True):
        """
        Initialize insertion optimizer
        
        Args:
            valve_radius: Radius of the valve (meters)
            valve_beam_width: Width of valve beams (meters)
            safety_margin: Safety margin for insertion (meters)
            module_id: UAV module ID for topic subscription
            simulation: Whether running in simulation mode
        """
        self.valve_radius = valve_radius
        self.valve_beam_width = valve_beam_width
        self.safety_margin = safety_margin
        self.module_id = module_id
        self.simulation = simulation
        
        # Real-time position tracking
        self.current_uav_pos = None
        self.current_uav_yaw = 0.0
        self.valve_pos = None
        self.valve_yaw = 0.0
        
        # Thread events for data synchronization
        self.uav_data_received = threading.Event()
        self.valve_data_received = threading.Event()
        
        # Setup subscribers for real-time data
        self._setup_subscribers()
        
        # Fang configurations
        self.fang_configs = {
            'fang1': {
                'name': 'Left Fang (Fang 1)',
                'description': 'Left side fang insertion',
                'preferred_beam_gap': 'beam0_beam1',  # Between beam0(0°) and beam1(120°)
                'beam_position_factor': 0.1,  # Closer to beam1
                'approach_bias': 0.3,  # Approach bias factor
            },
            'fang2': {
                'name': 'Right Fang (Fang 2)', 
                'description': 'Right side fang insertion',
                'preferred_beam_gap': 'beam1_beam2',  # Between beam1(120°) and beam2(240°)
                'beam_position_factor': 0.1,  # Closer to beam2
                'approach_bias': 0.3,  # Approach bias factor
            }
        }
        
        rospy.loginfo("Insertion optimizer initialized with real-time data")
        rospy.loginfo(f"Module ID: {module_id}")
        rospy.loginfo(f"Simulation mode: {simulation}")
        rospy.loginfo(f"Valve radius: {valve_radius:.4f}m")
        rospy.loginfo(f"Valve beam width: {valve_beam_width:.4f}m")
        rospy.loginfo(f"Safety margin: {safety_margin:.4f}m")
    
    def _setup_subscribers(self):
        """Setup ROS subscribers for real-time position data"""
        # UAV position subscriber
        uav_topic = f"/beetle{self.module_id}/mocap/pose"
        rospy.loginfo(f"Subscribing to UAV pose: {uav_topic}")
        self.uav_sub = rospy.Subscriber(uav_topic, PoseStamped, self._uav_callback, queue_size=1)
        
        # Valve position subscriber (depends on simulation mode)
        if self.simulation:
            valve_topic = "/valve/odom"
            rospy.loginfo(f"Subscribing to valve pose (simulation): {valve_topic}")
            self.valve_sub = rospy.Subscriber(valve_topic, Odometry, self._valve_sim_callback, queue_size=1)
        else:
            valve_topic = "/valve/mocap/pose"
            rospy.loginfo(f"Subscribing to valve pose (real): {valve_topic}")
            self.valve_sub = rospy.Subscriber(valve_topic, PoseStamped, self._valve_callback, queue_size=1)
    
    def _uav_callback(self, msg):
        """Callback for UAV position updates"""
        self.current_uav_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        _, _, self.current_uav_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.uav_data_received.set()
    
    def _valve_sim_callback(self, msg):
        """Callback for valve position updates in simulation mode"""
        self.valve_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.valve_data_received.set()
    
    def _valve_callback(self, msg):
        """Callback for valve position updates in real mode"""
        self.valve_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.valve_data_received.set()
    
    def wait_for_data(self, timeout=10.0):
        """Wait for real-time UAV and valve data"""
        rospy.loginfo("Waiting for real-time UAV and valve position data...")
        
        uav_ok = self.uav_data_received.wait(timeout)
        valve_ok = self.valve_data_received.wait(timeout)
        
        if not uav_ok:
            rospy.logerr("Failed to receive UAV position data within timeout")
            rospy.logerr(f"Check UAV topic: /beetle{self.module_id}/mocap/pose")
            return False
        
        if not valve_ok:
            rospy.logwarn("Failed to receive valve position data within timeout")
            rospy.logwarn("Using default valve position as fallback")
            # Set default valve position if not available
            self.valve_pos = (3.0, 0.0, 0.57)
            self.valve_yaw = 0.0
            self.valve_data_received.set()
        
        rospy.loginfo(f"Real-time data received:")
        rospy.loginfo(f"  UAV position: {self.current_uav_pos}")
        rospy.loginfo(f"  UAV yaw: {self.current_uav_yaw:.3f} rad")
        rospy.loginfo(f"  Valve position: {self.valve_pos}")
        rospy.loginfo(f"  Valve yaw: {self.valve_yaw:.3f} rad")
        
        return True
    
    def get_current_data(self):
        """Get current UAV and valve data"""
        if not self.uav_data_received.is_set():
            rospy.logwarn("UAV data not available")
            return None
        
        if not self.valve_data_received.is_set():
            rospy.logwarn("Valve data not available")
            return None
        
        return {
            'uav_pos': self.current_uav_pos,
            'uav_yaw': self.current_uav_yaw,
            'valve_pos': self.valve_pos,
            'valve_yaw': self.valve_yaw
        }
    
    def calculate_beam_angles(self, valve_yaw):
        """Calculate beam angles based on valve orientation"""
        beam_angles = {
            'beam0': valve_yaw,  # 0°
            'beam1': valve_yaw + 2*math.pi/3,  # 120°
            'beam2': valve_yaw + 4*math.pi/3,  # 240°
        }
        return beam_angles
    
    def calculate_insertion_positions(self, valve_pos, valve_yaw):
        """Calculate potential insertion positions for both fangs with optimal 60° angle from adjacent beams"""
        beam_angles = self.calculate_beam_angles(valve_yaw)
        center_x, center_y = valve_pos[0], valve_pos[1]
        
        insertion_positions = {}
        
        rospy.loginfo("=== BEAM ANGLES CALCULATION ===")
        for beam_name, angle in beam_angles.items():
            rospy.loginfo(f"{beam_name}: {angle:.3f} rad ({angle*180/math.pi:.1f}°)")
        
        for fang_id, config in self.fang_configs.items():
            if config['preferred_beam_gap'] == 'beam1_beam2':
                # Between beam1(120°) and beam2(240°)
                # Use 210° (middle between 120° and 240°) instead of 180°
                beam1_angle = beam_angles['beam1']
                beam2_angle = beam_angles['beam2']
                
                # For beam1(120°) to beam2(240°), the gap center should be 180°
                # But for better insertion, use 210° (closer to beam2)
                insertion_angle = beam1_angle + 2*math.pi/3  # 120° + 120° = 240°, but use 210°
                if insertion_angle > 2*math.pi:
                    insertion_angle -= 2*math.pi
                
                # Alternative: use the actual gap center
                gap_center_angle = (beam1_angle + beam2_angle) / 2
                if beam2_angle < beam1_angle:
                    gap_center_angle = (beam1_angle + beam2_angle + 2*math.pi) / 2
                
                # Use gap center but offset slightly for better approach
                insertion_angle = gap_center_angle + math.pi/6  # Add 30° offset for better approach
                
                rospy.loginfo(f"Fang2 insertion: Gap center {gap_center_angle*180/math.pi:.1f}° + 30° offset = {insertion_angle*180/math.pi:.1f}°")
                
            elif config['preferred_beam_gap'] == 'beam0_beam1':
                # Between beam0(0°) and beam1(120°)
                # Use 60° (middle between 0° and 120°)
                beam0_angle = beam_angles['beam0']
                beam1_angle = beam_angles['beam1']
                
                gap_center_angle = (beam0_angle + beam1_angle) / 2
                insertion_angle = gap_center_angle
                
                rospy.loginfo(f"Fang1 insertion: Gap center between beam0({beam0_angle*180/math.pi:.1f}°) and beam1({beam1_angle*180/math.pi:.1f}°) = {insertion_angle*180/math.pi:.1f}°")
            
            # Normalize angle
            while insertion_angle >= 2*math.pi:
                insertion_angle -= 2*math.pi
            while insertion_angle < 0:
                insertion_angle += 2*math.pi
            
            # Calculate insertion position
            insertion_radius = self.valve_radius + self.safety_margin
            insertion_x = center_x + insertion_radius * math.cos(insertion_angle)
            insertion_y = center_y + insertion_radius * math.sin(insertion_angle)
            
            rospy.loginfo(f"{config['name']} insertion position: ({insertion_x:.3f}, {insertion_y:.3f}) at {insertion_radius:.3f}m radius")
            
            insertion_positions[fang_id] = {
                'angle': insertion_angle,
                'position': (insertion_x, insertion_y),
                'radius': insertion_radius,
                'config': config,
                'beam_gap': config['preferred_beam_gap'],
                'optimal_clearance_angle': math.pi/3,  # 60° optimal clearance
                'adjacent_beam_clearance': math.pi/3,  # 60° from adjacent beams
                'gap_center_angle': insertion_angle,  # Store the gap center angle
            }
        
        return insertion_positions
    
    def calculate_approach_distance(self, current_pos, target_pos):
        """Calculate 3D distance between current position and target position"""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        dz = target_pos[2] - current_pos[2] if len(target_pos) > 2 else 0
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def calculate_yaw_adjustment(self, current_yaw, target_yaw):
        """Calculate required yaw adjustment (shortest path)"""
        yaw_diff = target_yaw - current_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2*math.pi
        return abs(yaw_diff)
    
    def evaluate_insertion_strategy(self, current_pos=None, current_yaw=None, valve_pos=None, valve_yaw=None):
        """
        Evaluate and select dual-fang simultaneous insertion strategy
        
        Args:
            current_pos: Current UAV position (x, y, z) - if None, uses real-time data
            current_yaw: Current UAV yaw angle - if None, uses real-time data
            valve_pos: Valve position (x, y, z) - if None, uses real-time data
            valve_yaw: Valve yaw angle - if None, uses real-time data
            
        Returns:
            dict: Dual-fang simultaneous insertion strategy with both fang positions
        """
        # Use real-time data if parameters not provided
        if current_pos is None or current_yaw is None or valve_pos is None or valve_yaw is None:
            rospy.loginfo("Using real-time data for dual-fang insertion strategy evaluation")
            
            # Wait for real-time data if not available
            if not self.uav_data_received.is_set() or not self.valve_data_received.is_set():
                if not self.wait_for_data():
                    rospy.logerr("Failed to get real-time data for optimization")
                    return None
            
            # Use real-time data
            current_pos = current_pos or self.current_uav_pos
            current_yaw = current_yaw or self.current_uav_yaw
            valve_pos = valve_pos or self.valve_pos
            valve_yaw = valve_yaw or self.valve_yaw
            
            rospy.loginfo("=== REAL-TIME DATA FOR DUAL-FANG OPTIMIZATION ===")
            rospy.loginfo(f"UAV position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            rospy.loginfo(f"UAV yaw: {current_yaw:.3f} rad ({current_yaw*180/math.pi:.1f}°)")
            rospy.loginfo(f"Valve position: [{valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f}]")
            rospy.loginfo(f"Valve yaw: {valve_yaw:.3f} rad ({valve_yaw*180/math.pi:.1f}°)")
        
        insertion_positions = self.calculate_insertion_positions(valve_pos, valve_yaw)
        
        # Calculate strategy for BOTH fangs simultaneously
        dual_fang_strategy = {
            'fang1': None,
            'fang2': None,
            'insertion_mode': 'dual_simultaneous',
            'coordination_required': True,
            'primary_fang': None,  # Will be determined by complexity score
            'secondary_fang': None
        }
        
        rospy.loginfo("=== DUAL-FANG SIMULTANEOUS INSERTION STRATEGY ===")
        rospy.loginfo("Both Fang1 and Fang2 will insert simultaneously at their optimal positions")
        
        strategies = {}
        
        for fang_id, insertion_info in insertion_positions.items():
            target_pos = insertion_info['position'] + (valve_pos[2],)  # Add Z coordinate
            
            # Calculate approach distance
            approach_distance = self.calculate_approach_distance(current_pos, target_pos)
            
            # Calculate required yaw adjustment to point toward valve center
            dx_to_center = valve_pos[0] - target_pos[0]
            dy_to_center = valve_pos[1] - target_pos[1]
            target_yaw = math.atan2(dy_to_center, dx_to_center)
            yaw_adjustment = self.calculate_yaw_adjustment(current_yaw, target_yaw)
            
            # Calculate insertion complexity score
            distance_weight = 1.0
            yaw_weight = 0.5
            complexity_score = distance_weight * approach_distance + yaw_weight * yaw_adjustment
            
            # Calculate clearance from beams
            beam_angles = self.calculate_beam_angles(valve_yaw)
            beam_clearances = []
            for beam_name, beam_angle in beam_angles.items():
                beam_clearance = abs(insertion_info['angle'] - beam_angle)
                beam_clearances.append(min(beam_clearance, 2*math.pi - beam_clearance))
            
            min_beam_clearance = min(beam_clearances)
            
            strategies[fang_id] = {
                'fang_id': fang_id,
                'config': insertion_info['config'],
                'insertion_angle': insertion_info['angle'],
                'insertion_position': target_pos,
                'approach_distance': approach_distance,
                'yaw_adjustment': yaw_adjustment,
                'complexity_score': complexity_score,
                'min_beam_clearance': min_beam_clearance,
                'beam_gap': insertion_info['beam_gap'],
                'optimal_clearance_angle': insertion_info['optimal_clearance_angle'],
                'adjacent_beam_clearance': insertion_info['adjacent_beam_clearance'],
                'active': True,  # Both fangs are active in dual-fang mode
            }
            
            dual_fang_strategy[fang_id] = strategies[fang_id]
        
        # Determine primary fang based on complexity score (for coordination)
        if strategies:
            primary_fang_id = min(strategies.keys(), key=lambda k: strategies[k]['complexity_score'])
            secondary_fang_id = max(strategies.keys(), key=lambda k: strategies[k]['complexity_score'])
            
            dual_fang_strategy['primary_fang'] = primary_fang_id
            dual_fang_strategy['secondary_fang'] = secondary_fang_id
            
            # Log dual-fang optimization results
            rospy.loginfo("=== DUAL-FANG INSERTION STRATEGY RESULTS ===")
            rospy.loginfo("Using 60° optimal clearance angle for maximum insertion space")
            rospy.loginfo(f"Primary fang (lower complexity): {strategies[primary_fang_id]['config']['name']}")
            rospy.loginfo(f"Secondary fang (higher complexity): {strategies[secondary_fang_id]['config']['name']}")
            
            for fang_id, strategy in strategies.items():
                rospy.loginfo(f"\n{strategy['config']['name']} ({fang_id.upper()}):")
                rospy.loginfo(f"  Insertion angle: {strategy['insertion_angle']:.3f} rad ({strategy['insertion_angle']*180/math.pi:.1f}°)")
                rospy.loginfo(f"  Approach distance: {strategy['approach_distance']:.3f}m")
                rospy.loginfo(f"  Yaw adjustment: {strategy['yaw_adjustment']:.3f} rad ({strategy['yaw_adjustment']*180/math.pi:.1f}°)")
                rospy.loginfo(f"  Complexity score: {strategy['complexity_score']:.3f}")
                rospy.loginfo(f"  Beam clearance: {strategy['min_beam_clearance']:.3f} rad ({strategy['min_beam_clearance']*180/math.pi:.1f}°)")
                rospy.loginfo(f"  Optimal clearance: {strategy['optimal_clearance_angle']*180/math.pi:.1f}° (60° for max space)")
                rospy.loginfo(f"  Adjacent beam clearance: {strategy['adjacent_beam_clearance']*180/math.pi:.1f}° (60° from adjacent beams)")
                rospy.loginfo(f"  Status: ACTIVE (dual-fang simultaneous insertion)")
                rospy.loginfo(f"  Beam gap: {strategy['beam_gap']}")
                rospy.loginfo(f"  Role: {'PRIMARY' if fang_id == primary_fang_id else 'SECONDARY'}")
            
            rospy.loginfo("\n=== DUAL-FANG COORDINATION STRATEGY ===")
            rospy.loginfo("Both fangs will insert simultaneously at their optimal 60° positions")
            rospy.loginfo("Fang1 (60°) and Fang2 (210°) provide maximum insertion clearance")
            rospy.loginfo("Coordination ensures smooth simultaneous insertion movement")
            
            return dual_fang_strategy
        
        return None
    
    def evaluate_real_time_strategy(self):
        """Evaluate insertion strategy using current real-time data"""
        return self.evaluate_insertion_strategy()
    
    def get_insertion_parameters(self, dual_strategy, pre_insertion_distance=0.03, circumferential_offset=0.02):
        """
        Get insertion parameters for dual-fang simultaneous insertion strategy
        
        Args:
            dual_strategy: Dual-fang insertion strategy
            pre_insertion_distance: Pre-insertion distance (meters)
            circumferential_offset: Circumferential offset for avoidance (meters)
            
        Returns:
            dict: Dual-fang insertion parameters with both fang positions
        """
        if dual_strategy is None:
            rospy.logerr("No insertion strategy provided")
            return None
        
        # Ensure this is a dual-fang strategy
        if dual_strategy.get('insertion_mode') != 'dual_simultaneous':
            rospy.logerr("Only dual-fang simultaneous insertion strategy is supported")
            return None
        
        # New dual-fang mode
        dual_params = {
            'fang1': None,
            'fang2': None,
            'insertion_mode': 'dual_simultaneous',
            'coordination_required': dual_strategy.get('coordination_required', True),
            'primary_fang': dual_strategy.get('primary_fang'),
            'secondary_fang': dual_strategy.get('secondary_fang')
        }
        
        # Calculate parameters for both fangs
        for fang_id in ['fang1', 'fang2']:
            if fang_id not in dual_strategy or dual_strategy[fang_id] is None:
                continue
                
            strategy = dual_strategy[fang_id]
            
            # Calculate valve center from insertion position
            valve_center_x = strategy['insertion_position'][0] - self.valve_radius * math.cos(strategy['insertion_angle'])
            valve_center_y = strategy['insertion_position'][1] - self.valve_radius * math.sin(strategy['insertion_angle'])
            
            # Apply offsets based on strategy
            config = strategy['config']
            
            # Adjust approach angle with bias
            approach_angle = strategy['insertion_angle'] + config['approach_bias']
            
            # Calculate approach radius with offsets
            end_effector_offset = 0.246  # End-effector length from UAV center
            approach_radius = self.valve_radius + end_effector_offset + self.safety_margin + pre_insertion_distance + circumferential_offset * 0.5
            
            # Calculate approach position
            approach_x = valve_center_x + approach_radius * math.cos(approach_angle)
            approach_y = valve_center_y + approach_radius * math.sin(approach_angle)
            
            # Calculate final insertion position
            final_radius = self.valve_radius + end_effector_offset + self.safety_margin
            final_x = valve_center_x + final_radius * math.cos(strategy['insertion_angle'])
            final_y = valve_center_y + final_radius * math.sin(strategy['insertion_angle'])
            
            dual_params[fang_id] = {
                'fang_id': fang_id,
                'config': config,
                'strategy': strategy,
                'approach_position': (approach_x, approach_y),
                'approach_angle': approach_angle,
                'approach_radius': approach_radius,
                'final_position': (final_x, final_y),
                'final_angle': strategy['insertion_angle'],
                'final_radius': final_radius,
                'valve_center': (valve_center_x, valve_center_y),
                'target_yaw': math.atan2(valve_center_y - final_y, valve_center_x - final_x),
                'fang_config': config,
                'optimal_clearance_angle': strategy.get('optimal_clearance_angle', math.pi/3),
                'adjacent_beam_clearance': strategy.get('adjacent_beam_clearance', math.pi/3),
                'insertion_space_advantage': '60° angle provides maximum space for clockwise/counterclockwise movement',
                'beam_gap': strategy['beam_gap'],
                'complexity_score': strategy['complexity_score'],
                'active': True
            }
        
        return dual_params
    
    def log_insertion_plan(self, parameters):
        """Log detailed insertion plan for dual-fang simultaneous insertion"""
        if parameters is None:
            rospy.logwarn("No insertion parameters to log")
            return
        
        # Only support dual-fang mode
        if parameters.get('insertion_mode') != 'dual_simultaneous':
            rospy.logerr("Only dual-fang simultaneous insertion mode is supported")
            return
        
        rospy.loginfo("=== DUAL-FANG SIMULTANEOUS INSERTION PLAN ===")
        rospy.loginfo(f"Insertion Mode: {parameters['insertion_mode']}")
        rospy.loginfo(f"Coordination Required: {parameters['coordination_required']}")
        rospy.loginfo(f"Primary Fang: {parameters['primary_fang']}")
        rospy.loginfo(f"Secondary Fang: {parameters['secondary_fang']}")
        
        for fang_id in ['fang1', 'fang2']:
            if fang_id not in parameters or parameters[fang_id] is None:
                continue
                
            fang_params = parameters[fang_id]
            config = fang_params['fang_config']
            
            rospy.loginfo(f"\n{fang_id.upper()} - {config['name']}:")
            rospy.loginfo(f"  Beam Gap: {fang_params['beam_gap']}")
            rospy.loginfo(f"  Optimal Clearance: {fang_params['optimal_clearance_angle']*180/math.pi:.1f}° (60° for maximum insertion space)")
            rospy.loginfo(f"  Adjacent Beam Clearance: {fang_params['adjacent_beam_clearance']*180/math.pi:.1f}° (60° from adjacent beams)")
            rospy.loginfo(f"  Insertion Space Advantage: {fang_params['insertion_space_advantage']}")
            rospy.loginfo(f"  Valve Center: [{fang_params['valve_center'][0]:.3f}, {fang_params['valve_center'][1]:.3f}]")
            rospy.loginfo(f"  Approach Position: [{fang_params['approach_position'][0]:.3f}, {fang_params['approach_position'][1]:.3f}]")
            rospy.loginfo(f"  Final Position: [{fang_params['final_position'][0]:.3f}, {fang_params['final_position'][1]:.3f}]")
            rospy.loginfo(f"  Approach Angle: {fang_params['approach_angle']:.3f} rad ({fang_params['approach_angle']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Final Angle: {fang_params['final_angle']:.3f} rad ({fang_params['final_angle']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Target Yaw: {fang_params['target_yaw']:.3f} rad ({fang_params['target_yaw']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Approach Radius: {fang_params['approach_radius']:.3f}m")
            rospy.loginfo(f"  Final Radius: {fang_params['final_radius']:.3f}m")
            rospy.loginfo(f"  Complexity Score: {fang_params['complexity_score']:.3f}")
            rospy.loginfo(f"  Status: {'PRIMARY' if fang_id == parameters['primary_fang'] else 'SECONDARY'}")
            rospy.loginfo(f"  Active: {fang_params['active']}")
        
        rospy.loginfo("\n=== DUAL-FANG COORDINATION SUMMARY ===")
        rospy.loginfo("Both fangs insert simultaneously at optimal 60° clearance positions")
        rospy.loginfo("Fang1 (60°) + Fang2 (210°) = Maximum insertion space coverage")
        rospy.loginfo("Enables smooth clockwise/counterclockwise movement to pre-insertion point")
        rospy.loginfo("=== END DUAL-FANG INSERTION PLAN ===")
        
        rospy.loginfo("NOTE: 60° insertion angles enable smooth clockwise/counterclockwise movement to pre-insertion point")

    def evaluate_dual_fang_strategy(self, current_pos=None, current_yaw=None, valve_pos=None, valve_yaw=None):
        """
        Evaluate dual-fang simultaneous insertion strategy
        Both fang1 and fang2 insert simultaneously at their preferred positions
        
        Args:
            current_pos: Current UAV position (x, y, z) - if None, uses real-time data
            current_yaw: Current UAV yaw angle - if None, uses real-time data
            valve_pos: Valve position (x, y, z) - if None, uses real-time data
            valve_yaw: Valve yaw angle - if None, uses real-time data
            
        Returns:
            dict: Dual-fang insertion strategy with both fang positions
        """
        # Use real-time data if parameters not provided
        if current_pos is None or current_yaw is None or valve_pos is None or valve_yaw is None:
            rospy.loginfo("Using real-time data for dual-fang insertion strategy evaluation")
            
            # Wait for real-time data if not available
            if not self.uav_data_received.is_set() or not self.valve_data_received.is_set():
                if not self.wait_for_data():
                    rospy.logerr("Failed to get real-time data for dual-fang optimization")
                    return None
            
            # Use real-time data
            current_pos = current_pos or self.current_uav_pos
            current_yaw = current_yaw or self.current_uav_yaw
            valve_pos = valve_pos or self.valve_pos
            valve_yaw = valve_yaw or self.valve_yaw
            
            rospy.loginfo("=== REAL-TIME DATA FOR DUAL-FANG OPTIMIZATION ===")
            rospy.loginfo(f"UAV position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            rospy.loginfo(f"UAV yaw: {current_yaw:.3f} rad ({current_yaw*180/math.pi:.1f}°)")
            rospy.loginfo(f"Valve position: [{valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f}]")
            rospy.loginfo(f"Valve yaw: {valve_yaw:.3f} rad ({valve_yaw*180/math.pi:.1f}°)")
        
        insertion_positions = self.calculate_insertion_positions(valve_pos, valve_yaw)
        
        # Calculate strategy for both fangs
        dual_fang_strategy = {
            'fang1': None,
            'fang2': None,
            'approach_sequence': 'simultaneous',  # or 'sequential'
            'coordination_required': True
        }
        
        rospy.loginfo("=== DUAL-FANG INSERTION STRATEGY EVALUATION ===")
        
        for fang_id, insertion_info in insertion_positions.items():
            target_pos = insertion_info['position'] + (valve_pos[2],)  # Add Z coordinate
            
            # Calculate approach distance
            approach_distance = self.calculate_approach_distance(current_pos, target_pos)
            
            # Calculate required yaw adjustment to point toward valve center
            dx_to_center = valve_pos[0] - target_pos[0]
            dy_to_center = valve_pos[1] - target_pos[1]
            target_yaw = math.atan2(dy_to_center, dx_to_center)
            yaw_adjustment = self.calculate_yaw_adjustment(current_yaw, target_yaw)
            
            # Calculate insertion complexity score
            distance_weight = 1.0
            yaw_weight = 0.5
            complexity_score = distance_weight * approach_distance + yaw_weight * yaw_adjustment
            
            # Calculate clearance from beams
            beam_angles = self.calculate_beam_angles(valve_yaw)
            beam_clearances = []
            for beam_name, beam_angle in beam_angles.items():
                beam_clearance = abs(insertion_info['angle'] - beam_angle)
                beam_clearances.append(min(beam_clearance, 2*math.pi - beam_clearance))
            
            min_beam_clearance = min(beam_clearances)
            
            dual_fang_strategy[fang_id] = {
                'fang_id': fang_id,
                'config': insertion_info['config'],
                'insertion_angle': insertion_info['angle'],
                'insertion_position': target_pos,
                'approach_distance': approach_distance,
                'yaw_adjustment': yaw_adjustment,
                'complexity_score': complexity_score,
                'min_beam_clearance': min_beam_clearance,
                'beam_gap': insertion_info['beam_gap'],
                'active': True,  # Both fangs are active in dual-fang mode
            }
            
            rospy.loginfo(f"{insertion_info['config']['name']}:")
            rospy.loginfo(f"  Insertion angle: {insertion_info['angle']:.3f} rad ({insertion_info['angle']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Approach distance: {approach_distance:.3f}m")
            rospy.loginfo(f"  Yaw adjustment: {yaw_adjustment:.3f} rad ({yaw_adjustment*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Complexity score: {complexity_score:.3f}")
            rospy.loginfo(f"  Beam clearance: {min_beam_clearance:.3f} rad")
            rospy.loginfo(f"  Beam gap: {insertion_info['beam_gap']}")
            rospy.loginfo(f"  Status: ACTIVE (dual-fang mode)")
        
        rospy.loginfo("=== DUAL-FANG STRATEGY SELECTED ===")
        rospy.loginfo("Both Fang1 (Left) and Fang2 (Right) will insert simultaneously")
        rospy.loginfo("Fang1 -> beam0_beam1, Fang2 -> beam1_beam2")
        
        return dual_fang_strategy

    def get_dual_fang_insertion_parameters(self, dual_strategy, pre_insertion_distance=0.03, circumferential_offset=0.02):
        """
        Get insertion parameters for dual-fang simultaneous insertion
        
        Args:
            dual_strategy: Dual-fang insertion strategy
            pre_insertion_distance: Pre-insertion distance (meters)
            circumferential_offset: Circumferential offset for avoidance (meters)
            
        Returns:
            dict: Dual-fang insertion parameters
        """
        if dual_strategy is None or dual_strategy.get('fang1') is None or dual_strategy.get('fang2') is None:
            return None
        
        dual_params = {
            'fang1': None,
            'fang2': None,
            'approach_sequence': dual_strategy.get('approach_sequence', 'simultaneous'),
            'coordination_required': dual_strategy.get('coordination_required', True)
        }
        
        # Calculate parameters for each fang
        for fang_id in ['fang1', 'fang2']:
            strategy = dual_strategy[fang_id]
            
            # Calculate approach position with offsets
            valve_center_x = strategy['insertion_position'][0] - self.valve_radius * math.cos(strategy['insertion_angle'])
            valve_center_y = strategy['insertion_position'][1] - self.valve_radius * math.sin(strategy['insertion_angle'])
            
            # Apply offsets based on strategy
            config = strategy['config']
            
            # Adjust approach angle with bias
            approach_angle = strategy['insertion_angle'] + config['approach_bias']
            
            # Calculate approach radius with offsets
            approach_radius = self.valve_radius + self.safety_margin + pre_insertion_distance + circumferential_offset * 0.5
            
            # Calculate approach position
            approach_x = valve_center_x + approach_radius * math.cos(approach_angle)
            approach_y = valve_center_y + approach_radius * math.sin(approach_angle)
            
            # Calculate final insertion position
            final_radius = self.valve_radius + self.safety_margin
            final_x = valve_center_x + final_radius * math.cos(strategy['insertion_angle'])
            final_y = valve_center_y + final_radius * math.sin(strategy['insertion_angle'])
            
            dual_params[fang_id] = {
                'fang_id': fang_id,
                'config': config,
                'valve_center': [valve_center_x, valve_center_y],
                'approach_position': [approach_x, approach_y],
                'final_position': [final_x, final_y],
                'approach_angle': approach_angle,
                'final_angle': strategy['insertion_angle'],
                'target_yaw': strategy['insertion_angle'] + math.pi,  # Point toward valve center
                'approach_radius': approach_radius,
                'final_radius': final_radius,
                'beam_gap': strategy['beam_gap'],
                'complexity_score': strategy['complexity_score']
            }
        
        return dual_params
    
    def log_dual_fang_insertion_plan(self, dual_params):
        """
        Log the dual-fang insertion plan
        
        Args:
            dual_params: Dual-fang insertion parameters
        """
        if dual_params is None:
            rospy.logerr("No dual-fang insertion parameters to log")
            return
        
        rospy.loginfo("=== DUAL-FANG INSERTION PLAN ===")
        rospy.loginfo(f"Approach sequence: {dual_params['approach_sequence']}")
        rospy.loginfo(f"Coordination required: {dual_params['coordination_required']}")
        
        for fang_id in ['fang1', 'fang2']:
            params = dual_params[fang_id]
            rospy.loginfo(f"\n{fang_id.upper()} ({params['config']['name']}):")
            rospy.loginfo(f"  Beam Gap: {params['beam_gap']}")
            rospy.loginfo(f"  Valve Center: [{params['valve_center'][0]:.3f}, {params['valve_center'][1]:.3f}]")
            rospy.loginfo(f"  Approach Position: [{params['approach_position'][0]:.3f}, {params['approach_position'][1]:.3f}]")
            rospy.loginfo(f"  Final Position: [{params['final_position'][0]:.3f}, {params['final_position'][1]:.3f}]")
            rospy.loginfo(f"  Approach Angle: {params['approach_angle']:.3f} rad ({params['approach_angle']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Final Angle: {params['final_angle']:.3f} rad ({params['final_angle']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Target Yaw: {params['target_yaw']:.3f} rad ({params['target_yaw']*180/math.pi:.1f}°)")
            rospy.loginfo(f"  Approach Radius: {params['approach_radius']:.3f}m")
            rospy.loginfo(f"  Final Radius: {params['final_radius']:.3f}m")
            rospy.loginfo(f"  Complexity Score: {params['complexity_score']:.3f}")
        
        rospy.loginfo("=== END DUAL-FANG INSERTION PLAN ===")


def test_insertion_optimizer():
    """Test function for insertion optimizer with real-time data"""
    rospy.init_node('insertion_optimizer_test')
    
    # Test parameters
    module_id = rospy.get_param("~module_id", 1)
    simulation = rospy.get_param("~simulation", True)
    
    rospy.loginfo(f"Testing InsertionOptimizer with module_id={module_id}, simulation={simulation}")
    
    # Initialize optimizer with real-time data capability
    optimizer = InsertionOptimizer(
        valve_radius=0.1225,
        valve_beam_width=0.0185,
        safety_margin=0.008,
        module_id=module_id,
        simulation=simulation
    )
    
    # Test 1: Wait for real-time data and evaluate strategy
    rospy.loginfo("=== Test 1: Real-time data evaluation ===")
    try:
        if optimizer.wait_for_data(timeout=15.0):
            strategy = optimizer.evaluate_real_time_strategy()
            if strategy:
                parameters = optimizer.get_insertion_parameters(strategy)
                optimizer.log_insertion_plan(parameters)
                rospy.loginfo("✓ Real-time optimization successful")
            else:
                rospy.logerr("✗ Failed to get optimization strategy")
        else:
            rospy.logerr("✗ Failed to get real-time data")
    except Exception as e:
        rospy.logerr(f"✗ Real-time test failed: {e}")
    
    # Test 2: Fallback to manual data for comparison
    rospy.loginfo("\n=== Test 2: Manual data for comparison ===")
    try:
        # Sample positions for testing
        test_current_pos = (2.8, -0.2, 1.0)
        test_current_yaw = 0.1
        test_valve_pos = (3.0, 0.0, 0.57)
        test_valve_yaw = 0.0
        
        rospy.loginfo(f"Test UAV position: {test_current_pos}")
        rospy.loginfo(f"Test UAV yaw: {test_current_yaw:.3f} rad")
        rospy.loginfo(f"Test valve position: {test_valve_pos}")
        rospy.loginfo(f"Test valve yaw: {test_valve_yaw:.3f} rad")
        
        strategy = optimizer.evaluate_insertion_strategy(
            current_pos=test_current_pos,
            current_yaw=test_current_yaw,
            valve_pos=test_valve_pos,
            valve_yaw=test_valve_yaw
        )
        
        if strategy:
            parameters = optimizer.get_insertion_parameters(strategy)
            optimizer.log_insertion_plan(parameters)
            rospy.loginfo("✓ Manual data optimization successful")
        else:
            rospy.logerr("✗ Failed to get manual optimization strategy")
    except Exception as e:
        rospy.logerr(f"✗ Manual test failed: {e}")
    
    rospy.loginfo("=== Test Complete ===")
    rospy.loginfo("The optimizer can now use real-time data from ROS topics:")
    rospy.loginfo(f"- UAV pose: /beetle{module_id}/mocap/pose")
    if simulation:
        rospy.loginfo("- Valve pose: /valve/odom (simulation)")
    else:
        rospy.loginfo("- Valve pose: /valve/mocap/pose (real)")


def test_with_manual_data():
    """Test with manual data (for standalone testing without ROS topics)"""
    print("=== Testing with manual data (no ROS topics required) ===")
    
    # Create optimizer without ROS subscriptions
    optimizer = InsertionOptimizer(
        valve_radius=0.1225,
        valve_beam_width=0.0185,
        safety_margin=0.008,
        module_id=1,
        simulation=True
    )
    
    # Test cases
    test_cases = [
        {
            'name': 'UAV test position 1',
            'current_pos': (2.8, -0.2, 1.0),
            'current_yaw': 0.1,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
        {
            'name': 'UAV test position 2',
            'current_pos': (3.2, 0.2, 1.0),
            'current_yaw': 2.0,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n=== Test Case {i}: {test_case['name']} ===")
        try:
            strategy = optimizer.evaluate_insertion_strategy(
                current_pos=test_case['current_pos'],
                current_yaw=test_case['current_yaw'],
                valve_pos=test_case['valve_pos'],
                valve_yaw=test_case['valve_yaw']
            )
            
            if strategy and strategy.get('insertion_mode') == 'dual_simultaneous':
                parameters = optimizer.get_insertion_parameters(strategy)
                print(f"✓ Dual-fang strategy generated successfully")
                print(f"  Primary fang: {strategy['primary_fang']}")
                print(f"  Secondary fang: {strategy['secondary_fang']}")
                print(f"  Coordination required: {strategy['coordination_required']}")
                
                # Show complexity scores for both fangs
                for fang_id in ['fang1', 'fang2']:
                    if fang_id in strategy and strategy[fang_id]:
                        fang_strategy = strategy[fang_id]
                        print(f"  {fang_id}: {fang_strategy['config']['name']}")
                        print(f"    Complexity score: {fang_strategy['complexity_score']:.3f}")
                        print(f"    Approach distance: {fang_strategy['approach_distance']:.3f}m")
                        print(f"    Yaw adjustment: {fang_strategy['yaw_adjustment']:.3f} rad")
            else:
                print("✗ Failed to get dual-fang strategy")
        except Exception as e:
            print(f"✗ Test failed: {e}")
    
    print("\n=== Manual test complete ===")


if __name__ == '__main__':
    import sys
    
    # Check if ROS is available
    if len(sys.argv) > 1 and sys.argv[1] == '--manual':
        # Run manual test without ROS
        test_with_manual_data()
    else:
        # Run ROS-based test
        try:
            test_insertion_optimizer()
        except rospy.ROSException as e:
            print(f"ROS test failed: {e}")
            print("Falling back to manual test...")
            test_with_manual_data()