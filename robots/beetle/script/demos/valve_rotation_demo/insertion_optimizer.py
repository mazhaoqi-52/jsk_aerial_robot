#!/usr/bin/env python
"""
Insertion Strategy Optimizer for Valve Rotation Task
Optimizes insertion strategy selection between different fang configurations and beam positions
"""

import math
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class InsertionOptimizer:
    """
    Optimizer for selecting optimal insertion strategy based on UAV position and valve configuration
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
                'preferred_beam_gap': 'beam1_beam2',  # Between beam1(120°) and beam2(240°)
                'beam_position_factor': 0.1,  # Closer to beam2
                'approach_bias': 0.3,  # Approach bias factor
            },
            'fang2': {
                'name': 'Right Fang (Fang 2)', 
                'description': 'Right side fang insertion',
                'preferred_beam_gap': 'beam0_beam1',  # Between beam0(0°) and beam1(120°)
                'beam_position_factor': 0.1,  # Closer to beam1
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
        """Calculate potential insertion positions for both fangs"""
        beam_angles = self.calculate_beam_angles(valve_yaw)
        center_x, center_y = valve_pos[0], valve_pos[1]
        
        insertion_positions = {}
        
        for fang_id, config in self.fang_configs.items():
            if config['preferred_beam_gap'] == 'beam1_beam2':
                # Between beam1(120°) and beam2(240°)
                beam_start = beam_angles['beam2']
                beam_end = beam_angles['beam1']
                # Adjust for angle wrapping
                if beam_end < beam_start:
                    beam_end += 2*math.pi
                
            elif config['preferred_beam_gap'] == 'beam0_beam1':
                # Between beam0(0°) and beam1(120°)
                beam_start = beam_angles['beam1']
                beam_end = beam_angles['beam0']
                # Adjust for angle wrapping
                if beam_end < beam_start:
                    beam_end += 2*math.pi
            
            # Calculate insertion angle (closer to start beam)
            gap_angle = beam_end - beam_start
            insertion_angle = beam_start - config['beam_position_factor'] * gap_angle
            
            # Normalize angle
            while insertion_angle >= 2*math.pi:
                insertion_angle -= 2*math.pi
            while insertion_angle < 0:
                insertion_angle += 2*math.pi
            
            # Calculate insertion position
            insertion_radius = self.valve_radius + self.safety_margin
            insertion_x = center_x + insertion_radius * math.cos(insertion_angle)
            insertion_y = center_y + insertion_radius * math.sin(insertion_angle)
            
            insertion_positions[fang_id] = {
                'angle': insertion_angle,
                'position': (insertion_x, insertion_y),
                'radius': insertion_radius,
                'config': config,
                'beam_gap': config['preferred_beam_gap'],
                'beam_start_angle': beam_start,
                'beam_end_angle': beam_end,
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
        Evaluate and select optimal insertion strategy
        
        Args:
            current_pos: Current UAV position (x, y, z) - if None, uses real-time data
            current_yaw: Current UAV yaw angle - if None, uses real-time data
            valve_pos: Valve position (x, y, z) - if None, uses real-time data
            valve_yaw: Valve yaw angle - if None, uses real-time data
            
        Returns:
            dict: Optimal insertion strategy with details
        """
        # Use real-time data if parameters not provided
        if current_pos is None or current_yaw is None or valve_pos is None or valve_yaw is None:
            rospy.loginfo("Using real-time data for insertion strategy evaluation")
            
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
            
            rospy.loginfo("=== REAL-TIME DATA FOR OPTIMIZATION ===")
            rospy.loginfo(f"UAV position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            rospy.loginfo(f"UAV yaw: {current_yaw:.3f} rad ({current_yaw*180/math.pi:.1f}°)")
            rospy.loginfo(f"Valve position: [{valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f}]")
            rospy.loginfo(f"Valve yaw: {valve_yaw:.3f} rad ({valve_yaw*180/math.pi:.1f}°)")
        
        insertion_positions = self.calculate_insertion_positions(valve_pos, valve_yaw)
        
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
            # Lower score = better strategy
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
                'recommended': False,
            }
        
        # Select optimal strategy (lowest complexity score)
        if strategies:
            optimal_fang = min(strategies.keys(), key=lambda k: strategies[k]['complexity_score'])
            strategies[optimal_fang]['recommended'] = True
            
            # Log optimization results
            rospy.loginfo("=== Insertion Strategy Optimization Results ===")
            for fang_id, strategy in strategies.items():
                rospy.loginfo(f"{strategy['config']['name']}:")
                rospy.loginfo(f"  Insertion angle: {strategy['insertion_angle']:.3f} rad ({strategy['insertion_angle']*180/math.pi:.1f}°)")
                rospy.loginfo(f"  Approach distance: {strategy['approach_distance']:.3f}m")
                rospy.loginfo(f"  Yaw adjustment: {strategy['yaw_adjustment']:.3f} rad ({strategy['yaw_adjustment']*180/math.pi:.1f}°)")
                rospy.loginfo(f"  Complexity score: {strategy['complexity_score']:.3f}")
                rospy.loginfo(f"  Beam clearance: {strategy['min_beam_clearance']:.3f} rad")
                rospy.loginfo(f"  Recommended: {'YES' if strategy['recommended'] else 'NO'}")
                rospy.loginfo(f"  Beam gap: {strategy['beam_gap']}")
            
            optimal_strategy = strategies[optimal_fang]
            rospy.loginfo(f"=== SELECTED STRATEGY: {optimal_strategy['config']['name']} ===")
            rospy.loginfo(f"Reason: Lowest complexity score ({optimal_strategy['complexity_score']:.3f})")
            
            return optimal_strategy
        
        return None
    
    def evaluate_real_time_strategy(self):
        """Evaluate insertion strategy using current real-time data"""
        return self.evaluate_insertion_strategy()
    
    def get_insertion_parameters(self, strategy, pre_insertion_distance=0.03, circumferential_offset=0.02):
        """
        Get insertion parameters for the selected strategy
        
        Args:
            strategy: Selected insertion strategy
            pre_insertion_distance: Pre-insertion distance (meters)
            circumferential_offset: Circumferential offset for avoidance (meters)
            
        Returns:
            dict: Insertion parameters
        """
        if strategy is None:
            return None
        
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
        
        return {
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
        }
    
    def log_insertion_plan(self, parameters):
        """Log detailed insertion plan"""
        if parameters is None:
            rospy.logwarn("No insertion parameters to log")
            return
        
        strategy = parameters['strategy']
        config = parameters['fang_config']
        
        rospy.loginfo("=== INSERTION PLAN ===")
        rospy.loginfo(f"Selected Fang: {config['name']}")
        rospy.loginfo(f"Beam Gap: {strategy['beam_gap']}")
        rospy.loginfo(f"Valve Center: [{parameters['valve_center'][0]:.3f}, {parameters['valve_center'][1]:.3f}]")
        rospy.loginfo(f"Approach Position: [{parameters['approach_position'][0]:.3f}, {parameters['approach_position'][1]:.3f}]")
        rospy.loginfo(f"Final Position: [{parameters['final_position'][0]:.3f}, {parameters['final_position'][1]:.3f}]")
        rospy.loginfo(f"Approach Angle: {parameters['approach_angle']:.3f} rad ({parameters['approach_angle']*180/math.pi:.1f}°)")
        rospy.loginfo(f"Final Angle: {parameters['final_angle']:.3f} rad ({parameters['final_angle']*180/math.pi:.1f}°)")
        rospy.loginfo(f"Target Yaw: {parameters['target_yaw']:.3f} rad ({parameters['target_yaw']*180/math.pi:.1f}°)")
        rospy.loginfo(f"Approach Radius: {parameters['approach_radius']:.3f}m")
        rospy.loginfo(f"Final Radius: {parameters['final_radius']:.3f}m")
        rospy.loginfo("=== END INSERTION PLAN ===")


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
            'name': 'UAV closer to left side (should select Fang1)',
            'current_pos': (2.8, -0.2, 1.0),
            'current_yaw': 0.1,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
        {
            'name': 'UAV closer to right side (should select Fang2)',
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
            
            if strategy:
                parameters = optimizer.get_insertion_parameters(strategy)
                print(f"✓ Selected: {strategy['config']['name']}")
                print(f"  Complexity score: {strategy['complexity_score']:.3f}")
                print(f"  Approach distance: {strategy['approach_distance']:.3f}m")
                print(f"  Yaw adjustment: {strategy['yaw_adjustment']:.3f} rad")
            else:
                print("✗ Failed to get strategy")
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
