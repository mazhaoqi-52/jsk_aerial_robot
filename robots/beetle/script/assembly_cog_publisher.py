#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
from collections import defaultdict

class AssemblyCogPublisher:
    """
    Assembly Center of Gravity Publisher
    
    This node calculates and publishes the assembly center of gravity (CoG) 
    by taking the geometric center of all assembled UAV modules.
    
    Subscribes to: /beetle{ID}/mocap/pose for each module
    Publishes to: /assemble/cog/odom
    """
    
    def __init__(self):
        rospy.init_node('assembly_cog_publisher', anonymous=True)
        
        # Get module IDs parameter
        module_ids_param = rospy.get_param('~module_ids', '1,2')
        try:
            self.module_ids = [int(x.strip()) for x in module_ids_param.split(',')]
        except ValueError:
            rospy.logerr(f"Invalid module_ids parameter: {module_ids_param}")
            rospy.signal_shutdown("Invalid parameters")
            return
            
        rospy.loginfo(f"Assembly CoG Publisher initialized for modules: {self.module_ids}")
        
        # Data storage
        self.module_poses = {}  # {module_id: PoseStamped}
        self.last_update_time = {}  # {module_id: timestamp}
        
        # Publisher for assembly CoG
        self.cog_pub = rospy.Publisher('/assemble/cog/odom', Odometry, queue_size=1)
        
        # Subscribers for each module's pose
        self.pose_subs = {}
        for module_id in self.module_ids:
            topic_name = f'/beetle{module_id}/mocap/pose'
            self.pose_subs[module_id] = rospy.Subscriber(
                topic_name, PoseStamped, 
                lambda msg, mid=module_id: self.pose_callback(msg, mid), 
                queue_size=1
            )
            rospy.loginfo(f"Subscribed to {topic_name}")
        
        # Configuration parameters
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)  # 50Hz
        self.timeout_threshold = rospy.get_param('~timeout_threshold', 0.5)  # 0.5s timeout
        
        # Timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        rospy.loginfo(f"Assembly CoG Publisher ready - publishing at {self.publish_rate}Hz")
        
    def pose_callback(self, msg, module_id):
        """Callback for individual module pose updates"""
        self.module_poses[module_id] = msg
        self.last_update_time[module_id] = rospy.Time.now()
        
    def timer_callback(self, event):
        """Timer callback to publish assembly CoG at regular intervals"""
        current_time = rospy.Time.now()
        
        # Check which modules have recent data
        active_modules = []
        for module_id in self.module_ids:
            if (module_id in self.module_poses and 
                module_id in self.last_update_time and
                (current_time - self.last_update_time[module_id]).to_sec() < self.timeout_threshold):
                active_modules.append(module_id)
        
        if len(active_modules) < 2:
            # Need at least 2 modules for assembly
            return
            
        # Calculate assembly center of gravity
        assembly_cog = self.calculate_assembly_cog(active_modules)
        if assembly_cog is not None:
            self.publish_assembly_cog(assembly_cog, current_time)
    
    def calculate_assembly_cog(self, active_modules):
        """
        Calculate the geometric center of all active modules
        
        Args:
            active_modules: List of module IDs with recent pose data
            
        Returns:
            dict: Assembly CoG data with position and orientation
        """
        if len(active_modules) == 0:
            return None
            
        # Sum positions for geometric center calculation
        sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
        sum_qx, sum_qy, sum_qz, sum_qw = 0.0, 0.0, 0.0, 0.0
        
        for module_id in active_modules:
            pose = self.module_poses[module_id].pose
            
            # Sum positions
            sum_x += pose.position.x
            sum_y += pose.position.y
            sum_z += pose.position.z
            
            # Sum orientations (quaternion averaging)
            sum_qx += pose.orientation.x
            sum_qy += pose.orientation.y
            sum_qz += pose.orientation.z
            sum_qw += pose.orientation.w
        
        # Calculate geometric center
        n_modules = len(active_modules)
        cog_x = sum_x / n_modules
        cog_y = sum_y / n_modules
        cog_z = sum_z / n_modules
        
        # Average orientation (simple averaging - good enough for small angle differences)
        cog_qx = sum_qx / n_modules
        cog_qy = sum_qy / n_modules
        cog_qz = sum_qz / n_modules
        cog_qw = sum_qw / n_modules
        
        # Normalize quaternion
        q_norm = math.sqrt(cog_qx**2 + cog_qy**2 + cog_qz**2 + cog_qw**2)
        if q_norm > 0:
            cog_qx /= q_norm
            cog_qy /= q_norm
            cog_qz /= q_norm
            cog_qw /= q_norm
        else:
            # Default to no rotation if norm is zero
            cog_qx, cog_qy, cog_qz, cog_qw = 0.0, 0.0, 0.0, 1.0
        
        return {
            'position': [cog_x, cog_y, cog_z],
            'orientation': [cog_qx, cog_qy, cog_qz, cog_qw],
            'active_modules': active_modules
        }
    
    def publish_assembly_cog(self, assembly_cog, timestamp):
        """
        Publish assembly CoG as Odometry message
        
        Args:
            assembly_cog: Assembly CoG data from calculate_assembly_cog
            timestamp: Current timestamp
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'assembly_cog'
        
        # Set position
        pos = assembly_cog['position']
        odom_msg.pose.pose.position.x = pos[0]
        odom_msg.pose.pose.position.y = pos[1]
        odom_msg.pose.pose.position.z = pos[2]
        
        # Set orientation
        quat = assembly_cog['orientation']
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Set covariance (identity for now)
        for i in range(36):
            if i % 7 == 0:  # Diagonal elements
                odom_msg.pose.covariance[i] = 0.01  # 1cm position uncertainty
            else:
                odom_msg.pose.covariance[i] = 0.0
        
        # Velocity is zero for now (could be calculated from position derivatives)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Publish the message
        self.cog_pub.publish(odom_msg)
        
        # Debug logging (throttled) - log every 2 seconds (100 calls at 50Hz)
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter = (self.log_counter + 1) % 100
        if self.log_counter == 0:
            rospy.loginfo(f"Assembly CoG: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
                         f"from modules {assembly_cog['active_modules']}")

if __name__ == '__main__':
    try:
        publisher = AssemblyCogPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Assembly CoG Publisher shutdown")
    except Exception as e:
        rospy.logerr(f"Assembly CoG Publisher error: {e}")
        rospy.signal_shutdown("Fatal error")