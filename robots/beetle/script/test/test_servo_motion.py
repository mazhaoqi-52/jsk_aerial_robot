#!/usr/bin/env python

"""
Test script for 6 servos simultaneous motion on Beetle Hyper.
Servos will move in a sinusoidal pattern within ±60 degrees range.

Usage:
    rosrun beetle test_servo_motion.py
    or
    python test_servo_motion.py
"""

from spinal.msg import ServoControlCmd
import rospy
import math
import time

class ServoMotionTest():
    def __init__(self):
        # Servo configuration
        self.servo_ids = [1, 2, 3, 4, 5, 6]  # 6 servos
        self.robot_name = "beetle2"
        
        # Dynamixel position parameters
        # Position range: 0 ~ 4096, center position: 2048
        # Full range: 0 ~ 360 degrees (2*pi radians)
        self.DYNAMIXEL_CENTER_POSITION = 2048
        self.DYNAMIXEL_POSITION_PER_RAD = 4096.0 / (2.0 * math.pi)
        
        # Motion parameters
        self.amplitude = math.pi / 3.0  # ±60 degrees = ±pi/3 radians
        self.frequency = 0.2  # 0.2 Hz = 5 seconds per cycle
        self.control_rate = 40  # Hz
        
        # ROS publisher
        self.servo_pub = rospy.Publisher(
            '/' + self.robot_name + '/servo/target_states',
            ServoControlCmd,
            queue_size=1
        )
        
        # Message initialization
        self.servo_cmd_msg = ServoControlCmd()
        self.servo_cmd_msg.index = self.servo_ids
        
        # Rate control
        self.rate = rospy.Rate(self.control_rate)
        
        # Wait for publisher to be ready
        time.sleep(0.5)
        
        rospy.loginfo("ServoMotionTest initialized")
        rospy.loginfo("  Robot: %s", self.robot_name)
        rospy.loginfo("  Servo IDs: %s", str(self.servo_ids))
        rospy.loginfo("  Amplitude: ±60 degrees (±%.3f rad)", self.amplitude)
        rospy.loginfo("  Frequency: %.2f Hz (%.1f sec/cycle)", self.frequency, 1.0/self.frequency)

    def rad_to_dynamixel_position(self, angle_rad):
        """
        Convert angle in radians to Dynamixel position.
        Center (0 rad) = 2048
        """
        position = int(self.DYNAMIXEL_CENTER_POSITION + angle_rad * self.DYNAMIXEL_POSITION_PER_RAD)
        # Clamp to valid range
        position = max(0, min(4096, position))
        return position

    def main(self):
        rospy.loginfo("Starting servo motion test...")
        rospy.loginfo("Press Ctrl+C to stop")
        
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            # Calculate elapsed time
            t = rospy.get_time() - start_time
            
            # Calculate target angle using sinusoidal function
            # angle(t) = A * sin(2 * pi * f * t)
            target_angle_rad = self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)
            
            # Convert to Dynamixel position
            target_position = self.rad_to_dynamixel_position(target_angle_rad)
            
            # Apply same angle to all 6 servos
            self.servo_cmd_msg.angles = [target_angle_rad] * len(self.servo_ids)
            
            # Publish command
            self.servo_pub.publish(self.servo_cmd_msg)
            
            # Log status every second
            if int(t * 10) % 10 == 0:
                angle_deg = math.degrees(target_angle_rad)
                rospy.loginfo_throttle(1.0, "t=%.1fs, angle=%.1f deg (%.3f rad)", 
                                       t, angle_deg, target_angle_rad)
            
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("test_servo_motion")
    
    try:
        test = ServoMotionTest()
        test.main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Servo motion test stopped")
