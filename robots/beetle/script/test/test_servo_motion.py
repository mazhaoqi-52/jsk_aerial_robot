#!/usr/bin/env python

"""
Test script for 6 servos simultaneous motion on Beetle Hyper.
Servos 0-3 will move in a sinusoidal pattern within ±60 degrees range.
Servo 4 (male) will move between lock (1550) and unlock (1850) positions.
Servo 5 (female) will move between unlock (2000) and lock (3200) positions.

Usage:
    rosrun beetle test_servo_motion.py _robot_name:=beetle1
    or
    python test_servo_motion.py _robot_name:=beetle1
"""

from spinal.msg import ServoControlCmd
import rospy
import math
import time
import sys

class ServoMotionTest():
    def __init__(self, robot_name="beetle2"):
        # Servo configuration
        self.servo_ids = [0, 1, 2, 3, 4, 5]  # 6 servos
        self.robot_name = robot_name
        
        # Dynamixel position parameters
        # Position range: 0 ~ 4096, center position: 2048
        # Full range: 0 ~ 360 degrees (2*pi radians)
        self.DYNAMIXEL_CENTER_POSITION = 2048
        self.DYNAMIXEL_POSITION_PER_RAD = 4096.0 / (2.0 * math.pi)
        
        # Motion parameters for servo 0-3 (±60 degrees sinusoidal)
        self.amplitude = math.pi / 3.0  # ±60 degrees = ±pi/3 radians
        self.frequency = 0.2  # 0.2 Hz = 5 seconds per cycle
        self.control_rate = 40  # Hz
        
        # Servo 4 (male) lock/unlock positions (from assembly_api.py)
        self.male_servo_id = 4
        self.unlock_servo_angle_male = 1850
        self.lock_servo_angle_male = 1550
        
        # Servo 5 (female) lock/unlock positions (from assembly_api.py)
        self.female_servo_id = 5
        self.unlock_servo_angle_female = 2000
        self.lock_servo_angle_female = 3200
        
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
        rospy.loginfo("  Servo 0-3: ±60 degrees (±%.3f rad)", self.amplitude)
        rospy.loginfo("  Servo 4 (male): %d ~ %d", self.lock_servo_angle_male, self.unlock_servo_angle_male)
        rospy.loginfo("  Servo 5 (female): %d ~ %d", self.unlock_servo_angle_female, self.lock_servo_angle_female)
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

    def get_male_servo_position(self, t):
        """
        Calculate male servo (ID 4) position oscillating between lock and unlock.
        Uses sinusoidal interpolation between lock_servo_angle_male (1550) and unlock_servo_angle_male (1850).
        """
        # sin value ranges from -1 to 1, map to 0 to 1
        sin_val = (math.sin(2.0 * math.pi * self.frequency * t) + 1.0) / 2.0
        # Interpolate between lock and unlock positions
        position = int(self.lock_servo_angle_male + sin_val * (self.unlock_servo_angle_male - self.lock_servo_angle_male))
        return position

    def get_female_servo_position(self, t):
        """
        Calculate female servo (ID 5) position oscillating between unlock and lock.
        Uses sinusoidal interpolation between unlock_servo_angle_female (2000) and lock_servo_angle_female (3200).
        """
        # sin value ranges from -1 to 1, map to 0 to 1
        sin_val = (math.sin(2.0 * math.pi * self.frequency * t) + 1.0) / 2.0
        # Interpolate between unlock and lock positions
        position = int(self.unlock_servo_angle_female + sin_val * (self.lock_servo_angle_female - self.unlock_servo_angle_female))
        return position

    def main(self):
        rospy.loginfo("Starting servo motion test...")
        rospy.loginfo("Press Ctrl+C to stop")
        
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            # Calculate elapsed time
            t = rospy.get_time() - start_time
            
            # Calculate target angle for servo 0-3 using sinusoidal function
            # angle(t) = A * sin(2 * pi * f * t)
            target_angle_rad = self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)
            
            # Convert to Dynamixel position for servo 0-3
            target_position_0_3 = self.rad_to_dynamixel_position(target_angle_rad)
            
            # Get positions for male (ID 4) and female (ID 5) servos
            male_position = self.get_male_servo_position(t)
            female_position = self.get_female_servo_position(t)
            
            # Build angles list: [servo0, servo1, servo2, servo3, servo4_male, servo5_female]
            self.servo_cmd_msg.angles = [target_position_0_3, target_position_0_3, 
                                         target_position_0_3, target_position_0_3,
                                         male_position, female_position]
            
            # Publish command
            self.servo_pub.publish(self.servo_cmd_msg)
            
            # Log status every second
            if int(t * 10) % 10 == 0:
                angle_deg = math.degrees(target_angle_rad)
                rospy.loginfo_throttle(1.0, "t=%.1fs, servo0-3=%.1f deg, male=%d, female=%d", 
                                       t, angle_deg, male_position, female_position)
            
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("test_servo_motion")
    
    # Get robot_name from ROS parameter (command line: _robot_name:=beetle1)
    robot_name = rospy.get_param("~robot_name", "beetle2")
    
    try:
        test = ServoMotionTest(robot_name)
        test.main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Servo motion test stopped")
