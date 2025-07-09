#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from task.assembly_motion import *

#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from task.assembly_motion import *

class PolynomialTrajectory:
    def __init__(self, duration):
        self.duration = duration
        self.coeffs_x = None
        self.coeffs_y = None
        self.coeffs_z = None
        self.coeffs_scalar = None  
        self.start_time = None
        self.is_scalar = False  

    def compute_coefficients(self, start, target):
        if abs(target - start) < 1e-6:
            return np.array([0, 0, 0, 0, 0, start])
        T = self.duration
        if T <= 0:
            raise ValueError("Duration must be positive.")
        A = np.array([
            [0,         0,      0,    0,  0, 1],
            [T**5,     T**4,   T**3,  T**2, T, 1],
            [0,         0,      0,    0,  1, 0],
            [5*T**4,   4*T**3, 3*T**2, 2*T, 1, 0],
            [0,         0,      0,    2,   0, 0],
            [20*T**3, 12*T**2, 6*T,    2,   0, 0]
        ])
        B = np.array([start, target, 0, 0, 0, 0])
        return np.linalg.solve(A, B)
    
    def generate_trajectory(self, start_pos, target_pos):
        if isinstance(start_pos, (int, float)) and isinstance(target_pos, (int, float)):
            self.is_scalar = True
            self.coeffs_scalar = self.compute_coefficients(start_pos, target_pos)
        else:
            self.is_scalar = False
            self.coeffs_x = self.compute_coefficients(start_pos[0], target_pos[0])
            self.coeffs_y = self.compute_coefficients(start_pos[1], target_pos[1])
            self.coeffs_z = self.compute_coefficients(start_pos[2], target_pos[2])
        self.start_time = rospy.Time.now().to_sec()

    def evaluate(self):
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        if elapsed_time > self.duration:
            return None 
        T = np.array([elapsed_time**5, elapsed_time**4, elapsed_time**3, 
                      elapsed_time**2, elapsed_time, 1])
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T)
        else:
            return (
                np.dot(self.coeffs_x, T),
                np.dot(self.coeffs_y, T),
                np.dot(self.coeffs_z, T)
            )

class ValveRotationTrajectory:
    def __init__(self, duration):
        self.trajectory = PolynomialTrajectory(duration)
        self.start_pos = None
        self.target_pos = None

    def set_start_target(self, start_pos, target_pos):
        self.start_pos = start_pos
        self.target_pos = target_pos
        self.trajectory.generate_trajectory(start_pos, target_pos)

    def get_next_position(self):
        return self.trajectory.evaluate()

    def is_complete(self):
        return self.get_next_position() is None